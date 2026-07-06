#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// 广播模式: 单播实测在强干扰下 ACK 丢失导致重传等待拖垮转发速率 (串口积压溢出),
// 广播无 ACK 等待, 配合两端 WIFI_PS_NONE 已足够可靠。
static uint8_t gcs_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 双机协议帧: [0xAA] [DRONE_ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD...] [CRC8]
// 帧头 5 字节 + payload + CRC 1 字节 = plen + 6
#define FRAME_HEADER   0xAA
#define FRAME_HDR_SIZE 5
#define TYPE_HEARTBEAT 0x01
#define TYPE_ODOM      0x02
#define TYPE_STATE     0x03
#define TYPE_ESP_STATS 0xF0

#define USB_BUF_SIZE   8192  // 512B 仅 ~100ms 余量, 信道繁忙 CSMA 退避时会溢出丢字节
#define ESPNOW_MAX     250
#define STATS_INTERVAL 1000

static uint8_t usb_buf[USB_BUF_SIZE];
static int usb_buf_pos = 0;
static volatile bool espnow_send_done = true;

static uint32_t rx_frame_count = 0;
static uint32_t tx_espnow_count = 0;
static uint32_t tx_espnow_fail = 0;
static uint32_t crc_errors = 0;
static uint8_t last_frame_type = 0;
static uint32_t last_stats_time = 0;

static uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

// 双机帧解析: [AA] [ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD] [CRC]
static int find_frame(const uint8_t *buf, int len, int *frame_start, int *frame_len) {
    for (int i = 0; i < len; i++) {
        if (buf[i] != FRAME_HEADER) continue;
        if (i + FRAME_HDR_SIZE > len) return 0;

        int plen = (buf[i + 3] << 8) | buf[i + 4];
        int total = FRAME_HDR_SIZE + plen + 1;

        if (plen > ESPNOW_MAX) {
            *frame_start = i;
            *frame_len = 1;
            return -1;
        }
        if (i + total > len) return 0;

        uint8_t expected = crc8(buf + i, FRAME_HDR_SIZE + plen);
        if (buf[i + total - 1] != expected) {
            *frame_start = i;
            *frame_len = 1;
            return -1;
        }

        *frame_start = i;
        *frame_len = total;
        return 1;
    }
    return 0;
}

void on_espnow_send(const uint8_t *mac, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS)
        tx_espnow_count++;
    else
        tx_espnow_fail++;
    espnow_send_done = true;
}

void on_espnow_recv(const uint8_t *mac, const uint8_t *data, int len) {
    Serial.write(data, len);
}

static void send_stats() {
    // 统计帧不经过 ESP-NOW，只回传给本机 Jetson，用旧格式（无 DRONE_ID）也行
    // 但为保持一致，统一用新格式，ID=0x00 表示本机统计
    uint8_t payload[21];
    uint32_t uptime = millis() / 1000;
    memcpy(payload + 0,  &rx_frame_count, 4);
    memcpy(payload + 4,  &tx_espnow_count, 4);
    memcpy(payload + 8,  &tx_espnow_fail, 4);
    memcpy(payload + 12, &crc_errors, 4);
    payload[16] = last_frame_type;
    memcpy(payload + 17, &uptime, 4);

    uint8_t frame[FRAME_HDR_SIZE + 21 + 1]; // 27 bytes
    frame[0] = FRAME_HEADER;
    frame[1] = 0x00;         // DRONE_ID=0 for local stats
    frame[2] = TYPE_ESP_STATS;
    frame[3] = 0;
    frame[4] = 21;
    memcpy(frame + 5, payload, 21);
    frame[26] = crc8(frame, 26);
    Serial.write(frame, 27);
}

void setup() {
    // 默认 RX 环形缓冲仅 256B, 单播重传等待期间会被串口数据灌爆导致丢帧
    Serial.setRxBufferSize(8192);
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);  // 默认 modem sleep 会周期性关射频, ESP-NOW 漏收帧/丢ACK

    if (esp_now_init() != ESP_OK) return;

    esp_now_register_send_cb(on_espnow_send);
    esp_now_register_recv_cb(on_espnow_recv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, gcs_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

void loop() {
    uint32_t now = millis();
    if (now - last_stats_time >= STATS_INTERVAL) {
        send_stats();
        last_stats_time = now;
    }

    int avail = Serial.available();
    if (avail > 0) {
        int to_read = min(avail, USB_BUF_SIZE - usb_buf_pos);
        if (to_read > 0) {
            Serial.readBytes(usb_buf + usb_buf_pos, to_read);
            usb_buf_pos += to_read;
        }

        while (usb_buf_pos > 0 && espnow_send_done) {
            int fs, fl;
            int result = find_frame(usb_buf, usb_buf_pos, &fs, &fl);

            if (result == 1) {
                last_frame_type = usb_buf[fs + 2]; // TYPE is at offset 2 now
                espnow_send_done = false;
                esp_now_send(gcs_mac, usb_buf + fs, fl);
                rx_frame_count++;

                int remaining = usb_buf_pos - (fs + fl);
                if (remaining > 0)
                    memmove(usb_buf, usb_buf + fs + fl, remaining);
                usb_buf_pos = remaining;
            } else if (result == -1) {
                crc_errors++;
                int remaining = usb_buf_pos - (fs + 1);
                if (remaining > 0)
                    memmove(usb_buf, usb_buf + fs + 1, remaining);
                usb_buf_pos = remaining;
            } else {
                break;
            }
        }

        if (usb_buf_pos >= USB_BUF_SIZE)
            usb_buf_pos = 0;
    }
}
