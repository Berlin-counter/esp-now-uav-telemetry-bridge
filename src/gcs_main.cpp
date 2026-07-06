#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

// TJC 串口屏引脚 (UART1)
#define TJC_TX  18
#define TJC_RX  17
#define TJC_BAUD 115200

// 协议常量
#define FRAME_HEADER   0xAA
#define FRAME_HDR_SIZE 5
#define ESPNOW_MAX     250
#define USB_BUF_SIZE   256

#define TYPE_HEARTBEAT 0x01
#define TYPE_ODOM      0x02
#define TYPE_STATE     0x03
#define TYPE_BATTERY   0x04
#define TYPE_EKF       0x05
#define TYPE_CLOUD_META 0x06
#define TYPE_CLOUD_BATCH 0x07
#define TYPE_CMD       0x80
#define TYPE_CMD_ACK   0x81
#define TYPE_ESP_STATS 0xF0

#define CMD_START_MISSION 0x20

// TJC RGB565 颜色
#define COLOR_GREEN  2016
#define COLOR_RED    63488
#define COLOR_YELLOW 65504

// 点云绘制区域 (像素坐标，需与 TJC 编辑器中 pcView 位置一致)
#define PC_X0  535
#define PC_Y0  34
#define PC_W   489
#define PC_H   540

// xpic 俯仰姿态指示器 (屏幕坐标 + 大图参数)
// 大图: pitch_strip.png, 100x600, 地平线在 y=300
// 显示窗口: 100x100, crop_y=250 时为 pitch=0°
#define PITCH_X      107   // 屏幕上绘制位置 x
#define PITCH_Y      57    // 屏幕上绘制位置 y
#define PITCH_W      100   // 显示宽度
#define PITCH_H      100   // 显示高度
#define PITCH_IMG_H  600   // 大图总高度
#define PITCH_CENTER 118   // pitch=0° 时 crop_y (校准: 从163再-45修正-20°)
#define PITCH_PPD    3.33f // 每度对应的像素数
#define PITCH_PIC_ID 8     // 图片库编号

// 双机状态
struct DroneState {
    float x, y, z;
    float roll, pitch, yaw;
    float vx, vy, vz, speed;
    char mode[17];
    bool armed;
    float batt_v, batt_pct, batt_i;
    uint16_t ekf_flags;
    float ekf_vel, ekf_ph, ekf_pv, ekf_comp;
    uint32_t odom_count;
    uint32_t last_seen;
    bool dirty_odom, dirty_state, dirty_batt, dirty_ekf, dirty_ack;
    uint8_t ack_result;
};

static DroneState drones[2];
static uint8_t sel_drone = 1;

// 点云缓存 (每架最多 3000 点，约 12KB/架)
#define MAX_CLOUD_PTS 14000
static int16_t cloud_px[2][MAX_CLOUD_PTS];
static int16_t cloud_py[2][MAX_CLOUD_PTS];
static uint16_t cloud_cc[2][MAX_CLOUD_PTS];
static volatile uint16_t cloud_count[2] = {0, 0};
static uint16_t cloud_drawn[2] = {0, 0};        // 绘制泵进度
static bool cloud_need_clear[2] = {false, false};
static uint32_t cloud_draw_hold[2] = {0, 0};    // page 切换后延迟开画

static uint32_t rx_count = 0, rx_drone1 = 0, rx_drone2 = 0, tx_cmd_count = 0;
static uint8_t broadcast_mac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static uint8_t usb_buf[USB_BUF_SIZE];
static int usb_pos = 0;

static uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

// ---------- TJC 指令 ----------

static char _tb[64];

static void tjc_raw(const char* cmd) {
    Serial1.print(cmd);
    Serial1.write(0xFF);
    Serial1.write(0xFF);
    Serial1.write(0xFF);
}

static void tjc_val(const char* id, int val) {
    snprintf(_tb, sizeof(_tb), "%s.val=%d", id, val);
    tjc_raw(_tb);
}

static void tjc_pco(const char* id, uint16_t color) {
    snprintf(_tb, sizeof(_tb), "%s.pco=%d", id, color);
    tjc_raw(_tb);
}

static void tjc_txt(const char* id, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = snprintf(_tb, sizeof(_tb), "%s.txt=\"", id);
    n += vsnprintf(_tb + n, sizeof(_tb) - n, fmt, ap);
    va_end(ap);
    if (n < (int)sizeof(_tb) - 1) _tb[n++] = '"';
    _tb[n] = 0;
    tjc_raw(_tb);
}

// ---------- 点云重绘 ----------

// 重绘 = 重置绘制泵进度, 实际绘制由 loop 里的泵分片完成 (不阻塞)
static void redraw_cloud(uint8_t di) {
    cloud_drawn[di] = 0;
    cloud_need_clear[di] = true;
    cloud_draw_hold[di] = millis() + 250;  // 等 TJC page 切换渲染完成
    Serial.printf("\n[REDRAW] queued di=%d pts=%d\n", di, cloud_count[di]);
}

// ---------- 四元数 → 欧拉角 ----------

static void quat2rpy(float qx, float qy, float qz, float qw,
                     float &r, float &p, float &y) {
    r = atan2f(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)) * 57.2958f;
    float sp = 2*(qw*qy-qz*qx);
    p = asinf(sp > 1 ? 1 : (sp < -1 ? -1 : sp)) * 57.2958f;
    y = atan2f(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)) * 57.2958f;
}

// ---------- ESP-NOW 接收 ----------

void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
    Serial.write(data, len);
    rx_count++;

    if (len < FRAME_HDR_SIZE + 1) return;
    uint8_t did = data[1];
    uint8_t ftype = data[2];
    int plen = (data[3] << 8) | data[4];
    const uint8_t *pl = data + 5;

    if (did < 1 || did > 2) return;
    DroneState &d = drones[did - 1];
    d.last_seen = millis();
    if (did == 1) rx_drone1++; else rx_drone2++;

    switch (ftype) {
    case TYPE_ODOM:
        if (plen >= 40) {
            float v[10]; memcpy(v, pl, 40);
            d.x = v[0]; d.y = v[1]; d.z = v[2];
            quat2rpy(v[3], v[4], v[5], v[6], d.roll, d.pitch, d.yaw);
            d.vx = v[7]; d.vy = v[8]; d.vz = v[9];
            d.speed = sqrtf(v[7]*v[7] + v[8]*v[8] + v[9]*v[9]);
            d.odom_count++;
            d.dirty_odom = true;
        }
        break;
    case TYPE_STATE:
        if (plen >= 17) {
            d.armed = pl[0];
            memset(d.mode, 0, 17);
            memcpy(d.mode, pl + 1, 16);
            d.dirty_state = true;
        }
        break;
    case TYPE_BATTERY:
        if (plen >= 12) {
            float b[3]; memcpy(b, pl, 12);
            d.batt_v = b[0]; d.batt_pct = b[1]; d.batt_i = b[2];
            d.dirty_batt = true;
        }
        break;
    case TYPE_EKF:
        if (plen >= 18) {
            memcpy(&d.ekf_flags, pl, 2);
            float e[4]; memcpy(e, pl + 2, 16);
            d.ekf_vel = e[0]; d.ekf_ph = e[1];
            d.ekf_pv = e[2]; d.ekf_comp = e[3];
            d.dirty_ekf = true;
        }
        break;
    case TYPE_CMD_ACK:
        if (plen >= 2) {
            d.ack_result = pl[1];
            d.dirty_ack = true;
        }
        break;
    // 点云帧只存内存 (微秒级, 不阻塞 WiFi 任务不丢帧), 绘制由 loop 泵完成
    case TYPE_CLOUD_META:
        if (plen >= 8) {
            int di = did - 1;
            cloud_count[di] = 0;
            cloud_drawn[di] = 0;
            cloud_need_clear[di] = (did == sel_drone);
            cloud_draw_hold[di] = millis();
        }
        break;
    case TYPE_CLOUD_BATCH:
        if (plen >= 6) {
            int n_pts = plen / 6;
            int di = did - 1;
            for (int i = 0; i < n_pts; i++) {
                if (cloud_count[di] >= MAX_CLOUD_PTS) break;
                memcpy(&cloud_px[di][cloud_count[di]], pl + i*6, 2);
                memcpy(&cloud_py[di][cloud_count[di]], pl + i*6 + 2, 2);
                memcpy(&cloud_cc[di][cloud_count[di]], pl + i*6 + 4, 2);
                cloud_count[di]++;
            }
        }
        break;
    }
}

// ---------- 发送任务指令 ----------

static void send_mission() {
    uint8_t f[7] = {FRAME_HEADER, sel_drone, TYPE_CMD, 0, 1, CMD_START_MISSION, 0};
    f[6] = crc8(f, 6);
    esp_now_send(broadcast_mac, f, 7);
    tx_cmd_count++;
    tjc_txt("cmdStatus", "已发送");
}

// ---------- setup ----------

void setup() {
    Serial.begin(115200);
    // USB-CDC 无人读取时 write 默认阻塞 ~100ms/帧, 会卡死 WiFi 任务导致 ESP-NOW 大量丢帧
    Serial.setTxTimeoutMs(0);
    Serial1.begin(TJC_BAUD, SERIAL_8N1, TJC_RX, TJC_TX);

    memset(drones, 0, sizeof(drones));

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);  // 默认 modem sleep 会周期性关射频, ESP-NOW 漏收帧
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(on_recv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    delay(300);
    tjc_txt("dLink", "● LINK");
    tjc_txt("cmdStatus", "就绪");
}

// ---------- loop ----------

void loop() {
    uint32_t now = millis();

    // ===== TJC 按钮事件 =====
    // TJC 按钮 Touch Press Event 中配置:
    //   btnMission: printh 4D    (ASCII 'M')
    //   d1btn:      printh 31    (ASCII '1')
    //   d2btn:      printh 32    (ASCII '2')
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == 'M') send_mission();
        else if (c == '1') { sel_drone = 1; tjc_txt("cmdStatus", "就绪"); redraw_cloud(0); }
        else if (c == '2') { sel_drone = 2; tjc_txt("cmdStatus", "就绪"); redraw_cloud(1); }
    }

    // ===== USB 指令转发 (PC 调试/gcs_display.py) =====
    int avail = Serial.available();
    if (avail > 0) {
        int to_read = min(avail, USB_BUF_SIZE - usb_pos);
        if (to_read > 0) {
            Serial.readBytes(usb_buf + usb_pos, to_read);
            usb_pos += to_read;
        }
        while (usb_pos >= FRAME_HDR_SIZE + 1) {
            int i = 0;
            for (; i < usb_pos; i++) if (usb_buf[i] == FRAME_HEADER) break;
            if (i > 0) { memmove(usb_buf, usb_buf + i, usb_pos - i); usb_pos -= i; }
            if (usb_pos < FRAME_HDR_SIZE + 1) break;
            int plen = (usb_buf[3] << 8) | usb_buf[4];
            int total = FRAME_HDR_SIZE + plen + 1;
            if (plen > ESPNOW_MAX) { usb_pos = 0; break; }
            if (usb_pos < total) break;
            if (usb_buf[total - 1] == crc8(usb_buf, FRAME_HDR_SIZE + plen)) {
                esp_now_send(broadcast_mac, usb_buf, total);
                tx_cmd_count++;
            }
            int rem = usb_pos - total;
            if (rem > 0) memmove(usb_buf, usb_buf + total, rem);
            usb_pos = rem;
        }
        if (usb_pos >= USB_BUF_SIZE) usb_pos = 0;
    }

    // ===== 点云绘制泵 (唯一的点云 TJC 写入点, 每次最多 20 点保持 loop 响应) =====
    {
        uint8_t di = sel_drone - 1;
        if ((int32_t)(now - cloud_draw_hold[di]) >= 0) {
            if (cloud_need_clear[di]) {
                snprintf(_tb, sizeof(_tb), "fill %d,%d,%d,%d,0", PC_X0, PC_Y0, PC_W, PC_H);
                tjc_raw(_tb);
                cloud_need_clear[di] = false;
            }
            uint16_t budget = 20;
            bool drew = false;
            while (cloud_drawn[di] < cloud_count[di] && budget--) {
                uint16_t i = cloud_drawn[di]++;
                int sx = PC_X0 + cloud_px[di][i];
                int sy = PC_Y0 + cloud_py[di][i];
                if (sx >= PC_X0 && sx < PC_X0+PC_W && sy >= PC_Y0 && sy < PC_Y0+PC_H) {
                    snprintf(_tb, sizeof(_tb), "cirs %d,%d,2,%u", sx, sy, cloud_cc[di][i]);
                    tjc_raw(_tb);
                }
                drew = true;
            }
            static uint16_t last_reported[2] = {0, 0};
            if (drew && cloud_drawn[di] == cloud_count[di]
                && last_reported[di] != cloud_count[di]) {
                last_reported[di] = cloud_count[di];
                tjc_txt("pcPts", "%d", cloud_count[di]);
                Serial.printf("\n[CLOUD] drawn %d pts complete\n", cloud_count[di]);
            }
        }
    }

    // ===== TJC 显示刷新 =====
    DroneState &d = drones[sel_drone - 1];

    // 里程计 10Hz 限速
    static uint32_t last_odom_push = 0;
    if (d.dirty_odom && now - last_odom_push >= 100) {
        tjc_txt("dX", "%+.2fm", d.x);
        tjc_txt("dY", "%+.2fm", d.y);
        tjc_txt("dZ", "%+.2fm", d.z);
        tjc_txt("dR", "%+.1f", d.roll);
        tjc_txt("dP", "%+.1f", d.pitch);
        tjc_txt("dW", "%+.0f", d.yaw);
        tjc_txt("dHdg", "%03.0f", d.yaw < 0 ? d.yaw + 360 : d.yaw);
        tjc_txt("dVx", "%+.2f", d.vx);
        tjc_txt("dVy", "%+.2f", d.vy);
        tjc_txt("dVz", "%+.2f", d.vz);
        tjc_txt("dSpd", "%.2fm/s", d.speed);
        // Gauge 组件数值 (指针旋转)
        int hdg = (int)(d.yaw < 0 ? d.yaw + 360 : d.yaw);
        tjc_val("compass", hdg);
        tjc_val("gaugeR", (int)(d.roll + 180));
        // 俯仰: xpic 裁剪大图实现天地线滚动
        int crop_y = PITCH_CENTER + (int)(d.pitch * PITCH_PPD);
        if (crop_y < 0) crop_y = 0;
        if (crop_y > PITCH_IMG_H - PITCH_H) crop_y = PITCH_IMG_H - PITCH_H;
        snprintf(_tb, sizeof(_tb), "xpic %d,%d,%d,%d,7,%d,%d",
                 PITCH_X, PITCH_Y, PITCH_W, PITCH_H, crop_y, PITCH_PIC_ID);
        tjc_raw(_tb);
        d.dirty_odom = false;
        last_odom_push = now;
    }

    if (d.dirty_state) {
        tjc_txt("dMode", "%s", d.mode);
        tjc_txt("dArm", "%s", d.armed ? "ARMED" : "DISARMED");
        d.dirty_state = false;
    }

    if (d.dirty_batt) {
        tjc_txt("dBatV", "%.1fV", d.batt_v);
        tjc_txt("dBatP", "%.0f%%", d.batt_pct * 100);
        tjc_txt("dBatI", "%.1fA", d.batt_i);
        d.dirty_batt = false;
    }

    if (d.dirty_ekf) {
        tjc_txt("dEkfF", "%d", d.ekf_flags);
        tjc_txt("dEkfV", "%.3f", d.ekf_vel);
        d.dirty_ekf = false;
    }

    if (d.dirty_ack) {
        const char* s = d.ack_result == 0 ? "任务执行中" :
                        d.ack_result == 1 ? "拒绝:未解锁" : "指令失败";
        tjc_txt("cmdStatus", "%s", s);
        d.dirty_ack = false;
    }

    // ===== 1Hz 统计 =====
    static uint32_t last_hz = 0;
    static uint32_t prev_odom = 0;
    if (now - last_hz >= 1000) {
        uint32_t cnt = d.odom_count;
        tjc_txt("dHz", "%dHz", cnt - prev_odom);
        prev_odom = cnt;

        bool online = d.last_seen > 0 && (now - d.last_seen) < 3000;
        tjc_pco("dLink", online ? COLOR_GREEN : COLOR_RED);
        tjc_txt("dLink", "%s", online ? "● LINK" : "× OFFLINE");
        tjc_txt("dUp", "%ds", now / 1000);
        tjc_txt("dTxOk", "%d", rx_count);

        Serial.printf("[GCS] rx=%d d1=%d d2=%d tx=%d sel=D%d\n",
            rx_count, rx_drone1, rx_drone2, tx_cmd_count, sel_drone);
        last_hz = now;
    }
}
