#!/usr/bin/env python3
"""
地面站显示脚本 — 双机协议版
帧格式: [0xAA] [DRONE_ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD...] [CRC8]
"""

import serial
import struct
import math
import os
import sys
import time
import select
import tty
import termios

FRAME_HEADER   = 0xAA
TYPE_HEARTBEAT = 0x01
TYPE_ODOM      = 0x02
TYPE_STATE     = 0x03
TYPE_BATTERY   = 0x04
TYPE_CMD       = 0x80
TYPE_CMD_ACK   = 0x81

CMD_START_MISSION = 0x20
ACK_NAMES = {0: "OK", 1: "REJECTED", 2: "TIMEOUT"}

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc

def make_cmd_frame(drone_id, cmd_id):
    payload = bytes([cmd_id])
    length = len(payload)
    frame = bytes([FRAME_HEADER, drone_id, TYPE_CMD, (length >> 8) & 0xFF, length & 0xFF])
    frame += payload
    frame += bytes([crc8(frame)])
    return frame

def quat_to_euler(qx, qy, qz, qw):
    sr_cp = 2 * (qw * qx + qy * qz)
    cr_cp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sr_cp, cr_cp)
    sp = max(-1, min(1, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)
    sy_cp = 2 * (qw * qz + qx * qy)
    cy_cp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(sy_cp, cy_cp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class DroneState:
    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.speed = 0.0
        self.mode = "---"
        self.armed = False
        self.odom_count = 0
        self.state_count = 0
        self.odom_hz = 0.0
        self.last_odom_time = None
        self.last_any_time = None
        self.batt_voltage = 0.0
        self.batt_percent = 0.0
        self.batt_current = 0.0

    @property
    def online(self):
        if self.last_any_time is None:
            return False
        return (time.time() - self.last_any_time) < 3.0

class GCSDisplay:
    def __init__(self, port='/dev/ttyACM1', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.buf = bytearray()
        self.drones = {1: DroneState(), 2: DroneState()}
        self.crc_errors = 0
        self.start_time = time.time()
        self.cmd_ack_status = ""
        self.active_drone = 1

    def parse_frames(self):
        while len(self.buf) >= 6:
            idx = self.buf.find(FRAME_HEADER)
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                self.buf = self.buf[idx:]

            if len(self.buf) < 6:
                break

            drone_id = self.buf[1]
            frame_type = self.buf[2]
            plen = (self.buf[3] << 8) | self.buf[4]
            total = 5 + plen + 1

            if plen > 250:
                self.buf.pop(0)
                continue

            if len(self.buf) < total:
                break

            frame = bytes(self.buf[:total])
            expected = crc8(frame[:5 + plen])
            if frame[total - 1] != expected:
                self.crc_errors += 1
                self.buf.pop(0)
                continue

            payload = frame[5:5 + plen]
            self.handle_frame(drone_id, frame_type, payload)
            self.buf = self.buf[total:]

    def handle_frame(self, drone_id, frame_type, payload):
        if frame_type == TYPE_CMD_ACK and len(payload) == 2:
            cmd_id, result = payload[0], payload[1]
            self.cmd_ack_status = "D%d: %s" % (drone_id, ACK_NAMES.get(result, "?"))
            return

        if drone_id not in self.drones:
            return
        d = self.drones[drone_id]
        d.last_any_time = time.time()

        if frame_type == TYPE_ODOM and len(payload) == 40:
            vals = struct.unpack('<10f', payload)
            d.pos = list(vals[0:3])
            d.rpy = list(quat_to_euler(vals[3], vals[4], vals[5], vals[6]))
            d.vel = list(vals[7:10])
            d.speed = math.sqrt(sum(v**2 for v in d.vel))
            d.odom_count += 1

            now = time.time()
            if d.last_odom_time:
                dt = now - d.last_odom_time
                if dt > 0:
                    d.odom_hz = 0.9 * d.odom_hz + 0.1 * (1.0 / dt)
            d.last_odom_time = now

        elif frame_type == TYPE_STATE and len(payload) == 17:
            d.armed = payload[0] == 1
            d.mode = payload[1:17].rstrip(b'\x00').decode('utf-8', errors='replace')
            d.state_count += 1

        elif frame_type == TYPE_BATTERY and len(payload) == 12:
            d.batt_voltage, d.batt_percent, d.batt_current = struct.unpack('<3f', payload)
            d.batt_percent *= 100.0

    def render_drone(self, drone_id):
        d = self.drones[drone_id]
        link = "\033[92mONLINE\033[0m" if d.online else "\033[91mOFFLINE\033[0m"
        armed = "\033[91mARMED\033[0m" if d.armed else "\033[92mDISARM\033[0m"

        lines = []
        lines.append(f"\033[1m--- Drone {drone_id} [{link}\033[1m] ---\033[0m")
        lines.append(f"  {d.mode:10s} {armed}")
        lines.append(f"  X:{d.pos[0]:+7.2f} Y:{d.pos[1]:+7.2f} Z:{d.pos[2]:+7.2f}")
        lines.append(f"  R:{d.rpy[0]:+6.1f}  P:{d.rpy[1]:+6.1f}  Y:{d.rpy[2]:+6.1f}")
        lines.append(f"  Spd:{d.speed:.2f} m/s   {d.odom_hz:.0f}Hz")
        lines.append(f"  Bat:{d.batt_voltage:.1f}V {d.batt_percent:.0f}%")
        lines.append(f"  odom:{d.odom_count}  state:{d.state_count}")
        return lines

    def refresh_display(self):
        elapsed = int(time.time() - self.start_time)
        os.system('clear')
        print(f"\033[1m============ GCS Ground Station ============\033[0m")
        print(f"  Uptime: {elapsed}s   CRC errors: {self.crc_errors}")
        print()

        d1 = self.render_drone(1)
        d2 = self.render_drone(2)

        for l1, l2 in zip(d1, d2):
            print(f"{l1:40s}  {l2}")
        if self.cmd_ack_status:
            print(f"  CMD: {self.cmd_ack_status}")
        print()
        print("\033[93m[M] 执行任务 (Drone {})   [1/2] 切换飞机   [Q] 退出\033[0m".format(self.active_drone))

    def send_cmd(self, drone_id, cmd_id):
        frame = make_cmd_frame(drone_id, cmd_id)
        self.ser.write(frame)
        self.cmd_ack_status = "D%d: SENDING..." % drone_id

    def run(self):
        last_display = 0
        old_settings = termios.tcgetattr(sys.stdin)
        print("GCS starting on", self.ser.port)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                n = self.ser.in_waiting
                if n > 0:
                    self.buf.extend(self.ser.read(n))
                    self.parse_frames()

                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'q':
                        break
                    elif key == 'm':
                        self.send_cmd(self.active_drone, CMD_START_MISSION)
                    elif key == '1':
                        self.active_drone = 1
                    elif key == '2':
                        self.active_drone = 2

                now = time.time()
                if now - last_display >= 0.5:
                    self.refresh_display()
                    last_display = now

                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.ser.close()
            print("\nStopped.")

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM1'
    GCSDisplay(port).run()
