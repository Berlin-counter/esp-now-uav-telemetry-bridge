#!/usr/bin/env python3
"""
ESP32 桥接脚本 — 双机协议版 + 点云传输
帧格式: [0xAA] [DRONE_ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD...] [CRC8]

点云传输: 降落后写入触发文件即可发送 PCD 到地面站
  echo auto > ~/esp32_bridge/.cloud_trigger          # 自动查找 PCD
  echo /path/to/scans.pcd > ~/esp32_bridge/.cloud_trigger  # 指定路径
"""

import rospy
import serial
import struct
import math
import threading
import sys
import os
import time
import argparse
import glob
import subprocess
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, Mavlink
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import BatteryState

FRAME_HEADER   = 0xAA
TYPE_HEARTBEAT = 0x01
TYPE_ODOM      = 0x02
TYPE_STATE     = 0x03
TYPE_BATTERY   = 0x04
TYPE_EKF       = 0x05
TYPE_CLOUD_META  = 0x06
TYPE_CLOUD_BATCH = 0x07
TYPE_CMD       = 0x80
TYPE_CMD_ACK   = 0x81
TYPE_ESP_STATS = 0xF0

CMD_START_MISSION = 0x20

CLOUD_TRIGGER = os.path.expanduser("~/esp32_bridge/.cloud_trigger")
CLOUD_CANVAS_W = 489
CLOUD_CANVAS_H = 540
CLOUD_BATCH_SIZE = 40
CLOUD_Z_MIN = 0.0
CLOUD_Z_MAX = 3.0
CLOUD_VOXEL = 0.01
CLOUD_FRAME_DELAY = 0.08  # 单播重传占用空口时间, 过快会挤爆机载 ESP32 发送队列
PCD_SEARCH_PATHS = [
    os.path.expanduser("~/catkin_ws/src/FAST_LIO/PCD/scans.pcd"),
    os.path.expanduser("~/FAST_LIO_SLAM/FAST-LIO/PCD/scans.pcd"),
    os.path.expanduser("~/PCD/scans.pcd"),
]

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc

def make_frame(drone_id, msg_type, payload):
    length = len(payload)
    frame = bytes([FRAME_HEADER, drone_id, msg_type, (length >> 8) & 0xFF, length & 0xFF])
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

def find_pcd():
    for p in PCD_SEARCH_PATHS:
        if os.path.isfile(p) and os.path.getsize(p) > 100:
            return p
    candidates = glob.glob(os.path.expanduser("~/*/PCD/*.pcd"))
    candidates += glob.glob(os.path.expanduser("~/catkin_ws/src/*/PCD/*.pcd"))
    candidates = [c for c in candidates if os.path.getsize(c) > 100]
    if candidates:
        return max(candidates, key=os.path.getmtime)
    return None


def read_pcd(path):
    with open(path, 'rb') as f:
        fields, sizes, types, counts = [], [], [], []
        n_points = 0
        data_fmt = 'binary'
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            key = parts[0]
            if key == 'FIELDS':
                fields = parts[1:]
            elif key == 'SIZE':
                sizes = [int(v) for v in parts[1:]]
            elif key == 'TYPE':
                types = parts[1:]
            elif key == 'COUNT':
                counts = [int(v) for v in parts[1:]]
            elif key == 'POINTS':
                n_points = int(parts[1])
            elif key == 'DATA':
                data_fmt = parts[1] if len(parts) > 1 else 'binary'
                break
        if n_points == 0 or 'x' not in fields:
            return np.empty((0, 3), dtype=np.float32)
        if data_fmt == 'ascii':
            xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
            rows = []
            for line in f:
                vals = line.decode('ascii', errors='ignore').split()
                if len(vals) > max(xi, yi, zi):
                    rows.append([float(vals[xi]), float(vals[yi]), float(vals[zi])])
                if len(rows) >= n_points:
                    break
            return np.array(rows, dtype=np.float32) if rows else np.empty((0, 3), dtype=np.float32)
        type_map = {
            ('F', 4): '<f4', ('F', 8): '<f8',
            ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4',
            ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4',
        }
        dt_list = []
        for i in range(len(fields)):
            np_t = type_map.get((types[i], sizes[i]), '<f4')
            c = counts[i] if i < len(counts) else 1
            dt_list.append((fields[i], np_t, (c,)) if c > 1 else (fields[i], np_t))
        dtype = np.dtype(dt_list)
        raw = np.frombuffer(f.read(n_points * dtype.itemsize), dtype=dtype, count=n_points)
        return np.column_stack([raw['x'].astype(np.float32),
                                raw['y'].astype(np.float32),
                                raw['z'].astype(np.float32)])


def z_to_rgb565(z, z_lo, z_hi):
    t = max(0.0, min(1.0, (z - z_lo) / (z_hi - z_lo))) if z_hi > z_lo else 0.5
    if t < 0.25:
        r, g, b = 0, t * 4, 1
    elif t < 0.5:
        r, g, b = 0, 1, 1 - (t - 0.25) * 4
    elif t < 0.75:
        r, g, b = (t - 0.5) * 4, 1, 0
    else:
        r, g, b = 1, 1 - (t - 0.75) * 4, 0
    return (int(r * 31) << 11) | (int(g * 63) << 5) | int(b * 31)


# 总点数硬预算: 必须 < GCS 固件 MAX_CLOUD_PTS (14000), 且决定传输时长 (~0.08s/40点)
CLOUD_POINT_BUDGET = 13000

def project_to_canvas(points, z_min, z_max, canvas_w, canvas_h, voxel):
    # 分层下采样: 墙壁全密度, 地面/天花板稀疏点缀
    layers = [
        (0.0,  0.3,  0.05),
        (0.3,  2.5,  0),
        (2.5,  3.2,  0.15),
    ]
    all_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    all_pts = points[all_mask]
    if len(all_pts) == 0:
        return [], 0.0, 0.0
    # 包围盒用 1%~99% 分位数: 穿门窗的远距离离群点会把 min/max 撑大数倍,
    # 导致真实房间被缩成画布一角; 分位数外的点投影出画布自然被丢弃
    x_min, x_max = np.percentile(all_pts[:, 0], [1, 99])
    y_min, y_max = np.percentile(all_pts[:, 1], [1, 99])
    x_range = x_max - x_min
    y_range = y_max - y_min
    if x_range < 0.01 or y_range < 0.01:
        return [], x_range, y_range
    margin = 5
    scale = min((canvas_w - 2 * margin) / x_range,
                (canvas_h - 2 * margin) / y_range)
    cx, cy = canvas_w / 2.0, canvas_h / 2.0
    mx, my = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
    seen = set()
    layer_coords = []
    for zlo, zhi, vx in layers:
        mask = (points[:, 2] >= max(zlo, z_min)) & (points[:, 2] <= min(zhi, z_max))
        pts = points[mask]
        cur = []
        if len(pts) > 0:
            if vx > 0:
                grid = np.floor(pts[:, :2] / vx).astype(np.int32)
                _, idx = np.unique(grid, axis=0, return_index=True)
                pts = pts[idx]
            px = ((pts[:, 0] - mx) * scale + cx).astype(np.int16)
            py = (-(pts[:, 1] - my) * scale + cy).astype(np.int16)
            for i in range(len(px)):
                key = (int(px[i]), int(py[i]))
                if 0 <= key[0] < canvas_w and 0 <= key[1] < canvas_h and key not in seen:
                    seen.add(key)
                    color = z_to_rgb565(float(pts[i, 2]), z_min, z_max)
                    cur.append((key[0], key[1], color))
        layer_coords.append(cur)

    floor_c, wall_c, ceil_c = layer_coords
    total = len(floor_c) + len(wall_c) + len(ceil_c)
    if total > CLOUD_POINT_BUDGET:
        # 大场地保护: 均匀抽稀墙壁层 (地面/天花板本来就稀), 保持轮廓形状
        wall_keep = max(1000, CLOUD_POINT_BUDGET - len(floor_c) - len(ceil_c))
        if len(wall_c) > wall_keep:
            step = len(wall_c) / float(wall_keep)
            wall_c = [wall_c[int(i * step)] for i in range(wall_keep)]
        coords = (floor_c + wall_c + ceil_c)[:CLOUD_POINT_BUDGET]
    else:
        coords = floor_c + wall_c + ceil_c
    return coords, x_range, y_range


def open_serial(port, baud):
    while not rospy.is_shutdown():
        try:
            s = serial.Serial(port, baud, timeout=0.1)
            rospy.loginfo("Serial opened: %s", port)
            return s
        except (serial.SerialException, OSError):
            rospy.logwarn("Waiting for %s ...", port)
            time.sleep(3)
    return None

class ESP32Bridge:
    def __init__(self, drone_id, port, baud):
        rospy.init_node('esp32_bridge', anonymous=True)

        self.drone_id = drone_id
        self.port = port
        self.baud = baud
        self.ser = open_serial(port, baud)
        self.ser_lock = threading.Lock()
        self.serial_ok = self.ser is not None

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
        self.batt_voltage = 0.0
        self.batt_percent = 0.0
        self.batt_current = 0.0

        self.esp_rx = 0
        self.esp_tx_ok = 0
        self.esp_tx_fail = 0
        self.esp_crc_err = 0
        self.esp_uptime = 0

        rospy.Subscriber('/Odometry', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)
        rospy.Subscriber('/mavlink/from', Mavlink, self.on_mavlink, queue_size=10)

        self.ekf_flags = 0
        self.ekf_vel_var = 0.0
        self.ekf_pos_h_var = 0.0
        self.ekf_pos_v_var = 0.0
        self.ekf_comp_var = 0.0
        self.ekf_ok = False

        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/mavros/set_mode', timeout=5)
                break
            except rospy.ROSException:
                rospy.logwarn("Waiting for /mavros/set_mode ...")
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.last_cmd_status = ""

        self._prev_armed = False
        self.cloud_status = "IDLE"
        self.cloud_pts = 0
        self._cloud_send_lock = threading.Lock()
        self._cloud_thread = threading.Thread(target=self._cloud_watcher, daemon=True)
        self._cloud_thread.start()

        rospy.Timer(rospy.Duration(1.0), self.send_heartbeat)
        rospy.Timer(rospy.Duration(0.05), self.read_esp32)
        rospy.Timer(rospy.Duration(0.5), self.refresh_display)

    def _reconnect(self):
        self.serial_ok = False
        with self.ser_lock:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = open_serial(self.port, self.baud)
            if self.ser:
                self.serial_ok = True

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        self.pos = [p.x, p.y, p.z]
        self.rpy = list(quat_to_euler(q.x, q.y, q.z, q.w))
        self.vel = [v.x, v.y, v.z]
        self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

        now = rospy.get_time()
        if self.last_odom_time:
            dt = now - self.last_odom_time
            if dt > 0:
                self.odom_hz = 0.9 * self.odom_hz + 0.1 * (1.0 / dt)
        self.last_odom_time = now

        payload = struct.pack('<10f',
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            v.x, v.y, v.z
        )
        self._send(TYPE_ODOM, payload)
        self.odom_count += 1

    def on_state(self, msg):
        self.mode = msg.mode
        was_armed = self._prev_armed
        self.armed = msg.armed
        self._prev_armed = msg.armed
        if was_armed and not msg.armed:
            rospy.loginfo("DISARM detected, triggering PCD save + cloud send")
            threading.Thread(target=self._on_disarm, daemon=True).start()
        mode_bytes = msg.mode.encode('utf-8')[:16].ljust(16, b'\x00')
        payload = struct.pack('<B', 1 if msg.armed else 0) + mode_bytes
        self._send(TYPE_STATE, payload)
        self.state_count += 1

    def on_battery(self, msg):
        self.batt_voltage = msg.voltage
        self.batt_percent = msg.percentage * 100.0
        self.batt_current = msg.current
        payload = struct.pack('<3f', msg.voltage, msg.percentage, msg.current)
        self._send(TYPE_BATTERY, payload)

    def on_mavlink(self, msg):
        if msg.msgid != 193:
            return
        raw = b''
        for v in msg.payload64:
            raw += struct.pack('<Q', v)
        raw = raw[:msg.len]
        if len(raw) < 18:
            return
        vel, ph, pv, comp = struct.unpack('<ffff', raw[0:16])
        flags_bytes = raw[20:22] if len(raw) >= 22 else raw[20:21] + b'\x00'
        flags = struct.unpack('<H', flags_bytes)[0]

        self.ekf_vel_var = vel
        self.ekf_pos_h_var = ph
        self.ekf_pos_v_var = pv
        self.ekf_comp_var = comp
        self.ekf_flags = flags
        self.ekf_ok = vel < 1.0 and ph < 1.0

        payload = struct.pack('<Hffff', flags, vel, ph, pv, comp)
        self._send(TYPE_EKF, payload)

    def send_heartbeat(self, event):
        payload = struct.pack('<I', self.odom_count)
        self._send(TYPE_HEARTBEAT, payload)

    def _send(self, msg_type, payload):
        if not self.serial_ok:
            return
        frame = make_frame(self.drone_id, msg_type, payload)
        with self.ser_lock:
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError):
                rospy.logwarn("Serial write failed, reconnecting...")
                threading.Thread(target=self._reconnect, daemon=True).start()

    def read_esp32(self, event):
        if not self.serial_ok:
            return
        with self.ser_lock:
            try:
                n = self.ser.in_waiting
                if n == 0:
                    return
                data = self.ser.read(n)
            except (serial.SerialException, OSError):
                rospy.logwarn("Serial read failed, reconnecting...")
                threading.Thread(target=self._reconnect, daemon=True).start()
                return
        i = 0
        while i < len(data):
            if data[i] != FRAME_HEADER:
                i += 1
                continue
            if i + 5 > len(data):
                break
            frame_type = data[i + 2]
            plen = (data[i + 3] << 8) | data[i + 4]
            total = 5 + plen + 1
            if i + total > len(data):
                break

            expected_crc = crc8(bytes(data[i:i + 5 + plen]))
            if data[i + total - 1] != expected_crc:
                i += 1
                continue

            payload = data[i + 5: i + 5 + plen]

            if frame_type == TYPE_ESP_STATS and plen == 21:
                self.esp_rx, self.esp_tx_ok, self.esp_tx_fail, self.esp_crc_err = \
                    struct.unpack('<IIII', payload[:16])
                self.esp_uptime = struct.unpack('<I', payload[17:21])[0]

            elif frame_type == TYPE_CMD and plen >= 1:
                self.handle_cmd(payload)

            i += total

    def handle_cmd(self, payload):
        cmd_id = payload[0]
        if cmd_id == CMD_START_MISSION:
            if not self.armed:
                rospy.logwarn("CMD: START_MISSION rejected - not armed")
                self.last_cmd_status = "REJECTED: NOT ARMED"
                self.send_cmd_ack(cmd_id, 1)
                return
            try:
                resp = self.set_mode_srv(custom_mode='GUIDED')
                if resp.mode_sent:
                    rospy.loginfo("CMD: START_MISSION -> GUIDED OK")
                    self.last_cmd_status = "GUIDED SENT"
                    self.send_cmd_ack(cmd_id, 0)
                else:
                    rospy.logwarn("CMD: START_MISSION -> GUIDED FAILED")
                    self.last_cmd_status = "GUIDED FAILED"
                    self.send_cmd_ack(cmd_id, 1)
            except rospy.ServiceException as e:
                rospy.logerr("CMD: set_mode service failed: %s", e)
                self.last_cmd_status = "SERVICE ERROR"
                self.send_cmd_ack(cmd_id, 2)
        else:
            rospy.logwarn("CMD: unknown cmd_id 0x%02X", cmd_id)

    def send_cmd_ack(self, cmd_id, result):
        payload = struct.pack('<BB', cmd_id, result)
        self._send(TYPE_CMD_ACK, payload)

    def _on_disarm(self):
        try:
            pid = subprocess.check_output(
                ['pidof', 'fastlio_mapping'], text=True).strip()
        except subprocess.CalledProcessError:
            rospy.logwarn("Cloud: FAST-LIO not running, skip SIGUSR1")
            pcd_path = find_pcd()
            if pcd_path:
                self._send_cloud(pcd_path)
            return

        # 新鲜度判定: 文件必须写于"本次信号之后" (与当前时钟比较, 同一开机会话内自洽)
        # 注意不能与旧文件 mtime 比较: Jetson 时钟每次开机重置 1970, 跨重启比较无意义
        expect = PCD_SEARCH_PATHS[0]
        t_signal = time.time()

        rospy.loginfo("Cloud: sending SIGUSR1 to FAST-LIO (pid %s)", pid)
        os.kill(int(pid.split()[0]), 10)  # SIGUSR1 = 10
        self.cloud_status = "WAIT PCD"
        fresh = False
        last_size = -1
        for _ in range(60):
            time.sleep(1)
            if not os.path.isfile(expect):
                continue
            st = os.stat(expect)
            if st.st_mtime < t_signal - 2:
                continue  # 不是本次信号触发写出的文件
            if st.st_size == last_size and st.st_size > 100:
                fresh = True  # 大小两次采样不变 = 写完了
                break
            last_size = st.st_size
        if not fresh:
            self.cloud_status = "PCD STALE/TIMEOUT"
            rospy.logwarn("Cloud: fresh PCD not ready after SIGUSR1, abort send")
            return
        self._send_cloud(expect)

    def _cloud_watcher(self):
        while not rospy.is_shutdown():
            time.sleep(2)
            if not os.path.isfile(CLOUD_TRIGGER):
                continue
            try:
                with open(CLOUD_TRIGGER, 'r') as f:
                    content = f.read().strip()
                os.remove(CLOUD_TRIGGER)
            except Exception:
                continue
            if content == 'auto' or content == '':
                pcd_path = find_pcd()
            else:
                pcd_path = content
            if not pcd_path or not os.path.isfile(pcd_path):
                self.cloud_status = "PCD NOT FOUND"
                rospy.logwarn("Cloud: PCD not found: %s", pcd_path or "auto")
                continue
            self._send_cloud(pcd_path)

    def _send_cloud(self, pcd_path):
        # 发送互斥: 传输可能持续 ~30s, 期间新触发直接丢弃 (防批次交错)
        if not self._cloud_send_lock.acquire(blocking=False):
            rospy.logwarn("Cloud: send already in progress, skip trigger")
            return
        try:
            self._send_cloud_inner(pcd_path)
        finally:
            self._cloud_send_lock.release()

    def _send_cloud_inner(self, pcd_path):
        self.cloud_status = "READING"
        rospy.loginfo("Cloud: reading %s", pcd_path)
        try:
            points = read_pcd(pcd_path)
        except Exception as e:
            self.cloud_status = "READ ERR"
            rospy.logerr("Cloud: read failed: %s", e)
            return
        if len(points) == 0:
            self.cloud_status = "EMPTY PCD"
            return
        self.cloud_status = "PROJECTING"
        coords, xr, yr = project_to_canvas(
            points, CLOUD_Z_MIN, CLOUD_Z_MAX,
            CLOUD_CANVAS_W, CLOUD_CANVAS_H, CLOUD_VOXEL)
        if not coords:
            self.cloud_status = "NO POINTS"
            return
        total = len(coords)
        self.cloud_status = f"SENDING 0/{total}"
        rospy.loginfo("Cloud: sending %d pts", total)
        meta = struct.pack('<H', min(total, 65535))
        meta += struct.pack('<ff', xr, yr)
        self._send(TYPE_CLOUD_META, meta)
        time.sleep(0.1)
        sent = 0
        for i in range(0, total, CLOUD_BATCH_SIZE):
            if rospy.is_shutdown():
                break
            chunk = coords[i:i + CLOUD_BATCH_SIZE]
            payload = b''.join(struct.pack('<hhH', px, py, c) for px, py, c in chunk)
            self._send(TYPE_CLOUD_BATCH, payload)
            sent += len(chunk)
            self.cloud_status = f"SENDING {sent}/{total}"
            time.sleep(CLOUD_FRAME_DELAY)
        self.cloud_pts = sent
        self.cloud_status = f"DONE {sent} pts"
        rospy.loginfo("Cloud: done, %d pts sent", sent)

    def refresh_display(self, event):
        armed_str = "\033[91mARMED\033[0m" if self.armed else "\033[92mDISARMED\033[0m"

        os.system('clear')
        print(f"\033[1m========== ESP32 Bridge — Drone {self.drone_id} ==========\033[0m")
        print()
        print("\033[1m--- Flight ---\033[0m")
        print(f"  Mode:     {self.mode}  {armed_str}")
        print()
        print("\033[1m--- Position (m) ---\033[0m")
        print(f"  X: {self.pos[0]:+8.3f}   Y: {self.pos[1]:+8.3f}   Z: {self.pos[2]:+8.3f}")
        print()
        print("\033[1m--- Attitude (deg) ---\033[0m")
        print(f"  Roll: {self.rpy[0]:+7.2f}   Pitch: {self.rpy[1]:+7.2f}   Yaw: {self.rpy[2]:+7.2f}")
        print()
        print("\033[1m--- Velocity (m/s) ---\033[0m")
        print(f"  Vx: {self.vel[0]:+7.3f}   Vy: {self.vel[1]:+7.3f}   Vz: {self.vel[2]:+7.3f}")
        print(f"  Speed: {self.speed:.3f}")
        print()
        print("\033[1m--- Battery ---\033[0m")
        print(f"  {self.batt_voltage:.1f}V  {self.batt_percent:.0f}%  {self.batt_current:.1f}A")
        print()
        ekf_color = "\033[92m" if self.ekf_ok else "\033[91m"
        print("\033[1m--- EKF ---\033[0m")
        print(f"  {ekf_color}{'OK' if self.ekf_ok else 'BAD'}\033[0m  flags={self.ekf_flags}  vel={self.ekf_vel_var:.4f}  pos_h={self.ekf_pos_h_var:.4f}  comp={self.ekf_comp_var:.4f}")
        print()
        serial_status = "\033[92mOK\033[0m" if self.serial_ok else "\033[91mDISCONNECTED\033[0m"
        print("\033[1m--- Bridge Stats ---\033[0m")
        print(f"  Odom: {self.odom_count} frames ({self.odom_hz:.1f} Hz)")
        print(f"  State: {self.state_count} frames")
        print(f"  Serial: {serial_status}")
        print()
        if self.last_cmd_status:
            print(f"\033[1m--- Last CMD ---\033[0m")
            print(f"  {self.last_cmd_status}")
            print()
        print("\033[1m--- ESP32 ---\033[0m")
        print(f"  RX: {self.esp_rx}  TX_OK: {self.esp_tx_ok}  TX_FAIL: {self.esp_tx_fail}  CRC_ERR: {self.esp_crc_err}")
        print(f"  Uptime: {self.esp_uptime}s")
        print()
        cloud_color = "\033[92m" if "DONE" in self.cloud_status else (
            "\033[93m" if "SEND" in self.cloud_status else "\033[0m")
        print("\033[1m--- Point Cloud ---\033[0m")
        print(f"  {cloud_color}{self.cloud_status}\033[0m")
        print(f"  trigger: echo auto > ~/esp32_bridge/.cloud_trigger")
        print()
        print("\033[90mCtrl+C to stop\033[0m")

    def run(self):
        rospy.spin()
        if self.ser:
            self.ser.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', type=int, default=1, help='Drone ID (1 or 2)')
    parser.add_argument('--port', default='/dev/esp32_bridge', help='ESP32 serial port')
    parser.add_argument('--baud', type=int, default=115200)
    args, _ = parser.parse_known_args()

    try:
        ESP32Bridge(args.id, args.port, args.baud).run()
    except rospy.ROSInterruptException:
        pass
