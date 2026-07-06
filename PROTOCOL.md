# ESP32 Drone Bridge 通信协议（双机版 v2）

## 帧格式

```
[0xAA] [DRONE_ID] [TYPE] [LEN_H] [LEN_L] [PAYLOAD...] [CRC8]
  1B      1B        1B     1B      1B       N bytes      1B
```

- DRONE_ID: 0x01=飞机1, 0x02=飞机2, 0x00=本机统计(不经过ESP-NOW)
- LEN = payload 长度（大端）
- CRC8 校验范围：从 0xAA 到 payload 末尾
- CRC8 多项式：0x07，初始值 0
- 总帧长 = N + 6

## 帧类型

### TYPE 0x01 — 心跳 (1Hz)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 4 | uint32 LE | 已发送 odom 帧总数 |

payload = 4 bytes

### TYPE 0x02 — 里程计 (~20Hz)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 4 | float32 LE | X (m) |
| 4 | 4 | float32 LE | Y (m) |
| 8 | 4 | float32 LE | Z (m) |
| 12 | 4 | float32 LE | qx |
| 16 | 4 | float32 LE | qy |
| 20 | 4 | float32 LE | qz |
| 24 | 4 | float32 LE | qw |
| 28 | 4 | float32 LE | Vx (m/s) |
| 32 | 4 | float32 LE | Vy (m/s) |
| 36 | 4 | float32 LE | Vz (m/s) |

payload = 40 bytes

四元数转欧拉角:
```
roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx² + qy²))
pitch = asin(clamp(2*(qw*qy - qz*qx), -1, 1))
yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
```

### TYPE 0x03 — 飞控状态 (~1Hz)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 1 | uint8 | 0=未解锁, 1=已解锁 |
| 1 | 16 | char[16] | 飞行模式 UTF-8, \0填充 |

payload = 17 bytes

常见模式: STABILIZE, ALT_HOLD, LOITER, AUTO, GUIDED, LAND

### TYPE 0x04 — 电池 (~1Hz)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 4 | float32 LE | 电压 (V)，如 15.28 |
| 4 | 4 | float32 LE | 百分比 (0.0~1.0) |
| 8 | 4 | float32 LE | 电流 (A) |

payload = 12 bytes

### TYPE 0x05 — EKF 状态 (~3Hz)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 2 | uint16 LE | EKF flags 标志位 |
| 2 | 4 | float32 LE | 速度方差 |
| 6 | 4 | float32 LE | 水平位置方差 |
| 10 | 4 | float32 LE | 垂直位置方差 |
| 14 | 4 | float32 LE | 罗盘方差 |

payload = 18 bytes

### TYPE 0x06 — 点云元数据 (降落后传输)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 2 | uint16 LE | 总点数 |
| 2 | 4 | float32 LE | X 范围 (米) |
| 6 | 4 | float32 LE | Y 范围 (米) |

payload = 10 bytes

收到此帧后地面站清空点云画布，准备接收坐标数据。

### TYPE 0x07 — 点云坐标批次 (降落后传输)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 6×N | int16 LE, int16 LE, uint16 LE | N 组 (px, py, rgb565) |

payload = 6×N bytes, N = payload_len / 6

每组 6 字节: [px_lo] [px_hi] [py_lo] [py_hi] [color_lo] [color_hi]。
坐标原点为点云画布左上角，单位像素；颜色为 RGB565。
每帧最多约 40 个点 (ESP-NOW 250B 限制)。

### TYPE 0xF0 — ESP32 统计 (仅 USB 回传，不经过 ESP-NOW)

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 4 | uint32 LE | 接收帧计数 |
| 4 | 4 | uint32 LE | ESP-NOW 发送成功 |
| 8 | 4 | uint32 LE | ESP-NOW 发送失败 |
| 12 | 4 | uint32 LE | CRC 错误计数 |
| 16 | 1 | uint8 | 最后帧类型 |
| 17 | 4 | uint32 LE | 运行时间 (秒) |

payload = 21 bytes, 1Hz, DRONE_ID=0x00

## 地面站指令帧（GCS → 飞机）

方向：TJC 触控屏 → ESP32-B → ESP-NOW → ESP32-A → USB → Jetson 桥接脚本

### 安全机制

- ARM/DISARM 只能通过遥控器操作，地面站不控制解锁
- 地面站只发"执行任务"指令（切 GUIDED 模式）
- 遥控器随时可切模式中止，优先级最高

### 操作流程

```
1. 遥控器 ARM 解锁
2. 地面站按"执行任务"
3. ESP-NOW 发送 TYPE 0x80 → Jetson 切 GUIDED
4. auto_mission 检测到 GUIDED+ARMED → 自动执行预设航线
5. 任务完成自动降落 / 遥控器切模式中止
```

### TYPE 0x80 — 执行任务指令

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 1 | uint8 | 指令码: 0x20=执行任务 |

payload = 1 byte

Jetson 收到后执行: `set_mode GUIDED` → auto_mission 自动触发

### TYPE 0x81 — 指令应答（飞机 → GCS）

| 偏移 | 长度 | 类型 | 说明 |
|------|------|------|------|
| 0 | 1 | uint8 | 原始指令码 (0x20) |
| 1 | 1 | uint8 | 结果: 0=已发送, 1=未ARM拒绝 |

payload = 2 bytes

地面站收到应答后显示执行状态。

### 串口屏指令控件 ID

| 控件 ID | 说明 |
|---------|------|
| btnMission | "执行任务" 按钮 |
| cmdStatus | 指令状态 ("已发送"/"未解锁") |

## CRC8 实现

```c
uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}
```

```python
def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc
```

## 数据链路

```
遥测数据 (飞机→地面站):
  飞机1 Jetson ──USB──> ESP32-A1 ──┐
                                    ├── ESP-NOW ──> ESP32-B ──UART──> TJC 串口屏
  飞机2 Jetson ──USB──> ESP32-A2 ──┘                       ──USB───> PC (调试)

指令下发 (地面站→飞机):
  TJC 触控屏 ──UART──> ESP32-B ──ESP-NOW──> ESP32-A ──USB──> Jetson 桥接脚本
                                                                ↓
                                                        MAVROS 服务调用
```

## 串口屏控件 ID (TJC)

### 顶部状态栏
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dMode | 飞行模式 | "LAND" |
| dArm | 解锁状态 | "DISARMED" |
| dLink | 连接状态 | "● LINK" |
| dBatV | 电池电压 | "15.3V" |
| dBatP | 电池百分比 | "98%" |
| dHz | 数据频率 | "20Hz" |

### 姿态面板
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dR | Roll 角 | "-5.2°" |
| dP | Pitch 角 | "+2.1°" |
| dW | Yaw 角 | "+035°" |

### 罗盘面板
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dHdg | 航向角大字 | "035°" |

### 位置/速度面板
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dX / dY / dZ | 位置 | "+1.234m" |
| dVx | X 方向速度 | "+0.05m/s" |
| dVy | Y 方向速度 | "-0.02m/s" |
| dVz | 垂直速度 | "+0.01m/s" |
| dSpd | 合速度 | "0.12m/s" |

### 系统面板
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dEkfV | EKF 方差 | "0.12" |
| dEkfF | EKF 标志位 | "128" |
| dViso | 视觉定位频率 | "20Hz" |
| dDist | 测距高度 | "1.23m" |
| dBatI | 电池电流 | "3.2A" |
| dCell | 电芯配置 | "4S" |

### 点云面板
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| pcView | 点云画布 | 2D 俯视图 |
| pcPts | 点数 | "12,847" |
| pcRate | 传输速率 | "2.4KB/s" |

### ESP-NOW 链路
| 控件 ID | 内容 | 示例值 |
|---------|------|--------|
| dTxOk | 发送成功 | "12,847" |
| dTxFl | 发送失败 | "0" |
| dLoss | 丢包率 | "0.0%" |
| dUp | 运行时长 | "642s" |

### 指令区
| 控件 ID | 类型 | 说明 |
|---------|------|------|
| btnMission | 触控按钮 | "执行任务" |
| cmdStatus | 文本 | "就绪"/"已发送"/"拒绝:未解锁"/"任务执行中" |
| missionStep | 文本 | "起飞中"/"飞往目标"/"返航中"/"已完成" |

### 飞机切换
| 控件 ID | 说明 |
|---------|------|
| d1btn | 飞机1按钮 |
| d2btn | 飞机2按钮 |

## 启动命令

飞机1 (Jetson):
```bash
python3 ~/esp32_bridge/esp32_bridge.py --id 1
```

飞机2 (Jetson):
```bash
python3 ~/esp32_bridge/esp32_bridge.py --id 2
```

地面站 (PC 调试):
```bash
python3 gcs/gcs_display.py /dev/ttyACM1
```
