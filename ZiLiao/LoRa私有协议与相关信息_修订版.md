# LoRa 私有通信协议规范（修订版）

> 本协议是为"基于 LoRa 的森林火灾监测系统"设计的完整私有通信协议，涵盖终端节点、中继节点与网关节点之间的双向数据传输、网络自组织与维护。
>
> 修订记录：
> - v1: 初始版本
> - v2: 完善下行指令集（0x20-0x2A）、新增 TLV 0x0E/0x0F/0x33/0x34、修改 0xFF 格式、明确功率映射、规范上行 JSON 格式
> - v3: 修正 Class A 时序（50ms+2s/2s 替代 1s/2s）、载荷限制 120B、注册流程细化(含 GNSS 稳定性)、单帧多历史块、0x10 含电池电压、指令经 ACK 下发、终端状态机完整化、低功耗 STOP2+WFI 双级休眠、协议常量更新

---

## 1. 物理层参数

| 参数 | 规定 |
|---|---|
| 工作频段 | CN470 (470-510 MHz)，固定 471.5 MHz，单信道 |
| 调制方式 | LoRa，带宽 125 kHz |
| 扩频因子 | 主 SF=7（调试），终端可在链路恶化时切换备用 SF（11~12）；中继/网关固定主 SF |
| **发射功率（网关/中继）** | 模块内置 PA 增益 +11 dB。SX1268 芯片输出 -9~+22 dBm，天线端实际输出 **+2~+33 dBm**。调试 14 dBm，部署 33 dBm |
| **发射功率（终端）** | 最大 +22 dBm，可调 5 档：+14/+16/+18/+20/+22 dBm |
| 信道接入 | 纯 ALOHA，可选 CAD 退避 100~500 ms |
| 接收灵敏度 | SF7: -123, SF8: -126, SF9: -129, SF10: -132, SF11: -134.5, SF12: -137 (dBm, 典型值) |
| 收发模式 | Class A: 上行后延迟 50ms 开 RX1（窗口 2s），RX1 结束 50ms 后开 RX2（窗口 2s） |
| **注册阶段 RX** | 注册帧发送后延迟 10ms 开 15s RX 窗口；若收到代理 ACK 则延长至 3min |

---

## 2. 帧结构（通用）

```
┌─────────────┬──────────┬────────────┬────────┬─────────┬─────┐
│ FrameControl │ DestAddr │ SenderAddr │ MsgSeq │ Payload │ CRC │
│    1 B      │  2 B LE  │   2 B LE   │ 2 B LE │ TLV变长 │ 2 B │
└─────────────┴──────────┴────────────┴────────┴─────────┴─────┘
```

> CRC 由硬件自动附加/校验，软件不参与。
> 单帧最大载荷 120 字节（帧头 7B + TLV 最多 120B），LoRa 缓冲区 128B，PHY 硬上限 255B。

### FrameControl (1 字节)

| 位 | 名称 | 说明 |
|---|---|---|
| 7-6 | FrameType | 00=数据帧, 01=ACK帧, 10=命令帧, 11=注册帧 |
| 5-4 | AckType | 仅 FrameType=01 有效: 00=无ACK, 01=代理ACK(中继), 10=端到端ACK(网关), 11=保留 |
| 3 | SF_Flag | 0=主SF, 1=备用SF |
| 2-0 | PowerLevel | 发送者功率档位 (0~4) |

### 短地址保留

| 地址 | 含义 |
|---|---|
| 0x0000 | 网关 |
| 0xFFFF | 广播 / 未注册 |
| 其他 | 网关动态分配 |

### 帧构建宏

```c
LORA_FC_BUILD(ftype, acktype, sf, power)
```

ftype: `LORA_FTYPE_DATA(0x00)`, `LORA_FTYPE_ACK(0x40)`, `LORA_FTYPE_CMD(0x80)`, `LORA_FTYPE_REGISTER(0xC0)`

---

## 3. 载荷格式（TLV）

每个 TLV 单元: `[Type 1B] [Length 1B] [Value ... Length字节]`。TLV 个数受 120B 载荷限制自动截断，
典型数据帧固定 0x01(11B) + extra(0~44B) + 0x10×N(15B/个) + 0xFF(2+2N)B。

### 3.1 完整 TLV 类型表

| Type | 名称 | Value 结构 | 方向 |
|---|---|---|---|
| **0x01** | SENSOR_DATA | 温度(int16,×0.1°C)+湿度(uint16,×0.1%RH)+CO(uint16,ppm)+电池(uint16,mV)+警报(uint8) | 终端→网关 |
| **0x0E** | RELAY_CONFIG_REPORT | **预留**（中继对 0x29 的回复，TBD） | 中继→网关 |
| **0x0F** | CONFIG_REPORT | 见 §3.2，固定 24 字节 packed | 终端→网关 |
| **0x10** | HISTORY_BLOCK | 序列号(uint16)+时间戳(uint16,相对秒)+传感器数据(同0x01,含电池电压)，共 13B。单帧可携带多个，剩余空间填满为止 | 终端→网关 |
| **0x20** | SET_COLLECT_INTERVAL | normal(uint16,秒)+warn(uint16)+fire(uint16)，共 6B | 网关→终端 |
| **0x21** | SET_SEND_INTERVAL | normal(uint16,秒)+warn(uint16)+fire(uint16)，共 6B | 网关→终端 |
| **0x22** | SET_TEMP_THRESHOLD | warn(int16,×0.1°C)+warn_recov(int16)+alarm(int16)+alarm_recov(int16)，共 8B | 网关→终端 |
| **0x23** | SET_CO_THRESHOLD | warn(uint16,ppm)+recover(uint16)，共 4B | 网关→终端 |
| **0x24** | SET_HUMI_THRESHOLD | warn(uint16,×0.1%RH)+recover(uint16)，共 4B | 网关→终端 |
| **0x25** | SET_RF_PARAMS | sf_main(uint8)+sf_backup(uint8)+power_level(uint8)，共 3B | 网关→终端 |
| **0x26** | REPORT_NOW | 无 Value (0B) | 网关→终端 |
| **0x27** | REQ_RESEND | 目标序列号(uint16)，共 2B | 网关→终端 |
| **0x28** | UPDATE_PARENT | 新父节点短地址(uint16)，共 2B | 网关→终端 |
| **0x29** | QUERY_CONFIG | 无 Value (0B)，终端回复 0x0F，中继回复 0x0E | 网关→终端 |
| **0x2A** | REMOTE_RESET | 无 Value (0B) | 网关→终端 |
| **0x30** | REGISTER_ID | 12 字节唯一硬件 ID | 终端/中继→网关 |
| **0x31** | TERM_CAPABILITY | 最大SF(uint8)+最大功率档(uint8)+传感器掩码(uint8)，共 3B | 终端→网关 |
| **0x32** | ASSIGN_ADDR | 分配的短地址(uint16)，共 2B | 网关→终端/中继 |
| **0x33** | CURRENT_TXPOWER | 当前发射功率(int8,dBm)。**终端不使用此TLV**（功率档位已编码在 FrameControl.PowerLevel 中） | (预留) |
| **0x34** | GNSS_POSITION | 见 §3.3，固定 11 字节 packed | 终端→网关 |
| **0x35** | NODE_ROLE | 角色(uint8): 0=终端, 1=中继，共 1B | 终端/中继→网关 |
| **0x36** | LINK_PROBE | 类型(uint8): 0=请求, 1=应答，共 1B | 中继↔父节点 |
| **0x40** | BEACON_INFO | 见 §4 | 网关/中继→广播 |
| **0xAC** | ACK_INFO | LastInSeq(uint16)+RSSI(int8,dBm)+SNR(int8,0.25dB步进)+GwTxPower(int8,dBm)+[SuggestPower(uint8)] | 网关/中继→终端 |
| **0xFF** | CMD_STATUS | 见 §3.4，2×N 字节 [CmdType Status] 对 (N≤8) | 终端→网关 |

### 3.2 CONFIG_REPORT (0x0F) — 终端配置上报

对 QUERY_CONFIG (0x29) 的回复，固定 24 字节 packed，小端序：

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---|---|---|---|
| 0 | 2 | measure_interval_normal | uint16 | 常规采集间隔 (s) |
| 2 | 2 | measure_interval_warning | uint16 | 预警采集间隔 (s) |
| 4 | 2 | measure_interval_alarm | uint16 | 火警采集间隔 (s) |
| 6 | 2 | transmit_interval_normal | uint16 | 常规发送间隔 (s) |
| 8 | 2 | transmit_interval_warning | uint16 | 预警发送间隔 (s) |
| 10 | 2 | transmit_interval_alarm | uint16 | 火警发送间隔 (s) |
| 12 | 2 | temp_alarm_threshold | int16 | 温度火警阈值 (×0.1°C) |
| 14 | 2 | temp_alarm_recovery | int16 | 温度火警恢复 (×0.1°C) |
| 16 | 2 | co_warning_threshold | uint16 | CO 预警阈值 (ppm) |
| 18 | 2 | co_warning_recovery | uint16 | CO 预警恢复 (ppm) |
| 20 | 2 | hum_warning_threshold | uint16 | 湿度预警阈值 (×0.1%RH) |
| 22 | 2 | hum_warning_recovery | uint16 | 湿度预警恢复 (×0.1%RH) |

### 3.3 GNSS_POSITION (0x34) — 位置信息

固定 11 字节 packed，在注册帧中携带：

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---|---|---|---|
| 0 | 1 | lat_dir | uint8 | 纬度方向 'N'(78) 或 'S'(83) |
| 1 | 1 | lat_deg | uint8 | 纬度度 (0-90) |
| 2 | 1 | lat_min | uint8 | 纬度分 (0-59) |
| 3 | 2 | lat_frac | uint16 LE | 纬度分的分数 (×1/10000) |
| 5 | 1 | lon_dir | uint8 | 经度方向 'E'(69) 或 'W'(87) |
| 6 | 2 | lon_deg | uint16 LE | 经度度 (0-180) |
| 8 | 1 | lon_min | uint8 | 经度分 (0-59) |
| 9 | 2 | lon_frac | uint16 LE | 经度分的分数 (×1/10000) |

**十进制度转换:**
```
decimal = deg + (min + frac / 10000) / 60
若 dir='S' 或 'W' 则取负
```

### 3.4 CMD_STATUS (0xFF) — 指令执行状态

新格式，支持单帧回复多个下行指令的执行结果 (N≤8)：

```
Value = [CmdType₀ Status₀ CmdType₁ Status₁ ... CmdTypeₙ₋₁ Statusₙ₋₁]
长度 = 2 × N
```

| 字段 | 长度 | 说明 |
|---|---|---|
| CmdType | 1B | 执行的指令 TLV 类型 |
| Status | 1B | 0=成功, 1=失败 |

**示例:** 网关下发 0x20 和 0x21 均成功 → `0xFF 04 20 00 21 00`
**示例:** 网关下发 0x20 成功、0x23 失败 → `0xFF 04 20 00 23 01`

---

## 4. Beacon 与网络发现

网关和中继周期广播 Beacon 帧（FrameType=命令帧，附加 TLV 0x40），用于拓扑发现与链路评估。

### Beacon TLV (0x40) 字段

| 字段 | 长度 | 说明 |
|---|---|---|
| BeaconType | 1B | 0=网关, 1=中继 |
| TxPower | 1B | 发送此 Beacon 的实际天线端功率 (dBm) |
| GatewayID | 2B | 可达网关短地址（网关填 0x0000） |
| HopsToGateway | 1B | 到网关跳数（网关=0） |
| Flags | 1B | bit0=IsValid (1=可提供服务) |
| LoadFactor | 1B | 可选，已接入终端数 |

### 广播参数

- 网关 Beacon 间隔: 20s
- 仅 IsValid=1 的父节点可被终端/中继选择

---

## 5. 网络加入与注册流程

### 5.1 终端注册流程

1. **GNSS 定位**: 终端上电后首先获取 GNSS 位置。需满足稳定性条件（卫星≥6、HDOP<2.0、连续 5 次位置漂移<10m）后才保存到 EEPROM 并进入扫描。
2. **扫描 Beacon**: 终端以主 SF 连续接收 ≥45s（>2 个 Beacon 周期），收集 Beacon 候选列表（RSSI/SNR/跳数/负载/IsValid）
3. **父节点选择**: 优先 IsValid=1 的节点，其次直连网关(BeaconType=0)，综合跳数少→负载低→RSSI 强评分排序；预估上行 RSSI ≥ SF 灵敏度 + 6dB 方为可达
4. **发送注册帧**: FrameType=0xC0, DestAddr=父节点, SenderAddr=0xFFFF
   - Payload: TLV 0x30 (12B长ID) + 0x31 (3B能力) + **0x34 (11B GNSS位置)** + 0x35 (1B角色=终端)
5. **等待分配**: 注册帧发出后，开 RX 窗口监听回复
   - 直连网关: 15s RX 窗口等待 ACK+TLV 0x32
   - 收到中继代理 ACK: 延长至 3min 等待最终确认
   - 超时: 排除当前候选 → 回扫描重选父节点
6. **注册完成**: 收到分配短地址后，置 `registration_done = true`
   - ACK 中可能附有初始下行指令（间隔/阈值/RF 参数等）
   - **调试阶段**: 短地址仅存入 RAM，每次上电重新注册；部署后改为写入 EEPROM 持久化

### 5.2 中继注册

1. 中继同样扫描、选择 IsValid=1 的父节点
2. 发送注册帧: TLV 0x30 + 0x35(角色=1) + 0x31 + 0x34
3. 网关分配短地址，记录跳数（父节点+1），校验 MAX_HOPS(3) 与环路
4. 注册完成后中继设 IsValid=1，开始广播自身 Beacon

### 5.3 中继连通性维护

- **被动监测**: 2 分钟内无端到端 ACK 则触发主动探测
- **主动探测**: 60s 发 TLV 0x36(请求)，无应答 N 次则判定失联
- **失联处理**: IsValid→0，停止代理，重扫描

---

## 6. 数据上报与确认

### 6.1 上行数据帧

终端构建数据帧 (FrameType=0x00)，组装顺序:

```
[帧头7B] → [0x01实时11B] → [0x0F配置26B/0xFF状态等(可选)] → [0x10历史×N(填满剩余空间)]
```

- **0x01**: 必须携带（温度/湿度/CO/电池/警报标志）
- **0x0F/0xFF**: 由下行指令触发（0x29 触发配置上报、指令执行后触发状态上报）
- **0x10**: 遍历重传队列，每条 15B，用剩余载荷空间尽量填充，历史块较多时自动裁剪
- 全帧 ≤ 120B 载荷

### 6.2 确认机制

| ACK 类型 | 发送者 | 触发时机 | 含义 |
|---|---|---|---|
| 代理 ACK (01) | 中继 | 收到终端上行帧后 | 告知终端已收到，LastInSeq=中继记录的连续序列号 |
| 端到端 ACK (10) | 网关 | 收到完整上行帧后 | 确认数据已送达网关 |

- ACK 帧可附带下行指令 TLV (0x20-0x2A)
- 终端发完上行后进入 Class A RX 窗口等待 ACK（见 §1）
- 两个窗口都超时 → 终端将当前数据缓存到 EEPROM
- ACK 确认后，若重传队列为空，逐条弹出 EEPROM 缓存继续补发

### 6.3 重传与缓存

- **重传队列** (RAM FIFO, 16 条): 每次发送后推入，收到 ACK 的 LastInSeq 后移除已确认条目
- **EEPROM 缓存** (环形队列, ~32KB): TX 失败或 ACK 超时时备份；每次成功 ACK 后弹出一条重新推入重传队列，逐步排出

### 6.4 上行 JSON 格式（网关→云平台，见 §9）

---

## 7. 下行指令

### 7.1 传输方式

下行指令**附着在 ACK 帧** (FrameType=0x40) 的 TLV payload 中发送，作为对终端上行数据的响应一并下发。
终端收到后逐条执行，执行结果缓存到状态队列（最多 8 条），在下次上行数据帧中通过 TLV 0xFF 统一回复。
一帧 ACK 可携带多条指令，终端按序执行。

### 7.2 下发路径

```
Web上位机 → OneNET属性设置(downlink_cmd) → MQTT → 网关 → LoRa → 终端
   或
上位机调试 → USB CDC → 网关 → LoRa → 终端
```

### 7.3 JSON 格式（USB 和 MQTT 通用）

```json
{
  "dst": 1,
  "cmd": "set_collect_interval",
  "normal": 60,
  "warn": 15,
  "fire": 5
}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `dst` | uint | 目标终端短地址 |
| `cmd` | string | 指令名称（见下表） |
| 其余字段 | — | 指令参数 |

### 7.4 完整指令列表

| cmd | TLV | 参数 | 类型 |
|---|---|---|---|
| `set_collect_interval` | 0x20 | `normal`, `warn`, `fire` | uint16 (秒) |
| `set_send_interval` | 0x21 | `normal`, `warn`, `fire` | uint16 (秒) |
| `set_temp_threshold` | 0x22 | `warn`, `warn_recover`, `alarm`, `alarm_recover` | float (°C) |
| `set_co_threshold` | 0x23 | `warn`, `recover` | uint16 (ppm) |
| `set_humi_threshold` | 0x24 | `warn`, `recover` | float (%RH) |
| `set_rf_params` | 0x25 | `sf_main`, `sf_backup`, `power` | uint8 |
| `report_now` | 0x26 | 无参数 | — |
| `resend` | 0x27 | `seq` | uint16 |
| `update_parent` | 0x28 | `parent` | uint16 |
| `query_config` | 0x29 | 无参数 (终端回复 0x0F) | — |
| `remote_reset` | 0x2A | 无参数 | — |

---

## 8. 自适应链路控制（终端侧）

- **功率调节**: 根据 ACK 中的 RSSI/SNR 评估链路预算，余量不足升功率，过大降功率
- **SF 切换**: 连续丢失达阈值切备用 SF，稳定后切回
- **父节点失效**: 连续 MAX_LOSS 次无 ACK → ISOLATED → 重扫描

---

## 9. 网关云平台接口（OneNET MQTT）

### 9.1 概述

网关使用 OneNET **事件 (Event)** 机制上报数据。数据内容通过 struct+string 分片透传——
网关将上行 JSON 按 480 字节分片填入 `payload_0/1/2` 三个 string 字段，
Web 端拼接后即得完整 JSON。

- 上报 Topic: `$sys/{pid}/{device-name}/thing/event/post`
- 下行 Topic: `$sys/{pid}/{device-name}/thing/property/set`

### 9.2 OneNET 物模型定义

**事件 `sensor_uplink`（信息）：**

| 参数 | 类型 |
|---|---|
| `msg_type` | enum: sensor_data(0), protocol_ack(1), protocol_cmd(2), other(3) |
| `payload_0` | string(512) |
| `payload_1` | string(512) |
| `payload_2` | string(512) |

**属性 `downlink_cmd`：**

| 参数 | 类型 |
|---|---|
| `downlink_cmd` | string(512) |

### 9.3 上云消息（4 种）

#### (1) 终端传感器数据

```json
{
  "dir": "up",
  "src": 1,
  "seq": 484,
  "rssi": -7,
  "snr": 12,
  "type": "data",
  "payload": {
    "sensor": {
      "temp": 26.6,
      "humi": 50.8,
      "co": 0,
      "bat": 2919,
      "alarm": 0
    }
  }
}
```

可选字段: `payload.history[]` (历史补发数组), `payload.config_report` (终端配置), `payload.cmd_results[]` (指令执行结果)。

#### (2) 终端注册

```json
{
  "dir": "event",
  "event": "node_register",
  "gw": 0,
  "node_addr": 1,
  "long_id": "AABBCCDDEEFF00112233",
  "role": "terminal",
  "gnss": {
    "lat": 39.0437,
    "lon": 117.2054
  }
}
```

#### (3) 网关心跳 (60s)

```json
{
  "dir": "status",
  "event": "heartbeat",
  "gw": 0,
  "uptime": 3600,
  "nodes": 3,
  "relays": 1
}
```

#### (4) 网关启动上线

```json
{
  "dir": "status",
  "event": "online",
  "gw": 0,
  "tick": 123456
}
```

### 9.4 不上云消息（仅 USB CDC 调试）

ACK 帧、Beacon 帧、注册帧的完整 JSON 仅通过 USB CDC 输出，不作为 MQTT 事件上报。格式与上行 JSON 一致，`type` 字段分别为 `"ack"`、`"cmd"`（Beacon）、`"register"`。

### 9.5 事件包装格式

所有上云 JSON 在 MQTT 发布时被包装为 OneNET 事件：

```json
{
  "id": "123456",
  "version": "1.0",
  "params": {
    "sensor_uplink": {
      "value": {
        "msg_type": 0,
        "payload_0": "{\"dir\":\"up\",\"src\":1,...}",
        "payload_1": "",
        "payload_2": ""
      }
    }
  }
}
```

分片策略: 单包 ≤480 字节，超出则分片填入 payload_0/1/2（最多 1440 字节）。Web 端拼接所有非空字段即得完整 JSON。

---

## 10. 终端完整状态机

```
上电 → INIT → GNSS定位 → SCAN(45s) → REGISTERING → IDLE ⇄ SLEEP
                   ↑            ↑            ↓         ↓    ↑
                   └──超时重试──┘      注册超时回SCAN   MEASURE→TRANSMIT→RECEIVE
                                                              ↑         ↓
                                                              └───ACK───┘
```

| 状态 | 说明 |
|---|---|
| INIT | 初始化外设/SHT40/EEPROM/OLED/LoRa/CO传感器/协议模块 |
| GNSS | 获取位置（需通过稳定性过滤），保存到 EEPROM |
| SCAN | 45s 连续接收 Beacon，收集候选父节点列表，评分择优 |
| REGISTERING | 发送注册帧 → 等待分配短地址 (15s RX / 3min 扩展) |
| IDLE | 空闲轮询，判断采集/发送时机，无任务则→SLEEP |
| MEASURE | 采集 SHT40 + CO + 电池电压 → 判定运行模式(正常/预警/火警) |
| TRANSMIT | 构建数据帧(含历史补发+可选配置/状态) → LoRa 发送 |
| RECEIVE | Class A RX1(2s) + RX2(2s)，处理 ACK/指令，超时则缓存 |
| SLEEP | ≤5s: WFI 浅睡；>5s: STOP2 深睡(RTC闹钟唤醒)；CO 预热提前 30s 唤醒 |
| SETTINGS | OLED 菜单操作(设置间隔/阈值/LoRa参数/重新定位/恢复默认) |
| ERROR | 硬件故障指示 |

**运行模式** (根据传感器数据自动切换):
- NORMAL: 常规采集/发送间隔
- EARLY_WARNING: CO 或湿度超阈值，缩短间隔
- FIRE_ALARM: 温度超火警阈值，最高频率采集/发送，不休眠

---

## 11. 中继协议状态机

```
上电 → SCAN → REGISTERING → ACTIVE → LOST → SCAN
                  ↑                        ↓
                  └──── 注册失败 ──────────┘
```

ACTIVE 状态下: IsValid=1，广播 Beacon，代理终端 ACK，持续监测父节点链路。

---

## 12. 网关功能概要

| 功能 | 说明 |
|---|---|
| 短地址管理 | 管理 0x0001~0xFFFE 地址池，查重复用 |
| 注册处理 | 处理终端/中继注册，提取 GNSS 和角色，触发云平台上报 |
| Beacon 广播 | 20s 周期，携带网关参数 |
| ACK 回复 | 收到数据帧回复端到端 ACK，携带 RSSI/SNR |
| 数据路由 | 上行: LoRa→解析→JSON→USB(全部)+MQTT(仅data帧)；下行: USB/MQTT→JSON→TLV→LoRa |
| 云平台 | OneNET MQTT 事件上报 + 属性设置下行 |
| 心跳 | 60s，含终端数/中继数/运行时间 |
| 本地调试 | USB CDC 输出所有帧 JSON + 系统日志 |

---

## 13. 安全简述

- 注册时通过长 ID 白名单过滤非法节点
- 预共享密钥帧认证为未来增强项

---

## 附录 A: 协议常量速查

| 常量 | 值 | 说明 |
|---|---|---|
| PROTO_HEADER_LEN | 7 | 帧头字节数 |
| PROTO_MAX_PAYLOAD_LEN | 120 | 单帧最大 TLV 载荷 |
| LORA_RX_BUF_SIZE | 128 | 接收缓冲区 |
| PROTO_RX_DELAY_MS | 50 | 发送完成→RX1 延迟 |
| PROTO_RX_WINDOW_MS | 2000 | 单个 RX 窗口时长 |
| PROTO_SCAN_DURATION_MS | 45000 | Beacon 扫描时长 |
| PROTO_REG_RX15_MS | 15000 | 注册直连 RX 窗口 |
| PROTO_REG_RX_EXTEND_MS | 180000 | 注册中继扩展 RX 窗口 |
| GW_BEACON_INTERVAL_S | 20 | Beacon 广播间隔 |
| GW_TX_POWER_DBM | 14 / 33 | 调试/部署 |
| LORA_PA_GAIN_DB | 11 | 模块 PA 增益 |
| GW_ADDR_POOL_START | 0x0001 | 地址池起 |
| GW_ADDR_POOL_END | 0xFFFE | 地址池止 |
| MAX_HOPS | 3 | 最大跳数 |
| PROTO_RETRANSMIT_QUEUE_SIZE | 16 | 重传队列容量 |
| CMD_STATUS_MAX | 8 | 单帧最大指令执行结果数 |
| GNSS_MIN_SATELLITES | 6 | 定位最低卫星数 |
| GNSS_MAX_HDOP_X10 | 20 | HDOP < 2.0 |
| GNSS_STABLE_COUNT | 5 | 连续稳定次数 |
| SLEEP_STOP2_THRESHOLD_S | 5 | 超过此秒数用 STOP2 |

## 附录 B: 网关代码分层

```
main/app_task.c       — 初始化协调
components/
  protocol/           — 帧编解码 + TLV 编解码
  node_core/          — lora_link(收发任务), gateway_mgr(注册/Beacon/ACK), node_common
  periph_hal/         — lora_hal(SX1268+PA映射), display_hal, gnss_hal, usb_hal, led_hal, button_hal
  services/           — data_router(上下行路由+JSON序列化), mqtt_service(OneNET), usb_service(CDC)
```
