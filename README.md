# 基于 LoRa 的森林火灾监测系统 — 终端节点

[![MCU](https://img.shields.io/badge/MCU-STM32WLE5-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32wle5.html)
[![Framework](https://img.shields.io/badge/Framework-STM32CubeMX-orange)](https://www.st.com/en/development-tools/stm32cubemx.html)
[![Lang](https://img.shields.io/badge/Language-C11-green)]()

终端节点是"基于 LoRa 的森林火灾监测系统"的组成部分之一，负责**传感器数据采集、火灾预警判定、GNSS 定位**和**LoRa 组网通信**。

* [网关/中继节点代码](https://github.com/your-account/lora-forestfire-gateway)（待上传）
* [通信协议规范](ZiLiao/LoRa私有协议与相关信息_修订版.md)
* [软件设计说明书](ZiLiao/终端节点软件设计说明书.md)

## 硬件平台

| 组件 | 型号 | 接口 |
|------|------|------|
| MCU | STM32WLE5JCI6 | Cortex-M4 @ 48 MHz, 内置 LoRa |
| 温湿度传感器 | SHT40 | I2C1 |
| CO 传感器 | PS-CO-5000 | USART2 (9600 bps) |
| GNSS 模块 | ATGM336H | USART1 (115200 bps) |
| EEPROM | BL24C256A (32 KB) | I2C1 |
| OLED 显示屏 | SSD1306 128×64 | I2C2 |
| 按键 | 3 个 (UP / OK / DOWN) | EXTI |

## 功能特性

- **传感器采集** — 定期读取 SHT40 (温湿度)、PS-CO-5000 (CO 浓度)、电池电压
- **火灾预警判定** — 温度预警/火警、CO 预警、湿度预警，三级运行模式切换，含迟滞
- **GNSS 定位** — GPS/北斗双模，多层稳定性过滤（卫星数 ≥6、HDOP < 2.0、连续 5 次漂移 < 10 m）
- **LoRa 组网** — 完整私有协议栈：Beacon 扫描、父节点选择、注册、端到端 ACK、下行指令
- **OLED 菜单** — 主页 11 项实时数据展示 + 8 项参数配置（采集/发送间隔、阈值、LoRa 参数）
- **低功耗** — STOP2 深度休眠 (< 1 μA) + WFI 浅睡双级管理，RTC 闹钟/按键唤醒
- **离线缓存** — EEPROM 环形队列，通信中断时缓存数据，恢复后自动补发

## 项目结构

```
├── App/                        # 应用层代码
│   ├── Inc/                    # 头文件
│   └── Src/                    # 源文件
├── Core/                       # HAL 层 (CubeMX 生成)
│   ├── Inc/
│   └── Src/
├── Drivers/                    # STM32WL HAL 驱动库
├── Lib/                        # 第三方库
│   ├── Easy_Menu/              # 菜单框架 (预留)
│   └── STM32_HAL_SSD1306/     # OLED 驱动 + 中文字库
├── SubGHz_Phy/                 # Sub-GHz 射频协议栈
├── Utilities/                  # 工具库 (sequencer/timer/lpm/trace)
├── Middlewares/                # 中间件
├── cmake/                      # CMake 工具链 + CubeMX 生成文件
├── ZiLiao/                     # 设计资料
│   ├── 终端节点软件设计说明书.md
│   ├── LoRa私有协议与相关信息_修订版.md
│   └── PS-CO-5000 规格书.pdf
├── CMakeLists.txt              # CMake 构建入口
├── CMakePresets.json
├── LoRa_SenLinFanghuo.ioc     # CubeMX 项目文件
└── STM32WLE5XX_FLASH.ld       # 链接脚本
```

### 应用层文件

| 文件 | 职责 |
|------|------|
| `app_main.c/h` | 主状态机、主循环、按键分发、调试输出 |
| `app_menu.c/h` | OLED 菜单（主页 + 设置页） |
| `app_config.c/h` | EEPROM 配置持久化、离线数据缓存队列 |
| `lora_protocol.c/h` | 帧编解码、TLV 编解码、重传队列、Beacon 候选管理、RX 分发 |
| `app_lora.c/h` | LoRa Radio 收发 |
| `board_io.c/h` | 板级 IO 抽象 (GPIO/ADC/电源/RF 开关/LED) |
| `driver_sht40.c/h` | SHT40 温湿度传感器驱动 |
| `driver_co.c/h` | PS-CO-5000 CO 传感器驱动 |
| `driver_gnss.c/h` | ATGM336H GNSS 驱动 |
| `driver_eeprom.c/h` | BL24C256A EEPROM 驱动 |
| `driver_button.c/h` | 按键驱动 (EXTI + 消抖) |

## 构建

### 前置条件

- **STM32CubeCLT** 或 **GNU Tools for STM32** (arm-none-eabi-gcc)
- **CMake** ≥ 3.22
- **STM32CubeMX**（仅修改硬件配置时需要）

### 编译

```bash
# 配置 (Debug)
cmake --preset default

# 编译
cmake --build build

# 烧录 (ST-LINK)
STM32_Programmer_CLI -c port=SWD -w build/Debug/LoRa_SenLinFanghuo.elf
```

### CubeMX 代码生成

修改 `.ioc` 后重新生成代码，注意选择 **"Generate peripheral initialization as a pair of .c/.h files"** 以保持与当前项目结构一致。

## 调试

终端通过 **USART2 (PA2, 9600 bps)** 输出调试信息:

```
[SHT40] T=26.5C H=55.3%RH
[CO] 12 ppm
[ADC] V_bat=3.72V
[LORA] TX len=42
[PROTO] ACK last_seq=5 rssi=-12 snr=8
[SLEEP] STOP2 59s
[STATE] IDLE -> MEASURE
```

CO 传感器与调试串口共用 USART2（全双工），驱动调试输出已默认关闭（`CO_DEBUG=0`）。

## 已规划功能

见 [终端节点软件设计说明书 §10](ZiLiao/终端节点软件设计说明书.md)：
- 短地址持久化（调试→部署切换）
- 下行指令联调测试
- 中文字库补充 (6 字)
- 采集/发送间隔单位切换 (秒 → 分钟)
- 自适应链路控制 (SF 切换、功率调节)
- 系统看门狗 (IWDG)
- 远程固件升级 (FOTA)

## 许可证

本项目仅用于毕业设计/学术目的。
