# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

HB_chuchai -- 基于 HC32F460 (Cortex-M4) 的直流电机推窗控制系统，RS485/Modbus RTU 通信。

**当前代码状态：整合板版本**，通过 `main.h` 中 `BOARD_VERSION = 1` 控制。

## 硬件板本切换

`template/source/main.h` 第30行：
```c
#define BOARD_VERSION   1   // 0=原HB_chuchai板, 1=整合板(当前)
```

该宏控制以下所有差异（无需单独修改各模块）：
- 电机控制: GPIO(0) / PWM(1)
- 电流传感器: 霍尔(0) / 差分运放(1)
- 分压电阻: 110k(0) / 150k(1)
- 电压ADC: PA04(0) / PA06(1)
- RS485 DIR: PB14(0) / PA3(1)

## 编译/烧录

- IDE: Keil MDK V5.06
- 主工程文件: `HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx`
- 芯片包: HC32F460PETB

## 代码架构

### Adp 层 - 硬件适配层
直接操作 MCU 外设寄存器，与 HC32F460 DDL 及 CMSIS 交互。
- `Pwm.c/h`, `Adc.c/h`, `Gpio_io.c/h` -- PWM/ADC/GPIO 驱动
- `Motor_hall.c/h` -- 霍尔传感器 GPIO 中断 + 转速/方向计算 + 脉冲累积
- `rs485.c/h` -- USART + RS485 DIR 引脚控制
- `Dma.c/h`, `Sysclk.c/h`, `Timer*`, `hc32f46x_flash.c/h`, `timer6_timebase.c/h` -- 系统外设

### Dev 层 - 设备抽象层
每个设备封装为 `DeviceNode_t` 注册到 DeviceManager，通过 EventBus 解耦通信。
- `dev_motor.c/h` -- 电机仲裁器（占空比缓升/缓降、方向切换 pending 机制）
- `dev_rturn.c/h` -- 旋转角度追踪（**当前使用 Pulse-Direct 方案**，过流 -> 重新校准角度）
- `dev_sensor.c/h` -- 电流传感器（窗口滤波 + 滞回）
- `dev_voltage.c/h` -- 电压监控（过压/欠压）
- `EventBus.c/h` -- 发布/订阅事件总线（Topic 优先级队列）
- `device_manager.c/h` -- 设备注册/调度/互斥
- `dev_motor_hall.c/h` -- 电机霍尔设备封装
- `dev_adc.c/h`, `dev_pwm.c/h`, `dev_power.c/h`, `dev_hall.c/h`, `dev_io.c/h`

### App 层 - 应用协调层
- `App_Motor_Project.c/h` -- 系统初始化(ESystem_Init)、主循环(ESystem_MainLoop)、热重载(App_ReloadConfig)
- `App_Modbus.c/h` -- Modbus RTU 协议处理，CRC16，帧解析
- `App_FaultHandler.c/h` -- 故障聚合与清除（订阅 EventBus）

### Utils 层 - 基础设施
- `App_Params.c/h` -- Flash 参数持久化 + Modbus 寄存器读写映射 + 实时数据结构
- `param_manager.c/h` -- Flash 读写引擎（磨损均衡）
- `param_validator.c/h` -- Modbus 写入值校验（限幅 + 精度取整）
- `rtt_manager.c/h` -- RTT 日志 + 模块级调试开关
- `TickTimer.c/h`, `msg_queue.c/h`, `ring_buf.c/h`, `lock.c/h`

## 关键改动记录 (2026-06-21 更新)

### 角度获取: Pulse-Direct 方案 (方案 C)
- `dev_rturn.c`: 角度不再用 RPM 积分，改为 `fCurrentAngle = fMinAngle + g_s32HallPulseAccum * 360 / 12 / ratio`
- RPM 计算链路保留不动（用于转速显示和堵转检测）
- 开窗限位不再压制角度（保留真实超调值），关窗限位保持硬下限
- 过流校准时同步复位 `g_s32HallPulseAccum = 0`
- 详见 `电机霍尔方案.md`

### 霍尔脉冲累计修复
- `Motor_hall.c`: `last_total` 和 `first_run` 从 static 局部变量改为实例成员
- 累加器从 1 秒定时器移出，改为每次 update (1ms) 都跑
- 方向为 STOP/NONE 时回退到 `last_valid_direction`
- 修复了复位后脉冲跳跃的问题

### Modbus 参数校验
- `Utils/param_validator.c/h`: 新增独立校验模块
- 四个寄存器有校验规则：
  - 过压阈值 0x2714: 25.0~27.0V, 步进 0.2V
  - 欠压阈值 0x2715: 21.0~23.0V, 步进 0.2V
  - 过流阈值 0x2716: 0~2300mA, 步进 50mA
  - 过流判定时间 0x271E: 0~2000ms, 步进 20ms
- 减速比 0x3712: 1.0~6553.5 (单位 0.1)
- 极对数 0x3713: 1~100
- `App_Modbus.c`: 写入路径集成 Param_Validate，回令携带处理后的值
- 流程: CRC 校验 -> 最大最小值限幅 -> 精度取整 -> 写入 Flash -> 回令

### 减速比精度
- 寄存器 0x3712 单位改为 0.1 (值 x10)，原 1183 对应新值 11830
- 涉及文件: App_Params.h, App_Motor_Project.c/h, param_validator.c

## /init 自动阅读清单

执行 /init 或开始新会话时，应阅读以下关键代码以理解当前状态：

### 参数 Flash 存储
- `Utils/App_Params.h` -- 寄存器地址宏、默认值宏、AppParamRecord_t/AppRealTimeData_t 结构体
- `Utils/App_Params.c` -- Param_ReadByReg/Param_WriteByReg 读写分发
- `Utils/param_manager.h` -- Flash 读写引擎接口
- `Utils/param_validator.h` -- 参数校验规则

### 角度 & 转速获取
- `Adp/Motor_hall.c` -- 霍尔中断、脉冲计数、RPM 计算、g_s32HallPulseAccum 累加
- `Adp/Motor_hall.h` -- motor_hall_config_t 结构体、方向枚举
- `Dev/dev_motor_hall.c` -- MotorHall 设备封装
- `Dev/dev_rturn.c` -- 角度计算（Pulse-Direct）、限位检测、过流处理
- `Dev/dev_rturn.h` -- RTurn_Device_t 结构体

### 电流传感器
- `Dev/dev_sensor.h` -- Sensor_Device_t、传感器类型选择、过流模式、校准
- `Dev/dev_sensor.c` -- 电流计算、过流检测（计数/时间窗口两种模式）、EventBus 发布

### 电压传感器
- `Dev/dev_voltage.h` -- Voltage_Device_t、分压比配置、过压/欠压阈值
- `Dev/dev_voltage.c` -- 电压计算、过压/欠压检测、EventBus 发布

### 系统入口
- `template/source/main.c` -- 初始化顺序、主循环
- `template/source/main.h` -- BOARD_VERSION

## Modbus 寄存器地址分区

| 地址范围 | 用途 | 读写 |
|---------|------|------|
| 0x2700-0x271F | 可配置参数 (Flash 持久化) | R/W (0x03/0x06) |
| 0x2720 | 控制命令 | W (0x06) |
| 0x2730-0x273F | 实时数据 (RAM, 只读) | R (0x03) |
| 0x2740 | 故障状态 | R/W |
| 0x3700-0x371F | 高级参数 (Flash 持久化) | R/W (0x03/0x06) |

## EventBus 核心主题

| Topic | 发布者 | 订阅者 | 用途 |
|-------|--------|--------|------|
| TOPIC_CURRENT_ALARM | dev_sensor | dev_motor, dev_rturn, App_FaultHandler | 过流报警/解除 |
| TOPIC_VOLTAGE_ALARM | dev_voltage | dev_motor, App_FaultHandler | 过压/欠压 |
| TOPIC_RTURN_LIMIT | dev_rturn | dev_motor | 角度限位 -> 电机阻塞 |
| TOPIC_FAULT_CLEAR | App_FaultHandler | dev_sensor | 手动清除故障 |
| TOPIC_MOTOR_SPEED_FEEDBACK | dev_motor_hall | dev_rturn, dev_motor | 转速反馈 |
| TOPIC_MANUAL_RS485 | App_Modbus/App_Params | dev_motor | RS485 手动控制 |

## 辅助工具

- `modbus_tool.py` / `dist/modbus_tool.exe` -- 交互式 Modbus 指令生成器
  - 主菜单 7: 开发者选项（密码 5858）
  - 含读/写配置寄存器、计算霍尔脉冲、读/重置霍尔脉冲、计算实时角度
- `modbus_test_cmds.py` -- 脚本式 Modbus 指令生成器
- `实时数据使用说明.md` -- 实时数据 API 使用指南
- `电流控制逻辑说明.md` -- 过流检测三层处理详细时序
- `电机霍尔方案.md` -- 角度计算方案分析（含 ABC 方案对比、Pulse-Direct 实现总结）
- `电机霍尔脉冲.md` -- 双链路（Path 1 角度 + Path 2 脉冲）架构说明

## 调试日志

通过 `rtt_manager.h` 中的宏控制模块级调试输出（SEGGER RTT 通道 0）：

| 宏 | 控制的模块 |
|----|-----------|
| DEV_MOTOR | 电机仲裁器 |
| DEV_SENSOR | 电流传感器 |
| DEV_VOLTAGE | 电压监控 |
| DEV_EVENT_BUS | EventBus 订阅/发布 |
| APP_PARAMS_DBG | 参数读写 |
| APP_MODBUS_DBG | Modbus 帧收发 |
| DEV_MOTOR_HALL | 电机霍尔设备 |

## 仿真模式

`App_Motor_Project.h` 中 `ENABLE_SIMULATION_MODE` 默认为 1（开启）。
- 开启时：`Sim_ProcessInput()` / `Sim_PublishEvents()` 生成模拟传感器数据
- 关闭时（=0）：使用真实硬件 ADC/GPIO
- 模拟数据通过全局 `SystemSim_t g_sim` 控制

## 已知问题/注意事项

1. 新增 Flash 字段后需 `Modbus_Init` 检测并强制保存默认值
2. dev_motor.c/h 中有部分 UTF-16LE 编码文件，修改时需注意编码
3. App_Modbus.c 为 UTF-16LE 编码，param_validator.c 为 UTF-8 编码
4. PWM 停止使用 50% 交替极性，预驱芯片 SDH21263 需确认刹车效果
5. 减速比改为 0.1 精度后，已有设备 Flash 中的旧值需重新写入

## 预驱芯片 SDH21263 真值表

SDH21263 是 3 相 BLDC 预驱，本项目用其中 2 个半桥组成 H 桥驱动直流有刷电机。
PWM 频率 20kHz，低有效 (active LOW)。

MCU 引脚到 SDH21263 输入：

| MCU 引脚 | PWM 通道 | SDH21263 输入 | 半桥 |
|----------|---------|---------------|------|
| PB6 | TMRA_4 CH1 | HIN1 (高侧) / LIN1 (低侧) | Phase U |
| PB7 | TMRA_4 CH2 | HIN2 (高侧) / LIN2 (低侧) | Phase V |
| PB8 | TMRA_4 CH3 | HIN3 (高侧) / LIN3 (低侧) | Phase W |
| PB9 | TMRA_4 CH4 | EN (使能) 或互补输入 | — |

电机接线：Phase U + Phase V 跨接直流电机两端（H 桥），Phase W 未用或作刹车通道。

**运行真值表 (PWM active LOW, duty 表示低电平占比)：**

| 模式 | CH1 (PB6) | CH2 (PB7) | CH3 (PB8) | CH4 (PB9) | 电机 |
|------|-----------|-----------|-----------|-----------|------|
| 正转 (开窗) | 低 duty | 低 duty | 高 duty | 高 duty | FWD |
| 反转 (关窗) | 高 duty | 高 duty | 低 duty | 低 duty | REV |
| 停止 (刹车) | 98% | 98% | 98% | 98% | Brake |
| 停止 (旧版本) | 50% 交替 | 50% 交替 | 50% 交替 | 50% 交替 | Brake (交替) |

正转时 CH1/2 低 duty = 上桥臂关/下桥臂开，CH3/4 高 duty = 上桥臂开/下桥臂关，
电流从 Phase W 经电机流向 Phase U/V，电机正转。

反转时极性互换，电流反向。

停止时 4 通道全部 98% 高电平 → 所有下桥臂导通 → 电机制动短路刹车。

**注意：** 上述真值表基于代码逻辑推导，SDH21263 的实际引脚对应关系需对照硬件原理图确认。
SDH21263 支持 3 个半桥独立控制，HINx/LINx 为互补带死区输入。
