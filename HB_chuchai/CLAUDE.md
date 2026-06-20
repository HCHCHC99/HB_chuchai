# CLAUDE.md

## 项目概述

HB_chuchai — 基于 HC32F460 (Cortex-M4) 的直流电机推窗控制系统，RS485/Modbus RTU 通信。

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
- 工程文件: `HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx`

## 关键修改记录

### 通信栈 (App_Modbus.c/.h, App_Params.c/.h)
- 寄存器写入后改为**热重载**而非软件复位（`App_ReloadConfig()`）
- 保留 CTRL_CMD RESET 位 (0x0008) 的真复位功能
- `Modbus_SetParamDefaults` 需同步新增字段的默认值

### PWM 电机控制 (dev_motor.c/.h)
- `MOTOR_CONTROL_MODE` 由 `BOARD_VERSION` 管理
- **停止(急停)**: 瞬间设置 `Motor_SetStopDuty()`，50% 占空比交替极性
  - CH1 PB6: 低有效 50%, CH2 PB7: 高有效 50%
  - CH3 PB8: 低有效 50%, CH4 PB9: 高有效 50%
- **方向切换**: 非阻塞 pending 机制（`s_bPolaritySwitchPending`）
  - 缓降到0 → 切极性 → 缓升到目标，避免抽动
- **正转**: CH1/2 低占空比, CH3/4 高占空比（全高有效）
- **反转**: CH1/2 高占空比, CH3/4 低占空比（全高有效）
- `MOTOR_DIRECTION_INVERT` 宏已废弃，改用运行时 `g_AppParam.motor_dir`

### 霍尔方向 (Motor_hall.c/.h)
- `HALL_DIRECTION_INVERT` 宏已废弃，改用运行时 `g_AppParam.motor_hall_dir`
- 方向检测: Hall B 下降沿读 Hall A 电平
- 新增 `motor_hall_set_pole_pairs()` 支持热更新极对数

### 电流传感器 (dev_sensor.h)
- `SENSOR_TYPE_DIFF_AMP_ENABLE` 由 `BOARD_VERSION` 管理
- 整合板: 差分运放模式 (0mV零点, 100mV/A)
- 原板: 霍尔电流传感器模式 (1650mV零点, 66mV/A)

### 电压检测 (dev_voltage.c/.h)
- `VOLTAGE_DIVIDER_TOP_OHM` 由 `BOARD_VERSION` 管理
- 新增 `VOLTAGE_COMPENSATION_MV = 400` (0.4V 补偿)

### RTurn 角度 (dev_rturn.c/h, App_Motor_Project.h)
- 减速比 `RTURN_REDUCTION_RATIO = 1183.0f`，现可运行时修改 (0x3712)
- 角度范围: -2° ~ 88°
- 角速度公式: `RPM × 360 / 减速比 / 60`

### 霍尔脉冲累计 (新增)
- 变量: `volatile int32_t g_s32HallPulseAccum` (RAM, 上电=0)
- 寄存器: 0x3714 (低16位) + 0x3715 (高16位)，读2寄存器得uint32
- 开窗(逆时针)→加, 关窗(顺时针)→减
- 写 0 到 0x3714 可重置
- 公式: `角度(°) = 脉冲数 / 12 / 减速比 × 360°`

### Flash 可配置寄存器 (App_Params.h)
所有配置寄存器通过 Modbus 0x03/0x06 读写:

| 地址 | 名称 | 默认值 | 说明 |
|------|------|--------|------|
| 0x2710 | 设备地址 | 1 | 1~247 |
| 0x2714 | 电压上限 | 270 (27.0V) | 0.1V |
| 0x2715 | 电压下限 | 210 (21.0V) | 0.1V |
| 0x2716 | 电流上限 | 3000 (3A) | mA |
| 0x271C | 关窗极限角度 | -20 (-2.0°) | 0.1° |
| 0x271D | 开窗极限角度 | 880 (88.0°) | 0.1° |
| 0x271E | 电流检测时间 | 20 | ms |
| 0x3710 | 霍尔方向 | 0 | 0=正常 1=反转 |
| 0x3711 | 电机方向 | 0 | 0=正常 1=反转 |
| 0x3712 | 减速比 | 1183 | — |
| 0x3713 | 电机极对数 | 3 | — |

### 实时数据 (只读)
| 地址 | 名称 | 类型 |
|------|------|------|
| 0x2730 | 实时转速 | int16 (r/min) |
| 0x2731 | 实时角度 | int16 (0.1°) |
| 0x2732 | 实时电压 | uint16 (0.1V) |
| 0x2733 | 实时电流 | uint16 (mA) |
| 0x2737 | 实时方向 | int16 |
| 0x2740 | 故障状态 | uint16 |
| 0x3714+0x3715 | 霍尔脉冲累计 | uint32 |

### 控制命令 (0x2720 写入)
- 0x0001=START, 0x0002=STOP, 0x0004=ESTOP
- 0x0011=FWD(开窗), 0x0021=REV(关窗), 0x0008=RESET

### 辅助工具
- `modbus_test_cmds.py` — Modbus 指令生成器源码
- `dist/modbus_tool.exe` — 打包好的交互式 EXE

### 已知问题/注意事项
1. 新增 Flash 字段后需 `Modbus_Init` 检测并强制保存默认值
2. 霍尔脉冲累计的 `last_total` 是 static 变量，当前只支持单实例
3. PWM 停止使用 50% 交替极性，预驱芯片 SDH21263 需确认刹车效果
4. dev_motor.c/h 中有部分 UTF-16LE 编码文件，修改时需注意编码
