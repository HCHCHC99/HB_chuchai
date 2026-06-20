#include "App_Motor_Project.h"
#include "hc32_ll.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "rtt_log.h"
#include "dev_rturn.h"          // 添加这行
#include "App_Params.h"         // 访问 g_AppParam 读取 Modbus 寄存器配置
// ========== è???±?á??¨ò? ==========
SystemSim_t g_sim = {
    .sim_pwr_pos = 0,
    .sim_pwr_neg = 0,
    .sim_hall_up = 0,
    .sim_hall_down = 0,
    .sim_io_fwd = 0,
    .sim_io_rev = 0,
    .sim_io_speed = 85.0f,
    .sim_adc_val = 1500,
    .sim_motor_speed = 0,
    .sim_motor_dir = 0
};

SystemStatus_t g_status = {
    .motor_status = 0,
    .power_status = 0,
    .hall_status = 0,
    .io_status = 0,
    .current_duty = 0.0f
};

// 在文件开头，其他静态变量定义附近添加
RTurn_Device_t* g_rturn_dev = NULL;


// ========== ?2ì?éè±?êμày ==========
static PWM_Device_t g_pwm_dev;
static MotorDevice_t g_motor_dev;
// Keil Watch 调试用：指向电机仲裁器实例的全局指针
MotorDevice_t* volatile g_pMotorDevWatch = NULL;
static Power_Device_t* g_pwr_pos_dev = NULL;
static Power_Device_t* g_pwr_neg_dev = NULL;
static IO_Device_t* g_io_fwd_dev = NULL;
static IO_Device_t* g_io_rev_dev = NULL;
static Hall_Device_t* g_hall_up_dev = NULL;
static Hall_Device_t* g_hall_down_dev = NULL;
static Voltage_Device_t* g_voltage_bus_dev = NULL;
static Sensor_Device_t* g_sensor_current_dev = NULL;

// ========== ?ú2?oˉêyéù?÷ ==========
static void RegisterAllDevices(void);
static void SetupEventBusSubscriptions(void);
static void UpdateStatusIndicators(void);
static void ProcessDeviceUpdates(void);
static void SetDeviceUpdateIntervals(void);

// ========== éè±?×￠2áoˉêy ==========

static void RegisterAllDevices(void) {
    // // 1. ×￠2áPWMéè±?
    // PWM_Config_t pwmConfig = {
    //     .tmr = NULL,  // Dèòa?ù?Yêμ?êó2?t????
    //     .periphClk = 0,
    //     .channel = 0,
    //     .port = 0,
    //     .pin = 0,
    //     .pinFunc = 0,
    //     .countMode = 0,
    //     .countDir = 0,
    //     .defaultFreqHz = 20000,
    //     .defaultDutyPercent = 0,
    //     .activePolarity = PWM_ACTIVE_HIGH
    // };
    
    // memcpy(&g_pwm_dev.config, &pwmConfig, sizeof(PWM_Config_t));
    
    // DeviceOps_t pwm_ops = {
    //     .init = PWM_Device_Init,
    //     .deinit = PWM_Device_Deinit,
    //     .read = PWM_Device_Read,
    //     .write = PWM_Device_Write,
    //     .control = PWM_Device_Control,
    //     .update = PWM_Device_Update
    // };
    // DeviceManager_Register(ID_PWM_MOTOR, "PWM_Motor", DEVICE_TYPE_PWM, &g_pwm_dev, pwm_ops);

    // 2. ×￠2á?yμ??′éè±?
    Power_Config_t pwrPosCfg = {
        .port = GPIO_PORT_C,
        .pin = GPIO_PIN_13,
        .active_level = 1,
        .power_id = 0,
        .debounce_ms = 20,
        .window_size = 5,
        .sample_interval = 5
    };
    g_pwr_pos_dev = Power_Device_Create(&pwrPosCfg);
    DeviceManager_Register(ID_PWR_POS, "PowerPositive", DEVICE_TYPE_POWER, 
                           g_pwr_pos_dev, g_power_ops);

    // // 3. ×￠2á?oμ??′éè±? (使用PB2引脚)
    // Power_Config_t pwrNegCfg = {
    //     .port = GPIO_PORT_C,
    //     .pin = GPIO_PIN_14,
    //     .active_level = 1,
    //     .power_id = 1,
    //     .debounce_ms = 20,
    //     .window_size = 5,
    //     .sample_interval = 5
    // };
    // g_pwr_neg_dev = Power_Device_Create(&pwrNegCfg);
    // DeviceManager_Register(ID_PWR_NEG, "PowerNegative", DEVICE_TYPE_POWER, 
    //                        g_pwr_neg_dev, g_power_ops);



    // 原来的霍尔设备注册
    
    // Hall_Config_t upCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_02,
    //     .active_level = 0,           // μíμ???óDD§￡¨′￥·￠ê±à-μí￡?
    //     .bind_dir = DIR_FWD,
    //     .is_soft_limit = 0,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .hall_id = 0
    // };
    // g_hall_up_dev = Hall_Device_Create(&upCfg);
    // DeviceManager_Register(ID_HALL_UP, "LimitUp", DEVICE_TYPE_HALL, 
    //                     g_hall_up_dev, g_hall_ops);

    // // 5. ×￠2á???T?????? (PC14)
    // Hall_Config_t downCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_10,
    //     .active_level = 0,           // μíμ???óDD§￡¨′￥·￠ê±à-μí￡?
    //     .bind_dir = DIR_REV,
    //     .is_soft_limit = 0,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .hall_id = 1
    // };
    // g_hall_down_dev = Hall_Device_Create(&downCfg);
    // DeviceManager_Register(ID_HALL_DOWN, "LimitDown", DEVICE_TYPE_HALL, 
    //                     g_hall_down_dev, g_hall_ops);
    

    // // 6. ×￠2á?y×aIOéè±?
    // IO_Config_t ioFwdCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_13,
    //     .active_level = 1,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .io_id = 0
    // };
    // g_io_fwd_dev = IO_Device_Create(&ioFwdCfg);
    // DeviceManager_Register(ID_IO_FWD, "IO_Forward", DEVICE_TYPE_IO, 
    //                        g_io_fwd_dev, g_io_ops);

    // // 7. ×￠2á·′×aIOéè±?
    // IO_Config_t ioRevCfg = {
    //     .port = GPIO_PORT_B,
    //     .pin = GPIO_PIN_12,
    //     .active_level = 1,
    //     .debounce_ms = 20,
    //     .window_size = 3,
    //     .sample_interval = 2,
    //     .io_id = 1
    // };
    // g_io_rev_dev = IO_Device_Create(&ioRevCfg);
    // DeviceManager_Register(ID_IO_REV, "IO_Reverse", DEVICE_TYPE_IO, 
    //                        g_io_rev_dev, g_io_ops);

    // 8. ×￠2áμ??ú?ù2?éè±?
    DeviceOps_t motor_ops = {
        .init = Motor_Init,
        .deinit = Motor_Deinit,
        .read = Motor_Read,
        .write = Motor_Write,
        .control = Motor_Control,
        .update = Motor_Update
    };
    DeviceManager_Register(ID_MOTOR, "MotorArbitrator", DEVICE_TYPE_MOTOR, 
                           &g_motor_dev, motor_ops);
    // Keil Watch 调试：让全局指针指向电机仲裁器实例
    g_pMotorDevWatch = &g_motor_dev;

    // 注册电机霍尔设备
    motor_hall_config_t motorHallCfg = {
        .hall_a_port = GPIO_PORT_A,
        .hall_a_pin = GPIO_PIN_10,
        .hall_b_port = GPIO_PORT_A,
        .hall_b_pin = GPIO_PIN_09,
        .eirq_ch_a = EXTINT_CH10,
        .eirq_ch_b = EXTINT_CH09,
        .irqn_a = INT010_IRQn,
        .irqn_b = INT009_IRQn,
        .irq_src_a = INT_SRC_PORT_EIRQ10,
        .irq_src_b = INT_SRC_PORT_EIRQ9,
        .irq_priority = 2,
        .pole_pairs = 3,
        .hall_count = 2,
        .custom_pulses_per_rev = 0
    };

    MotorHall_DeviceConfig_t motorHallDevCfg = {
        .motor_id = 0,
        .update_interval_ms = 1
    };

    MotorHall_Device_t* motor_hall_dev = MotorHall_Device_Create(&motorHallCfg, &motorHallDevCfg);

    MAIN_D("Before Register: motor_hall_dev=0x%p\r\n", motor_hall_dev);
    MAIN_D("Before Register: motor_hall_dev->config.hall_a_pin=0x%04X\r\n", motor_hall_dev->config.hall_a_pin);

    DeviceManager_Register(ID_MOTOR_HALL, "MotorHall", DEVICE_TYPE_HALL, 
                        motor_hall_dev, g_motor_hall_ops);

    // 注册后立即验证
    DeviceNode_t* check_node = DeviceManager_Get(ID_MOTOR_HALL);
    if (check_node && check_node->private_data) {
        MotorHall_Device_t* check_dev = (MotorHall_Device_t*)check_node->private_data;
        MAIN_D("After Register: check_dev=0x%p\r\n", check_dev);
        MAIN_D("After Register: check_dev->config.hall_a_pin=0x%04X\r\n", check_dev->config.hall_a_pin);
    }                    

    // ============================================================
    // ADC 设备注册
    // 注意：ADC 底层硬件初始化由 dev_adc.c 的 ADC_AdpLayerInit() 自动完成
    // 第一个 ADC 设备初始化时会自动调用 Adc_Create/Adc_Init/Adc_Start
    // ============================================================

    // 注册电流检测ADC设备 (PA5, CH5) - 中断模式
    ADC_Config_t adcCurrentCfg = {
        .u8AdcId = 0,                       // 由 ADC_AdpLayerInit 自动分配
        .u8Channel = 5,                     // ADC_CH5 = PA5
        .u8Port = GPIO_PORT_A,
        .u16Pin = GPIO_PIN_05,
        .enAcqMode = ADC_ACQ_MODE_INTERRUPT, // 中断模式
        .u16DmaBufferSize = 0,              // 中断模式不需要DMA缓冲区
        .u8DmaChannel = 0                   // 中断模式不需要DMA通道
    };

    ADC_Device_t* adc_current_dev = ADC_Device_Create(&adcCurrentCfg);
    DeviceManager_Register(ID_ADC_CURRENT, "ADC_Current", DEVICE_TYPE_ADC, 
                        adc_current_dev, g_adc_ops);

    // 注册电压检测ADC设备
    //   HB_chuchai 原始配置：PA04, CH4
    //   整合板（与 HandB 一致）：PA06, CH6
    //   取消注释可还原为 PA04 + CH4
    ADC_Config_t adcVoltageCfg = {
        .u8AdcId = 1,                       // 由 ADC_AdpLayerInit 自动分配
        .u8Channel = 6,                     // ADC_CH6 = PA06 (整合板); 原 HB_chuchai: CH4 = PA04
        .u8Port = GPIO_PORT_A,
        .u16Pin = GPIO_PIN_06,              // PA06 (整合板); 原 HB_chuchai: GPIO_PIN_04
        .enAcqMode = ADC_ACQ_MODE_INTERRUPT, // 中断模式
        .u16DmaBufferSize = 0,              // 中断模式不需要DMA缓冲区
        .u8DmaChannel = 0                   // 中断模式不需要DMA通道
    };

    ADC_Device_t* adc_voltage_dev = ADC_Device_Create(&adcVoltageCfg);
    DeviceManager_Register(ID_ADC_VOLTAGE, "ADC_Voltage", DEVICE_TYPE_ADC, 
                        adc_voltage_dev, g_adc_ops);

    // 注册电压母线设备（基于ADC_VOLTAGE计算实际母线电压）
    // 告警阈值从 g_AppParam 读取（支持 Flash 配置）
    {
        // 从 Flash 读取电压配置，单位转换：0.1V -> mV
        uint32_t u32OvervoltageThresholdMv = (uint32_t)g_AppParam.voltage_upper_limit * 100UL;
        uint32_t u32UndervoltageThresholdMv = (uint32_t)g_AppParam.voltage_lower_limit * 100UL;
        uint32_t u32OvervoltageHysteresisMv = (uint32_t)g_AppParam.voltage_upper_hysteresis * 100UL;
        uint32_t u32UndervoltageHysteresisMv = (uint32_t)g_AppParam.voltage_lower_hysteresis * 100UL;
        uint8_t u8OvervoltageTriggerCount = g_AppParam.overvoltage_trigger_count;
        uint8_t u8UndervoltageTriggerCount = g_AppParam.undervoltage_trigger_count;
        
        // 如果 Flash 中的值为 0，回退到默认值（保证设备能正常工作）
        if (u32OvervoltageThresholdMv == 0) {
            u32OvervoltageThresholdMv = PARAM_DEFAULT_VOLTAGE_UPPER_LIMIT * 100UL;
            PARAMS_DBG("[VOLTAGE] Overvoltage threshold is 0, using default: %lu mV", u32OvervoltageThresholdMv);
        }
        if (u32UndervoltageThresholdMv == 0) {
            u32UndervoltageThresholdMv = PARAM_DEFAULT_VOLTAGE_LOWER_LIMIT * 100UL;
            PARAMS_DBG("[VOLTAGE] Undervoltage threshold is 0, using default: %lu mV", u32UndervoltageThresholdMv);
        }
        if (u32OvervoltageHysteresisMv == 0) {
            u32OvervoltageHysteresisMv = PARAM_DEFAULT_VOLTAGE_UPPER_HYSTERESIS * 100UL;
        }
        if (u32UndervoltageHysteresisMv == 0) {
            u32UndervoltageHysteresisMv = PARAM_DEFAULT_VOLTAGE_LOWER_HYSTERESIS * 100UL;
        }
        if (u8OvervoltageTriggerCount == 0) {
            u8OvervoltageTriggerCount = PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT;
        }
        if (u8UndervoltageTriggerCount == 0) {
            u8UndervoltageTriggerCount = PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT;
        }
        
        MAIN_D("[APP] Voltage config from Flash: upper=%lu mV (0.1V=%d), lower=%lu mV (0.1V=%d), "
            "upper_hys=%lu mV, lower_hys=%lu mV, ov_cnt=%d, uv_cnt=%d",
            u32OvervoltageThresholdMv, g_AppParam.voltage_upper_limit,
            u32UndervoltageThresholdMv, g_AppParam.voltage_lower_limit,
            u32OvervoltageHysteresisMv, u32UndervoltageHysteresisMv,
            u8OvervoltageTriggerCount, u8UndervoltageTriggerCount);
        
        Voltage_Config_t voltageBusCfg = {
            .u8AdcDevId = ID_ADC_VOLTAGE,
            
            // 过压检测配置
            .u32OvervoltageThresholdMv = u32OvervoltageThresholdMv,
            .u32OvervoltageHysteresisMv = u32OvervoltageHysteresisMv,
            .u8OvervoltageTriggerCount = u8OvervoltageTriggerCount,
            
            // 欠压检测配置
            .u32UndervoltageThresholdMv = u32UndervoltageThresholdMv,
            .u32UndervoltageHysteresisMv = u32UndervoltageHysteresisMv,
            .u8UndervoltageTriggerCount = u8UndervoltageTriggerCount,
        };
        g_voltage_bus_dev = Voltage_Device_Create(&voltageBusCfg);
        DeviceManager_Register(ID_VOLTAGE_BUS, "VoltageBus", DEVICE_TYPE_SENSOR,
                            g_voltage_bus_dev, g_voltage_ops);
    }

    // 注册电流传感器设备
    // 从 Modbus 寄存器读取电流阈值和检测窗口，支持运行时动态配置
    {
        int32_t s32ThresholdMa = (int32_t)g_AppParam.current_upper_limit;
        uint32_t u32TriggerMs = (uint32_t)g_AppParam.current_detect_ms;
        uint32_t u32ReleaseMs = (uint32_t)g_AppParam.current_release_ms;
        int32_t s32HysteresisMa = (int32_t)g_AppParam.current_hysteresis_ma;
        uint8_t u8TriggerCount = g_AppParam.overcurrent_trigger_count;
        
        // 如果寄存器值为 0（未配置），回退到默认值
        // 注意：0 是有效值（用于测试），但正常情况下不应设为 0
        // 如果 Flash 未初始化（全 0xFF），g_AppParam.current_upper_limit 会是 0xFFFFFFFF（int32_t 的 -1）
        // 所以这里判断 < 0 表示未初始化，= 0 表示用户有意设为 0
        if (s32ThresholdMa < 0) {
            s32ThresholdMa = PARAM_DEFAULT_CURRENT_UPPER_LIMIT;
            PARAMS_DBG("[CURRENT] Threshold is invalid (%ld), using default: %ld mA", 
                       (long)s32ThresholdMa, (long)PARAM_DEFAULT_CURRENT_UPPER_LIMIT);
        }
        if (u32TriggerMs == 0) {
            u32TriggerMs = PARAM_DEFAULT_CURRENT_DETECT_MS;
        }
        if (u32ReleaseMs == 0) {
            u32ReleaseMs = PARAM_DEFAULT_CURRENT_RELEASE_MS;
        }
        if (s32HysteresisMa == 0) {
            s32HysteresisMa = PARAM_DEFAULT_CURRENT_HYSTERESIS_MA;
        }
        if (u8TriggerCount == 0) {
            u8TriggerCount = PARAM_DEFAULT_OVERCURRENT_TRIGGER_CNT;
        }
        
        MAIN_D("[APP] Sensor config from Flash: threshold=%ldmA, trigger_window=%ldms, release_window=%ldms, hysteresis=%ldmA, trigger_cnt=%d",
            (long)s32ThresholdMa, (long)u32TriggerMs, (long)u32ReleaseMs, (long)s32HysteresisMa, (int)u8TriggerCount);
        
        Sensor_Config_t sensorCurrentCfg = {
            .u8AdcDevId = ID_ADC_CURRENT,
            
            .s32OvercurrentThresholdMa = s32ThresholdMa,
            .s32OvercurrentHysteresisMa = s32HysteresisMa,
            
            .u8OvercurrentMode = OVERCURRENT_MODE_TIME_WINDOW,  // 编译时固定使用时间模式
            
            // 点数模式配置（时间模式下不使用）
            .u16TriggerWindowSize = 0,
            .u16ReleaseWindowSize = 0,
            
            // 时间模式配置
            .u32TriggerWindowMs = u32TriggerMs,
            .u32ReleaseWindowMs = u32ReleaseMs,
        };
        g_sensor_current_dev = Sensor_Device_Create(&sensorCurrentCfg);
        DeviceManager_Register(ID_SENSOR_CURRENT, "SensorCurrent", DEVICE_TYPE_SENSOR,
                            g_sensor_current_dev, g_sensor_ops);
    }

    // 注册圆弧转动机构设备
    RTurn_Config_t rturnCfg = {
        .u8MotorHallDevId = ID_MOTOR_HALL,                      // 绑定电机霍尔设备
        .u8MotorArbiterDevId = ID_MOTOR,                        // 期望方向（电机仲裁器）        
        .u8SensorDevId = ID_SENSOR_CURRENT,                     // 绑定电流传感器设备
        .fReductionRatio = RTURN_REDUCTION_RATIO,               // 减速比
        .fMaxAngle = RTURN_MAX_ANGLE,                           // 最大角度
        .fMinAngle = RTURN_MIN_ANGLE,                           // 最小角度
        .u8ReverseOutput = RTURN_REVERSE_OUTPUT,                // 输出方向反向标志
        .u8DeviceId = ID_RTURN,
        .u16UpdateIntervalMs = RTURN_UPDATE_INTERVAL_MS         // 更新间隔
    };

    // RTurn_Device_t* rturn_dev = RTurn_Device_Create(&rturnCfg);
    g_rturn_dev = RTurn_Device_Create(&rturnCfg);
    DeviceManager_Register(ID_RTURN, "RTurn", DEVICE_TYPE_SENSOR, 
                        g_rturn_dev, g_rturn_ops);


                        
}

// ========== EventBus ???????? ==========
static void SetupEventBusSubscriptions(void) {
    // 电机设备订阅各主题（全部使用优先级0，后续可根据需要调整）
    EventBus_Subscribe(TOPIC_POWER, Motor_OnPowerEvent, 0);
    EventBus_Subscribe(TOPIC_LIMIT_HARD, Motor_OnHardLimit, 0);
    EventBus_Subscribe(TOPIC_LIMIT_SOFT, Motor_OnHardLimit, 0);
    EventBus_Subscribe(TOPIC_MANUAL_IO, Motor_OnManualIO, 0);
    EventBus_Subscribe(TOPIC_MOTOR_SPEED_FEEDBACK, Motor_OnSpeedFeedback, 0);    
    // 例如电机设备订阅过流事件
    EventBus_Subscribe(TOPIC_ALARM, Motor_OnOvercurrent, 0);
    EventBus_Subscribe(TOPIC_VOLTAGE_ALARM, Motor_OnVoltageAlarm, 0);
    // 圆弧转动机构订阅过流事件
    EventBus_Subscribe(TOPIC_CURRENT_ALARM, RTurn_OnCurrentAlarm, 1);	
    EventBus_Subscribe(TOPIC_CURRENT_ALARM, Motor_OnCurrentAlarm, 1);

    // 电机仲裁器订阅圆弧转动机构限位事件
    EventBus_Subscribe(TOPIC_RTURN_LIMIT, Motor_OnRTurnLimit, 0);
    // RS485 手动控制（通过 Modbus REG_CTRL_CMD 触发）
    EventBus_Subscribe(TOPIC_MANUAL_RS485, Motor_OnManualIO, 0);
}

// ========== éè±??üD??μ?ê???? ==========
static void SetDeviceUpdateIntervals(void) {
    DeviceManager_SetUpdateInterval(ID_PWR_POS, 1);
    DeviceManager_SetUpdateInterval(ID_PWR_NEG, 1);
    // ????éè±? - éè???a0￡???′????üD? (已注释)
    // DeviceManager_SetUpdateInterval(ID_HALL_UP, 1);    // é??T??
    // DeviceManager_SetUpdateInterval(ID_HALL_DOWN, 1);  // ???T??
    // DeviceManager_SetUpdateInterval(ID_IO_FWD, 1000);
    // DeviceManager_SetUpdateInterval(ID_IO_REV, 1000);
    // DeviceManager_SetUpdateInterval(ID_PWM_MOTOR, 10000);
    DeviceManager_SetUpdateInterval(ID_MOTOR, 1);
    DeviceManager_SetUpdateInterval(ID_MOTOR_HALL, 1);  // 每10ms更新
    // 在 SetDeviceUpdateIntervals 函数中添加
    DeviceManager_SetUpdateInterval(ID_ADC_CURRENT, 1);   // ADC电流采样
    DeviceManager_SetUpdateInterval(ID_ADC_VOLTAGE, 1);   // ADC电压采样
    
    // 电压母线和电流传感器设备 - 每10ms从ADC读取并计算一次
    DeviceManager_SetUpdateInterval(ID_VOLTAGE_BUS, 10);   // 每10ms计算一次母线电压
    DeviceManager_SetUpdateInterval(ID_SENSOR_CURRENT, 1); // 每10ms计算一次电流值
    DeviceManager_SetUpdateInterval(ID_RTURN, 1);  // 每10ms更新一次角度
}

// ========== ?￡?a?÷′|àí ==========
#if ENABLE_SIMULATION_MODE

void Sim_ProcessInput(void) {
    static uint32_t last_update = 0;
    uint32_t now = tickTimer_GetCount();
    
    if (now - last_update < 100) return;
    last_update = now;
}

void Sim_PublishEvents(void) {
    uint32_t now = tickTimer_GetCount();
    (void)now;  // 避免未使用警告
    
    // 电源事件
    static uint8_t last_pwr_pos = 0;
    static uint8_t last_pwr_neg = 0;
    
    if (g_sim.sim_pwr_pos != last_pwr_pos) {
        MotorPowerEvent_t pwrEvent = {.power_id = 0, .is_on = (g_sim.sim_pwr_pos != 0)};
        EventBus_Publish(TOPIC_POWER, &pwrEvent);
        MAIN_D("[SIM] POWER_POS changed: %d -> %d\r\n", last_pwr_pos, g_sim.sim_pwr_pos);
        last_pwr_pos = g_sim.sim_pwr_pos;
    }
    
    if (g_sim.sim_pwr_neg != last_pwr_neg) {
        MotorPowerEvent_t pwrEvent = {.power_id = 1, .is_on = (g_sim.sim_pwr_neg != 0)};
        EventBus_Publish(TOPIC_POWER, &pwrEvent);
        MAIN_D("[SIM] POWER_NEG changed: %d -> %d\r\n", last_pwr_neg, g_sim.sim_pwr_neg);
        last_pwr_neg = g_sim.sim_pwr_neg;
    }
    
    // 限位事件
    static uint8_t last_hall_up = 0;
    static uint8_t last_hall_down = 0;
    
    if (g_sim.sim_hall_up != last_hall_up) {
        MotorLimitEvent_t limitEvent = {.dir = DIR_FWD, .is_active = (g_sim.sim_hall_up != 0)};
        EventBus_Publish(TOPIC_LIMIT_HARD, &limitEvent);
        MAIN_D("[SIM] HALL_UP changed: %d -> %d\r\n", last_hall_up, g_sim.sim_hall_up);
        last_hall_up = g_sim.sim_hall_up;
    }
    
    if (g_sim.sim_hall_down != last_hall_down) {
        MotorLimitEvent_t limitEvent = {.dir = DIR_REV, .is_active = (g_sim.sim_hall_down != 0)};
        EventBus_Publish(TOPIC_LIMIT_HARD, &limitEvent);
        MAIN_D("[SIM] HALL_DOWN changed: %d -> %d\r\n", last_hall_down, g_sim.sim_hall_down);
        last_hall_down = g_sim.sim_hall_down;
    }
    
    // IO事件 - 只在有变化时打印
    static uint8_t last_io_fwd = 0;
    static uint8_t last_io_rev = 0;
    
    // 正转IO变化检测
    if (g_sim.sim_io_fwd != last_io_fwd) {
        MAIN_D("[SIM] IO_FWD changed: %d -> %d\r\n", last_io_fwd, g_sim.sim_io_fwd);
        
        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));  // 初始化所有字段
        
        if (g_sim.sim_io_fwd) {
            ioEvent.dir = DIR_FWD;
            ioEvent.type = CMD_TYPE_RUN_FWD;
            ioEvent.speed = g_sim.sim_io_speed;
        } else {
            ioEvent.dir = DIR_FWD;
            ioEvent.type = CMD_TYPE_STOP;
            ioEvent.speed = 0.0f;
        }
        
        // 打印发布的事件内容
        int32_t speed_int = (int32_t)(ioEvent.speed * 10);
        MAIN_D("[SIM] Publishing IO_FWD event: dir=%d, type=%d, speed=%ld.%ld%%\r\n",
               ioEvent.dir, ioEvent.type, (long)(speed_int / 10), (long)(speed_int % 10));
        
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
        last_io_fwd = g_sim.sim_io_fwd;
    }
    
    // 反转IO变化检测
    if (g_sim.sim_io_rev != last_io_rev) {
        MAIN_D("[SIM] IO_REV changed: %d -> %d\r\n", last_io_rev, g_sim.sim_io_rev);
        
        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));  // 初始化所有字段
        
        if (g_sim.sim_io_rev) {
            ioEvent.dir = DIR_REV;
            ioEvent.type = CMD_TYPE_RUN_REV;
            ioEvent.speed = g_sim.sim_io_speed;
        } else {
            ioEvent.dir = DIR_REV;
            ioEvent.type = CMD_TYPE_STOP;
            ioEvent.speed = 0.0f;
        }
        
        // 打印发布的事件内容
        int32_t speed_int = (int32_t)(ioEvent.speed * 10);
        MAIN_D("[SIM] Publishing IO_REV event: dir=%d, type=%d, speed=%ld.%ld%%\r\n",
               ioEvent.dir, ioEvent.type, (long)(speed_int / 10), (long)(speed_int % 10));
        
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
        last_io_rev = g_sim.sim_io_rev;
    }
}

#endif

// ========== ×′ì??üD?oˉêy ==========

static void UpdateStatusIndicators(void) {
    static uint32_t last_update = 0;
    uint32_t now = tickTimer_GetCount();
    
    if (now - last_update < 50) return;
    last_update = now;
    
    // ??è?μ??úμ÷ê?D??￠
    const MotorDebugInfo_t* dbg = Motor_GetDebugInfo(&g_motor_dev);
    if (dbg) {
        if (dbg->state == MS_IDLE) {
            g_status.motor_status = MOTOR_STOPPED;
        } else if (dbg->active_dir == DIR_FWD) {
            g_status.motor_status = MOTOR_FORWARD;
        } else if (dbg->active_dir == DIR_REV) {
            g_status.motor_status = MOTOR_REVERSE;
        }
        g_status.current_duty = dbg->current_duty;
    }
    
    // ?üD?μ??′×′ì?
    if (g_sim.sim_pwr_pos && g_sim.sim_pwr_neg) {
        g_status.power_status = POWER_BOTH_ON;
    } else if (g_sim.sim_pwr_pos) {
        g_status.power_status = POWER_POS_ON;
    } else if (g_sim.sim_pwr_neg) {
        g_status.power_status = POWER_NEG_ON;
    } else {
        g_status.power_status = POWER_BOTH_OFF;
    }
    
    // ?üD?????×′ì?
    if (g_sim.sim_hall_up && g_sim.sim_hall_down) {
        g_status.hall_status = HALL_BOTH_LIMIT;
    } else if (g_sim.sim_hall_up) {
        g_status.hall_status = HALL_UP_LIMIT;
    } else if (g_sim.sim_hall_down) {
        g_status.hall_status = HALL_DOWN_LIMIT;
    } else {
        g_status.hall_status = HALL_NO_LIMIT;
    }
    
    g_status.io_status = (g_sim.sim_io_fwd ? 1 : (g_sim.sim_io_rev ? 2 : 0));
}

static void ProcessDeviceUpdates(void) {
    DeviceManager_UpdateAll();
}

// ========== ?μí33?ê??ˉ ==========
void ESystem_Init(void) {
    // 1. 先初始化 EventBus（只初始化一次）
    EventBus_Init();
    
    // 2. 初始化 DeviceManager（不再重复初始化 EventBus）
    DeviceManagerConfig_t config = {
        .operation_timeout_ms = 500,
        .enable_mutex = 1,
        .auto_subscribe = 0
    };
    DeviceManager_Init(&config);
    
    // 3. 注册所有设备
    RegisterAllDevices();
    
    // 4. 配置主题订阅（订阅必须在电源设备初始化之前完成！）
    SetupEventBusSubscriptions();
    

    
    // 6. 分阶段初始化设备硬件（先初始化电机仲裁器，再初始化电源）
    // 先初始化电机仲裁器
    MAIN_D("[APP] Initializing motor arbiter first...\r\n");
    Device_Init(ID_MOTOR);
    
    // 再初始化其他设备（包括电源）
    MAIN_D("[APP] Initializing remaining devices...\r\n");
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (i == ID_MOTOR) continue;  // 已初始化
        Device_Init(i);
    }
    
    // 6. 配置设备更新频率
    SetDeviceUpdateIntervals();
    
    // 7. 启用所有设备更新
    DeviceManager_EnableAllUpdate();
    
    memset(&g_status, 0, sizeof(SystemStatus_t));
}

// ========== ?μí3?÷?-?· ==========
// ========== 热重载配置（替代软件复位） ==========
/**
 * @brief 将 g_AppParam 的最新值推送到各个设备模块
 *        Modbus 写入新配置后调用，避免软件复位
 */
void App_ReloadConfig(void)
{
    MAIN_D("[RELOAD] Reloading config from g_AppParam...\r\n");

    /* --- 电压母线设备：更新过压/欠压阈值 --- */
    if (g_voltage_bus_dev != NULL)
    {
        g_voltage_bus_dev->stcConfig.u32OvervoltageThresholdMv =
            (uint32_t)g_AppParam.voltage_upper_limit * 100UL;
        g_voltage_bus_dev->stcConfig.u32UndervoltageThresholdMv =
            (uint32_t)g_AppParam.voltage_lower_limit * 100UL;
        g_voltage_bus_dev->stcConfig.u32OvervoltageHysteresisMv =
            (uint32_t)g_AppParam.voltage_upper_hysteresis * 100UL;
        g_voltage_bus_dev->stcConfig.u32UndervoltageHysteresisMv =
            (uint32_t)g_AppParam.voltage_lower_hysteresis * 100UL;
        g_voltage_bus_dev->stcConfig.u8OvervoltageTriggerCount =
            g_AppParam.overvoltage_trigger_count;
        g_voltage_bus_dev->stcConfig.u8UndervoltageTriggerCount =
            g_AppParam.undervoltage_trigger_count;

        MAIN_D("[RELOAD] Voltage: over=%lu mV, under=%lu mV\r\n",
               g_voltage_bus_dev->stcConfig.u32OvervoltageThresholdMv,
               g_voltage_bus_dev->stcConfig.u32UndervoltageThresholdMv);
    }

    /* --- 电流传感器设备：更新过流阈值 --- */
    if (g_sensor_current_dev != NULL)
    {
        g_sensor_current_dev->stcConfig.s32OvercurrentThresholdMa =
            (int32_t)g_AppParam.current_upper_limit;
        g_sensor_current_dev->stcConfig.u32TriggerWindowMs =
            (uint32_t)g_AppParam.current_detect_ms;
        g_sensor_current_dev->stcConfig.u32ReleaseWindowMs =
            (uint32_t)g_AppParam.current_release_ms;
        g_sensor_current_dev->stcConfig.s32OvercurrentHysteresisMa =
            (int32_t)g_AppParam.current_hysteresis_ma;

        MAIN_D("[RELOAD] Current: threshold=%ld mA, trigger=%lu ms, release=%lu ms\r\n",
               (long)g_sensor_current_dev->stcConfig.s32OvercurrentThresholdMa,
               (unsigned long)g_sensor_current_dev->stcConfig.u32TriggerWindowMs,
               (unsigned long)g_sensor_current_dev->stcConfig.u32ReleaseWindowMs);
    }

    /* --- 转速/角度目标值直接从 g_AppParam 读取，无需推送 --- */
    /* --- 设备地址 (node_id) 已写入 Flash，下次 Param_Init 时生效 --- */

    MAIN_D("[RELOAD] Config reload complete.\r\n");
}


void ESystem_MainLoop(void) {
    static uint32_t last_loop_time = 0;
    uint32_t now = tickTimer_GetCount();
    
    if (now - last_loop_time < 1) return;
    last_loop_time = now;
    
#if ENABLE_SIMULATION_MODE
    Sim_ProcessInput();
    Sim_PublishEvents();
#endif
    
    ProcessDeviceUpdates();
    UpdateStatusIndicators();
}
