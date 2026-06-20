#ifndef DEV_MOTOR_H_
#define DEV_MOTOR_H_

#include "device_manager.h"
#include "EventBus.h"
#include "dev_power.h"
#include "dev_hall.h"
#include "dev_voltage.h"
#include "dev_adc.h"
#include "dev_sensor.h"
#include "dev_rturn.h"
#include "dev_motor_hall.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== ���Ժ궨�� ==========
// ������ rtt_manager.h ��ͳһ������DEV_MOTOR

#ifdef DEV_MOTOR
    #define MOTOR_DEBUG(fmt, ...)    MAIN_D("[MOTOR_DEBUG] " fmt, ##__VA_ARGS__)
    #define MOTOR_OUT(fmt, ...)      MAIN_D("[MOTOR_OUT] " fmt, ##__VA_ARGS__)
#else
    #define MOTOR_DEBUG(fmt, ...)    ((void)0)
    #define MOTOR_OUT(fmt, ...)      ((void)0)
#endif


// ========== 硬件板本：电机控制模�? ==========
// �? main.h �? BOARD_VERSION 统一管理
#include "main.h"
#if BOARD_VERSION == 0
    // 原HB_chuchai板：GPIO PB8/PB9 直接控制正反�?/停止
    #define MOTOR_CONTROL_MODE  0
#else
    // 整合板：4通道 PWM 占空比控制，缓启�?/缓停
    #define MOTOR_CONTROL_MODE  1
#endif

// ========== ����豸�����붨��? ==========
// ע�⣺CMD_BASE_MOTOR �� device_manager.h ��û��Ԥ���壬���ﶨ��
#define CMD_BASE_MOTOR              0x9000
#define CMD_MOTOR_STOP              (CMD_BASE_MOTOR + 0x01)
#define CMD_MOTOR_RUN_FWD           (CMD_BASE_MOTOR + 0x02)
#define CMD_MOTOR_RUN_REV           (CMD_BASE_MOTOR + 0x03)
#define CMD_MOTOR_SET_SPEED         (CMD_BASE_MOTOR + 0x04)
#define CMD_MOTOR_EMERGENCY_STOP    (CMD_BASE_MOTOR + 0x05)
// ���ӵ� dev_motor.h �������붨������
#define CMD_MOTOR_GET_DESIRED_DIR   (CMD_BASE_MOTOR + 0x06)   // ��ȡ��������������


// ========== ����豸���ú�? ==========
// ģʽ�л��꣺0=�����ԣ�����Դȫ�ţ���1=˫���ԣ�˫��Դ���ţ�
#ifndef MOTOR_MODE_BIPOLAR
#define MOTOR_MODE_BIPOLAR      0
#endif

// ���ȼ�ģʽ���ã�1=IO���ȼ���, 0=CAN���ȼ��ߣ�
#ifndef MOTOR_PRIORITY_IO_HIGH
#define MOTOR_PRIORITY_IO_HIGH  1
#endif

// �豸�������ã���λ��ϣ�?
#ifndef MOTOR_MANUAL_CAPABILITY
#define MOTOR_MANUAL_CAPABILITY  (CAP_ALLOW | CAP_BLOCK)  // IO�豸������������Ҳ����ֹ
#endif

#ifndef MOTOR_CAN_CAPABILITY
#define MOTOR_CAN_CAPABILITY     (CAP_ALLOW)  // CAN�豸����
#endif

// �豸����λ����
#define CAP_BLOCK      (1 << 0)
#define CAP_ALLOW      (1 << 1)

// ========== ������ö�ٶ��� ==========
// ���״̬�ṹ�壨����? Device_Read һ���Զ�ȡ��
// ========== ���״̬�ṹ�壨����? Device_Read һ���Զ�ȡ�� ==========


typedef enum {
    DIR_NONE = 0,
    DIR_FWD = 1,
    DIR_REV = 2
} MotorDir_t;

typedef enum {
    MODE_NONE = 0,
    MODE_AUTO = 1,
    MODE_REMOTE = 2,
    MODE_MANUAL = 3
} MotorMode_t;

typedef enum {
    MS_IDLE = 0,
    MS_RAMPING = 1,
    MS_RUNNING = 2,
    MS_LOCKED = 3
} MotorState_t;

typedef enum {
    CMD_TYPE_NONE_USE = 255,
    CMD_TYPE_STOP = 1,
    CMD_TYPE_RUN_FWD = 2,
    CMD_TYPE_RUN_REV = 3,
    CMD_TYPE_RAMP_FWD = 4,
    CMD_TYPE_RAMP_REV = 5,
    CMD_TYPE_BLOCK_FWD = 6,
    CMD_TYPE_BLOCK_REV = 7,
    CMD_TYPE_BLOCK_BOTH = 8,
} CmdType_t;

// ========== �豸IDö�� ==========
typedef enum {
    DEV_ID_NONE             = 255,
    DEV_ID_POWER_POS        = 1,
    DEV_ID_POWER_NEG        = 2,
    DEV_ID_LIMIT_FWD        = 3,
    DEV_ID_LIMIT_REV        = 4,
    DEV_ID_CAN              = 5,
    DEV_ID_IO_FWD           = 6,    // ��תIO�豸
    DEV_ID_IO_REV           = 7,    // ��תIO�豸
    DEV_ID_EMERGENCY        = 8,
    DEV_ID_RTURN_FWD        = 9,    // Բ��ת��������ת��λ
    DEV_ID_RTURN_REV        = 10,   // Բ��ת��������ת��λ
    DEV_ID_OVERVOLTAGE_FWD  = 11,   // ��ѹ������ת
    DEV_ID_OVERVOLTAGE_REV  = 12,   // ��ѹ������ת
    DEV_ID_UNDERVOLTAGE_FWD = 13,   // Ƿѹ������ת
    DEV_ID_UNDERVOLTAGE_REV = 14,   // Ƿѹ������ת
    DEV_ID_OVERCUR_FWD      = 15,   // ����������ת������ת����ת��������Ԥ�ڹ�����
    DEV_ID_MAX
} MotorDeviceId_t;

// ���ȼ�ö��
typedef enum {
    PRIO_NONE = 255,
    PRIO_EMERGENCY = 0,
    PRIO_LIMIT = 2,
    PRIO_MANUAL = 3,
    PRIO_CAN = 4,
    PRIO_POWER = 5
} MotorPriority_t;

// ========== ���������? ==========
typedef struct {
    MotorDeviceId_t device_id;
    MotorPriority_t priority;
    CmdType_t type;
    float param;
    uint32_t timestamp;
} MotorControlCommand_t;

typedef struct {
    MotorDir_t desired_dir;     // ���������ٲ��������?
    MotorState_t state;         // ���״̬��IDLE/RAMPING/RUNNING��
    MotorDir_t active_dir;      // ��ǰ�����
    float current_duty;         // ��ǰռ�ձ�
    uint8_t enable;             // ���ʹ��״�?
} Motor_StateInfo_t;

#define MAX_COMMANDS_PER_DIRECTION 20

typedef struct {
    MotorControlCommand_t commands[MAX_COMMANDS_PER_DIRECTION];
    uint8_t count;
} MotorCommandList_t;

// ========== ������Ϣ�ṹ ==========
typedef struct {
    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } block_fwd;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } block_rev;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t priorities[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } allow_fwd;

    struct {
        MotorDeviceId_t device_ids[MAX_COMMANDS_PER_DIRECTION];
        uint8_t priorities[MAX_COMMANDS_PER_DIRECTION];
        uint8_t count;
    } allow_rev;

    MotorDeviceId_t active_device_id;
    MotorState_t state;
    MotorDir_t active_dir;
    float current_duty;
    bool conflict_fault;
} MotorDebugInfo_t;

// ========== ����豸�ṹ��? ==========
typedef struct {
    // �ٲ�������
    MotorCommandList_t block_fwd;
    MotorCommandList_t block_rev;
    MotorCommandList_t allow_fwd;
    MotorCommandList_t allow_rev;
    MotorState_t state;
    MotorDir_t active_dir;
    MotorDeviceId_t active_device_id;
    float current_duty;

    // ������Ϣ
    MotorDebugInfo_t debug_info;

    // �ڲ�״̬
    float last_sent_duty;
    uint32_t last_arbitration_time;

    // �豸����
    uint8_t motor_id;           // ���ID������ж�������?
    uint8_t enable;             // ���ʹ��?
} MotorDevice_t;

// ========== �¼����ݽṹ������EventBus�� ==========
typedef struct {
    MotorDir_t dir;
    bool is_active;
} MotorLimitEvent_t;

typedef struct {
    MotorDeviceId_t power_id;
    bool is_on;
} MotorPowerEvent_t;

typedef struct {
    MotorDir_t dir;
    CmdType_t type;
    float speed;
} MotorManualIOEvent_t;

typedef struct {
    MotorDir_t dir;
    CmdType_t type;
    float speed;
} MotorCANEvent_t;

typedef struct {
    uint8_t adc_id;             // ADC�豸ID���ĸ�ADC��⵽�ģ�?
    uint16_t current_ma;        // ��ǰ����(mA)
    uint16_t threshold_ma;      // ������ֵ(mA)
    uint32_t duration_ms;       // ����ʱ��(ms)
} MotorOvercurrentEvent_t;

// ========== ����ٲý���ص������������壬�û�����д�� ==========
/**
 * @brief ����ٲò��ж����ֹͣʱ����
 * @param motor ����豸ָ��?
 */
void Motor_OnArbitrationStop(MotorDevice_t* motor);

/**
 * @brief ����ٲò��ж������תʱ����
 * @param motor ����豸ָ��?
 * @param duty ��ǰռ�ձ�
 */
void Motor_OnArbitrationFwd(MotorDevice_t* motor, float duty);

/**
 * @brief ����ٲò��ж������תʱ����
 * @param motor ����豸ָ��?
 * @param duty ��ǰռ�ձ�
 */
void Motor_OnArbitrationRev(MotorDevice_t* motor, float duty);

// ========== ����豸�ӿڣ�����DeviceManager�淶�� ==========
// ��׼�豸����
DeviceResult_t Motor_Init(void* handle);
DeviceResult_t Motor_Deinit(void* handle);
DeviceResult_t Motor_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Motor_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Motor_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Motor_Update(void* handle);  // ��ʱ��ѯ

// ����ض��ӿ�?
void Motor_SetSpeed(MotorDevice_t* motor, float duty);
void Motor_Start(MotorDevice_t* motor, MotorDir_t dir);
void Motor_Stop(MotorDevice_t* motor);
void Motor_EmergencyStop(MotorDevice_t* motor);

// �������ֹͣ�ӿڣ����ָ�������? allow ָ����� block��
void Motor_ClearAllowFwd(MotorDevice_t* motor);
void Motor_ClearAllowRev(MotorDevice_t* motor);

// ���Խӿ�
const MotorDebugInfo_t* Motor_GetDebugInfo(MotorDevice_t* motor);

// ========== EventBus�ص��������� ==========
void Motor_OnPowerEvent(void* payload);
void Motor_OnHardLimit(void* payload);
void Motor_OnManualIO(void* payload);
void Motor_OnCANEvent(void* payload);
void Motor_OnSpeedFeedback(void* payload);  // �����Ҫ�ٶȷ���?
void Motor_OnOvercurrent(void* payload);
void Motor_OnVoltageAlarm(void* payload);
void Motor_OnCurrentAlarm(void* payload);
void Motor_OnRTurnLimit(void* payload);  // Բ��ת��������λ�¼�
// ��ȡ����ٲ�����������������ִ�еķ���?
MotorDir_t Motor_GetDesiredDirection(MotorDevice_t* motor);

// ========== ��ѹ�澯�ֶ�����ӿ�? ==========
/**
 * @brief �����ѹ�澯�ڵ���ٲ��������õ� block ָ��
 * @param motor ����豸ָ��?
 * @param u8AlarmType �澯���ͣ�VOLTAGE_ALARM_OVERVOLTAGE �� VOLTAGE_ALARM_UNDERVOLTAGE
 * @note ���� VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_MANUAL ʱʹ��
 *       �� App_FaultHandler ���յ� TOPIC_FAULT_CLEAR �¼�ʱ����
 */
void Motor_ClearVoltageBlock(MotorDevice_t* motor, uint8_t u8AlarmType);

// ========== �����澯�ֶ�����ӿ�? ==========
/**
 * @brief ��������澯�ڵ���ٲ��������õ� block ָ��
 * @param motor ����豸ָ��?
 * @note �� App_FaultHandler ���յ� TOPIC_FAULT_CLEAR �¼�ʱ����
 *       ��������ٲ����������������������?
 */
void Motor_ClearOvercurrentBlock(MotorDevice_t* motor);

// ========== Keil Watch ����ȫ�ֱ��� ==========
// �� Watch �������� g_pMotorDevWatch ���ɲ鿴����ٲ����ڲ�״�?
extern MotorDevice_t* volatile g_pMotorDevWatch;

#endif /* DEV_MOTOR_H_ */
