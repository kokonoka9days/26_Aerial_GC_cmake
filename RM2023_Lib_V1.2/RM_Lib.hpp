/**
  ******************************************************************************
  * File Name				: RM_Lib.hpp
  * Description			: RM c++库，包含USART，CAN，麦轮底盘算法，PID，遥控器，
  陀螺仪，snail电调，裁判系统交互，达妙4310，瓴控6010等库函数
  * Version					: v1.4
  * Creation Date		: 2025.4.12
  ******************************************************************************
  */

#ifndef __RM_LIB_H
#define __RM_LIB_H

#include "main.h"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
#include "math.h"
#include "can.h"   // 这行报错请屏蔽
#include "usart.h" // 这行报错请屏蔽
#include "spi.h"   // 这行报错请屏蔽
#include "tim.h"   // 这行报错请屏蔽
#include "my_math.h"

/**************************************** USART **********************************************************/
#define USART_BUF_SIZE 128         // 数组大小，可修改
#define PRINTF_USART_HANDLE huart1 // 串口号，可修改,英雄是串口1
extern uint8_t info_ubuf[USART_BUF_SIZE];
#define INFO(...) HAL_UART_Transmit(&PRINTF_USART_HANDLE,                    \
                                    (uint8_t *)info_ubuf,                    \
                                    sprintf((char *)info_ubuf, __VA_ARGS__), \
                                    0xffff)

static const float m_pi = 3.1415926535897932384626433832795f;

float Rad2Angle(float Rad);
float Angle2Rad(float Angle);
// uint8_t INFO_DMA(const char *fmt, ...);
class Slow /*集成目标，斜坡目标*/
{
private:
    float inc_buf = 0;

public:
    bool init_flag = 1;
    bool once_reset = 0;
    bool ok = 1;
    bool en = 1;                      // 开关斜坡
    float add_inc, cut_inc, stop_err; // 内部+-实际值
    float targe = 0;                  // 突变目标值
    float s_targe = 0;                // 线性跟随后
    float Freq = 1000;
    float real_add = 0, real_cut = 0, real_stoperr = 0; // 外部按照单位化后的值

    void set_real(void) /*如果要将斜坡时间标准化，就调用*/
    {
        float T = 1.0f / Freq; // 执行频率，将速度统一
        add_inc = real_add * T;
        cut_inc = real_cut * T;
        stop_err = real_stoperr * T;
    }
    Slow(float add, float cut, float stop, float freq) : real_add(add), real_cut(cut), real_stoperr(stop), Freq(freq)
    {
        set_real(); // 我的默认就是标准化单位s
    };
    void init_S(float ref) /*pid进保护后，重置*/
    {
        s_targe = ref;
        if (abs(s_targe - targe) <= stop_err)
        {
            ok = 1;
        }
    }
    void init_T(float ref)
    {
        targe = ref;
        if (abs(s_targe - targe) <= stop_err)
        {
            ok = 1;
        }
    }

    void S_T(void) /*开关/复位跟随*/
    {
        s_targe = targe;
        ok = 1;
    }
    float F_slow(void) // 逐步调整 in 指向的值，使其逐渐接近 target 指向的值，add为跟随至差异值
    {

        if (!en || abs(s_targe - targe) <= stop_err) // 不启用斜坡也复位
        {
            S_T();
        }
        else
        {
            if (s_targe < targe)
                s_targe += add_inc;
            else
                s_targe -= cut_inc;
            ok = 0;
        }
        return s_targe;
    }
    int16_t I16_slow(int16_t target) // int整数形跟随
    {
        // static float *inc_buf = 0;         // 缓冲增量，可以按小数加存到缓冲变量中，过1再加
        if (abs(s_targe - target) < stop_err) // 小于误差
        {
            s_targe = target;
            bool ok = 1;
        }
        else
        {
            if (s_targe < target)
            {
                inc_buf += add_inc;
            }
            else
            {
                inc_buf -= cut_inc;
            }

            if (abs(inc_buf) >= 1) // 如果过1加整数部分
            {
                int int_inc;
                int_inc = (int)inc_buf;
                s_targe += int_inc;
                inc_buf -= int_inc;
            }
        }
        return (int16_t)s_targe;
    }
    int I_slow(int target) // int整数形跟随
    {
        // static float inc_buf = 0;          // 缓冲增量，可以按小数加存到缓冲变量中，过1再加
        if (abs(s_targe - target) < stop_err) // 小于误差
        {
            s_targe = target;
            bool ok = 1;
        }
        else
        {
            if (s_targe < target)
            {
                inc_buf += add_inc;
            }
            else
            {
                inc_buf -= cut_inc;
            }

            if (abs(inc_buf) >= 1) // 如果过1加整数部分
            {
                int int_inc;
                int_inc = (int)inc_buf;
                s_targe += int_inc;
                inc_buf -= int_inc;
            }
        }
        return (int)s_targe;
    }
};

#ifdef __CAN_H__
/**************************************** C A N **********************************************************/
class USER_CAN
{
public:
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_buf[8]; // 给RMD，RM，DM，LK电机接收数据使用
    uint8_t tx_buf[8]; // CAN发送数据使用
    uint8_t can_tx_enable = 1;
    uint32_t FIFO;
    uint8_t FreeTxNum = 255; // 空闲发送邮箱个数，最多3个
    uint32_t TxMailbox;
    uint32_t can_send_busy_cnt = 0;          // 空闲邮箱队列满溢出总计次，也可能是因无法发送导致的
    uint32_t can_send_error_cnt = 0;         // 是否有can发送错误总计次，一直发送失败导致空闲邮箱队列满溢出后，发送错误计次可能不会增加
    uint32_t can_user_error_cnt = 0;         // 接收fifo的信息和回调函数不对应/canid位溢出，一般为自身代码问题导致的错误
    uint32_t can_send_RM_error_cnt = 0;      // 大疆电机一拖四发送错误计次
    uint32_t can_send_LK_error_cnt = 0;      // 领控电机广播模式发送错误计次
    uint32_t can_send_Xbit_error_cnt = 0;    // 板件通信错误计次
    uint32_t can_getRxMessage_error_cnt = 0; // 接收错误计次

    HAL_StatusTypeDef can_send_error_state = HAL_TIMEOUT;         // 发送错误状态记录/初始化成功标志
    HAL_StatusTypeDef can_getRxMessage_error_state = HAL_TIMEOUT; // 接收错误状态记录/是否有数据接收标志
    HAL_StatusTypeDef motor_send_error_state = HAL_TIMEOUT;       // 广播模式电机发送状态记录，然后计次

    void Init(uint16_t t, uint16_t x);
    HAL_StatusTypeDef Send8Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint8_t uint8_d1, uint8_t uint8_d2, uint8_t uint8_d3, uint8_t uint8_d4, uint8_t uint8_d5, uint8_t uint8_d6, uint8_t uint8_d7, uint8_t uint8_d8);
    HAL_StatusTypeDef Send16Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint16_t uint16_d1, uint16_t uint16_d2, uint16_t uint16_d3, uint16_t uint16_d4);
    HAL_StatusTypeDef Send32Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint32_t uint32_d1, uint32_t uint32_d2);
    HAL_StatusTypeDef Send64Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint64_t uint64_data);
    HAL_StatusTypeDef Send_RM(uint16_t Id, int16_t M_201, int16_t M_202, int16_t M_203, int16_t M_204); // 大疆电机1拖4模式
    HAL_StatusTypeDef Broadcast_Send_LK(int16_t M_201, int16_t M_202, int16_t M_203, int16_t M_204);    // 领控电机广播模式,1拖4，不可使用Send_RM()

    HAL_StatusTypeDef STD_ID_Send(uint16_t Id, uint8_t *pData);
    HAL_StatusTypeDef EXT_ID_Send(uint32_t Id, uint8_t *pData);
    HAL_StatusTypeDef Receive(uint32_t fifo);

    USER_CAN(CAN_HandleTypeDef *p, uint32_t fifo) : hcan(p), FIFO(fifo) {}

private:
};
/*************************************  RM电机  ************************************************/
class MOTOR_RM
{
public:
    uint16_t ID; // 电机反馈ID
    USER_CAN *can_rev;

    int16_t mang = 0;   // (int16_t) ((Can1_Data[0] << 8) | Can1_Data[1]);
    int16_t sp;         // (int16_t) ((Can1_Data[2] << 8) | Can1_Data[3]);
    int16_t AT_current; // (int16_t) ((Can1_Data[4] << 8) | Can1_Data[5]);
    int8_t temp;        //  Can1_Data[6];
    uint8_t en = 0;
    uint8_t first = 0;
    int16_t out = 0;

    uint8_t online = 0;
    uint8_t loss_time = 0;

    void watchdog_run(void)
    {
        loss_time++;
        if (loss_time > 2)
        {
            online = 0;
        }
    }

    int mang_inf;
    int motor_number;
    int first_mang_inc; // 第一次上电或者被初始化的编码器值数据
    void if_en(void)
    {
        if (!en)
        {
            out = 0;
        }
    }

    // HAL_StatusTypeDef motor_send_state=HAL_TIMEOUT; // 电机发送状态/是否有调用标志
    HAL_StatusTypeDef update(void);
    void update_mang_inf(void);   // 以上电为原点,或自定义的绝对编码值
    void NSQD_8192mang_inf(void); // 优化update_mang_inf，效果一样
    void First(void);
    MOTOR_RM(const uint16_t id, class USER_CAN *CAN_rev) : ID(id), can_rev(CAN_rev) {}

private:
    int Last_mang;

    int nsqd_8192xCnt_mang; // 圈速*编码值
};

/*************************************  瓴控6010电机  ************************************************/
class MOTOR_LK
{
public:
    const uint16_t ID; // 电机反馈ID 321
    USER_CAN *can_rev;

    int16_t order; // 反馈的命令字节 161?
    int8_t temp;   // 温度
    int16_t iq;    // 转矩电流值
    int16_t sp;    // 速度
    uint16_t mang; // 角度
    int mang_inf;
    int motor_number;
    int nsqd_65535xCnt_mang;                          // 圈速*编码值
    uint32_t motor_send_error_cnt = 0;                // 领空单电机模式发送错误计次
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // 单电机控制发送状态/是否有调用单电机控制标志

    /*******************单电机控制*********************/
    HAL_StatusTypeDef LK_Close(uint16_t Id);                     // 电机关闭命令
    HAL_StatusTypeDef LK_Start(uint16_t Id);                     // 电机运行命令
    HAL_StatusTypeDef LK_Stop(uint16_t Id);                      // 电机停止命令
    HAL_StatusTypeDef LK_MIT_ClossControl(uint16_t Id);          // 转矩闭环控制命令 数值范围-2048~ 2048  对应MG电机实际转矩电流范围-33A~33A  电机在收到命令后回复主机
    HAL_StatusTypeDef LK_SP_ClossControl(uint16_t Id);           // 速度闭环控制命令
    HAL_StatusTypeDef LK_More_Mang_ClossControl_1(uint16_t Id);  // 多圈位置闭环控制命令 1
    HAL_StatusTypeDef LK_More_Mang_ClossControl_2(uint16_t Id);  // 多圈位置闭环控制命令 2
    HAL_StatusTypeDef LK_Alone_Mang_ClossControl_1(uint16_t Id); // 单圈位置闭环控制命令 1
    HAL_StatusTypeDef LK_Alone_Mang_ClossControl_2(uint16_t Id); // 单圈位置闭环控制命令 2
    HAL_StatusTypeDef LK_Read_PID(uint16_t Id);                  // 读取电机PID
    HAL_StatusTypeDef LK_Read_Acc(uint16_t Id);                  // 读取电机的加速度
    HAL_StatusTypeDef LK_Read_Encoder(uint16_t Id);              // 读取电机的编码器
    HAL_StatusTypeDef LK_Read_More_Mang(uint16_t Id);            // 读取电机多圈角度
    HAL_StatusTypeDef LK_Read_Alone_Mang(uint16_t Id);           // 读取电机单圈角度
    HAL_StatusTypeDef LK_Read_MotorState_1(uint16_t Id);         // 读取电机状态1，该命令读取当前电机的温度、电压和错误状态标志
    HAL_StatusTypeDef LK_Clear_MotorMismark(uint16_t Id);        // 该命令清除当前电机的错误状态，电机收到后返回
    HAL_StatusTypeDef LK_Read_MotorState_2(uint16_t Id);         // 读取电机状态2，该命令读取当前电机的温度、电压、转速、编码器位置。
    HAL_StatusTypeDef LK_Read_MotorState_3(uint16_t Id);         // 读取电机状态3，该命令读取当前电机的温度和相电流数据。

    HAL_StatusTypeDef LK_Broadcast_update(void);                                       // 接收数据时候用了什么函数就要在这里写
    void update_65535mang_inf_free(void);                                              // 以上电为原点,或自定义的绝对编码值
    void update_65535mang_inf_basic_zeromang(void);                                    // 以电机编码0点的绝对编码值
    MOTOR_LK(const uint16_t id, class USER_CAN *CAN_rev) : ID(id), can_rev(CAN_rev) {} // 不懂为什么要加这个
private:
    uint8_t first = 0; // 接收到反馈数据的状态
    int Last_mang;
};

/*************************************  达妙4310电机  ************************************************/
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -45.0f
// #define V_MAX 45.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -10.0f
// #define T_MAX 10.0f

class MOTOR_DM
{ // 达秒电机 ,在这里定义的东西需要使用this来提取
public:
    uint16_t CAN_ID;    // 控制电机id
    uint16_t MASTER_ID; // 电机反馈ID

    USER_CAN *can_rev;

    int16_t state_id; // 由达秒的串口助手设置
    int16_t ERR;      // 反馈回来的电机错误信息，8：超压 9：欠压 A：过电流 B：mos过温 C：线圈过温 D：通讯丢失 E：过载
    int p_int;
    int v_int;
    int t_int;
    float mang;    // 位置 16位
    float sp;      //   速度  12位
    float Torque;  // 扭矩 12位
    float T_Rotor; // 表示电机内部线圈的平均温度 单位：摄氏度
    float T_MOS;   // 表示驱动上 MOS 的平均温度

    float nsqd_8PI_Cnt_mang; // 圈速*编码值
    float mang_inf;          // 过圈编码值
    uint8_t first = 0;       // 初始标志
    float Last_mang;         // 上次的角度值，判断过圈用
    int16_t motor_number;    // 圈数

    uint8_t en = 0;

    /*默认4310参数*/
    float P_MIN = -12.566f,
          P_MAX = 12.566f,
          V_MIN = -30.0f,
          V_MAX = 30.0f,
          KP_MIN = 0.0f,
          KP_MAX = 500.0f,
          KD_MIN = 0.0f,
          KD_MAX = 5.0f,
          T_MIN = -10.0f,
          T_MAX = 10.0f;

    float mit_pos = 0, mit_vel = 0, mit_kp = 0, mit_kd = 0, mit_torq = 0;

    uint32_t motor_send_error_cnt = 0;                // 达妙单电机发送错误计次
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // 电机发送状态/是否有调用标志
    uint8_t online = 0;
    uint8_t loss_time = 0;

    void watchdog_run(void)
    {
        loss_time++;
        if (loss_time > 2)
        {
            online = 0;
        }
    }
    HAL_StatusTypeDef DM_Start(void);

    HAL_StatusTypeDef DM_End(void);
    HAL_StatusTypeDef DM_Savezero(void);
    HAL_StatusTypeDef DM_MIT(float _pos, float _vel, float _KP, float _KD, float _torq);
    HAL_StatusTypeDef DM_POS(float _pos, float _vel);
    HAL_StatusTypeDef DM_VEL(float _vel);
    HAL_StatusTypeDef DM_update(void); // 得到速度，位置等参数
    void DM_Param_Set(float p_max, float v_max, float t_max);
    void update_xPI_mang_inf_basic_zeromang(float x_pi); // 不改变0点的过圈检测,x_pi为几圈
    void if_en(void)
    {
        if (!en)
        {
            mit_torq = 0;
        }
    }

    MOTOR_DM(uint16_t can_id, uint16_t master_id, class USER_CAN *CAN_rev) : CAN_ID(can_id), MASTER_ID(master_id), can_rev(CAN_rev) {}

private:
    // 这里定义的东西是用给class类里面的函数参数定义
};
/****************************************Cyber_Gear**********************************************/

float uint_to_float(int value, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
// uint32_t Cyber_Gear_EXTID_SET(uint8_t mode, uint8_t Motor_id, uint16_t data);
// uint8_t Motor_Id_Get(uint32_t EXTID); // 针对通讯协议2,从ID里面读取电机ID
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

// 01电机无绝对编码，过圈阈值0.81
class Cyber_Gear // 小米，灵足时代电机
{
public:
    /*可读写单个参数列 (7019-701C 为最新版本固件可)*/
    struct
    {
        float voltage; // 电压
        float speed;   // 速度
        float temp;    // 温度

        float VBUS;    // 母线电压(只读)
        float mechVel; // 负载端转(只读)
        float iqf;     // iq滤波(只读)
        float mechPos; // 负载 计圈机械角度(只读)

        float limit_cur;     // 速度模式电流限制
        float limit_spd;     // 位置模式速度限制
        float loc_ref;       // 位置模式角度指令
        float cur_filt_gain; // 电流的滤波系
        float cur_ki;        // 电流的Ki
        float cur_kp;        // 电流的Kp
        float imit_torque;   // 转矩限制
        float spd_ref;       // 转模式转速指
        float iq_ref;        // 电流Iq指令(可读)
    } index;
    struct
    {
        uint8_t A_phase_sampling_overcurrent; // A相采样电
        uint8_t B_phase_sampling_overcurrent; // B相采样电
        uint8_t C_phase_sampling_overcurrent; // C相采样电
        uint8_t Mcu_Error;                    // 驱动芯片故障
        uint8_t Encoder_not_calibrated;       // 编码器未标定
        uint8_t OverLoad_Voltage;             // 过压故障
        uint8_t UnderLoad_Voltage;            // 欠压故障
        uint8_t Over_temperature_fault;       // 过温故障
        uint16_t Fault;
        uint16_t Overload_fault; // 过载故障
    } Error;
    struct
    {
        uint8_t Over_temperature_warning_80_degrees; // 80度过温警
        uint8_t Over_temperature_warning_75_degrees; // 75度过温警
        uint32_t WarningValue;
    } Temp_Warning;
    USER_CAN *can_rev;
    const uint16_t MOTOR_ID;            // 电机反馈ID
    const uint16_t MY_Master_ID = 0xFE; // 主机ID
    uint8_t RunMode;                    // 运行模式
    float sp;                           // 速度
    float mang;                         // 角度
    float torque;                       // 力矩
    float temp;                         // 温度

    float nsqd_8PI_Cnt_mang; // 圈速*编码值
    float mang_inf;          // 过圈编码值
    uint8_t first = 0;       // 初始标志
    float Last_mang;         // 上次的角度值，判断过圈用
    int16_t motor_number;    // 圈速

    uint32_t motor_send_error_cnt = 0;                // 达妙单电机发送错误计次
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // 电机发送状态/是否有调用标志
    uint8_t Motor_Id_Get(uint32_t EXTID);             // 针对通讯协议2,从ID里面读取电机ID
    HAL_StatusTypeDef torque_Send(uint8_t motor_id, float torque);
    HAL_StatusTypeDef Stop(uint8_t motor_id);
    HAL_StatusTypeDef Enable(uint8_t motor_id);
    uint32_t EXTID_SET(uint8_t mode, uint8_t Motor_id, uint16_t data);

    void update_4PI_mang_inf_basic_zeromang(void); // 不改变0点的过圈检测
    HAL_StatusTypeDef update(void);
    Cyber_Gear(const uint16_t id, class USER_CAN *CAN_rev) : MOTOR_ID(id), can_rev(CAN_rev) {}

private:
    float P_MIN = -12.5f,
          P_MAX = 12.5f,
          V_MIN = -30.0f,
          V_MAX = 30.0f,
          KP_MIN = 0.0f,
          KP_MAX = 500.0f,
          KD_MIN = 0.0f,
          KD_MAX = 5.0f,
          T_MIN = -12.0f,
          T_MAX = 12.0f; // 小米电机参数10
};

/*************************************  RMD结构体  ************************************************/

// 这里定义的东西是用给class类里面的函数参数定义
typedef struct
{

    uint16_t anglePidKp;
    uint16_t anglePidKi;
    uint16_t speedPidKp;
    uint16_t speedPidKi;
    uint16_t iqPidKp;
    uint16_t iqPidKi;
    int32_t Accel;
    uint16_t encoder;       // 编码器位置     0~65535
    uint16_t encoderRaw;    // 编码器原始位置 0~65535
    uint16_t encoderOffset; // 编码器零偏     0~65535
    int64_t motorAngle;
    uint16_t circleAngle;
    int8_t temperature;
    uint16_t voltage;
    uint8_t eerorState;
    int16_t iq;
    int16_t speed;
    uint16_t now_encoder; // 编码器位置值
    int16_t iA;
    int16_t iB;
    int16_t iC;
    int16_t iqControl;
    int32_t speedControl;
    int32_t angleControl;

} RMD_typedef;

/*************************************  RMD电机  ************************************************/

class MOTOR_RMD
{
public:
    const uint16_t ID; // 电机反馈ID
    USER_CAN *can_rev;
    RMD_typedef RMD_X;
    uint32_t motor_send_error_cnt = 0;                // 达妙单电机发送错误计次
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // 电机发送状态/是否有调用标志

    HAL_StatusTypeDef RMD_Read_and_Write_Things(uint16_t Id, uint16_t Order_Id);
    HAL_StatusTypeDef RMD_Write_PID(uint16_t Id, uint16_t Order_Id, uint16_t anglePidKp, uint16_t anglePidKi,
                                    uint16_t speedPidKp, uint16_t speedPidKi, uint16_t iqPidKp, uint16_t iqPidKi);
    HAL_StatusTypeDef RMD_Write_ACCLE_to_RAM(uint16_t Id, int32_t Accel);
    HAL_StatusTypeDef RMD_Write_EncoderOffset_to_ROM(uint16_t Id, uint16_t EncoderOffset); // 写入编码器值
    HAL_StatusTypeDef RMD_Iqcontrol_Motor(uint16_t Id, int16_t iqControl);
    // HAL_StatusTypeDef RMD_Speedcontrol_Motor(uint16_t Id, int32_t angleControl);//好像没有这个函数
    HAL_StatusTypeDef RMD_Speedcontrol_Motor(uint16_t Id, uint16_t Order_Id, uint16_t maxSpeed, int32_t angleControl);
    HAL_StatusTypeDef RMD_Anglecontrol_Motor(uint16_t Id, uint16_t Order_Id, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl);
    HAL_StatusTypeDef RMD_update(void);

    MOTOR_RMD(const uint16_t id, class USER_CAN *CAN_rev) : ID(id), can_rev(CAN_rev) {}

private:
};
#endif /*__CAN_H__*/

typedef struct
{
    float qy;
    float hy;
    float qz;
    float hz;
} ML_typedef;

class MOTOR_DiPan
{
public:
    ML_typedef ML;

    MOTOR_DiPan(void);
    void ML_Data_Deal(float lx, float ly, float lp, int MAX_rate);
};

#pragma region /*DL*/
/*************************************************** Duo Lun  ******************************************************/

// #define M_PI 3.1415926535897932384626433832795f
#define COS_45 0.70710678118654752440084436210485f
#define RAD2MANG 1303.7972938088065906186957895476f
#define SPEED_MAX 3000

struct wheel_dir_and_weight
{
    bool dir;
    float speed;
    int yaogan_speed;
};
struct lun_xy
{
    float x, y;
};

enum
{
    QZ = 0,
    HZ,
    HY,
    QY
};

class RUDDER_DiPan
{
public:
    int16_t g_angle_6020[4] = {0, 0, 0, 0};
    wheel_dir_and_weight g_wheel_3508[4] = {0, 0, 0, 0};
    lun_xy QZ_xy, HZ_xy, HY_xy, QY_xy;
    uint16_t ZERO[4] = {1685, 7564, 6822, 5802}; // qz,hz,hy,qy  每个舵轮都不一样，自己修改

    void Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2);
    void Xiaotuoluo_jie_Suan(uint16_t Mang_yaw, int16_t CH0, int16_t CH1, int16_t CH2);

private:
    float Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1);
    float Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2);
    float absf(float d0);
    float Get_Max_float(float d0, float d1);
    int16_t Get_Max_int16(int16_t d0, int16_t d1);
};
#pragma endregion

/**************************************** P I D **********************************************************/

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
// #define NL -3
// #define NM -2
// #define NS -1
// #define ZE 0
// #define PS 1
// #define PM 2
// #define PL 3

/*定义一个标准坐标系，规范好反馈的正方向，响应（输出执行）的正方向
定义好后，如果上述两个正方向相同，那么闭环必定是收敛的

一般流程、
反馈数据
坐标系
pid计算
坐标系后的响应数据
电机执行/下一级pid（转换过的响应数据）
*/
class PID_class
{
public:
    uint8_t enable_flag = 0; //
    float Goal = 0;          // 标准坐标系（反馈与响应有正负极性）
    float Ref = 0;           // 标准坐标系（反馈与响应有正负极性）
    float error;
    float KP, KI, KD;
    float LIMIT_P = 0, LIMIT_I = 0, LIMIT_D = 0, LIMIT_PID = 0;
    float Deadzoom = 0, Separate = 0; // 直接赋值
    float OUT_P = 0;
    /*标准坐标系（反馈与响应有正负极性）*/
    float OUT_I = 0;
    float OUT_D = 0;         // 1
    float OUT_PID = 0;       // 1
    float delta_OUT_PID = 0; // 1
    float D_LP_value = 1;
    float cutoff_freq = 0, sample_freq = 0; // 直接赋值
    float LP_out_last = 0;                  // 上一次滤波值
    float g_speed_3508[4] = {0, 0, 0, 0};
    int16_t g_angle_6020[4] = {0, 0, 0, 0};

    uint8_t ifReset(void) // 判断顺便带上复位
    {
        if (!enable_flag)
        {
            Reset();
        }
        return enable_flag;
    }
    void Reset(void)
    {
        OUT_I = 0;
        OUT_PID = 0;
        LAST_Error = 0;
        Goal = Ref;
        // 目标=反馈
    }
    void set_goal_func(void (*ptr)(void))
    {
        goal_func = ptr;
    }
    void pGoal(void) /*注意，函数指针是你决定的，enable_flag判断不可去掉，一定判断可靠再跑
                        虽然过程中可能会多次判断enable_flag*/
    {
        if (goal_func && enable_flag)
        {
            goal_func();
        }
    }
    void LP_param_set(void)
    {
        if (cutoff_freq != 0 && sample_freq != 0)
        {
            D_LP_value = get_cutoff_freq(cutoff_freq, sample_freq);
        }
        else
        {
            D_LP_value = 1;
        }
    }
    bool Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now);
    void PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight);
    void PID_new_update(float goal, float now);
    void PID_update(float goal, float now);
    void PID_update_LP(float goal, float now);
    void PID_update_void(void);

    float low_pass_filter(float value, float k_value);
    void PID_Inc_update(float goal, float now);
    float get_cutoff_freq(float cutoff_freq, float sample_freq)
    {
        float k = 1.0f - expf(-2.0f * 3.14159265358979323846f * cutoff_freq / sample_freq);
        return k;
    }
    PID_class(float kp, float ki, float kd, float limit_p, float limit_i, float limit_d, float limit_pid, float lp_kd = 0.01f, float i_eMAX = 0, float dz = 0) : KP(kp), KI(ki), KD(kd), LIMIT_P(limit_p), LIMIT_I(limit_i), LIMIT_D(limit_d), LIMIT_PID(limit_pid), D_LP_value(lp_kd), Separate(i_eMAX), Deadzoom(dz)
    {
    }

private:
    float LAST_Error = 0;     // 位置式
    float inc_last_error = 0; // 增量式
    float inc_lastlast_error; // 增量式上上次误差
    void (*goal_func)(void) = nullptr;
};

class PID_Fuzzy_class
{
public:
    float LastError;        // 前次误差
    float error;            // 当前误差
    float SumError;         // 积分误差
    float IMax;             // 积分限制
    float POut, IOut, DOut; // 比例输出
    float DOut_last;        // 上一次微分输出
    float OutMax;           // 限幅
    float Out;              // 总输出
    float Out_last;         // 上一次输出

    float I_U; // 变速积分上限
    float I_L; // 变速积分下限

    float Kp0, Ki0, Kd0; // PID初值
    float dKp, dKi, dKd; // PID变化量

    float stair, Kp_stair, Ki_stair, Kd_stair; // 动态调整梯度   //0.25f

    void FuzzyPID_update(float goal, float now);
    PID_Fuzzy_class(float kp, float ki, float kd, float limit_i, float limit_pid, float IL, float Stair, float KP_stair, float KI_stair, float KD_stair) : Kp0(kp), Ki0(ki), Kd0(kd), IMax(limit_i), OutMax(limit_pid), I_L(IL), stair(Stair), Kp_stair(KP_stair), Ki_stair(KI_stair), Kd_stair(KD_stair) {}

private:
    float NL = -3, NM = -2, NS = -1, ZE = 0, PS = 1, PM = 2, PL = 3; // 负大 负中 负小 零 正小 正中 正大
    const float fuzzyRuleKp[7][7] = {
        /******Kp隶属度规则表******/
        /*

        kp |   de/dt （e的速率）隶属度
        ---|-------------------------------
        e  | PL,PL,PM,	PM,	PS,	ZE,	ZE,
        隶 | PL,PL,	PM,	PS,	PS,	ZE,	NS,
        属 | PM,PM,	PM,	PS,	ZE,	NS,	NS,
        度 | PM,PM,	PS,	ZE,	NS,	NM,	NM,
           | PS,PS,	ZE,	NS,	NS,	NM,	NM,
           | PS,ZE,	NS,	NM,	NM,	NM,	NL,
           | ZE,ZE,	NM,	NM,	NM,	NL,	NL

        */
        PL, PL, PM, PM, PS, ZE, ZE, // NL(负大)NM(负中)NS(负小)ZE(零)PS(正小)PM(正中)PL(正大)
        PL, PL, PM, PS, PS, ZE, NS, // 中间是0，靠近0变小
        PM, PM, PM, PS, ZE, NS, NS,
        PM, PM, PS, ZE, NS, NM, NM,
        PS, PS, ZE, NS, NS, NM, NM,
        PS, ZE, NS, NM, NM, NM, NL,
        ZE, ZE, NM, NM, NM, NL, NL};

    const float fuzzyRuleKi[7][7] = {
        NL, NL, NM, NM, NS, ZE, ZE, // 中间是0，靠近0变大
        NL, NL, NM, NS, NS, ZE, ZE,
        NL, NM, NS, NS, ZE, PS, PS,
        NM, NM, NS, ZE, PS, PM, PM,
        NS, NS, ZE, PS, PS, PM, PL,
        ZE, ZE, PS, PS, PM, PL, PL,
        ZE, ZE, PS, PM, PM, PL, PL};

    const float fuzzyRuleKd[7][7] = {
        PS, NS, NL, NL, NL, NM, PS,
        PS, NS, NL, NM, NM, NS, ZE,
        ZE, NS, NM, NM, NS, NS, ZE,
        ZE, NS, NS, NS, NS, NS, ZE,
        ZE, ZE, ZE, ZE, ZE, ZE, ZE,
        PL, NS, PS, PS, PS, PS, PL,
        PL, PM, PM, PM, PS, PS, PL};

    void fuzzy(float goal, float now);
};

class CRC16_class
{
private:
    uint16_t crc16_init = 0xffff;
    const uint16_t crc16_tab[256] =
        {
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
            0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
            0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
            0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
            0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
            0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
            0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
            0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
            0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
            0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
            0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
            0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
            0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
            0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
            0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
            0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
            0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
            0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
            0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
            0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
            0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
            0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
            0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
            0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
            0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
            0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
            0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
            0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
            0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
            0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

public:
    uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16) // 得到crc校验的值
    {
        uint8_t data;
        if (p_msg == NULL)
            return 0xffff;
        while (len--)
        {
            data = *p_msg++;
            (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
        }
        return crc16;
    }

    bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len) // 校验
    {
        uint16_t w_expected = 0;

        if ((p_msg == NULL) || (len <= 2))
        {
            return false;
        }
        w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

        return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
    }

    CRC16_class();
};

#pragma region /*滑膜控制 SMC*/
class SMC
{
public:
    float C;          // 滑模面斜率,类似K
    float K;          // 趋近率增益,类似D
    float Target = 0; // 初始目标值
    float Target_vel = 0;
        float Target_ACC = 0;

    float error_eps; // 误差下限
    float u_max;     // 输出最大值
    float J;         // 估计惯量
    float epsilon;   // 切换增益,边界层厚度
    float u;         // 输出电流值
    float error_integral;
    float C2 = 0.0001f; // 滑模面斜率,类似K
    float integral_max = 1000.0f;

    // 初始化列表
    SMC(float C, float K, float c2, float error_eps, float u_max, float J, float epsilon) : C(C), K(K), C2(c2), error_eps(error_eps), u_max(u_max), J(J), epsilon(epsilon) {};

    // 更新函数
    void SMC_Tick(float Target_angle, float angle, float angle_vel);
    void SMC_Tick2(float Target_angle, float angle, float angle_vel);
    void SMC_Tick3(float Target_angle, float angle, float angle_vel);
    void SMC_Tick_ZM(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel);
    void SMC_Tick_I(float Target_angle, float angle, float angle_vel);
    void SMC_Tick_ZM_I(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel);

    void Reset(float ref)
    {
        Target = ref;
        error_integral = 0;
        Last_Target_angle = ref;
        Last_d_tge = 0;
        d_tge = 0;
        dd_tge = 0;
    }

private:
    float s;                     // 滑模面
    float error;                 // 当前误差
    float d_error;               // 当前误差一阶导
    float d_tge;                 // 目标值一阶导
    float dd_tge;                // 目标值二阶导
    float Last_d_tge = 0;        // 上一次目标值一阶导
    float Last_Target_angle = 0; // 上一次的目标值
    const float dt = 0.001f;

    // 饱和函数
    float Sat(float y)
    {
        if (fabs(y) <= 1)
            return y;
        else
            return (y > 0) ? 1.0f : -1.0f;
    }

    float Sat_v2(float s)
    {
        if (fabs(s) < epsilon)
        {
            return s / epsilon;
            // return s;
        }
        else
            return (s > 0) ? 1.0f : -1.0f;
    }

    // 符号函数,若有抖动可以换个陡峭的饱和函数
    int8_t Signal(float y)
    {
        if (y > 0)
            return 1;
        else if (y == 0)
            return 0;
        else
            return -1;
    }
};
extern SMC YawSMC;
#pragma endregion
#ifdef __USART_H__

#pragma region /*DBUS*/

/****************************************** D B U S **********************************************************/
#define YK_SW_UP ((uint16_t)1)
#define YK_SW_MID ((uint16_t)3)
#define YK_SW_DOWN ((uint16_t)2)

#define KEY_PRESSED_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_SHIFT ((uint16_t)0x01 << 4)
#define KEY_PRESSED_CTRL ((uint16_t)0x01 << 5)
#define KEY_PRESSED_Q ((uint16_t)0x01 << 6)
#define KEY_PRESSED_E ((uint16_t)0x01 << 7)
#define KEY_PRESSED_R ((uint16_t)0x01 << 8)
#define KEY_PRESSED_F ((uint16_t)0x01 << 9)
#define KEY_PRESSED_G ((uint16_t)0x01 << 10)
#define KEY_PRESSED_Z ((uint16_t)0x01 << 11)
#define KEY_PRESSED_X ((uint16_t)0x01 << 12)
#define KEY_PRESSED_C ((uint16_t)0x01 << 13)
#define KEY_PRESSED_V ((uint16_t)0x01 << 14)
#define KEY_PRESSED_B ((uint16_t)0x01 << 15)

typedef struct
{
    uint8_t s1; // 左开关
    uint8_t s2;
    int16_t ch0; // 横滚
    int16_t ch1; // 俯仰
    int16_t ch2; // 偏航
    int16_t ch3; // 油门 以上范围都为-660 600 11bit
    int16_t v;   // 波轮
} yaogan_typedef;
typedef struct
{
    uint8_t press_l;
    uint8_t press_r;
    int16_t x; // 左右
    int16_t y; // 前后
    int16_t z;

} shubiao_typedef;

class DBUS
{
public:
    bool dog = false;
    UART_HandleTypeDef *huart;
    uint8_t dbus_rx_buffer[25];

    yaogan_typedef yaogan;
    shubiao_typedef shubiao;
    uint16_t jianpan;
    uint32_t rx_error_cnt;

    void feed_watchdog(void)
    {
        time_100ms = 0;
    }
    // HAL_StatusTypeDef receive_run(void)
    // {
    //     return HAL_UART_Receive_IT(this->huart, this->dbus_rx_buffer, 18);
    // }
    // HAL_StatusTypeDef receive_refresh(void)
    // {
    //     return HAL_UART_AbortReceive_IT(this->huart);
    // }

    void Init(void);
    void DBUS_RxCplt_IRQHandler(void);
    void jianpan_deal(void);
    void set_zero(void);
    void data_deal(void);
    void watchdog_run(void);
    void can_receive_data_deal(uint8_t num, uint8_t *buf);

    uint8_t Pressed_Check(uint16_t key_value); // 按下状态 返回1

    DBUS(UART_HandleTypeDef *p) : huart(p) {}

private:
    uint8_t first;
    uint16_t time_100ms;
    uint8_t index; // 无用
    // uint16_t delaycount[16]; // 无用
    uint16_t last_jianpan; // 无用

    HAL_StatusTypeDef check_and_deal(void);
};
#pragma endregion

#pragma region /*RC*/
/****************************************** RC **********************************************************/
#define YK_MODE_SW_C ((uint16_t)0)
#define YK_MODE_SW_N ((uint16_t)1)
#define YK_MODE_SW_S ((uint16_t)2)

typedef struct
{
    uint8_t sof_1;   // 0xa9
    uint8_t sof_2;   // 0x53
    int16_t ch_0;    //+-660 横滚
    int16_t ch_1;    //+-660 俯仰
    int16_t ch_2;    //+-660 油门
    int16_t ch_3;    //+-660 偏航
    uint8_t mode_sw; // cns:012
    uint8_t pause;   // 暂停按键
    uint8_t fn_1;    // 左自定义按键
    uint8_t fn_2;    // 右自定义按键
    int16_t wheel;   // 拨轮
    uint8_t trigger; // 扳机键

    int16_t mouse_x;      //+-32768 右正
    int16_t mouse_y;      //+-32768 前正
    int16_t mouse_z;      //+-32768 鼠标滚轮滚动速度
    uint8_t mouse_left;   // 01
    uint8_t mouse_right;  // 01
    uint8_t mouse_middle; // 01鼠标中键
    uint16_t key;         // 键盘
    uint16_t crc16;
} remote_data_t;

static const uint16_t crc16_tab[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

typedef union
{
    float f;
    unsigned char c[4];
    int i;
} dat;
// 自定义控制器发送端，将数据赋值到此处,定时调用self_controler_data_tx即可
// 机器人接收端将接收到的数据会存入此处调用
typedef __PACKED_STRUCT
{
    // acc[3],gyro[3],24+3*2
    dat j0;
    dat j1;
    dat j2;
    dat j3;
    dat j4;
    dat j5;
    dat j6;
    uint8_t y, z;
    // bool KEY_1;
    // bool KEY_2;
}
custom_controller_data_t;
/****************************************tu  chuan***************************************************/
// 0xA5,0xC0,seq(0-255向上计数),crc8
// cmdid_L 4,cmdid_H 3
// 12data
#define FRAME_HEADER_LENGTH 5                                                                     // frame帧头                                                                // 帧头数据长度
#define CMD_ID_LENGTH 2                                                                           // 命令id                                                                // 命令码ID数据长度
#define DATA_LENGTH 30                                                                            // 数据位                                                               // 数据段长度
#define FRAME_TAIL_LENGTH 2                                                                       // 帧尾crc16整包校验                                                                // 帧尾数据长度
#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度
#define CONTROLLER_CMD_ID 0x0302                                                                  // 自定义控制器                                                           // 自定义控制器命令码
#define KEY_MOUSE_CMD_ID 0X304                                                                    // 键鼠发送
#define PASS_BACK_CMD_ID 0X309                                                                    // 机器人发自定义控制器
#define CUSTOM_ANALOG_KEY_MOUSE_CMD_ID 0X306                                                      // 自定义控制器模拟键鼠

typedef __PACKED_STRUCT
{
    __PACKED_STRUCT
    {
        uint8_t sof;          // 起始字节，固定值为0xA5
        uint16_t data_length; // 数据帧中data的长度
        uint8_t seq;          // 包序号
        uint8_t crc8;         // 帧头CRC8校验
    }
    frame_header;                 // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[30];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
}
Controller_t; // 图传链路帧格式结构体,0x302,0x304,0x309

typedef __PACKED_STRUCT
{
    dat j1_angle_iq;
    dat j2_angle_iq;
    dat j3_angle_iq;
    dat end_pitch_angle;
    dat end_roll_angle;
    dat end_yaw_angle;
}
force_feedback_t;

// 自定义控制器发机器人302 30 30hz_max
// 机器人发自定义控制器309 30 10hz
// 图传链路键鼠30hz
class RC
{
public:
    UART_HandleTypeDef *DT16_huart;
    UART_HandleTypeDef *TC_huart;

    /*抽象遥控*/
    uint8_t online = 0;
    yaogan_typedef yaogan = {0}; // 协议与dt7为准
    shubiao_typedef shubiao = {0};
    uint16_t jianpan = 0;

    /**DBUS **/
    uint8_t DR16_rx_buffer[40] = {0};
    uint8_t DR16_2rx_buffer[2][20] = {0};
    uint16_t dr16_fifo_index = 0;
    yaogan_typedef DT16_yaogan = {0};
    shubiao_typedef DT16_shubiao = {0};
    uint16_t DT16_jianpan = 0;
    uint8_t dt16_signal_flag = 0; /*DT7信号位*/
    /*波轮*/
    int16_t last_v = 0;
    int16_t first_v_inc = 0;
    int16_t nsqd_2048xCnt_mang = 0;
    int8_t v_number = 0;
    int16_t v_inf_buf = 0; /*中间变量*/
    int16_t v_inf = 0;     /*DT7遥控拨杆最终处理好的数据*/

    /*TC*/
    custom_controller_data_t Custom_Controller_data = {0}; // 自定义控制器数据
    remote_data_t VT13_Data = {0};                         // VT13遥控器数据
    uint16_t crc16_init = 0xffff;
    uint16_t DT16_time_100ms = 0;
    uint16_t VT13_time_100ms = 0;
    uint8_t VT13_rx_buffer[70] = {0};
    uint8_t VT13_2rx_buffer[2][40] = {0}; // 手册最大150+5字节
    uint8_t vt13_fifo_index = 0;
    uint8_t vt13yk_signal_flag = 0; /*图传链路信号位*/
    bool self_ctrl_enable_flag = 0;

    /*调试日志，不建议删*/
    uint32_t vt_yk_cnt = 0, selfctrl_cnt = 0, self_err_cnt = 0;
    uint32_t rx_cnt = 0;
    uint32_t err_cnt = 0;

    void DT16_Init(void);
    void DT16_2Init(void);

    void VT13_Init(void);
    void VT13_2Init(void);

    void DT16_RxCplt_IRQHandler(void);
    void DT16_2RxCplt_IRQHandler(DMA_HandleTypeDef *hdma);

    uint8_t VT13_RxCplt_IRQHandler(void);
    uint8_t VT13_2RxCplt_IRQHandler(DMA_HandleTypeDef *hdma);

    void VT13_YK_deal(void);
    void VT13_2YK_deal(uint8_t *rxbuf);

    void VT13_self_ctrl_deal(void);
    void VT13_2self_ctrl_deal(uint8_t *rxbuf);

    void DT16_watchdog_run(void);
    void DT16_2watchdog_run(void);

    void VT13_watchdog_run(void);
    void VT13_2watchdog_run(void);

    float lp_x_k = 1;
    float x_cutoff = 8;
    float lp_y_k = 1;
    float y_cutoff = 8;
    float lp_shubiao_x = 0;
    float lp_shubiao_y = 0;
    float get_cutoff_freq(float cutoff_freq, float sample_freq)
    {
        float k = 1.0f - expf(-2.0f * 3.14159265358979323846f * cutoff_freq / sample_freq);
        return k;
    }
    void low_pass_filter(float value, float *out_last, float k)
    {
        // static float out_last = 0; // 上一次滤波值
        float out;
        /***************** 如果第一次进入，则给 out_last 赋值 ******************/
        // static char fisrt_flag = 1;
        // if (fisrt_flag == 1)
        // {
        //     fisrt_flag = 0;
        //     out_last = value;
        // }

        /*************************** 一阶滤波 *********************************/
        out = *out_last + k * (value - *out_last);
        *out_last = out;
    }

    void DT16_v_deal(void);

    void can_receive_data_deal(uint8_t *buf);  // 没用
    uint8_t Pressed_Check(uint16_t key_value); // 按下状态 返回1

    // void dp_deal(void)
    // {
    //     uint8_t buffer[8] = {0};
    //     //(int16_t)((this->can_rev->rx_buf[0] << 8) | this->can_rev->rx_buf[1])
    //     // ch1234
    //     yaogan.ch0 = (int16_t)((buffer[0] << 8) | buffer[1]);
    //     yaogan.ch1 = (int16_t)((buffer[2] << 8) | buffer[3]);
    //     yaogan.ch2 = (int16_t)((buffer[4] << 8) | buffer[5]);
    //     yaogan.ch3 = (int16_t)((buffer[6] << 8) | buffer[7]);
    //     // shubiao x,y,jianpan,s1,s2
    //     shubiao.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    //     shubiao.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    //     jianpan = (uint16_t)((buffer[4] << 8) | buffer[5]);
    //     shubiao.press_r = buffer[6] & 0x01;
    //     shubiao.press_l = buffer[6] & 0x02;
    //     yaogan.s2 = buffer[6] & 0x0C;
    //     yaogan.s1 = buffer[6] & 0x30;
    //     // buffer[6] = (uint8_t)(((yaogan.s1 << 4) | (yaogan.s2 << 2) | (shubiao.press_l << 1) | (shubiao.press_r)) & 0x3f);
    // }

    // extKalman_t KF_Mouse_Y_Speed, KF_Mouse_X_Speed;   // 没用
    // float SF(float t, float *slopeFilter, float res); // 没用
    // float Mouse_X_Speed(float Xmax);                                            // 没用
    // float Mouse_Y_Speed(float Ymax);                                            // 没用
    bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);                  // 没用
    uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16); // 没用

    RC(UART_HandleTypeDef *f, UART_HandleTypeDef *s) : DT16_huart(f), TC_huart(s) {}

private:
    uint8_t first = 0;
    uint8_t index;
    // uint16_t delaycount[16];
    uint16_t last_jianpan;

    HAL_StatusTypeDef DT16_check_and_deal(void);
    HAL_StatusTypeDef DT16_2check_and_deal(uint8_t *rxbuf);

    HAL_StatusTypeDef VT13_check_and_deal(void);
    HAL_StatusTypeDef VT13_2check_and_deal(uint8_t *rxbuf);

    void DT16_data_deal(void);
    void DT16_2data_deal(uint8_t *rxbuf);

    void VT13_data_deal(void);
    void VT13_2data_deal(uint8_t *rxbuf);

    void DT16_set_zero(void);
    void fill_data(void); // 无用

    void VT13_YK_set_zero(void);

    void YK_ctrl(void)
    {
        if (!dt16_signal_flag && !vt13yk_signal_flag)
        {
            YK_set_zero();
        }
        else if (dt16_signal_flag && !vt13yk_signal_flag)
        {
            dt16_ctrl();
        }
        else if (!dt16_signal_flag && vt13yk_signal_flag)
        {
            vt13_ctrl();
        }
        else if (dt16_signal_flag && vt13yk_signal_flag)
        {
            if (this->VT13_Data.mode_sw == 0)
            {
                dt16_ctrl();
            }
            else if (this->VT13_Data.mode_sw == 1)
            {
                vt13_ctrl();
            }
            else if (this->VT13_Data.mode_sw == 2)
            {
                vt13_ctrl();
            }
        }
    }
    void dt16_ctrl(void)
    {
        yaogan.ch0 = DT16_yaogan.ch0;
        yaogan.ch1 = DT16_yaogan.ch1;
        yaogan.ch2 = DT16_yaogan.ch2;
        yaogan.ch3 = DT16_yaogan.ch3;
        yaogan.s1 = DT16_yaogan.s1;
        yaogan.s2 = DT16_yaogan.s2;
        shubiao.x = DT16_shubiao.x;
        shubiao.y = DT16_shubiao.y;
        shubiao.z = DT16_shubiao.z;
        shubiao.press_l = DT16_shubiao.press_l;
        shubiao.press_r = DT16_shubiao.press_r;
        jianpan = DT16_jianpan;
        yaogan.v = DT16_yaogan.v;
        online = 1;
        DT16_v_deal();
        // update_v_inf();
    }
    void vt13_ctrl(void)
    {
        yaogan.ch0 = VT13_Data.ch_0;
        yaogan.ch1 = VT13_Data.ch_1;
        yaogan.ch2 = VT13_Data.ch_3;
        yaogan.ch3 = VT13_Data.ch_2;
        jianpan = VT13_Data.key;
        if (VT13_Data.mode_sw == 1)
        {
            yaogan.s1 = YK_SW_DOWN;
            yaogan.s2 = YK_SW_DOWN; // this->VT13_Data.mouse_x
        }
        else
        {
            yaogan.s1 = YK_SW_UP;
            yaogan.s2 = YK_SW_UP; // this->VT13_Data.mouse_x
        }
        shubiao.x = VT13_Data.mouse_x;
        shubiao.y = VT13_Data.mouse_y;
        shubiao.z = VT13_Data.mouse_z;
        shubiao.press_l = VT13_Data.mouse_left;
        shubiao.press_r = VT13_Data.mouse_right;
        yaogan.v = VT13_Data.wheel;
        online = 1;
    }
    void YK_set_zero(void)
    {
        online = 0;
        yaogan.ch0 = 0;
        yaogan.ch1 = 0;
        yaogan.ch2 = 0;
        yaogan.ch3 = 0;
        yaogan.v = 0;
        yaogan.s1 = YK_SW_UP;
        yaogan.s2 = YK_SW_UP;
        shubiao.x = 0;
        shubiao.y = 0;
        shubiao.z = 0;
        shubiao.press_l = 0;
        shubiao.press_r = 0;
        jianpan = 0;
    }

    void DT16_feed_watchdog(void)
    {
        DT16_time_100ms = 0;
    }
    void VT13_feed_watchdog(void)
    {
        VT13_time_100ms = 0;
    }
};

#pragma endregion
#endif // __USART_H__

#pragma region /*ADXRS290*/
/**************************************** ADXRS290 **********************************************************/
#ifdef __SPI_H__

#define ADXRS290_ADI_ID 0x00
#define ADXRS290_MEMS_ID 0x01
#define ADXRS290_DEV_ID 0x02
#define ADXRS290_REV_ID 0x03
#define ADXRS290_SN0 0x04
#define ADXRS290_SN1 0x05
#define ADXRS290_SN2 0x06
#define ADXRS290_SN3 0x07
#define ADXRS290_DATAX0 0x08
#define ADXRS290_DATAX1 0x09
#define ADXRS290_DATAY0 0x0A
#define ADXRS290_DATAY1 0x0B
#define ADXRS290_TEMP0 0x0C
#define ADXRS290_TEMP1 0x0D
#define ADXRS290_POWER_CTL 0x10
#define ADXRS290_Filter 0x11
#define ADXRS290_DATA_READY 0x12

typedef struct gyro
{
    float v;
    float v_nonoise;
    float theta_euler;
    float bias;
    uint32_t dev_count;
} ADXRS290_TYPEDEF;

typedef enum
{
    ADXRS290_OK = 0x00U,
    ADXRS290_SET_ERROR = 0x01U,
    ADXRS290_ID_ERROR = 0x02U,
    ADXRS290_ERROR = 0x03U,
} ADXRS290_StatusTypeDef;

class ADXRS290
{
public:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;

    ADXRS290_TYPEDEF sensor_data_X;
    ADXRS290_TYPEDEF sensor_data_Y;

    ADXRS290_StatusTypeDef Init(uint8_t hpf_corner, uint8_t odr_lpf);
    void adxrs290_update(void);
    ADXRS290_StatusTypeDef adxrs290_writeByte(uint8_t subAddress, uint8_t data);
    uint8_t adxrs290_readByte(uint8_t subAddress);
    void adxrs290_readBytes(uint8_t subAddress, uint8_t count, uint8_t *spi_rev_buf);

    ADXRS290(SPI_HandleTypeDef *q, GPIO_TypeDef *w, uint16_t e, uint16_t t, float y, const char *u) : hspi(q), GPIOx(w), GPIO_Pin(e), SELF_TEST_NUM_290(t), DEAD_ZONE_290(y), string_check_290(u) {}

private:
    uint16_t SELF_TEST_NUM_290;
    float DEAD_ZONE_290;
    const char *string_check_290;
    void ADXRS290_SPI_ON()
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }
    void ADXRS290_SPI_OFF()
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }
};
#endif

#pragma endregion

#pragma region /*ADXRS453*/
/*****************************************ADXRS453*******************************************/
#ifdef __SPI_H__

typedef struct
{
    float v;
    float v_nonoise;
    float theta_euler;
    float bias;
    float offset_v;
    float offset_max;
    float offset_min;
    uint32_t dev_count;
    uint8_t calibration;
} ADXRS453_TYPEDEF;
typedef enum
{
    ADXRS453_OK = 0x00U,
    ADXRS453_RW_ERROR = 0x01U,
    ADXRS453_P0_ERROR = 0x02U, // P0是奇偶校验位，为比特建立奇偶校验。[31:16]设备响应的。
    ADXRS453_P1_ERROR = 0x03U, // P1是奇偶校验位，它为整个数据建立奇偶校验32位设备响应。
    ADXRS453_SPI_ERROR = 0x04U,
    ADXRS453_RE_ERROR = 0x05U,
    ADXRS453_DU_ERROR = 0x06U,
    ADXRS453_PLL_ERROR = 0x07U,
    ADXRS453_Q_ERROR = 0x08U,
    ADXRS453_NVM_ERROR = 0x09U,
    ADXRS453_POR_ERROR = 0x0AU,
    ADXRS453_PWR_ERROR = 0x0BU,
    ADXRS453_CST_ERROR = 0x0CU,
    ADXRS453_CHK_ERROR = 0x0DU,
    ADXRS453_ERROR
} ADXRS453_StatusTypeDef;

class ADXRS453
{
public:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    TIM_HandleTypeDef *htim;

    ADXRS453_TYPEDEF sensor_data;

    ADXRS453_StatusTypeDef Init();
    ADXRS453_StatusTypeDef adxrs453_update(void);
    ADXRS453_StatusTypeDef sensor(bool CHK, int16_t *date);
    uint32_t TransmitReceive(uint32_t address);
    ADXRS453_StatusTypeDef addread(uint8_t address, int16_t *date);

    ADXRS453(SPI_HandleTypeDef *q, GPIO_TypeDef *w, uint16_t e, TIM_HandleTypeDef *r, uint16_t t, float y, const char *u) : hspi(q), GPIOx(w), GPIO_Pin(e), htim(r), SELF_TEST_NUM(t), DEAD_ZONE(y), string_check(u) {}

private:
    uint16_t SELF_TEST_NUM;
    float DEAD_ZONE;
    const char *string_check;
    void SPI_ON()
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }
    void SPI_OFF()
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }
    bool odd_check(uint32_t date);
};
#endif
#pragma endregion

#pragma region /*BMI088*/
/**************************************BMI088************************************************/
#ifdef __SPI_H__

#define ACC_CHIP_ID 0x00
#define ACC_ERR_REG 0X02
#define ACC_STATUS 0X03
#define ACC_X_LSB 0X12
#define ACC_X_MSB 0X13
#define ACC_Y_LSB 0X14
#define ACC_Y_MSB 0X15
#define ACC_Z_LSB 0X16
#define ACC_Z_MSB 0X17
#define SENSORTIME_0 0X18
#define SENSORTIME_1 0X19
#define SENSORTIME_2 0X1A
#define ACC_INT_STAT_1 0X1D
#define TEMP_MSB 0X22
#define TEMP_LSB 0X23
#define ACC_CONF 0X40
#define ACC_RANGE 0X41
#define INT1_IO_CTRL 0X53
#define INT2_IO_CTRL 0X54
#define INT_MAP_DATA 0X58
#define ACC_SELF_TEST 0X6D
#define ACC_PWR_CONF 0X7C
#define ACC_PWR_CTRL 0X7D
#define ACC_SOFTRESET 0X7E

#define GYRO_CHIP_ID 0X00
#define RATE_X_LSB 0X02
#define RATE_X_MSB 0X03
#define RATE_Y_LSB 0X04
#define RATE_Y_MSB 0X05
#define RATE_Z_LSB 0X06
#define RATE_Z_MSB 0X07
#define GYRO_INT_STAT_1 0X0A
#define GYRO_RANGE 0X0F
#define GYRO_BANDWIDTH 0X10
#define GYRO_LPM1 0X11
#define GYRO_SOFTRESET 0X14
#define GYRO_INT_CTRL 0X15
#define INT3_INT4_IO_CONF 0X16
#define INT3_INT4_IO_MAP 0X18
#define GYRO_SELF_TEST 0X3C
float inVSqrt(float x);
typedef struct
{
    struct
    {
        float x;
        float y;
        float z;
        float LPF_x;
        float LPF_y;
        float LPF_z;
    } acc;

    struct
    {
        float x_nonoise;
        float y_nonoise;
        float z_nonoise;
        struct
        {
            float x;
            float y;
            float z;
        } calibration;
        struct
        {
            float x;
            float y;
            float z;
        } origin;
        struct
        {
            float x;
            float y;
            float z;
        } dynamicSum;
        struct
        {
            float x;
            float y;
            float z;
        } offset;
        struct
        {
            float x;
            float y;
            float z;
        } offset_max;
        struct
        {
            float x;
            float y;
            float z;
        } offset_min;
        struct
        {
            float x;
            float y;
            float z;
        } dps;
        struct
        {
            float x;
            float y;
            float z;
        } LPF;
    } gyro;

    struct
    {
        float x;
        float y;
        float z;
        float now_x;
        float now_y;
        float now_z;
        float last_x;
        float last_y;
        float last_z;
        float Real_x;
        float Real_y;
        float Real_z;
    } mang;

    uint16_t runningTimes;
    float temperature;
    uint8_t calibration;

} BMI088_TYPEDEF;

typedef enum
{
    BMI088_OK = 0x00U,
    BMI088_SET_ERROR = 0x01U,
    BMI088_ACC_ID_ERROR = 0x02U,
    BMI088_GYRO_ID_ERROR = 0x03U,
    BMI088_ERROR = 0x04U,
    BMI088_SELFTEXT_ERROR = 0x05U,
} BMI088_StatusTypeDef;

typedef enum
{
    BMI088_GYRO_RANGE_2000 = 0x00U,
    BMI088_GYRO_RANGE_1000 = 0x01U,
    BMI088_GYRO_RANGE_500 = 0x02U,
    BMI088_GYRO_RANGE_250 = 0x03U,
    BMI088_GYRO_RANGE_125 = 0x04U,
} BMI088_GyroRangeTypeDef;

typedef enum
{
    BMI088_ACC_RANGE_3 = 0X00U,
    BMI088_ACC_RANGE_6 = 0X01U,
    BMI088_ACC_RANGE_12 = 0X02U,
    BMI088_ACC_RANGE_24 = 0X03U,
} BMI088_AccRangeTypeDef;

struct
{
    float CUTOFF_FREQ = 50.0f;  // 截止频率
    float SAMPLE_RATE = 0.001f; // 采样周期
    float alpha;                // 滤波系数
} LPF_factor;

typedef struct
{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound_t;

/*四元数↓*/

typedef struct
{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound;

/*二维float向量结构体*/

// void BMI_CrossRound_err(void);

typedef struct vec2f
{
    float data[2];
} vec2f;

/*二维int16向量结构体*/

typedef struct vec2int16
{
    short data[2];
} vec2int16;

/*三维float向量结构体*/

typedef struct vec3f
{
    float data[3];
} vec3f;

/*三维int16向量结构体*/

typedef struct vec3int16
{
    short data[3];
} vec3int16;

/*四维float向量结构体*/

typedef struct vec4f
{
    float data[4];
} vec4f;

/*四维int16向量结构体*/

typedef struct vec4int16
{
    short data[4];
} vec4int16;

/*结构体*/

typedef struct accdata
{
    vec3int16 origin;      // 原始值
    vec3f offset_max;      // 零偏值最大值
    vec3f offset_min;      // 零偏值最小值
    vec3f offset;          // 零偏值
    vec3f calibration;     // 校准值
    vec3f filter;          // 滑动平均滤波值
    vec3f accValue;        // 加速度值，单位：m/s2
    vec3f dynamicSum;      // 校准时求和计算
    uint16_t runningTimes; // 运行次数
} accdata;

typedef struct gyrodata
{
    vec3int16 origin;  // 原始值
    vec3f offset_max;  // 零偏值最大值
    vec3f offset_min;  // 零偏值最小值
    vec3f offset;      // 零偏值
    vec3f calibration; // 校准值
    vec3f filter;      // 滑动平均滤波值
    vec3f dps;         // 度每秒
    vec3f radps;       // 弧度每秒
    vec3f dynamicSum;  // 校准时求和计算
} gyrodata;

typedef struct
{
    float x;
    float y;
    float z;
} Deal_acc_t;
typedef struct
{
    float x; // roll
    float y; // pitch
    float z; // yaw
} Deal_gyro_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaterInfo_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} eulerianAngles_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float Deal_pitch;
    float Deal_roll;
    float Deal_yaw;
} Anglespeed_t;
// NotchFilter.h
class LP
{
private:
    float LP_Last = 0;
    float freq = 1000;

public:
    float out;
    float K;
    float Cutoff_freq;
    uint8_t en = 1; // 使能标志
    float Set_LPK(float new_k)
    {
        K = LIMIT(new_k, 0, 1);
        return K;
    }
    LP(float sample_freq, float cutoff_freq) : freq(sample_freq), Cutoff_freq(cutoff_freq)
    {

        float klp = get_cutoff_freq(Cutoff_freq);
        Set_LPK(klp);
    }

    float low_pass_filter(float value)
    {
        if (!en)
        {
            Reset(value);
            return value;
        }
        out = LP_Last + K * (value - LP_Last);
        LP_Last = out;
        return out;
    }
    void Reset(float ref)
    {
        out = ref;
        LP_Last = ref;
    }
    float get_cutoff_freq(float cutoff_freq)
    {
        float klp = 1.0f - expf(-2.0f * m_pi * cutoff_freq / freq);
        return klp;
    }
};

class NotchFilter
{
public:
    // 构造函数
    // fs: 采样频率 (Hz)
    // f0: 陷波中心频率 (Hz)
    // Q:  品质因数（带宽 = f0 / Q），典型值 10~30

    float out = 0; /*滤波后输出*/
    float Q0 = 10; /*品质因数（带宽 = f0 / Q）*/
    float FS = 1000;
    float F0 = 100;
    float Min_Q = 10;
    float MAX_BW = 10;
    // float BW = 4;
    NotchFilter(float fs, float f0, float Q);
    void BW_Set(float BW)
    {
        Q0 = LIMIT(F0 / BW, Min_Q, 1000);
    }
    void NF_Param_update(float fs, float f0, float Q); /*更新参数*/
    void NF_update(void)                               /*更新参数*/
    {
        NF_Param_update(FS, F0, Q0);
    }

    // 重置滤波器状态
    void reset();

    // 滤波函数：输入一个样本，输出滤波后样本
    float process(float input);

private:
    float b0 = 0, b1 = 0, b2 = 0; // 分子系数
    float a1 = 0, a2 = 0;         // 分母系数 (a0 归一化为 1)
    float x1 = 0, x2 = 0;         // 输入延迟
    float y1 = 0, y2 = 0;         // 输出延迟
};
/*四元数↑*/

class BMI088
{
public:
    SPI_HandleTypeDef *hspi;
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *CSB1_GPIOx, *CSB2_GPIOx;
    uint16_t CSB1_GPIO_Pin, CSB2_GPIO_Pin;

    angleRound_t Round; // 过圈圈数
    BMI088_TYPEDEF sensor_data;
    BMI088_StatusTypeDef Init(uint8_t dwt_en);

    /*校准，滤波后的，输入姿态解算的*/
    Deal_acc_t Deal_acc;   // 单位化后的机体加速度
    Deal_gyro_t Deal_gyro; // 弧度机体角速度
    Deal_gyro_t Deg_gyro;  // 度机体角速度

    accdata acc = {0};
    gyrodata gyro;
    eulerianAngles_t eulerAngle;     // 欧拉角/无过圈
    eulerianAngles_t lastAngle;      // 上一次的欧拉角
    eulerianAngles_t last_realAngle; // 上一次的欧拉角

    // eulerianAngles_t nowAngle;   // 现在的欧拉角
    eulerianAngles_t realAngle; // 现在真实的欧拉角（已经叠加了圈数）

    Anglespeed_t Anglespeed;
    Anglespeed_t E_DAngle;

    BMI088_StatusTypeDef state = BMI088_ERROR;
    uint8_t enable_dwt = 1;
    // float q0_t;
    // float q1_t;
    // float q2_t;
    // float q3_t;
    // float err_last;
    const uint16_t freq = 1000;
    float delta_T = 1.0f / freq;         // 1khz
    float lpf_gyro_cutoff_freq = 135.0f; /*配置滤波器截止频率*/
    float lpf_acc_cutoff_freq = 30.0f;
    float lpf_gyro_param = 1;
    float lpf_acc_param = 1;
    quaterInfo_t Q_info = {1, 0, 0, 0}; // 全局四元数
    NotchFilter *gyroNotchX;            // X轴陷波滤波器
    NotchFilter *gyroNotchY;
    NotchFilter *gyroNotchZ;

    void set_zero(void);
    void low_pass_filter_init(void);
    void low_pass_filter(float value, float *out_last, float k);
    void BMI088_update(void);
    void BMI088_New_update(void);
    void getValues(void);         // imu校准，得到零偏值
    void QuatToEulerAngles(void); //
    void analyse(void);
    float get_cutoff_freq(float cutoff_freq, float sample_freq)
    {
        float k = 1.0f - expf(-2.0f * m_pi * cutoff_freq / sample_freq);
        return k;
    }
    // void BMI_Get_EulerAngle(void);
    // void BMI_analyse(void);
    // void BMI_QuatToEulerAngles(void);
    // void BMI_CrossRound(void);
    void BMI_CrossRound_err(void);

    void Analyse_speed(void);
    // void TEST_D_EulerAngles(void);
    void D_EulerAngles(void);

    float DWT_Get_time(void);

    void BMI088_AHRS(float gx, float gy, float gz, float ax, float ay, float az);
    BMI088(SPI_HandleTypeDef *q, TIM_HandleTypeDef *t, GPIO_TypeDef *w1, uint16_t p1, GPIO_TypeDef *w2, uint16_t p2, uint16_t num, float dz, BMI088_GyroRangeTypeDef gyrorange, BMI088_AccRangeTypeDef accrange) : hspi(q), htim(t), CSB1_GPIOx(w1), CSB1_GPIO_Pin(p1), CSB2_GPIOx(w2), CSB2_GPIO_Pin(p2), CALIBRATE_TIMES(num), dead_zoom(dz), GyroRange(gyrorange), AccRange(accrange)
    {
        float rpm = 6200.0f;
        float Q = 27.0F;
        gyroNotchX = new NotchFilter(freq, rpm / 60.0f, Q);
        gyroNotchY = new NotchFilter(freq, rpm / 60.0f, Q);
        gyroNotchZ = new NotchFilter(freq, rpm / 60.0f, Q);
    }

private:
    BMI088_GyroRangeTypeDef GyroRange;      // 陀螺仪量程
    BMI088_AccRangeTypeDef AccRange;        // 加速度量程
    float GyroResolution = 16.384f;         // 2000dps
    float AccRangsetting = 0.007177734375f; // 24g
    float Acc_Temperature_Offset = 0, Gyro_Temperature_Offset = 0;
    float dead_zoom = 0;
    uint16_t SELF_TEST_NUM;
    uint16_t timer_1ms = 0;
    uint8_t selftext_error_flag = 0, selftext_reset_step = 0;
    float last_gyro_x, last_gyro_y, last_gyro_z, last_temperature, filter_count_x, filter_count_y, filter_count_z, filter_count_temperature;
    float I_ex, I_ey, I_ez; // 误差积分

    uint32_t CALIBRATE_TIMES = 3000; // 校准的次数
    uint8_t calibrationState = 0;
    // uint8_t debug_acc[32] = {0};
    // uint8_t debug_gyro[32] = {0};

    void BMI088_SPI_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }
    void BMI088_SPI_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }
    void BMI088_write_Acc(uint8_t subAddress, uint8_t data);
    void BMI088_write_Gyro(uint8_t subAddress, uint8_t data);
    void BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf);
    void BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf);

    void BMI088_writeByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t subAddress, uint8_t data);
    void BMI088_readBytes(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf);
    void selftext_error_reset(void);
    void DWT_Init(void);
};

#endif
#pragma endregion

/**************************************Vision_LPF***********************************************/
class Vision_LPF
{
public:
    void Vision_Low_Pass_Filter_Init(void);
    float Vision_Low_Pass_Filter(float value);
    Vision_LPF(float CF, float SR) : CUTOFF_FREQ(CF), SAMPLE_RATE(SR) {}
    float CUTOFF_FREQ;                  // 截止频率
    float SAMPLE_RATE;                  // 采样周期
    float pi = 3.14159265358979323846f; // π
    float alpha;                        // 滤波系数
    float b;
    float out_last = 0; // 上一次滤波值
    float out;
};

/**************************************PWM_MCL***********************************************/
#ifdef __TIM_H__

class MCL_snail
{
public:
    TIM_HandleTypeDef *htim;
    TIM_HandleTypeDef *htim_z;
    uint32_t Channel_z;
    TIM_HandleTypeDef *htim_y;
    uint32_t Channel_y;

    void Init(void);
    void Init_XC_Calibration(uint8_t speed_z_max, uint8_t speed_y_max);
    void Init_Change_Steer(uint8_t dir, uint8_t speed_max);
    void stop(void);
    void run(uint8_t grade);
    HAL_StatusTypeDef shoot_state(void);
    void state_tick(TIM_HandleTypeDef *p);
    void set_speed(uint8_t speed_z, uint8_t speed_y);

    MCL_snail(TIM_HandleTypeDef *htim,
              TIM_HandleTypeDef *htim_z, uint32_t Channel_z,
              TIM_HandleTypeDef *htim_y, uint32_t Channel_y,
              uint8_t grade_1, uint8_t grade_2, uint8_t grade_3,
              uint8_t grade_1_error, uint8_t grade_2_error, uint8_t grade_3_error,
              const char *string_check) : htim(htim), htim_z(htim_z), Channel_z(Channel_z), htim_y(htim_y), Channel_y(Channel_y),
                                          grade_1(grade_1), grade_2(grade_2), grade_3(grade_3),
                                          grade_1_error(grade_1_error), grade_2_error(grade_2_error), grade_3_error(grade_3_error),
                                          string_check(string_check) {}

private:
    uint8_t grade_1, grade_2, grade_3;
    uint8_t grade_1_error, grade_2_error, grade_3_error;
    const char *string_check;
    uint8_t shoot_state_byte, run_stete;
    uint32_t time_20ms;
    uint8_t first_state;
};
#endif

/*************************************UD_check**********************************************/
typedef enum
{
    UpDown_check_nothing,
    UpDown_check_falling,
    UpDown_check_rising
} UpDown_check_state;
class UpDown_check_class
{
public:
    UpDown_check_class(bool initial_conditions) : bit(initial_conditions)
    {
    }
    UpDown_check_state UD_data = UpDown_check_nothing;
    bool enable_flag = 0;
    UpDown_check_state updata(bool Condition)
    {
        if (((Condition) != 0) && ((bit & 1) == 0))
        {
            bit |= 1;
            UD_data = UpDown_check_rising;
            return UpDown_check_rising;
        }
        else if (!((Condition) != 0) && ((bit & 1) != 0))
        {
            bit &= ~1;
            UD_data = UpDown_check_falling;
            return UpDown_check_falling;
        }
        else
        {
            UD_data = UpDown_check_nothing;
            return UpDown_check_nothing;
        }
    }
    void Init(void)
    {
    }

private:
    bool bit;
};

/*************************************RGB**********************************************/
#ifdef __TIM_H__

#define PIXEL_NUM 78              // 灯珠数
#define NUM (24 * PIXEL_NUM + 300) // Reset 280us（复位时间） / 1.25us = 224   NUM的值为单个灯珠的位宽（24）* 灯珠数量（PIXEL_NUM）+ 复位脉冲数    1/84M = 11.9ns
#define WS1 75                     // 重装值105
#define WS0 30

class RGB_UI
{
public:
    TIM_HandleTypeDef *htim;
    uint8_t R = 0, G = 0, B = 0;
    uint32_t Channel;
    HAL_StatusTypeDef DMAstatus = HAL_TIMEOUT;
    void RGB_UI_Init(void);
    void WS_Load(void);
    void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);
    void WS_CloseAll(void);
    void WS281x_SetPixelRGB(uint16_t n, uint8_t red, uint8_t green, uint8_t blue);

    RGB_UI(TIM_HandleTypeDef *htim, uint32_t Channel) : htim(htim), Channel(Channel) {}

private:
    uint16_t send_Buf[NUM];
    const char *string_check_rgb_ui;
    uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue);
    void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);
};

/**LPB60B激光测距 */
// class LPB60B
//{
// private:
//     UART_HandleTypeDef *huart;
//     DMA_HandleTypeDef *hdma_usart_rx;
//     uint8_t LP_rxbuffer[64];
//     uint8_t cmd_txbuffer[8];
//     const uint8_t cmdid_1[8]={0x55,0x01,0x00,0x00,0x00,0x00,0xD3,0xAA};
//     const uint8_t cmdid_2[8]={0x55,0x30,0x00,0x00,0x00,0x00,0x84,0xAA};
//     const uint8_t cmdid3

// public:
//     HAL_StatusTypeDef Init(void)
//     {
//         __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT); // 关闭dma传输过半中断
//         HAL_UARTEx_ReceiveToIdle_DMA(huart, LP_rxbuffer, sizeof(LP_rxbuffer));

//        // __HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);
//        // return HAL_UART_Receive_DMA(this->huart, LP_rxbuffer, 128);
//    }
//    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // 不定长接收视觉数据
//    {

//        if (huart == this->huart)
//        {
//            HAL_UARTEx_ReceiveToIdle_DMA(huart, LP_rxbuffer, sizeof(LP_rxbuffer));
//            __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT); // 关闭dma传输过半中断
//        }
//    }
//    // 0x55,Key,value[4],crc8,0xAA
//    // Key 字段，此处表示此数据包为接收数据。
//    // Value 字段的高字节，此处表示系统状态，0表示系统正常。
//    // 表示测量的距离值，16进制表示，单位是mm。

//    // 01获取设备信息 2获取温度信息 3设置测量频率
//    // 4设置数据格式 5启动测量 6停止测量
//    // 7测量数据返回 8保存设置 A获取序列号
//    // D设置测量模式 E高速测量数据返回
//    // 11配置设备地址 12设置波特率
//    HAL_StatusTypeDef cmdtx(uint8_t cmd_id)
//    {
//        switch (cmd_id)
//        {
//        case 0x01:
//        {
//            cmd_txbuffer[0] = 0x55;
//            cmd_txbuffer[1] =
//                cmd_txbuffer[2] =
//                    cmd_txbuffer[3] =
//                        cmd_txbuffer[4] =
//                            cmd_txbuffer[5] =
//                                cmd_txbuffer[6] =
//                                    cmd_txbuffer[7] = AA;
//            break;
//        }

//        case 0X02:
//        {
//            cmd_txbuffer[0] = 0x55;
//            cmd_txbuffer[1] =
//                cmd_txbuffer[2] =
//                    cmd_txbuffer[3] =
//                        cmd_txbuffer[4] =
//                            cmd_txbuffer[5] =
//                                cmd_txbuffer[6] =
//                                    cmd_txbuffer[7] = AA;
//            break;
//        }

//        default:
//            break;
//        }
//    }

//    /* 生成多项式为CRC-8x8+x5+x4+1 0x31(0x131) */
//    uint8_t crc_high_first(uint8_t *ptr, uint8_t data_len)
//    {
//        uint8_t i;
//        uint8_t crc = 0x00;
//        while (data_len--)
//        {
//            crc ^= *ptr++;
//            for (uint8_t ii = 8; ii > 0; --ii)
//            {
//                if (crc & 0x80)
//                    crc = (crc << 1) ^ 0x31;
//                else
//                    crc = (crc << 1);
//            }
//        }
//        return crc;
//    }

//    LPB60B(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma_usart_rx) : huart(uart), hdma_usart_rx(dma_usart_rx);
//};

#endif

#endif
