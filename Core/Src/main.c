/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "RM_Lib.hpp"
#include "communication.h"
#include "my_math.h"
#include "CP_System.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #define DEBUG
float Bullet_Speed = 24.6f;
float get_cutoff_freq(float cutoff_freq, float sample_freq)
{
    float wc = 1.0f / (2.0f * 3.1415926f * cutoff_freq);
    float T = 1.0f / sample_freq;
    float a;
    a = 1.0f - expf(-wc * T);
    return a;
}

float low_pass_filter(float value, float *out_last, float k_last)
{
    // static float out_last = 0; // 上一次滤波值
    float out;

    /***************** 如果第一次进入，则给 out_last 赋值 ******************/
    // static char fisrt_flag = 1;
    // if (fisrt_flag == 1)
    // {
    //     fisrt_flag = 0;
    //     *out_last = value;
    // }

    /*************************** 一阶滤波 *********************************/
    out = *out_last + k_last * (value - *out_last);
    *out_last = out;

    return out;
}

/*S形平滑（不是滤波器，类似斜坡）*/
// float SOUT;
// float spF = 0.01F; // 0.1-0.01
float smooth_step(float current, float target, float speed_factor)
{
    // speed_factor: 0.01~0.1 之间，越大越陡
    float diff = target - current;
    return current + diff * speed_factor * (1.0f - fabsf(diff) / 180.0f); // 误差大时加速，误差小时减速
}

float map_double(float x, float in_min, float in_max,
                 float out_min, float out_max)
{
    // 处理输入范围为零的情况
    if (in_max - in_min == 0.0)
    {
        return (out_min + out_max) / 2.0f;
    }
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}
/*自瞄加速度平滑*/
LP ZM_TXACCP(1000, 30);
LP ZM_TXACCY(1000, 30);

/*自瞄目标平滑*/
LP ZM_RXP(1000, 30);
LP ZM_RXY(1000, 30);
/***********倍镜舵机宏定义*************/

// #define Gyro_Mode 0x00
// #define Encoder_Mode 0x01
/************修改P Y轴的鼠标速度******************/

/********************云台无陀螺仪情况下底盘跟随云台的速度（没写）**********************/
// #define Homing_Speed 0.0043
// #define Erro_Spin_Speed 4

/*********************物理UI*********************/
// #define RGB_goal 0x32            // ws2812需要的
// RGB_UI RGB_UI(&htim3, TIM_CHANNEL_3, "put the Update function in 800kHz interrupt");

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma region /*RC*/
RC YK(&huart4, &huart3);

// 一帧can大概需要140us
USER_CAN CAN_1(&hcan1, 0), // 4.1khz 51%
    CAN_2(&hcan2, 1);

BMI088 IMU(&hspi3, &htim9, GPIOC, GPIO_PIN_10, GPIOA, GPIO_PIN_15, 3000, 0.0f, BMI088_GYRO_RANGE_500, // 088陀螺仪英雄是spi3,单独使用A15是可以的，但是D2就不行
           BMI088_ACC_RANGE_24);

float MCL_SP = 6150.0f; // 调转速

/*遥控控制模式*/
typedef enum
{
    LOST_MODE = -1,
    PROTECT_MODE = 0,
    DAN_YUN_TAI_MODE = 1,
    SHANG_XIA_MODE = 2,
    DI_PAN_L_MODE = 3,
    SHUANG_ZHONG_MODE = 4,
    DIAO_SHE_MODE = 5,
    DI_PAN_H_MODE = 6,
    XIAO_TUO_LUO_MODE = 7,
    ZHAN_DOU_MODE = 8

} RM_MODE_e;
typedef enum
{
    DISABLE_e = 0,
    ENABLE_e = 1

} ENABLE_Enum;
int8_t YKmode = PROTECT_MODE; // 默认保护模式

void MYMODE_Task(void)
{
    if (!YK.online)
    {
        YKmode = LOST_MODE;
        return;
    }
    if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID) // 上中，单云台(自瞄)
    {
        YKmode = DAN_YUN_TAI_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_DOWN) // 上下
    {
        YKmode = SHANG_XIA_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_UP) // 中上，低速单底盘
    {
        YKmode = DI_PAN_L_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_MID) // 双中，底盘跟随云台，云台imu闭环
    {
        YKmode = SHUANG_ZHONG_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_DOWN) // 中下，吊射，编码器值，小pitch
    {
        YKmode = DIAO_SHE_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_UP) // 下上，高速单底盘
    {

        YKmode = DI_PAN_H_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_MID) // 下中
    {
        YKmode = XIAO_TUO_LUO_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_DOWN) // 双下战斗
    {
        YKmode = ZHAN_DOU_MODE;
    }
    else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_UP) // 双上保险
    {
        YKmode = PROTECT_MODE;
    }
    else // 没遥控器等，保险
    {
        YKmode = LOST_MODE;
    }
    if (YK.Pressed_Check(KEY_PRESSED_R) && YK.Pressed_Check(KEY_PRESSED_CTRL))
    {
        YKmode = PROTECT_MODE;
        // __set_FAULTMASK(1); // 关闭所有中断

        for (uint8_t i = 0; i < 32; i++)
        {
            //            PITCH_M.DM_MIT(0, 0, 0, 0, 0);
            CAN_1.Send_RM(0x1ff, 0, 0, 0, 0);
            CAN_2.Send_RM(0X1FE, 0, 0, 0, 0);
            // HAL_Delay(1);
        }

        // __set_FAULTMASK(1); // 关闭所有中断
        NVIC_SystemReset(); // 复位
    }
}

#pragma endregion

#pragma region /*new pid frame*/
/*pid类的pid目标值处理用函数指针传入类里，指向自定义的函数？*/

typedef enum
{
    Param_1 = 0,
    Param_2 = 1,
    Param_3 = 2,
    Param_4 = 3,
    Param_5 = 4
} PID_Ctrl_Param_e;

typedef enum
{
    IN = 1,
    OUT = 0
} LOOP_Index_e; // 内外环索引

/*此结构体操作PID_Ctrl_Index控制结构体的总保护开关，实现内外环切换，不需要内外环切换时，此结构体不需要创建
注意使用此结构体时，会操作PID_Ctrl_Index结构体的protect_flag成员，不建议单独操作PID_Ctrl_Index结构体的protect_flag成员
而是使用此结构体loop_set=NOLOOP_SET实现保护
还是遵循只进行模块化，自己搭建特定框架
*/
typedef enum
{
    NOLOOP_SET = 0,
    INLOOP_SET = 1,
    OUTLOOP_SET = 2
    // 如果有3环，自己加枚举

} PID_loopCtrl_e;

typedef struct
{
    PID_Ctrl_Index *out_ctrl_index; // 外环控制索引
    PID_Ctrl_Index *in_ctrl_index;  // 内环控制索引
    PID_loopCtrl_e loop_set;        // 0保护，1单环，2双环
} PID_loopCtrl_Index;

typedef enum
{
    NONE_OVERSTEP = 0,
    PARAM_OVERSTEP = 1, // 检查当前是否越界标志
    REF_OVERSTEP = 2,
    PARAM_MODIFY_OVERSTEP = 4, // 切换修改操作时越界
    REF_MODIFY_OVERSTEP = 8

} PID_IndexErr_e; // 掩码方式

/*切换单双环/保护*/
void set_loop(PID_loopCtrl_Index *loop_index)
{
    if (loop_index->loop_set == OUTLOOP_SET)
    {
        loop_index->out_ctrl_index->protect_flag = 0;
        loop_index->in_ctrl_index->protect_flag = 0;
    }
    else if (loop_index->loop_set == INLOOP_SET)
    {
        loop_index->out_ctrl_index->protect_flag = 1;
        loop_index->in_ctrl_index->protect_flag = 0;
    }
    else if (loop_index->loop_set == NOLOOP_SET)
    {
        loop_index->out_ctrl_index->protect_flag = 1;
        loop_index->in_ctrl_index->protect_flag = 1;
    }
    else
    {
        loop_index->out_ctrl_index->protect_flag = 1;
        loop_index->in_ctrl_index->protect_flag = 1;
    }
}
/*自动初始化&参数反馈切换模块的判断数组越界的保护措施*/
void pid_set_protect(PID_Ctrl_Index *_ctrl_Index)
{
    _ctrl_Index->protect_flag = 1;
    _ctrl_Index->ref_type = 0;
    _ctrl_Index->param_type = 0;
}

/*判断数组越界处理*/
uint8_t pidclass_index_overstep_protect(PID_Ctrl_Index *_ctrl_Index) /*数组越界保护*/
{
    uint8_t overstep_log = NONE_OVERSTEP;

    if (_ctrl_Index->ref_type >= _ctrl_Index->ref_num)
    {
        _ctrl_Index->ref_index_overstep_log++; // 记录关键日志
        overstep_log |= REF_OVERSTEP;

        /*deal，直接上保护，并吧ref_type和param_type强制给0*/
        pid_set_protect(_ctrl_Index);
    }
    if (_ctrl_Index->param_type >= _ctrl_Index->param_num)
    {
        _ctrl_Index->param_index_overstep_log++; // 记录关键日志
        overstep_log |= PARAM_OVERSTEP;          // 索引越界保护
        /*其实此时可以，直接上保护，并吧ref_type和param_type强制给0*/
        pid_set_protect(_ctrl_Index);
    }

    return overstep_log;
}
/*
此只对单次操作做判断并处理错误，用途自己想，属于可选，但其他底层保护做好了
*/
void check_log_err(PID_Ctrl_Index *_ctrl_Index) // 对当次日志做处理/保护/记录（全可选）
{
    if (_ctrl_Index->log_err & REF_OVERSTEP) /*检查当前是否越界*/
    {
        _ctrl_Index->ref_index_overstep_log++;
        // pid_set_protect(_ctrl_Index);
        // deal();
    }
    if (_ctrl_Index->log_err & PARAM_OVERSTEP)
    {
        _ctrl_Index->param_index_overstep_log++;
        // pid_set_protect(_ctrl_Index);
        // deal();
    }
    if (_ctrl_Index->log_err & REF_MODIFY_OVERSTEP) /*修改操作越界也算越界*/
    {
        _ctrl_Index->ref_index_overstep_log++;
        // pid_set_protect(_ctrl_Index);
        // deal();
    }
    if (_ctrl_Index->log_err & PARAM_MODIFY_OVERSTEP)
    {
        _ctrl_Index->param_index_overstep_log++;
        // pid_set_protect(_ctrl_Index);
        // deal();
    }
}

/*
切换反馈数据不直接操作索引变量，而是调用此函数（防呆操作）
防止直接调用结构体导致数组越界发生意想不到的后果*/
uint8_t set_ref_type(PID_Ctrl_Index *_ctrl_Index, PID_Ctrl_Ref_e _ref_type)
{
    uint8_t overstep_log = 0;
    overstep_log |= pidclass_index_overstep_protect(_ctrl_Index);              // 这里冗余了，其实pid_setref_controler做了就行，但是安全起见
    overstep_log |= (_ref_type >= _ctrl_Index->ref_num) * REF_MODIFY_OVERSTEP; // 必
    if (!overstep_log)                                                         // 不越界才会传入
    {
        _ctrl_Index->ref_type = _ref_type;
    }
    else if (overstep_log & REF_MODIFY_OVERSTEP)
    {
        _ctrl_Index->ref_index_overstep_log++;
    }

    return overstep_log;
}
/*
切换参数数据不直接操作索引变量，而是调用此函数（防呆操作）
防止直接调用结构体导致数组越界发生意想不到的后果*/
uint8_t set_param_type(PID_Ctrl_Index *_ctrl_Index, uint8_t _param_type)
{
    uint8_t overstep_log = 0;
    overstep_log |= pidclass_index_overstep_protect(_ctrl_Index);
    overstep_log |= (_param_type >= _ctrl_Index->ref_num) * PARAM_MODIFY_OVERSTEP; // 必
    if (!overstep_log)                                                             // 不越界才会传入
    {
        _ctrl_Index->param_type = _param_type;
    }
    else if (overstep_log & PARAM_MODIFY_OVERSTEP)
    {
        _ctrl_Index->param_index_overstep_log++;
    }
    return overstep_log;
}

/*众多pid参数/反馈下，同时只会有一环是使能状态，其余全被自动初始化
有多种不同样的自动初始化&反馈数据切换函数，每个函数可能对PID_Ctrl_Index结构体的索引定义不同（我已经尽量相同化了，目前就功能3索引不一样）
*/
/*功能一*/
/*PID_class*为一维数组类，控制结构体*/
/*切换多个反馈，每个反馈只可固定对应一个pid参数
或者
切换多个pid参数，反馈固定
建议基本简单功能用功能1即可
*/
/*单环*/
uint8_t pid_setref_controler(PID_class *_pid, PID_Ctrl_Index *_ctrl_Index)
{
    /*判断是哪种用法*/
    bool select_param_flag = (_ctrl_Index->param_num > 1);
    bool select_ref_flag = (_ctrl_Index->ref_num > 1);
    bool conflict_flag = (select_param_flag && select_ref_flag); // 数据格式冲突标志
    uint8_t ctrl_num = 1;                                        // 决定循环次数
    uint8_t curr_index = 0;                                      // 得到当前索引
    uint8_t overstep_log = 0;
    overstep_log |= pidclass_index_overstep_protect(_ctrl_Index);

    if (overstep_log || conflict_flag) /*数据格式不对，用此模式多个反馈和多个pid参数只能二选一*/
    {
        overstep_log |= conflict_flag * (REF_MODIFY_OVERSTEP | PARAM_MODIFY_OVERSTEP); // 打日志，不错过任何错误信息
        return overstep_log;
    }
    else if (select_param_flag)
    {
        /*多个pid参数，单一反馈*/
        _ctrl_Index->ref_type = 0; // 反馈固定为0
        ctrl_num = _ctrl_Index->param_num;
        curr_index = _ctrl_Index->param_type;
    }
    else if (select_ref_flag)
    {

        /*多个反馈，单一pid参数*/
        _ctrl_Index->param_type = 0; // pid参数固定为0
        ctrl_num = _ctrl_Index->ref_num;
        curr_index = _ctrl_Index->ref_type;
    }
    for (uint8_t ifref = 0; ifref < ctrl_num; ifref++)
    {
        /*多选一*/
        if (_ctrl_Index->ref_type != ifref)
        {
            _pid[ifref].enable_flag = 0;
            _pid[ifref].Reset();
        }
    }

    /*总开关，因为ctrl_Index.ref_type索引没有保护选项，
    所以要用额外标志处理开关正在指向的pid反馈数据结构，
    实际上保护只要在当前调用一次outer_pid[ctrl_Index.ref_type].enable_flag = 0;
    就可以关闭该闭环
    */

    if (_ctrl_Index->protect_flag)
    {
        _pid[curr_index].enable_flag = 0;
        _pid[curr_index].Reset();
    }
    else
    {
        _pid[curr_index].enable_flag = 1;
    }
    // _pid[curr_index].ifReset();
    return overstep_log;
}

/*双环（可以合并一个总结构体）*/
void pid_setref2loop_controler(PID_class *outer_pid, PID_class *inner_pid, PID_Ctrl_Index *outloop_ctrl_Index, PID_Ctrl_Index *inloop_ctrl_Index)
{
    pid_setref_controler(outer_pid, outloop_ctrl_Index);
    pid_setref_controler(inner_pid, inloop_ctrl_Index);
    /*如果你要用3环，再来一个pid_setref_controler即可*/
}

/*功能二，反馈数据多选一&参数多选一（单点）*/
/*PID_class*为一个二维数组类*/
/*可实现[多少pid参数][反馈数据个数]的单环pid选择*/
/*双环在功能四*/
uint8_t pid_set1param_controler(PID_class *_pid, PID_Ctrl_Index *_ctrl_Index)
{
    /*法1，看看就好，法2更方便*/
    // for (uint8_t ifref = 0; ifref < ctrl_Index.ref_num; ifref++) // 遍历所有反馈索引
    // {
    //     for (uint8_t ifparam = 0; ifparam < ctrl_Index.param_num; ifparam++) // 遍历所有参数索引
    //     {
    //         uint8_t index = ifparam * ctrl_Index.ref_num + ifref;
    //         if (ctrl_Index.param_type != ifparam || ctrl_Index.ref_type != ifref) // if-else倒置，用&&更好理解，或者直接先找index=ctrl_Index.param_type * ctrl_Index.ref_num + ctrl_Index.ref_type;
    //         {
    //             // 计算一维索引
    //             _pid[index].enable_flag = 0;
    //         }
    //         else
    //         {
    //             /*总开关，因为ctrl_Index.ref_type索引没有保护选项，
    //             所以要用额外标志处理开关正在指向的pid反馈数据结构，
    //             实际上保护只要在当前调用一次_pid[当前索引].enable_flag = 0;
    //             就可以关闭该闭环
    //             */
    //             // ctrl_Index.param_type*ctrl_Index.ref_type+ctrl_Index.ref_type
    //             if (ctrl_Index.protect_flag)
    //             {
    //                 _pid[index].enable_flag = 0;
    //             }
    //             else
    //             {
    //                 _pid[index].enable_flag = 1;
    //             }
    //         }
    //     }
    // }

    /*法2*/
    /*越界保护*/
    uint8_t overstep_log = 0;
    overstep_log |= pidclass_index_overstep_protect(_ctrl_Index);
    if (overstep_log)
    {
        return overstep_log;
    }
    /* 先直接找到当前索引,将二位数组转为一维数组（本质是一样的）*/
    uint8_t now_index = _ctrl_Index->param_type * _ctrl_Index->ref_num + _ctrl_Index->ref_type; // 得到当前索引，
    /* 初始化就要吧所有参数遍历*/
    for (uint8_t ifindex = 0; ifindex < _ctrl_Index->param_num * _ctrl_Index->ref_num; ifindex++) // 两个for循环合并一个遍历
    {
        if (ifindex != now_index)
        {
            _pid[ifindex].enable_flag = 0;
            _pid[ifindex].Reset();
        }
        else
        {
            /*总开关，因为ctrl_Index.ref_type索引没有保护选项，
                所以要用额外标志处理开关正在指向的pid反馈数据结构，
                实际上保护只要在当前调用一次_pid[当前索引].enable_flag = 0;
                就可以关闭该闭环
                */
            if (_ctrl_Index->protect_flag)
            {
                _pid[ifindex].enable_flag = 0;
                _pid[ifindex].Reset();
            }
            else
            {
                _pid[ifindex].enable_flag = 1;
            }
        }
    }
    /*只操作当前索引进行判断复位*/
    // _pid[now_index].ifReset();
    return overstep_log;
}
/*END*/

/*功能四*/
/*建议但凡有些许复杂的需求，都推荐直接使用功能4，可以满足99%的需求
如单双多环切换
内外环参数的独立切换
多参数
多反馈
可同时实现
*/
/*PID_class*，PID_class*双二位数组（内环，外环）输入参数，反馈类型，不同pid参数
PID_class[pid参数个数][反馈数据个数]
 超级封装，内外环独立，不需要一一对应（但一般都有对应关系，此处选择过于自由，需注意反馈数据，参数有效性）
 如果需要内外环索引一一对应，选择PID_Ctrl_Index参数传入统一个结构体即可
*/
void pid_set2param_controler(PID_class *outer_pid, PID_class *inner_pid, PID_Ctrl_Index *out_loop_ctrl_Index, PID_Ctrl_Index *in_loop_ctrl_Index)
{
    /* 调用两次功能1即可*/
    pid_set1param_controler(outer_pid, out_loop_ctrl_Index);
    pid_set1param_controler(inner_pid, in_loop_ctrl_Index);
    /*如果你要用3环，再来一个pid_set1param_controler即可*/
}

/*功能三（不常用） 仅靠一个二位数组入参，实现内外多环&反馈数据选择(单列选择）*/
/*可实现[多少环pid（一般没用，固定2环）][反馈数据个数]选择*/
/*实际上是吧单环，双环等环数个数（参数选项）整合进PID_class二维数组的其中一维（行|列索引）
但是多少环直接套一维或者二维数组，再创个n维参数的函数就好了
反馈数据&pid参数会与PID_class互相耦合（不好拆成函数），相比之下环数更独立与固定*/
/*[多少环pid][反馈数据个数]*/
// PID_class pid_testloop[3][2]={
//     {{},{}},
//     {{},{}},
//     {{},{}}
// }
/*imu,motor
  gyro,msp
  ..   ..
*/
/*功能三用处不大*/
void pid_setloop_controler(PID_class *_pid, PID_Ctrl_Index *_ctrl_Index)
{
    /*越界保护*/
    uint8_t overstep_flag = pidclass_index_overstep_protect(_ctrl_Index);
    if (overstep_flag)
    {
        return;
    }
    /*先找列，找到后遍历行即可*/
    for (uint8_t ifparam = 0; ifparam < _ctrl_Index->param_num; ifparam++) // 遍历所有参数索引
    {
        for (uint8_t ifref = 0; ifref < _ctrl_Index->ref_num; ifref++) // 遍历所有反馈索引
        {
            uint8_t index = ifparam * _ctrl_Index->ref_num + ifref;
            if (_ctrl_Index->ref_type == ifref)
            {

                /*总开关，因为ctrl_Index.ref_type索引没有保护选项，
                所以要用额外标志处理开关正在指向的pid反馈数据结构，
                实际上保护只要在当前调用一次_pid[当前索引].enable_flag = 0;
                就可以关闭该闭环
                */
                if (_ctrl_Index->protect_flag)
                {
                    _pid[index].enable_flag = 0;
                    _pid[index].Reset();
                }
                else
                {

                    _pid[index].enable_flag = 1;
                }
            }
            else
            {
                _pid[index].enable_flag = 0;
            }
        }
    }
}
#pragma endregion

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*Aerial*/
#pragma region /*demo test*/
/*功能1示例*/
/*pid参数设置*/
MOTOR_RM test_M(0x201, &CAN_2);
const uint8_t test_ref_num = 2;

PID_class test_outer_pid[test_ref_num] = {
    PID_class(15, 0, 0, 10000, 0, 0, 10000), // imu反馈/参数1
    PID_class(2, 0, 0, 1000, 0, 0, 1000)     // 编码反馈/参数2
};
PID_class test_inner_pid[test_ref_num] = {
    PID_class(50, 0, 0, 3000, 0, 0, 10000), // gyro反馈/参数1
    PID_class(2, 0, 0, 1000, 0, 0, 1000)    // msp反馈/参数2
};

PID_Ctrl_Index y_ctrl = {Param_1, IMU_TYPE, 1, test_ref_num, 1};

void test_pid_setref2loop_controler_task(void) // yaw_while_application_layer
{

    /*********************************************            yaw控制逻辑             ********************************************/

    // if (Pitch_088_state == BMI088_OK && (YKmode == ZHAN_DOU_MODE || YKmode == SHUANG_ZHONG_MODE || YKmode == DAN_YUN_TAI_MODE || YKmode == XIAO_TUO_LUO_MODE) && deploy_flag != 1)
    // {
    //     // if (zimiao_mode == ZM_MODE) // 喵，不打弹
    //     // {
    //     //     yaw_control_mode = ZM_MODE;
    //     // }
    //     // else if (zimiao_mode == ZM_MODE_AUTO) // 喵，打弹
    //     // {
    //     //     yaw_control_mode = ZM_MODE_AUTO;
    //     // }
    //     // else
    //     // {
    //     //     yaw_control_mode = GYRO_MODE;
    //     // }
    //     // yaw_control_mode = IMU_TYPE;
    //     // Erro_Yaw = Motor_LK6010_Yaw.mang_inf; // 编码闭环复位
    //     // PID_YAW_Erro_IMU_MANG.OUT_I = 0;

    //     y_ctrl.ref_type = IMU_TYPE; // 就这么简单！
    //     if (temp_close)
    //     {
    //         y_ctrl.protect_flag = 1; // 可以操作protect_flag暂时关闭
    //     }
    //     else
    //     {
    //         y_ctrl.protect_flag = 1; // 可以操作protect_flag开启
    //     }
    // }
    // else if (YKmode == DIAO_SHE_MODE || YKmode == SHANG_XIA_MODE || (Pitch_088_state != BMI088_OK && YKmode == ZHAN_DOU_MODE) || deploy_flag == 1) // 下,或者控制时陀螺仪坏
    // {
    //     // Yaw_goal = IMU.realAngle.yaw;
    //     y_ctrl.ref_type = MOTOR_TYPE; // 就这么简单！
    //     if (temp_close)
    //     {
    //         y_ctrl.protect_flag = 1; // 可以操作protect_flag暂时关闭
    //     }
    //     else
    //     {
    //         y_ctrl.protect_flag = 1; // 可以操作protect_flag开启
    //     }
    // }
    // else
    // {
    //     // Erro_Yaw = Motor_LK6010_Yaw.mang_inf; // 编码闭环复位
    //     // PID_YAW_Erro_IMU_MANG.OUT_I = 0;
    //     // YAW_PID_OUT = 0;
    //     y_ctrl.protect_flag = 1; // 保护!
    // }
    /*此处内外环一一对应，合并成总控结构体（常用）*/
    pid_setref2loop_controler(test_outer_pid, test_inner_pid, &y_ctrl, &y_ctrl);
}
void test_pid_set_controler_imu(void)
{
    /*反馈值传递*/
    test_outer_pid[IMU_TYPE].Ref = -IMU.realAngle.yaw; // 可填+-号调整输入去调整pid极性（标准坐标系规范）
    test_inner_pid[IMU_TYPE].Ref = -IMU.Anglespeed.Deal_yaw;
    /*启用标志放自动初始化函数判断，属于多选一的结构*/
    if (test_outer_pid[IMU_TYPE].enable_flag)
    {
        test_outer_pid[IMU_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0f);
        test_outer_pid[IMU_TYPE].PID_update_void();
        test_inner_pid[IMU_TYPE].Goal = test_outer_pid[IMU_TYPE].OUT_PID;
        test_inner_pid[IMU_TYPE].PID_update_void();
        /*输出值传递*/
        test_M.out = test_inner_pid[IMU_TYPE].OUT_PID; // 可填+-号调整输出极性（标准坐标系规范）
    }
}
void test_pid_set_controler_can(void)
{
    /*反馈值传递*/
    test_outer_pid[MOTOR_TYPE].Ref = test_M.mang_inf;
    test_inner_pid[MOTOR_TYPE].Ref = test_M.sp;

    /*启用标志放自动初始化函数判断，属于多选一的结构*/
    if (test_outer_pid[MOTOR_TYPE].enable_flag) /*编码值与imu在不同地方算*/
    {
        /*目标传递值*/
        test_outer_pid[MOTOR_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0f);
        test_outer_pid[MOTOR_TYPE].PID_update_void();
        test_inner_pid[MOTOR_TYPE].Goal = test_outer_pid[MOTOR_TYPE].OUT_PID;
        test_inner_pid[MOTOR_TYPE].PID_update_void();
        /*输出值传递*/
        test_M.out = test_inner_pid[M_SP_TYPE].OUT_PID;
    }
}

/*功能二/四示例*/
const uint8_t test2p_param = 3;
const uint8_t test2p_ref = 2;
PID_Ctrl_Index pid_test2p_ctrl = {Param_1, MOTOR_TYPE, test2p_param, test2p_ref, 1}; // 可实现[多少环pid][反馈数据个数]选择
PID_class pid_test2p_out[test2p_param][test2p_ref] = {
    // imu1，motor1
    // imu2，motor2
    // imu3, motor3
    {PID_class(1, 0, 0, 10, 0, 0, 10), PID_class(25, 0, 0, 10, 0, 0, 10)},
    {PID_class(0.01f, 0, 0, 1, 0, 0, 1), PID_class(2, 0, 0, 1000, 0, 0, 1000)},
    {PID_class(0, 0, 0, 0, 0, 0, 0), PID_class(0, 0, 0, 0, 0, 0, 0)}}; // 参数全0，单环速度环
PID_class pid_test2p_in[test2p_param][test2p_ref] = {
    // gyro1，msp1
    // gyro2，msp2
    // gyro3, msp3
    {PID_class(1, 0, 0, 10, 0, 0, 10), PID_class(25, 0, 0, 10, 0, 0, 10)},
    {PID_class(0.01f, 0, 0, 1, 0, 0, 1), PID_class(2, 0, 0, 1000, 0, 0, 1000)},
    {PID_class(0.01f, 0, 0, 1, 0, 0, 1), PID_class(2, 0, 0, 1000, 0, 0, 1000)}};

void test_set2param_task(void) // OK!
{
    // while1

    // if (1) /*反馈，使能切换*/
    // {
    //     if (Pitch_088_state == BMI088_OK)
    //     {
    //         pid_test2p_ctrl.ref_type = IMU_TYPE;
    //         if (0) /*imu_pid参数切换*/
    //         {
    //             pid_test2p_ctrl.param_type = 0; // 正常
    //         }
    //         else if (1)
    //         {
    //             pid_test2p_ctrl.param_type = 1; // 自瞄
    //         }
    //         else if (2)
    //         {
    //             pid_test2p_ctrl.param_type = 2; // 单环
    //         } //...
    //     }
    //     else
    //     {
    //         pid_test2p_ctrl.ref_type = MOTOR_TYPE;
    //         if (0) /*motor_pid参数切换*/
    //         {
    //             pid_test2p_ctrl.param_type = 0; // 吊射
    //         }
    //         else if (1)
    //         {
    //             pid_test2p_ctrl.param_type = 1; // imu飞了的保底参数
    //         }
    //         else if (2)
    //         {
    //             pid_test2p_ctrl.param_type = 2; // 速度环
    //         } //...
    //     }
    //     pid_test2p_ctrl.protect_flag = 0;
    // }
    // else
    // {
    //     pid_test2p_ctrl.protect_flag = 1;
    // }
    pid_set2param_controler(pid_test2p_out[0], pid_test2p_in[0], &pid_test2p_ctrl, &pid_test2p_ctrl);

    /*中断*/
    /*反馈数据输入*/
    // pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].Ref = 0; // 只操作赋值当前在用的
    // pid_test2p_in[pid_test2p_ctrl.param_type][MOTOR_TYPE].Ref = 0;
    // pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].Ref = 0;
    // pid_test2p_in[pid_test2p_ctrl.param_type][IMU_TYPE].Ref = 0;

    // pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].Goal = 0; // imu_goal() 注意，不同反馈类型的目标值函数不能混用
    // pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].Goal = 0; // motor_goal() 注意，不同反馈类型的目标值函数不能混用

    /*注意内外环用的是同个索引（对齐）*/
    // pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].PID_update_void();
    // pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].Goal = pid_test2p_out[OUT][pid_test2p_ctrl.ref_type].OUT_PID;
    // pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].PID_update_void();
    // test_M.out = pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].OUT_PID;
}
void test_set2param_it(void)
{
    /*法4反馈传入
        格式可以和法1一样*/
    pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].Ref = -IMU.realAngle.yaw;
    pid_test2p_in[pid_test2p_ctrl.param_type][IMU_TYPE].Ref = -IMU.Anglespeed.Deal_yaw;

    /*格式1-能跑所有的pid*/
    pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0f); // imu_goal() 注意，不同反馈类型的目标值函数不能混用
    pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].PID_update_void();

    pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].Goal = pid_test2p_out[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].OUT_PID;
    pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].PID_update_void();
    test_M.out = pid_test2p_in[pid_test2p_ctrl.param_type][pid_test2p_ctrl.ref_type].OUT_PID;

    /*格式2-能跑imu/motor的pid，分开写提高实时性*/
    if (pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].enable_flag)
    {
        pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0f);
        pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].PID_update_void();
        pid_test2p_in[pid_test2p_ctrl.param_type][IMU_TYPE].Goal = pid_test2p_out[pid_test2p_ctrl.param_type][IMU_TYPE].OUT_PID;
        pid_test2p_in[pid_test2p_ctrl.param_type][IMU_TYPE].PID_update_void();
        test_M.out = pid_test2p_in[pid_test2p_ctrl.param_type][IMU_TYPE].OUT_PID;
    }
}
void test_set2param_can(void)
{
    /*法4反馈传递值*/
    pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].Ref = test_M.mang_inf; // 只操作赋值当前在用的
    pid_test2p_in[pid_test2p_ctrl.param_type][MOTOR_TYPE].Ref = test_M.sp;

    /*格式2-能跑imu/motor的pid，分开写提高实时性*/
    if (pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].enable_flag)
    {
        pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0f);
        pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].PID_update_void();
        pid_test2p_in[pid_test2p_ctrl.param_type][MOTOR_TYPE].Goal = pid_test2p_out[pid_test2p_ctrl.param_type][MOTOR_TYPE].OUT_PID;
        pid_test2p_in[pid_test2p_ctrl.param_type][MOTOR_TYPE].PID_update_void();
        test_M.out = pid_test2p_in[pid_test2p_ctrl.param_type][MOTOR_TYPE].OUT_PID;
    }
}
/*END*/

/*功能三示例*/
const uint8_t tloop_paramnum = 2;
const uint8_t tloop_refnum = 2;
PID_Ctrl_Index pid_loop_ctrl = {Param_1, IMU_TYPE, 2, 2, 1}; // 可实现[多少环pid][反馈数据个数]选择
PID_class pid_testloop[tloop_paramnum][tloop_refnum] = {
    // imu，motor
    // gyro，msp
    {PID_class(1, 0, 0, 10, 0, 0, 10), PID_class(25, 0, 0, 10, 0, 0, 10)},
    {PID_class(0.01f, 0, 0, 1, 0, 0, 1), PID_class(2, 0, 0, 1000, 0, 0, 1000)}};
void test_pid_setloop_controler_task(void)
{
    /*法1什么都不动，适合提高极限实时性（反馈数据立马算pid，不同反馈数据放不同处。写两次，每处都是固定的）*/
    // 在can接收中断
    // pid_testloop[OUT][MOTOR_TYPE].Ref = 0;
    // pid_testloop[IN][MOTOR_TYPE].Ref = 0;

    // pid_testloop[OUT][MOTOR_TYPE].Goal = 0; // 不同目标值传入
    // pid_testloop[OUT][MOTOR_TYPE].PID_update_void();
    // pid_testloop[IN][MOTOR_TYPE].Goal = pid_testloop[OUT][MOTOR_TYPE].OUT_PID;
    // pid_testloop[IN][MOTOR_TYPE].PID_update_void();
    // pid_testloop[IN][MOTOR_TYPE].OUT_PID;
    // // 在imu的定时器中断
    // pid_testloop[OUT][IMU_TYPE].Ref = 0;
    // pid_testloop[IN][IMU_TYPE].Ref = 0;

    // pid_testloop[OUT][IMU_TYPE].Goal = 0; // 不同目标值传入
    // pid_testloop[OUT][IMU_TYPE].PID_update_void();
    // pid_testloop[IN][IMU_TYPE].Goal = pid_testloop[OUT][MOTOR_TYPE].OUT_PID;
    // pid_testloop[IN][IMU_TYPE].PID_update_void();
    // pid_testloop[IN][IMU_TYPE].OUT_PID;

    // /*法2一套代码使用不同反馈数据，简洁*/
    // // 中断
    // pid_testloop[OUT][MOTOR_TYPE].Ref = 0;
    // pid_testloop[IN][MOTOR_TYPE].Ref = 0;
    // pid_testloop[OUT][IMU_TYPE].Ref = 0;
    // pid_testloop[IN][IMU_TYPE].Ref = 0;

    // pid_testloop[OUT][pid_loop_ctrl.ref_type].Goal = 0; // 选择使用的pid参数
    // pid_testloop[OUT][pid_loop_ctrl.ref_type].PID_update_void();
    // pid_testloop[IN][pid_loop_ctrl.ref_type].Goal = pid_testloop[OUT][pid_loop_ctrl.ref_type].OUT_PID;
    // pid_testloop[IN][pid_loop_ctrl.ref_type].PID_update_void();
    // pid_testloop[IN][pid_loop_ctrl.ref_type].OUT_PID;
    // // 法1法2通用while1
    // if (1)
    // {
    //     if (Pitch_088_state == BMI088_OK)
    //     {
    //         pid_loop_ctrl.ref_type = IMU_TYPE;
    //     }
    //     else
    //     {
    //         pid_loop_ctrl.ref_type = MOTOR_TYPE;
    //     }
    //     pid_loop_ctrl.protect_flag = 0;
    // }
    // else
    // {
    //     pid_loop_ctrl.protect_flag = 1;
    // }
    pid_setloop_controler(pid_testloop[0], &pid_loop_ctrl);
}
void test_pid_setloop_controler_imu(void)
{
    // 懒得做一样的
}
void test_pid_setloop_controler_can(void)
{
    // 懒得做一样的
}
#pragma endregion

#pragma region /*摩擦轮*/
/*掉线状态，启动但堵转状态，正常启动闭环*/
MOTOR_RM MCL_R(0x207, &CAN_1);
MOTOR_RM MCL_L(0x208, &CAN_1);
MOTOR_DM MCLDM_L(0x5, 0x15, &CAN_1);
MOTOR_DM MCLDM_R(0x6, 0x16, &CAN_1);
// P12.5,V200,T10
float MCL_R_sp = 0, MCL_L_sp = 0;
uint16_t MCL_targe_sp = MCL_SP;
// 发射机构电机在线标志
const uint8_t mcl_param_num = 1;
PID_Ctrl_Index MCL_ctrl_Index = {MOTOR_TYPE, Param_1, mcl_param_num, 1, 1};      // 摩擦轮闭环选择器
PID_class MCL_R_sp_pid[mcl_param_num] = {PID_class(15, 0, 0, 9000, 0, 0, 9000)}; // 掉速掉-350
PID_class MCL_L_sp_pid[mcl_param_num] = {PID_class(15, 0, 0, 9000, 0, 0, 9000)};
uint8_t MCL_4_motorflag = 0;           // 四个摩擦轮电机都更新完一次再发送
uint8_t MCL_ON_flag = DISABLE_e;       // 目标摩擦轮开启标志
uint8_t MCL_Normal_Onflag = 0;         // 检测摩擦轮堵转
Slow mcl_sp_s(8000, 8000, 8000, 1000); // 以右摩擦轮为准，速度单位化1s加8k
int8_t mcl_dir_init = -1;              // 摩擦轮转向，1正转，-1反转
uint8_t MCLBP_On = 1;                  // 摩擦轮拨盘一起关debug
uint8_t MCL_Task_en = 1;               // 摩擦轮关闭拨盘debug

float last_mcl_sp = Bullet_Speed;
void MCL_task(void)
{
    /*掉线保护,调试模式*/
    uint8_t protect = 0;
    uint8_t slow_sp_init = 0;
    if (!MCL_L.online || !MCL_R.online || !MCL_Task_en || !MCLBP_On)
    {
        protect = 1;
        slow_sp_init = 1;
    }

    if (YKmode == DI_PAN_H_MODE) /*保护*/
    {
        protect = 1;
        slow_sp_init = 1;
    }

    /*保护执行*/
    if (protect)
    {
        MCL_ctrl_Index.protect_flag = 1;
        MCL_R.out = 0;
        MCL_L.out = 0;
    }
    else
    {
        MCL_ctrl_Index.protect_flag = 0;
    }
    /*目标斜坡初始化*/
    if (slow_sp_init)
    {
        mcl_sp_s.init_S(mcl_dir_init * MCL_R.sp); /*不闭环时斜坡当然要复位*/
    }

    /*转速启停开关*/
    if (!protect && YKmode == ZHAN_DOU_MODE || (abs(YK.yaogan.ch1) > 600 && (YKmode == DIAO_SHE_MODE || YKmode == SHANG_XIA_MODE || YKmode == DAN_YUN_TAI_MODE))) /*转速启停*/
    {
        MCL_ON_flag = 1;
    }
    else
    {
        MCL_ON_flag = 0;
    }

    if (MCL_ON_flag) /*调摩擦轮转速功能*/
    {
        mcl_sp_s.targe = MCL_SP;
    }
    else
    {
        mcl_sp_s.targe = 0;
    }

    /*检测摩擦轮堵转*/
    if (MCL_ON_flag && (MCL_L.sp < 1000 || MCL_R.sp < 1000) && fabs(MCL_R_sp_pid[MCL_ctrl_Index.param_type].OUT_PID) + fabs(MCL_L_sp_pid[MCL_ctrl_Index.param_type].OUT_PID) > 9000)
    {
        MCL_Normal_Onflag = 0;
    }
    else
    {
        MCL_Normal_Onflag = 1;
    }

    /*根据弹速自动调整摩擦轮转速*/
    //
    /*弹丸跳变，用发给自瞄的数据，判断跳变沿*/
    // CP.ext_shoot_data_t.bullet_speed
    if (MCL_ON_flag)
    {
        if (Bullet_Speed != last_mcl_sp) // 前后两次弹速不一样，说明发射了一发子弹
        {
            if (Bullet_Speed < 24.55f && Bullet_Speed > 20)
            {
                MCL_SP += 10;
                MCL_SP = LIMIT(MCL_SP, 5990, 6280);
            }
            else if (Bullet_Speed > 24.85f)
            {
                MCL_SP -= 10;
                MCL_SP = LIMIT(MCL_SP, 5990, 6280);
            }
        }
        last_mcl_sp = Bullet_Speed;
    }

    pid_setref_controler(MCL_R_sp_pid, &MCL_ctrl_Index); // 1级处理，放最后
    pid_setref_controler(MCL_L_sp_pid, &MCL_ctrl_Index);
}

float MCL_adaption_sp(uint16_t targe_sp, float ref_sp)
{
    /*需要更新一次弹速再调用*/
    if (ref_sp > 24.85f)
    {
        targe_sp -= 50;
    }
    else if (ref_sp < 24.45f)
    {
        targe_sp += 50;
    }
    return (float)targe_sp;
}

#pragma endregion

#pragma region /*Notch*/
LP P_DY_LP(1000, 80);
// NotchFilter gyroNotch_P68(1000, 68.39f, 15);
// NotchFilter gyroNotch_P136(1000, 136.3f, 30);
NotchFilter gyroNotch_PRPM(1000, MCL_SP / 60.0f, 18);
// NotchFilter gyroNotch_PRPM2(1000, MCL_SP / 60.0f, 28);

// NotchFilter gyroNotch_P24(1000, 23.94f, 20);
// NotchFilter gyroNotch_P46(1000, 46.21f, 15);

NotchFilter gyroNotch_Y(1000, MCL_SP / 60.0f, 27); // 20%
// NotchFilter gyroNotch_YY(1000, MCL_SP / 60.0f, 27);        // 20%

NotchFilter gyroNotch_Y2(1000, (MCL_SP * 2 / 60.0f), 42); // 50%

void IMU_NF(void)
{
}
void Dynamic_Notach_Param_Task(void)
{
    /*还可固定BW或者Q*/
    float N_f0 = LIMIT((((abs(MCL_L.sp) + abs(MCL_R.sp)) / 2.0f) / 60.0f),
                       ((MCL_SP - 500) / 60.0f),
                       (9600.0f / 60.0f));
    gyroNotch_PRPM.F0 = N_f0;
    gyroNotch_PRPM.BW_Set(6);
    gyroNotch_Y.F0 = N_f0;
    gyroNotch_Y.BW_Set(4);
    gyroNotch_Y2.F0 = N_f0 * 2.0f;
    gyroNotch_Y2.BW_Set(5);
    // gyroNotch_Y2.BW_Set(6); // 定频宽确定Q(MCL_L).sp-(MCL_R).sp
    // gyroNotch_PRPM.BW_Set(4);
    IMU.gyroNotchX->F0 = N_f0;
    IMU.gyroNotchY->F0 = N_f0;
    IMU.gyroNotchZ->F0 = N_f0;

    IMU.gyroNotchX->BW_Set(5);
    IMU.gyroNotchY->BW_Set(4.2f);
    IMU.gyroNotchZ->BW_Set(3.5f);
    IMU.gyroNotchX->NF_update();
    IMU.gyroNotchY->NF_update();
    IMU.gyroNotchZ->NF_update();

    gyroNotch_PRPM.NF_update();

    gyroNotch_Y.NF_update();
    gyroNotch_Y2.NF_update();
}
#pragma endregion

#pragma region /*YAW*/

// mang 逆时针300-顺时针4540//186deg
uint16_t Y_limit_Rmang = 6470; // 以此为定位点
uint16_t Y_limit_Lmang = 460;

float Y_MIN_MAX_ANGLE = (Y_limit_Rmang - Y_limit_Lmang) * 360 / 8192.0f; // 可活动角度
float YLIMIT = 0;                                                        // 当前可活动范围

Slow YAW_Slow(150, 150, 150, 1000.0f); // 初始化斜坡对象

MOTOR_RM YAW_M(0x206, &CAN_2);
const uint8_t yaw_ref_num = 2;
const uint8_t yaw_param_num = 1;
PID_Ctrl_Index Y_ctrl_index = {Param_1, IMU_TYPE, yaw_param_num, yaw_ref_num, 1};
PID_class Y_outpid[yaw_param_num][yaw_ref_num] = {
    {PID_class(10, 0, 0, 10000, 0, 0, 10000), PID_class(60, 0, 0, 10000, 0, 0, 10000)}};
PID_class Y_inpid[yaw_param_num][yaw_ref_num] = {
    {PID_class(10, 0, 0, 5000, 0, 0, 5000), PID_class(100, 0, 0, 30000, 0, 0, 30000)}};

// C:80,20MS
// SMC YawSMC(80, 80, 0, 0.035f, 16000, 0.5f, 1);
SMC YawSMC(63, 63, 0, 0.035f, 16000, 0.5f, 1); //
// SMC YawSMC(33, 33, 0, 0.035f, 16000, 0.5f, 1);

SMC Yaw_ZM_SMC(60, 110, 0, 0.1f, 16000, 0.6f, 1);
// SMC YawSMC1(0.000000000001f, 0.000000000002f, 0, 0, 2.0f, 0.01f, 0.0000000000001f);

// 滑模面斜率,类似p,//趋近率增益,类似D//初始目标值//误差下限//输出最大值//估计惯量//切换增益,边界层厚度
float Yaw_goal = 0;
float alp = 0;
float cutoff = 80;
uint8_t YAW_SMC_or_PID = 1; // 1SMC,0PID

void YAW_PID_task(void)
{
    /*纯位置闭环*/
    /*内角速度外编码值*/
    /*内外imu*/
    if (YKmode <= PROTECT_MODE || !YAW_M.online)
    {
        Y_ctrl_index.protect_flag = 1;
    }
    else
    {
        Y_ctrl_index.protect_flag = 0;
    }
    /*最终输出置0*/
    if (Y_ctrl_index.protect_flag)
    {
        // YAW_M.out = 0;
        YAW_M.en = 0;
    }
    else
    {
        YAW_M.en = 1;
    }

    /*闭环方式*/
    if (IMU.state == BMI088_OK)
    {
        set_ref_type(&Y_ctrl_index, IMU_TYPE); // m
    }
    else
    {
        set_ref_type(&Y_ctrl_index, MOTOR_TYPE);
    }
    pid_set2param_controler(Y_outpid[0], Y_inpid[0], &Y_ctrl_index, &Y_ctrl_index);
}
void YAW_SMC_task(void)
{
    if (YKmode <= PROTECT_MODE || !YAW_M.online)
    {
        Y_ctrl_index.protect_flag = 1;
    }
    else
    {
        Y_ctrl_index.protect_flag = 0;
    }

    if (Y_ctrl_index.protect_flag)
    {
        YAW_M.en = 0;
        Yaw_goal = IMU.realAngle.yaw;
        YawSMC.Reset(IMU.realAngle.yaw);
        Yaw_ZM_SMC.Reset(IMU.realAngle.yaw);
        YAW_Slow.init_S(IMU.realAngle.yaw);
        YAW_Slow.init_T(IMU.realAngle.yaw);
    }
    else
    {
        YAW_M.en = 1;
        if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        {
            YAW_Slow.en = 1;
        }
        else
        {
            YAW_Slow.en = 0;
        }
    }

    /*IMU g了切回pid 编码值闭环*/
    if (IMU.state != BMI088_OK)
    {
        set_ref_type(&Y_ctrl_index, MOTOR_TYPE);
        set_param_type(&Y_ctrl_index, Param_1);
        YAW_SMC_or_PID = 0; // 切PID
    }
}

void YAW_Task(void)
{
    alp = Y_outpid[Y_ctrl_index.param_type][Y_ctrl_index.ref_type].get_cutoff_freq(cutoff, 1000);
    if (YAW_SMC_or_PID)
    {
        YAW_SMC_task();
    }
    else
    {
        YAW_PID_task();
    }
}
void RUN6020(void)
{
    YAW_M.out = YK.yaogan.ch0 * 20;
}

// double J = 0;
// const double Kt = 0.741;
// double Acc = 0;
// double Torque = 0;
// double Iq = 0;
// double loss = 0;
// double last_sp = 0;
int16_t T_2_6020U(float torque) // SMC控制输出量u为力矩，单位化，方便参数（效果）保持一致
{
    float value = torque * 7370.22f; // 1nm=7370.22mNm，单位化后就是这个数值
    int16_t i_value = LIMIT(value, -16000, 16000);
    return i_value;
}
void Test_J(void)
{
    // if (abs(YAW_M.sp) > 150)
    // {
    //     loss = 1; // 转速过大，恒力力矩下降，不足以维持额定力矩1.2nm（手册）
    // }
    // if (testout == 0)
    // {
    //     testout = 1;
    // }
    // Iq = testout * 0.00006103515625 * 3.0; // 5462
    // Torque = Kt * Iq;

    // Acc = (YAW_M.sp - last_sp) * 2.0 * 3.14159265358979323846 * 0.0166666666666667 * 1000.0;
    // if (Acc < 0.00000000001)
    // {
    //     Acc = -1;
    // }
    // J = Torque / Acc;
    // last_sp = YAW_M.sp;
}
void YAW_motor_pid(void)
{
    Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].Ref = YAW_M.mang_inf;
    Y_inpid[Y_ctrl_index.param_type][MOTOR_TYPE].Ref = YAW_M.sp;
    if (Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].enable_flag)
    {
        Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0F);
        // Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].Goal +=YK.lp_shubiao_x;
        Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].PID_update_void();
        Y_inpid[Y_ctrl_index.param_type][MOTOR_TYPE].Goal = Y_outpid[Y_ctrl_index.param_type][MOTOR_TYPE].OUT_PID;
        Y_inpid[Y_ctrl_index.param_type][MOTOR_TYPE].PID_update_void();
        YAW_M.out = Y_inpid[Y_ctrl_index.param_type][MOTOR_TYPE].OUT_PID;
    }
}
float Yaw_goal1;
float YAW_Angle_Limit(float goal)
{
    // if (YAW_M.mang > Y_limit_Rmang && YK.lp_shubiao_x > 0) // 顺时针限位
    // {
    //     YK.lp_shubiao_x = 0;
    // }
    // else if (YAW_M.mang < Y_limit_Lmang && YK.lp_shubiao_x < 0)
    // {
    //     YK.lp_shubiao_x = 0;
    // }
    // YLIMIT = ((Y_limit_Rmang - YAW_M.mang) * 360 / 8192.0f); // 当前可活动范围
    YLIMIT = IMU.realAngle.yaw - ((Y_limit_Rmang - YAW_M.mang) * 360 / 8192.0f);
    goal = LIMIT(goal, YLIMIT, (YLIMIT + Y_MIN_MAX_ANGLE));
    // Rad2Angle(Yaw_goal1);dyn_max-IMU.realAngle.yaw+YLIMIT
    return goal;
}
void YAW_imu_pid(void)
{
    /*法4反馈传入
        格式可以和法1一样*/
    Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Ref = IMU.realAngle.yaw;
    Y_inpid[Y_ctrl_index.param_type][IMU_TYPE].Ref = IMU.Anglespeed.Deal_yaw;
    // YK.low_pass_filter((float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -660, 660) / 660.0F / 3.0f), &YK.lp_shubiao_x, YK.lp_x_k);

    /*格式2-能跑imu/motor的pid，分开写提高实时性*/
    if (Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].enable_flag)
    {
        // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal -= (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0F);
        Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal -= YK.lp_shubiao_x;
        Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].PID_update_void();
        Y_inpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].OUT_PID;
        Y_inpid[Y_ctrl_index.param_type][IMU_TYPE].PID_update_void();
        YAW_M.out = -Y_inpid[Y_ctrl_index.param_type][IMU_TYPE].OUT_PID;
    }
}
void Yaw_imuSMC(void) // Yaw轴PID
{

    // if (ZM_Status && (Control_Mode.Control_Small_Gimbal_Mode == Gimbal_Shoot ||
    //                   Control_Mode.Control_Small_Gimbal_Mode == Gimbal_Automatic)) // 自瞄状态
    if (!Y_ctrl_index.protect_flag)
    {
        if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        {
            YawSMC.Target_vel = Y_Gyro_ZM; // LP插值处理,特别是自瞄
            YawSMC.Target_ACC = Y_AGyro_ZM;
            YAW_Slow.targe = YAW_Angle_Limit(Y_Angle_ZM); // 限位
        }
        else
        {
            YawSMC.Target_ACC = 0;
            YawSMC.Target_vel = -YK.lp_shubiao_x * 1000;
            YAW_Slow.targe = YAW_Angle_Limit(YAW_Slow.targe - YK.lp_shubiao_x); // 限位
        }
        YAW_Slow.F_slow(); // 限制最大速度
        // PitchSMC.Target_vel = P_Gyro_ZM; // LP插值处理,
        YawSMC.SMC_Tick_ZM_I(YAW_Slow.s_targe, YawSMC.Target_vel, YawSMC.Target_ACC, IMU.realAngle.yaw, gyroNotch_Y2.out);
        YAW_M.out = -YawSMC.u;

        // if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        // {
        //     // Y_Gyro_ZM，Y_AGyro_ZM需要平滑曲线

        //     // Yaw_goal = ZM_RXY.low_pass_filter(Y_Angle_ZM);
        //     Yaw_goal = YAW_Angle_Limit(ZM_RXY.low_pass_filter(Y_Angle_ZM));
        //     Yaw_ZM_SMC.SMC_Tick_ZM(Yaw_goal, Y_Gyro_ZM, 0, IMU.realAngle.yaw, gyroNotch_Y2.out);
        //     YAW_M.out = -Yaw_ZM_SMC.u;
        // }
        // else
        // {
        //     // Yaw_goal -= YK.lp_shubiao_x; // 速度
        //     float Y_Tvel = YK.lp_shubiao_x * 1000;
        //     // float Y_Tacc=YK.
        //     Yaw_goal = YAW_Angle_Limit(Yaw_goal - YK.lp_shubiao_x);
        //     ZM_RXY.Reset(IMU.realAngle.yaw);
        //     YawSMC.SMC_Tick2(Yaw_goal, IMU.realAngle.yaw, gyroNotch_Y2.out);

        //     // YAW_M.out = -T_2_6020U(YawSMC1.u);
        //     YAW_M.out = -YawSMC.u;
        // }
    }
}
void Yaw_imuCtrl(void)
{
    YK.low_pass_filter((float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -660, 660) / 660.0F / 3.0f), &YK.lp_shubiao_x, YK.lp_x_k);
    // F0 12.875HZ,10.5-13.2-14/BW3-3.5,Q3-4
    gyroNotch_Y.process(IMU.Anglespeed.Deal_yaw);
    gyroNotch_Y2.process(gyroNotch_Y.out);

    if (YAW_SMC_or_PID)
    {
        Yaw_imuSMC();
    }
    else
    {
        YAW_imu_pid(); // 1.15us
    }
}
#pragma endregion

#pragma region /*PITCH*/
// 上-0.72，下-1.72
//  4310 发送0x01，接收0x11
//  3507 发送0x04，接收0x14
Slow PITCH_Slow(75, 75, 150, 1000.0f); // 初始化斜坡对象

MOTOR_DM PITCH_M(0x4, 0x14, &CAN_2);

float Pitch_goal = 0;
// SMC PitchSMC(70, 80, 0.01f, 0.035f, 16000, 0.6f, 1);
SMC PitchSMC(90, 90, 0.01f, 0.035f, 16000, 0.6f, 1); //
// SMC PitchSMC(45, 45, 0.01f, 0.035f, 16000, 0.6f, 1);

// SMC PitchSMC(70, 80, 0.01f, 0.035f, 16000, 0.7f, 1);
// SMC Pitch_ZM_SMC(70, 80, 0, 0.035f, 7500, 1, 1);

// #define DM4310
#define DM3507
const uint8_t pitch_ref_num = 2;
const uint8_t pitch_param_num = 4;
PID_Ctrl_Index P_ctrl_index = {Param_4, MOTOR_TYPE, pitch_param_num, pitch_ref_num, 1};
/*M1,IMU1*/
PID_class P_outpid[pitch_param_num][pitch_ref_num] = {
    {PID_class(10, 0, 0, 10, 0, 0, 10), PID_class(40, 0, 0, 350, 0, 80, 350, 0.2f)}, // kp太软，kd太大,上场状态重量参数PID_class(10, 0, 0, 10, 0, 0, 10), PID_class(40, 0, 1500, 350, 0, 80, 350, 0.2f)
    {PID_class(10, 0, 0, 10, 0, 0, 10), PID_class(40, 0, 0, 350, 0, 0, 350, 0.2f)},
    {PID_class(10, 0, 0, 10, 0, 0, 10), PID_class(60, 0.1f, 0, 350, 120, 80, 350, 0.2f, 8)},
    {PID_class(10, 0, 0, 10, 0, 0, 10), PID_class(60, 0.1f, 800, 350, 120, 80, 350, 0.2f, 8)} // zm

};
PID_class P_inpid[pitch_param_num][pitch_ref_num] = {
    {PID_class(0.25f, 0, 0, 2, 0, 0, 2), PID_class(0.0075f, 0, 0, 3.5f, 0, 0, 3.5f, 0.015f)}, // PID_class(0.25f, 0, 0, 2, 0, 0, 2), PID_class(0.0075f, 0, 0.3, 3.5f, 0, 0, 3.5f, 0.015f)
    {PID_class(0.25f, 0, 0, 2, 0, 0, 2), PID_class(0.006f, 0, 0, 3.5f, 0, 0, 3.5f, 0.015f)},
    {PID_class(0.25f, 0, 0, 2, 0, 0, 2), PID_class(0.0075f, 0, 0.2f, 3.5f, 0, 0.15f, 3.5f, 0.015f)}, // now
    {PID_class(0.25f, 0, 0, 2, 0, 0, 2), PID_class(0.0075f, 0, 0.2f, 3.5f, 0, 0.15f, 3.5f, 0.015f)}, // now

};
uint8_t PITCH_SMC_or_PID = 1; /// 1SMC

void PITCH_SMC_task(void)
{
    if (YKmode <= PROTECT_MODE || !PITCH_M.online || PITCH_M.ERR != 1)
    {
        P_ctrl_index.protect_flag = 1;
    }
    else
    {
        P_ctrl_index.protect_flag = 0;
    }

    if (P_ctrl_index.protect_flag)
    {
        Pitch_goal = IMU.realAngle.pitch;
        PitchSMC.Reset(IMU.realAngle.pitch);
        //        Pitch_ZM_SMC.Reset(IMU.realAngle.pitch);
        PITCH_Slow.init_T(IMU.realAngle.pitch);
        PITCH_Slow.init_S(IMU.realAngle.pitch);

        PITCH_M.en = 0;
    }
    else
    {
        PITCH_M.en = 1;
        if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        {
            PITCH_Slow.en = 1;
        }
        else
        {
            PITCH_Slow.en = 0;
        }
    }

    /*IMU g了切回pid 编码值闭环*/
    if (IMU.state != BMI088_OK)
    {
        set_ref_type(&P_ctrl_index, MOTOR_TYPE);
        set_param_type(&P_ctrl_index, Param_1);
        PITCH_SMC_or_PID = 0; // 切PID
    }
}
void PITCH_PID_task(void)
{

    if (YKmode <= PROTECT_MODE)
    {
        P_ctrl_index.protect_flag = 1;
    }
    else
    {
        P_ctrl_index.protect_flag = 0;
    }
    /*最终输出置0*/
    if (P_ctrl_index.protect_flag || !PITCH_M.online || PITCH_M.ERR != 1)
    {
        PITCH_M.en = 0;
    }
    else
    {
        PITCH_M.en = 1;
    }

    /*闭环方式*/
    if (IMU.state == BMI088_OK)
    {
        set_ref_type(&P_ctrl_index, IMU_TYPE); // m
    }
    else
    {
        set_ref_type(&P_ctrl_index, MOTOR_TYPE);
    }

    pid_set2param_controler(P_outpid[0], P_inpid[0], &P_ctrl_index, &P_ctrl_index);
}

void PITCH_Task(void)
{
    if (PITCH_SMC_or_PID)
    {
        PITCH_SMC_task();
    }
    else
    {
        PITCH_PID_task();
    }
}

//-1.93云台上，-2.96云台下-36.6deg，-2.27水平，弧度
// IMU角度13，-40
float P_limit_upmang = -0.75f;  // 62.457md;56.37d
float P_limit_downmang = -1.7f; //-1.78m,-43.87d;-0.69m,12.5d

float P_0deg_mang = -0.9328f; // 限位定位点水平面
float PLIMIT;
float dyn_max;
float dyn_min;
float P_soft_uplimit = 12;
float P_soft_downlimit = -42;

float pitch_Angle_Limit(float targe)
{
    // if (PITCH_M.mang > P_limit_upmang && YK.lp_shubiao_y > 0) // 云台上限位
    // {
    //     YK.lp_shubiao_y = 0;
    // }
    // else if (PITCH_M.mang < P_limit_downmang && YK.lp_shubiao_y < 0)
    // {
    //     YK.lp_shubiao_y = 0;
    // }
    PLIMIT = Rad2Angle(PITCH_M.mang - P_0deg_mang) * 0.9025f;  // 当前可活动范围，编码值转换度数与imu度数约为0.9倍
    dyn_max = IMU.realAngle.pitch - PLIMIT + P_soft_uplimit;   // 上软件限位
    dyn_min = IMU.realAngle.pitch - PLIMIT + P_soft_downlimit; // 下软件限位
    targe = LIMIT(targe, dyn_min, dyn_max);
    return targe;
}

void PITCH_motor_pid(void)
{
    /*法4反馈传递值*/
    P_outpid[P_ctrl_index.param_type][MOTOR_TYPE].Ref = PITCH_M.mang_inf; // 只操作赋值当前在用的
    P_inpid[P_ctrl_index.param_type][MOTOR_TYPE].Ref = PITCH_M.sp;

    /*格式2-能跑imu/motor的pid，分开写提高实时性*/
    if (P_outpid[P_ctrl_index.param_type][MOTOR_TYPE].enable_flag)
    {

        P_outpid[P_ctrl_index.param_type][MOTOR_TYPE].Goal += ((float)YK.yaogan.ch3 / 300.0f / 1000.0f + (float)LIMIT(YK.shubiao.y, -500, 500) / 100.0f / 1000.0f);
        P_outpid[P_ctrl_index.param_type][MOTOR_TYPE].PID_update_void();
        P_inpid[P_ctrl_index.param_type][MOTOR_TYPE].Goal = P_outpid[P_ctrl_index.param_type][MOTOR_TYPE].OUT_PID;
        P_inpid[P_ctrl_index.param_type][MOTOR_TYPE].PID_update_void();
        PITCH_M.mit_torq = P_inpid[P_ctrl_index.param_type][MOTOR_TYPE].OUT_PID;
    }
}
void PITCH_imu_pid(void)
{
    /*法4反馈传入
        格式可以和法1一样*/
    P_outpid[P_ctrl_index.param_type][IMU_TYPE].Ref = IMU.realAngle.pitch;
    // P_inpid[P_ctrl_index.param_type][IMU_TYPE].Ref = IMU.Anglespeed.Deal_pitch;
    // gyroNotch_PRPM.process(IMU.Anglespeed.Deal_pitch);

    P_inpid[P_ctrl_index.param_type][IMU_TYPE].Ref = P_DY_LP.out; // 可以来个70HZ的LP

    /*格式2-能跑imu/motor的pid，分开写提高实时性*/
    if (P_outpid[P_ctrl_index.param_type][IMU_TYPE].enable_flag)
    {
        // pitch_Angle_Limit();
        // SOUT = smooth_step(SOUT, P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal, spF);
        if (SuperPower.Fire_Flag && request.zimiao_status)
        {
            // ZM_RXP.low_pass_filter(P_Angle_ZM);
            PITCH_Slow.targe = pitch_Angle_Limit(P_Angle_ZM); // 限位
            PITCH_Slow.F_slow();
            P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = PITCH_Slow.s_targe;
        }
        else
        {
            PITCH_Slow.init_S(IMU.realAngle.pitch);
            // SOUT = IMU.realAngle.pitch;
            P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal += YK.lp_shubiao_y;
            // ZM_RXP.Reset(IMU.realAngle.pitch);
        }
        // P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal += ((float)YK.yaogan.ch3 / 5000.0f + (float)LIMIT(YK.shubiao.y, -500, 500) / 1000.0f);

        P_outpid[P_ctrl_index.param_type][IMU_TYPE].PID_update_void();
        P_inpid[P_ctrl_index.param_type][IMU_TYPE].Goal = P_outpid[P_ctrl_index.param_type][IMU_TYPE].OUT_PID;
        // P_inpid[P_ctrl_index.param_type][IMU_TYPE].Goal = YK.yaogan.ch3 / 10.0f;
        P_inpid[P_ctrl_index.param_type][IMU_TYPE].PID_update_void();
        PITCH_M.mit_torq = P_inpid[P_ctrl_index.param_type][IMU_TYPE].OUT_PID; //- 0.35f;
    }
}

float gm6020to_torq(float u)
{
    float A = u / (16384.0f / 3.0f);
    float NM = A * 0.741f;
    return NM; // 4046
    // J0.001
}

void Pitch_imuSMC(void) // pitch轴PID
{
    if (!P_ctrl_index.protect_flag)
    {
        if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        {
            PitchSMC.Target_vel = P_Gyro_ZM; // LP插值处理,特别是自瞄
            PitchSMC.Target_ACC = P_AGyro_ZM;
            PITCH_Slow.targe = pitch_Angle_Limit(P_Angle_ZM); // 限位
        }
        else
        {
            PitchSMC.Target_vel = YK.lp_shubiao_y * 1000;
            PitchSMC.Target_ACC = 0;
            PITCH_Slow.targe = pitch_Angle_Limit(PITCH_Slow.targe + YK.lp_shubiao_y); // 限位
        }
        PITCH_Slow.F_slow(); // 限制最大速度
        // PitchSMC.Target_vel = P_Gyro_ZM; // LP插值处理,
        PitchSMC.SMC_Tick_ZM_I(PITCH_Slow.s_targe, PitchSMC.Target_vel, PitchSMC.Target_ACC, IMU.realAngle.pitch, gyroNotch_PRPM.out);
        PITCH_M.mit_torq = gm6020to_torq(PitchSMC.u);

        // if (SuperPower.Fire_Flag && request.zimiao_status) // 自瞄状态
        // {
        //     PITCH_Slow.targe = pitch_Angle_Limit(P_Angle_ZM); // 限位
        //     PITCH_Slow.F_slow();
        //     PitchSMC.Target_vel = P_Gyro_ZM; // LP插值处理
        //     PitchSMC.SMC_Tick_ZM_I(PITCH_Slow.s_targe, PitchSMC.Target_vel, 0, IMU.realAngle.pitch, P_DY_LP.out);
        //     PITCH_M.mit_torq = gm6020to_torq(Pitch_ZM_SMC.u);
        //     //		 PitchSMC.SMC_Tick2(Pitch_goal,BMI088_Pitch.realAngle.pitch,BMI088_Pitch.Anglespeed.Deal_pitch);
        // }
        // else
        // {
        //     // 0.0174533f
        //     // Pitch_goal += YK.lp_shubiao_y;
        //     PITCH_Slow.s_targe = pitch_Angle_Limit(PITCH_Slow.s_targe + YK.lp_shubiao_y);
        //     PitchSMC.Target_vel = YK.lp_shubiao_y * 1000;
        //     PitchSMC.SMC_Ticck_ZM_I(PITCH_Slow.s_targe, PitchSMC.Target_vel, 0, IMU.realAngle.pitch, P_DY_LP.out);

        //     PITCH_M.mit_torq = gm6020to_torq(PitchSMC.u);
        //     // PITCH_Slow.init_S(IMU.realAngle.pitch);
        // }
    }
}
void PITCH_imuCtrl(void)
{
    YK.low_pass_filter((float)YK.yaogan.ch3 / 5000.0f + (float)LIMIT(YK.shubiao.y, -660, 660) / 660.0f / 3.0f, &YK.lp_shubiao_y, YK.lp_y_k);

    // 111.6HZ,225.5HZ
    /**/
    // F0 12.875HZ,10.5-13.2-14/BW3-3.5,Q3-4

    // LP80
    gyroNotch_PRPM.process(IMU.Anglespeed.Deal_pitch);

    // P_DY_LP.low_pass_filter(gyroNotch_PRPM.out);
    if (PITCH_SMC_or_PID)
    {
        Pitch_imuSMC();
    }
    else
    {
        PITCH_imu_pid(); // 1.15us
    }
}

#pragma endregion

#pragma region /*BP热量管理*/

uint8_t heat_limit_en = 0; // 热量超限启停
int8_t cool_tim_start = 0; // cp接收后置0,更新数据后再开始计时

/*最低弹频=冷却速率，最高看场景*/
/*判断弹丸是否发射*/
// 拨盘编码器判断，但双发和卡弹会影响很大
// 看摩擦轮掉速，可能性，分析发射时的摩擦轮掉速特征，达到类似枪管反馈数据的效果
// 裁判系统弹速值变化判断，最直接稳妥，但调试需要裁判系统
// 裁判系统射频0x207数据可能能用
// 看cp允许发弹量，需裁判系统
// void heat_buf_deal(void)
// {
//     /*热量限制条件*/
//     if (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit - 5 > now_heat)
//     {
//     }

//     /*判断弹丸是否发射*/
//     static float last_bullet_sp = 0;
//     if (CP_Online && CP.ext_shoot_data_t.bullet_speed != last_bullet_sp) // 前后两次弹速不一样，说明发射了一发子弹
//     {
//         now_heat += 10;
//         fadan_cnt++;
//     }
//     last_bullet_sp = CP.ext_shoot_data_t.bullet_speed;
// }
// void shoot_cooling_Task(void) // 热量冷却
// {
//     // float freq = 10;
//     // static uint16_t cool_tim = 0; // 防止cp更新后，本地也更新导致数据错误
//     // /*10hz低频本地冷却*/
//     // if (!cool_tim_start)
//     // {
//     //     cool_tim_start = 1;
//     //     cool_tim = 0;
//     // }
//     // else
//     // {
//     //     cool_tim++;
//     // }
//     // if (heat_limit_en && now_heat > 0 && cool_tim >= 160 / freq)
//     // {
//     //     cooling_rate = (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value / freq);
//     //     now_heat -= cooling_rate;
//     //     cool_tim = 0;
//     // }

//     float cooling_rate;
//     /*连续更新冷却，再给点余量，比如cp热量限制200，你限制195*/
//     if (heat_limit_en && now_heat > 0)
//     {
//         cooling_rate = (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value / 160.0f);

//         now_heat -= cooling_rate; // 否则cp更新后立马减小，微小递减是最好的
//     }
// }

// UpDown_check_class UD_YK_BoPan_Master(0);
// float BoPan_goal; // 拨盘目标值
// int KD_Flag = 0, KD_Time = 0;
// #define SHOOT_ONE_DP 36.85 // 弹频一发每秒
// UpDown_check_class UD_Now_BP_Flag_0(0), UD_Now_BP_Flag_1(0);
// float now_heat = 0;
// float Now_BP_WeiZhi = 0;
// float Heat_Receive = 0;
// int Time;
// int Shoot_Heat = 0;
// int BP_flag_0 = 0, BP_flag_1 = 0;
// void KD_Deal()
// {
//     if (BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].OUT_PID <= -9000 || KD_Flag == 1)
//     {
//         KD_Time++;
//         if (KD_Time == 200) // 卡弹检测累加到300ms
//         {
//             BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal = BP_M.mang_inf;   // 初始化目标值为当前值
//             BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal -= one_shoot_mang; // 拨盘回退1格
//             KD_Flag = 1;                                                               // 卡弹标志
//         }
//         if (KD_Time >= 400)
//         {
//             BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal = BP_M.mang_inf; // 初始化目标值为当前值
//             KD_Flag = 0;
//             KD_Time = 0;
//         }
//     }
//     else
//         KD_Time = 0;
// }
// void Shoot_Fire_BiSai_Mode_0() // 正常10/秒的热量恢复
// {
//     if (UD_Now_BP_Flag_0.updata(BP_flag_0 == 1) == UpDown_check_rising)
//     {
//         Now_BP_WeiZhi = BP_M.mang_inf;
//         Heat_Receive = 0;
//     }
//    Time++;
//    if (Time == 100)
//    {
//        if (now_heat < 260)
//            Heat_Receive += 1;
//        else
//            Heat_Receive += 0;
//        Time = 0;
//    }
//    Shoot_Heat = -((BP_M.mang_inf - Now_BP_WeiZhi) / 36859) * 10; // 36,864
//    now_heat = 260 - Shoot_Heat + Heat_Receive;
//    if (now_heat >= 260)
//        now_heat = 260;
//    if ((UD_YK_BoPan_Master.updata(YK.yaogan.ch0 < -600) == UpDown_check_rising) && YK.yaogan.ch1 > 600)
//        BoPan_goal -= one_shoot_mang;
//    if (YK.yaogan.ch0 >= 600 && YK.yaogan.ch1 > 600 && now_heat > 0)
//    {
//        if (now_heat >= 200)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 10;
//        }
//        else if (now_heat >= 150 && now_heat < 200)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 8;
//        }
//        else if (now_heat >= 100 && now_heat < 150)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 6;
//        }
//        else if (now_heat >= 60 && now_heat < 100)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 4;
//        }
//        else if (now_heat >= 40 && now_heat < 60)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 2;
//        }
//        else if (now_heat >= 20 && now_heat < 40)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 1;
//        }
//        else
//        {
//            BoPan_goal = BP_M.mang_inf;
//        }
//    }
// }
// void Shoot_Fire_BiSai_Mode_1() // 攻击姿态下的90/秒热量恢复
// {
//    if (UD_Now_BP_Flag_1.updata(BP_flag_1 == 1) == UpDown_check_rising)
//    {
//        Now_BP_WeiZhi = BP_M.mang_inf;
//        Heat_Receive = 0;
//    }
//    Time++;
//    if (Time == 100)
//    {
//        if (now_heat < 260)
//            Heat_Receive += 9;
//        else
//            Heat_Receive += 0;
//        Time = 0;
//    }
//    Shoot_Heat = -((BP_M.mang_inf - Now_BP_WeiZhi) / 36859) * 10;
//    now_heat = 260 - Shoot_Heat + Heat_Receive;
//    if (now_heat >= 260)
//        now_heat = 260;
//    if ((UD_YK_BoPan_Master.updata(YK.yaogan.ch0 < -600) == UpDown_check_rising) && YK.yaogan.ch1 > 600)
//        BoPan_goal -= one_shoot_mang;
//    if (YK.yaogan.ch0 >= 600 && YK.yaogan.ch1 > 600 && now_heat > 0)
//    {
//        if (now_heat >= 150)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 20;
//        }
//        else if (now_heat >= 50 && now_heat < 150)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 15;
//        }
//        else if (now_heat >= 20 && now_heat < 50)
//        {
//            BoPan_goal -= SHOOT_ONE_DP * 9;
//        }
//        else
//        {
//            BoPan_goal = BP_M.mang_inf;
//        }
//    }
// }
// void Shoot_Fire_test()
//{
//    now_heat = 260;
//    if (KD_Flag == 0)
//    {
//        if ((UD_YK_BoPan_Master.updata(YK.yaogan.ch0 < -600) == UpDown_check_rising) && YK.yaogan.ch1 > 600)
//            BoPan_goal -= one_shoot_mang;
//        if (YK.yaogan.ch0 >= 600 && YK.yaogan.ch1 > 600)
//            BoPan_goal -= SHOOT_ONE_DP * 20;
//    }
//}
// void Gimbal_Shoot_Control() // 摩擦轮电机控制
//{
//    if (MCL_ON_flag)
//    {
//        if (YK.yaogan.ch1 > 600)
//            mcl_sp_s.targe = MCL_targe_sp;
//        else
//            mcl_sp_s.targe = 0;
//        /*-------正常测试20hz--------*/
//        if (YK.yaogan.s1 == 3 && YK.yaogan.s2 == 2)
//        {
//            KD_Deal();
//            Shoot_Fire_test();
//            BP_flag_0 = 0;
//            BP_flag_1 = 0;
//        }
//        /*-------1/3热量恢复-------*/
//        if (YK.yaogan.s1 == 2 && YK.yaogan.s2 == 2)
//        { // 1/3热量恢复
//            KD_Deal();
//            Shoot_Fire_BiSai_Mode_0();
//            BP_flag_0 = 1;
//            BP_flag_1 = 0;
//        }
//        /*-------3倍热量恢复--------*/
//        if (YK.yaogan.s1 == 2 && YK.yaogan.s2 == 3)
//        { // 3倍热量恢复
//            KD_Deal();
//            Shoot_Fire_BiSai_Mode_1();
//            BP_flag_0 = 0;
//            BP_flag_1 = 1;
//        }
//    }
//    else if (MCL_ON_flag) // 测试
//    {
//        if (YK.yaogan.ch1 > 600)
//            mcl_sp_s.targe = 2000;
//        else
//            mcl_sp_s.targe = 0;
//        KD_Deal();
//        Shoot_Fire_test();
//    }
//}
// void M_out_Task(void)
//{
//}

#pragma endregion

#pragma region /*BP*/
MOTOR_RM BP_M(0X205, &CAN_1);

float one_shoot_mang = (8192.0f * 36.0f / 8.0f); // 36864发射一发弹丸的编码值
const int32_t int_one_shoot_mang = 36864;        // 36864发射一发弹丸的编码值

float bp1hzsp = (8192.0f * 36.0f / 8.0f) / 8192.0f * 60.0f; // 270速度/弹频,单位匀速推弹rpm
float bp_thz = 5;                                           // 目标弹频
float bpzm_thz = 20;                                        // 底层限制20hz弹频率
float BP_tsp = bp_thz * bp1hzsp;                            // 弹频速度

float BP_intarge_sp = 0; // 内环目标速度
UpDown_check_class UD_OneShoot(0);
uint8_t YKshoot_flag = 0;     // 做单发标志的跳变沿判断
float lianfa_wait_tim = 0.2f; // 切换到连发的等待时间s
uint8_t one_shoot = 0;        // 申请单发标志
uint16_t shoot_tim = 0;       // 单环执行时间*BPsp=位置

/*内环发射时间自减，可调速*/
Slow BP_mang_slow(one_shoot_mang, one_shoot_mang, one_shoot_mang, 1000);
// Slow BPintui_tim_slow(1, 1, 2, 160); // 退弹时间
Slow BPin_tim_slow(1, 1, 1, 160); // 弹频周期

UpDown_check_class UDBP_tui(0); // 退弹检测BP_tui_flag
Slow BPwait_tim_slow(1, 1, 1, 160);

Slow BPwait20_tim_slow(1, 1, 1, 160);

uint8_t BPtui_on_flag = 1; // 启停退弹功能
// uint8_t BP_yuntui_way = 0; // 拨盘退弹方式；0快推，1匀推
bool BP_tui_flag = 0;    // 触发退弹
float BP_tui_sp = -6500; // 退单目标速度
const int16_t set_tui_tim = 320;
int16_t BP_tui_tim = set_tui_tim; // 退单时间
int16_t kdan_tim = 0;             // 卡单时间
UpDown_check_class UD_BPsp_up(0), UD_BPsp_down(0);

int8_t force_en_flag = 1; // 前馈使能

const uint8_t bp_ref_num = 1;
const uint8_t bp_param_num = 1;
PID_Ctrl_Index BP_outctrl_index = {Param_1, MOTOR_TYPE, bp_param_num, bp_ref_num, 1};
PID_Ctrl_Index BP_inctrl_index = {Param_1, MOTOR_TYPE, bp_param_num, bp_ref_num, 1};
PID_loopCtrl_Index BP_loop_t = {&BP_outctrl_index, &BP_inctrl_index, NOLOOP_SET};

/*哨兵参数，x*/
/*参数1，参数2*/
PID_class BP_outpid[bp_param_num][bp_ref_num] = {
    {PID_class(0.4f, 0, 0, 16000, 0, 0, 16000)}};
PID_class BP_inpid[bp_param_num][bp_ref_num] = {
    {PID_class(12, 0, 0, 30000, 0, 0, 30000)}};
UpDown_check_class UD_press_shoot(0);
UpDown_check_class UD_zm_shoot(0);
uint8_t bp2mcl_off = 0; // 置1关闭拨盘跟随摩擦轮启停，用于调试拨盘

void bp_ch_outgoal(void)
{
    if (!BP_tui_flag && (!MCL_ON_flag && !bp2mcl_off) && YK.yaogan.ch0 >= 0)
    {
        return;
    }
    BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal += YK.yaogan.ch0;
}
void voidgoal(void)
{
    BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal = BP_mang_slow.F_slow();
}

bool heat_flag = 1; // 热量标志//无用
#define BOOST_FLAG ((uint16_t)0x0001 << 3)
int32_t BP_calc_targe = 0; // 发弹后计算下一发弹的目标位置

int32_t BP_inc;          // 反馈编码值取余%,得到距离下个发弹点的距离
int32_t targe_inc;       // 发弹目标增量
int32_t bp_err_test = 0; // 计算目标值与当前值的误差

float calc_mang = 0;
float bili = 0;
void bp_calc_goal_debug(void) /*英雄带保留预置计算目标值拨盘*/
{
    BP_inc = int_one_shoot_mang - abs(BP_M.mang_inf % int_one_shoot_mang);
    // Fadan_cnt = Motor_BP.mang_inf / int_shot;
    bp_err_test = BP_M.mang_inf % int_one_shoot_mang;

    /*test*/
    if (BP_inc < int_one_shoot_mang * 2 / 3.0f) // 距离下个发弹点太近(1.5r，33%),给他多加一个shot
    {
        targe_inc = ((int_one_shoot_mang + BP_inc));
    }
    else
    {
        targe_inc = (BP_inc); // 否则到下一个发弹点
    }
    calc_mang = BP_M.mang_inf + targe_inc;
}
void bp_calc_goal(void) /*英雄带保留预置计算目标值拨盘*/
{
    bp_calc_goal_debug();

    /**新拨盘 申请发弹，算出更改目标交给斜坡处理*/
    // if (one_shoot && heat_flag) // 发弹时，如果当前误差过大，目标不增加，可额外进行处理
    // {
    // if (BP_inc < 30000) // 距离下个发弹点太近,给他多加一个shot
    // {
    //     targe_inc = -((int_one_shoot_mang + BP_inc) + int_one_shoot_mang);
    // }8
    // else
    // {
    //     targe_inc = -(int_one_shoot_mang + BP_inc); // 否则到下一个发弹点
    // }
    bili = targe_inc / (float)int_one_shoot_mang;
    BP_mang_slow.targe = BP_M.mang_inf + targe_inc; // 算好后给到定速发弹和变速发弹决策
    // one_shoot = 0;                                  // 发弹标志清0
    // }
}

void BP_tui_deal(uint8_t en);
uint8_t bp_ctrl_mode = 0;
uint8_t LF_on = 0;
UpDown_check_class UD_LF_start(0); // 连发开始跳变

uint16_t fadan_cnt = 0;
float last_bullet_sp = 0;
uint8_t CP_shoot_en = 1; // 热量限制内允许发弹
float mini_hz = 0;

uint8_t heat_debug = 0;  // 热量限制调试
uint8_t force_bp_on = 0; // 强制拨盘发弹,应急
uint8_t BP_ON = 1;
float min_Heat_buf = 35;
float Heat_buf = min_Heat_buf; // 缓冲热量

void BP_heat_limit(void)
{
    //    CP.ext_shoot_data_t.bullet_speed当前射速
    // CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value// 每秒冷却值
    // CP.ext_power_heat_data_t.shooter_id1_17mm_cooling_heat//当前热量
    // CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit热量上限
    // CP.ext_game_status_t.game_progress比赛阶段

    /*热量，弹频限制决策*/
    // if (((YKmode == ZHAN_DOU_MODE) || (CP.ext_game_status_t.game_progress >= 4) && CP_Online) || heat_debug) /*开比赛，或者在热量调试模式下*/
    if (((YKmode == ZHAN_DOU_MODE) && CP_Online) || heat_debug) /*开比赛，或者在热量调试模式下*/
    {
        heat_limit_en = 1;
    }
    else
    {
        heat_limit_en = 0;
    }

    float cooling_rate;
    /*连续更新冷却，再给点余量，比如cp热量限制200，你限制195*/
    // if (heat_limit_en && now_heat > 0)
    if (now_heat > 0)
    {
        cooling_rate = (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value * 0.6f / 160.0f); // CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value*0.6f，系数决定0.2s内本地弹丸热量判断是否发弹的激进程度0-1
        now_heat -= cooling_rate;                                                                          // 否则cp更新后立马减小，微小递减是最好的
    }

    /**热量限制**/
    if (heat_limit_en)
    {
        /*判断弹丸是否发射*/
        if (CP.ext_shoot_data_t.bullet_speed != last_bullet_sp) // 前后两次弹速不一样，说明发射了一发子弹
        {
            now_heat += 10;
            fadan_cnt++;
        }
        last_bullet_sp = CP.ext_shoot_data_t.bullet_speed;

        /*热量限制条件*/
        if (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit - Heat_buf > now_heat)
        {
            CP_shoot_en = 1;
        }
        else
        {
            CP_shoot_en = 0;
        }
    }
    else
    {
        CP_shoot_en = 1; // 裁判系统没有接入下不限制
    }
}

/*自适应弹频*/
// 每发射1发弹丸，获得1点经验
// 对机器人造成攻击伤害：每造成1点伤害，攻击方获得4点经验,80一发
// 对前哨站装甲模块造成攻击伤害：每造成1点伤害，攻击方获得2点经验。40一发
// 对基地装甲模块造成攻击伤害：每造成2点伤害，攻击方获得1点经验，伤害为奇数时，向上取整。
// 空中机器人的等级和经验
/*等级*/ /*升级所需经验*/
// 2，550
// 3，1100
// 4，1650
// 5，2200
// 6，2750
// 7，3300
// 8，3850
// 9，4400
// 10，5000

/*空中机器人17mm发射机构属性*/
/*等级*/ /*射击热量上限*/ /*射击热量冷却速率*/

/*1*/ /*100*/ /*20*/
/*2*/ /*110*/ /*30*/
/*3*/ /*120*/ /*40*/
/*4*/ /*130*/ /*50*/
/*5*/ /*140*/ /*60*/

/*6*/ /*150*/ /*70*/
/*7*/ /*160*/ /*80*/

/*8*/ /*170*/  /*90*/
/*9*/ /*180*/  /*100*/
/*10*/ /*200*/ /*120*/

/*射击热量上限*/ /*冷却*/ /*持续*/ /*爆发1*/ /*爆发2*/ /*2倍冷却弹频爆发*/

/*10*/ /*2*/ /*4*/ /*8*/ /*8*/   /*8*/
/*11*/ /*3*/ /*5*/ /*9*/ /*9*/   /*8*/
/*12*/ /*4*/ /*6*/ /*10*/ /*10*/ /*8*/
/*13*/ /*5*/ /*7*/ /*11*/ /*12*/ /*10*/
/*14*/ /*6*/ /*8*/ /*12*/ /*13*/ /*12*/

/*15*/ /*7*/ /*9*/ /*13*/ /*15*/      /*14*/
/*16*/ /*8*/ /*10*/ /*14-8*/ /*16-8*/ /*16-8*/

/*17*/ /*9*/ /*12*/ /*15-9*/ /*18-9*/    /*18-9*/
/*18*/ /*10*/ /*14*/ /*16-10*/ /*19-10*/ /*20-10*/
/*20*/ /*12*/ /*16*/ /*20-12*/ /*20-12*/ /*20-12*/

uint8_t self_AD_en = 1; // 功能启停
uint8_t limit_MIN_HZ = 6;
float MIN_HZ = limit_MIN_HZ;
float MAX_HZ = limit_MIN_HZ + 1; // 最大弹频，20，15，10高中低三档
float AD_F = MIN_HZ;             // 因变量，自适应单频

float BP_X = 0, // 当前热量
    BP_Y = 0,   // 当前弹频率
    BP_K = 0;   // 算出弹频变化率，代入得出当前弹丸频率
float ADD_Buf = 20;
/*拨轮调弹频*/
void BP_thz_set(void)
{
    bool ifud_BP_up;
    bool ifud_BP_down;
    AD_F = bp_thz;

    if (YK.v_inf > 3000)
    {
        ifud_BP_up = 1;
    }
    else
    {
        ifud_BP_up = 0;
    }

    if (YK.v_inf < -1000)
    {
        ifud_BP_down = 1;
    }
    else
    {
        ifud_BP_down = 0;
    }

    UD_BPsp_up.updata(ifud_BP_up);
    UD_BPsp_down.updata(ifud_BP_down);
    if (UD_BPsp_up.UD_data == UpDown_check_rising && AD_F < 20)
    {
        AD_F++;
    }
    if (UD_BPsp_down.UD_data == UpDown_check_rising && AD_F > 1)
    {
        AD_F--;
    }
}

uint8_t shoot_mode = 1;
const uint8_t shoot_type = 2;
UpDown_check_class UD_Q(0);
uint8_t level_F[shoot_type][10] = {{6, 7, 8, 8, 9, 9, 10, 12, 14, 16},      /*持续*/
                                                                            //    {8, 9, 10, 11, 12, 13, 14, 15, 16, 20}, /*爆发1*/
                                                                            //    {8, 9, 10, 12, 13, 15, 16, 18, 19, 20}, /*爆发2*/
                                   {8, 9, 10, 11, 12, 14, 16, 20, 20, 20}}; /*2倍冷却弹频爆发*/

/*操作手微调偏移弹频*/
/*根据等级设置最大弹频，3个预设*/
void MAX_HZ_F(void)
{

    uint8_t level = LIMIT(CP.ext_game_robot_status_t.robot_level, 1, 10);
    float max_buf = MAX_HZ;
    /*任何情况都可以选择预设，只有比赛生效*/
    if (UD_Q.updata(YK.Pressed_Check(KEY_PRESSED_G)) == UpDown_check_rising)
    {
        if (shoot_mode >= shoot_type)
        {
            shoot_mode = 1;
        }
        else
        {
            shoot_mode++;
        }
    }
    if (!heat_limit_en) /*当前不需要热量限制*/
    {
        MAX_HZ = 20;
        return;
    }
    /*热量限制下，发单激进程度，预设切换，等级决定弹频*/
    max_buf = level_F[shoot_mode - 1][level - 1];

    // switch (shoot_mode)
    // {
    // case 1:
    // {
    //     /*初始弹频持续*/
    //     if (!level)
    //     {
    //         max_buf = 10;
    //     }
    //     else if (level < 10)
    //     {
    //         max_buf = limit_MIN_HZ + level - 1;
    //     }
    //     else
    //     {
    //         max_buf = 20;
    //     }
    //     break;
    // }
    // case 2:
    // {
    //     /*初始弹频持续*/
    //     switch (level)
    //     {
    //     case 1:
    //         max_buf = limit_MIN_HZ;
    //         break;
    //     case 2:
    //         max_buf = limit_MIN_HZ + 1;
    //         break;
    //     case 3:
    //         max_buf = limit_MIN_HZ + 2;
    //         break;
    //     case 4:
    //         max_buf = limit_MIN_HZ + 4;
    //         break;
    //     case 5:
    //         max_buf = limit_MIN_HZ + 5;
    //         break;
    //     case 6:
    //         max_buf = limit_MIN_HZ + 7;
    //         break;
    //     case 7:
    //         max_buf = limit_MIN_HZ + 8;
    //         break;
    //     case 8:
    //         max_buf = 18;
    //         break;
    //     case 9:
    //         max_buf = 19;
    //         break;
    //     case 10:
    //         max_buf = 20;
    //         break;

    //     default:
    //         max_buf = 10;
    //         break;
    //     }
    //     break;
    // }
    // case 3:
    // { /*2倍冷却弹频爆发*/
    //     max_buf = LIMIT(level * 2, limit_MIN_HZ, 20);
    //     break;
    // }

    // default:
    // {
    //     max_buf = 10;
    // }
    // break;
    // }

    MAX_HZ = LIMIT(max_buf, limit_MIN_HZ, 20);
}

/*决定弹频函数，分开比赛与调试*/
float AD_BPHZ(void)
{
    /*根据等级设置最大弹频，3个预设*/
    MAX_HZ_F();
    if (heat_limit_en) /*如果有热量限制*/
    {
        /*单位转化*/
        MIN_HZ = LIMIT(CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value / 10.0f, limit_MIN_HZ, 12); // 最小弹频=冷却值

        float min_hz = MIN_HZ * 10;
        float max_hz = MAX_HZ * 10;

        float start_x = 0;                                                                                                          // 开始降低弹速起始点
        Heat_buf = min_Heat_buf + ((max_hz - CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_cooling_value + 10) * 0.2f) * 0.5f; // 因0.2s更新导致延误的弹丸余量，(最大弹频与冷却只差+冗余1发)/2
        BP_X = (CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit - ADD_Buf - Heat_buf);                                // 最大值持续范围，自适应范围BP_X，匀速缓冲区范围(已经=冷却蛋频，稳定蛋频所留的缓冲区)，不可发射范围（缓冲双发，急停，裁判系统数据延时，本地与云端相位差延时的区域）
        BP_Y = min_hz - max_hz;                                                                                                     // 弹频差
        BP_K = BP_Y / BP_X;
        // 点斜式，y-y1=k(x-x1),y1=max_hz,x1=start_x，LIMIT限制最大最小值
        AD_F = LIMIT(BP_K * (now_heat - start_x) + max_hz, min_hz, max_hz) / 10.0f; // 一次函数自适应弹速,F=K*(当前热量-起始自适应点)+最大弹频，k<0
        // CP.ext_game_robot_status_t.robot_level
    }
    else // 调试模式
    {
        /*拨轮调弹频*/
        BP_thz_set();
    }
    AD_F = LIMIT(AD_F, 1, 20);

    return AD_F;
}
/*操作手微调预置，防止双发*/
UpDown_check_class UD_B(0);
uint8_t YuZhi_flag = 0;
uint8_t YuZhi_Debug = 0;
/*操作手微调，防止双发，也可把堵转摩擦轮推出*/
void Restart_Mang(void)
{
    if ((UD_B.updata(YK.Pressed_Check(KEY_PRESSED_B)) == UpDown_check_rising || YuZhi_Debug) && !BP_tui_flag)
    {
        YuZhi_flag = 1;
        BP_mang_slow.targe -= one_shoot_mang * 0.1f;
        YuZhi_Debug = 0;
    }
}
void BP_YuZhi(void)
{
    /*电机回0点*/
    BP_M.first = 0; // 重新初始化
    BP_M.First();
    /*最终目标初始化*/
    BP_mang_slow.init_S(BP_M.mang_inf);
    BP_mang_slow.init_T(BP_M.mang_inf);
    /*PID部分，反馈初始化*/
    BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref = BP_M.mang_inf;
    BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Reset();
}
void BP_Task(void)
{
    /*最上层logic*/
    /*选择模式*/
    if (BP_M.online && MCLBP_On && BP_ON)
    {
        if (YKmode == DAN_YUN_TAI_MODE)
        {
            bp_ctrl_mode = 1; // 遥感控
        }
        else if (YKmode == DIAO_SHE_MODE)
        {
            bp_ctrl_mode = 2; // 发射测试连/单发+退弹
        }
        else if (YKmode == ZHAN_DOU_MODE || YKmode == SHANG_XIA_MODE)
        {
            bp_ctrl_mode = 3; // 战斗模式
        }
        else
        {
            bp_ctrl_mode = 0;
        }
    }
    else
    {
        bp_ctrl_mode = 0;
    }

    if (YKmode == PROTECT_MODE)
    {
        /*重新预置*/
        if (YK.yaogan.ch1 < -600 && YK.yaogan.ch0 < -600) // 右遥感往左下打，记录拨盘位置，
        {
            BP_YuZhi();
        }
    }

    /*目标，内外环*/
    if (bp_ctrl_mode == 1)
    {
        BP_loop_t.loop_set = OUTLOOP_SET;
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].set_goal_func(bp_ch_outgoal);
    }
    else if ((bp_ctrl_mode == 2 || bp_ctrl_mode == 3) && (MCL_ON_flag || bp2mcl_off))
    {
        BP_loop_t.loop_set = OUTLOOP_SET;
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].set_goal_func(voidgoal); // 目标交给while task处理
    }
    else
    {
        BP_loop_t.loop_set = NOLOOP_SET;
    }

    /*退弹开关*/

    if (BP_loop_t.loop_set != NOLOOP_SET)
    {
        if (bp_ctrl_mode == 1 || bp_ctrl_mode == 3) // 摇杆控
        {
            BPtui_on_flag = 1;
        }
        else if (bp_ctrl_mode == 2)
        {
            if (YK.yaogan.ch0 > -600)
            {
                BPtui_on_flag = 1;
            }
            else
            {
                BPtui_on_flag = 0;
            }
        }
    }
    else
    {
        BPtui_on_flag = 0;
    }
    /*拨轮调弹频*/
    Restart_Mang();
    // BP_thz_set();
    /*热量限制*/
    BP_heat_limit();
    /*自适应单频*/
    bp_thz = AD_BPHZ();
    /*退弹子任务*/
    BP_tui_deal(BPtui_on_flag);

    uint8_t P_L; // 手动左键打弹
    /*左右键同时按，自瞄火控*/
    if (request.zimiao_status != 2) // 不是自瞄
    {
        P_L = (YK.shubiao.press_l && !YK.shubiao.press_r); // 再来一层判断，zimiao_status可能来不及判断
    }
    else
    {
        P_L = 0;
    }
    // /*遥感+键鼠发弹准入，包括单连发*/
    if ((MCL_ON_flag || bp2mcl_off) && ((bp_ctrl_mode == 2 && (abs(YK.yaogan.ch0) > 600)) || ((P_L || (request.zimiao_status == 2 && SuperPower.Fire_Flag == 2)) && bp_ctrl_mode == 3))) /*内环使用发射时间*/
    {
        YKshoot_flag = 1;
    }
    else
    {
        YKshoot_flag = 0;
    }

    // if(UD_press_shoot.updata(YK.shubiao.press_l)==UpDown_check_rising && (MCL_ON_flag || bp2mcl_off) && bp_ctrl_mode==3)
    // {
    //     one_shoot=1;/*键鼠*/
    // }

    // /*连发*/
    // /*等待连发处理*/
    if (YKshoot_flag)
    {
        BPwait_tim_slow.F_slow();
    }
    else
    {
        BPwait_tim_slow.init_S(lianfa_wait_tim); // 等0.2s连发
    }
    /*连发准入*/
    if (YKshoot_flag && !BP_tui_flag && BPwait_tim_slow.ok) // 等待时间结束，不卡弹
    {
        /*连发标志*/
        LF_on = 1;
    }
    else
    {
        LF_on = 0;
    }

    /*快速连发，弹频设单发*/ /*自瞄火控*/
    if (LF_on || (request.zimiao_status == 2 && SuperPower.Fire_Flag == 2))
    {
        /*弹频one_shoot=1*/
        BPin_tim_slow.F_slow(); // 计时
        if (BPin_tim_slow.ok)   // 计时结束重新计时
        {
            one_shoot = 1;
            BPin_tim_slow.init_S(1.0f / bp_thz);
        }
    }
    else
    {
        BPin_tim_slow.init_S(1.0f / bp_thz); // 复位
    }
    // if (UD_LF_start.updata(LF_on) == UpDown_check_rising)
    // {
    //     one_shoot = 1;
    // }

    /*开启连发后，周期one_shoot=1*/
    /*处理YK输入单发*/
    if (UD_OneShoot.updata(YKshoot_flag) == UpDown_check_rising) // 单发
    {
        one_shoot = 2; // 单发对齐预置点
    }
    /*自瞄火控*/
    // if (request.zimiao_status == 2 && SuperPower.Fire_Flag == 2)
    // {
    //     one_shoot = 1;
    // }

    /*CTRL强制发弹*/
    force_bp_on = YK.Pressed_Check(KEY_PRESSED_CTRL);
    /*申请目标处理，底层限制20hz*/
    BPwait20_tim_slow.F_slow();
    // bp_calc_goal();
    if (one_shoot && (MCL_ON_flag || bp2mcl_off) && BPwait20_tim_slow.ok && ((CP_shoot_en && !BP_tui_flag && ((BP_mang_slow.targe - BP_M.mang_inf) < (one_shoot_mang * 3))) || force_bp_on))
    {
        if (one_shoot == 2)
        {
            if (YuZhi_flag) /*每次单发前判断是否要重新确定当前点为0点，如果需要重新确定位置，比如操作手微调，防止双发*/
            {
                BP_YuZhi();
                YuZhi_flag = 0;
            }
            bp_calc_goal();
        }
        else
        {
            BP_mang_slow.targe += one_shoot_mang;
        }
        // BP_mang_slow.targe += one_shoot_mang;

        BPwait20_tim_slow.init_S(1 / bpzm_thz); /*最底层限制20hz弹频*/
        one_shoot = 0;
    }
    else
    {
        one_shoot = 0;
    }

    // /*俯仰快慢推*/
    if ((bp_ctrl_mode == 2 && YK.yaogan.ch1 < -600 || bp_ctrl_mode == 3) && !BP_tui_flag)
    {
        // BP_yuntui_way = 1;
        BP_mang_slow.en = 1;
    }
    else
    {
        // BP_yuntui_way = 0;
        BP_mang_slow.en = 0;
    }
    /*设置匀速推弹速度*/
    // float s_set_sp = one_shoot_mang * bp_thz;
    float s_set_sp = one_shoot_mang * 20; // one_shoot_mang * bp_thz;
    BP_mang_slow.real_add = s_set_sp;
    BP_mang_slow.real_cut = s_set_sp;
    BP_mang_slow.real_stoperr = s_set_sp;
    BP_mang_slow.set_real();

    // if (BP_loop_t.loop_set == NOLOOP_SET)
    // {
    //     BP_M.out = 0;
    // }

    /*单双环切换*/ // 3级处理
    // if ((MCL_ON_flag || bp2mcl_off))
    // {
    //     if (YKmode == DAN_YUN_TAI_MODE || YKmode == SHUANG_ZHONG_MODE)
    //     {
    //         BP_loop_t.loop_set = INLOOP_SET;
    //     }
    //     else if (YKmode == SHANG_XIA_MODE)
    //     {
    //         BP_loop_t.loop_set = OUTLOOP_SET;
    //     }
    // }
    // else
    // {
    //     BP_loop_t.loop_set = NOLOOP_SET;
    // }

    /*编码值目标斜坡复位*/
    if (BP_loop_t.loop_set != OUTLOOP_SET)
    {

        BP_mang_slow.init_S(BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref);
        BP_mang_slow.init_T(BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref);
    }
    /*单双环控制*/
    set_loop(&BP_loop_t); // 2级处理
    /*自动保护控制*/      // 1级处理，放最后
    pid_set2param_controler(BP_outpid[0], BP_inpid[0], &BP_outctrl_index, &BP_inctrl_index);
}

void BP_pid(void)
{
    BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref = BP_M.mang_inf;
    BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].Ref = BP_M.sp;
    /*单双环切换示例，双环双enable_flag=1,单环内环=1，外环=0，判断内外环enable_flag*/
    if (BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].enable_flag) /*双环,使用.ifReset()复位*/
    {
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].pGoal(); /*需要切换目标值，退弹，将目标函数传入此结构体的goal_func函数指针*/
        // BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Goal += (float)YK.yaogan.ch2 / 4000.0f + ((float)LIMIT(YK.shubiao.x, -500, 500) / 1000.0F);
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].PID_update_void();
        BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].Goal = BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].OUT_PID; /*内外环输出传递目标*/
    }
    else if (BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].enable_flag) /*单环*/
    {
        BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].pGoal();
    }
    /*内置保护，不是多反馈不需要先行判断enable_flag*/
    BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].PID_update_void();
    BP_M.out = BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].OUT_PID;
}
UpDown_check_class UD_BPtui(0), UD_BPtui_end(0);
static int8_t kadan_forward = 0; // 卡弹方向1正，-1反

void BP_tui_deal(uint8_t en)
{
    static int16_t if_kadan_out_th = 30000 - 2; // 判断卡弹输出阈值
    static int16_t if_kadan_sp_th = 50;         // 判断卡弹速度阈值
    static uint16_t if_kadan_tim_th = 40;       // 卡弹时间阈值
    static uint16_t kadan_cnt = 0;              // 卡弹方向1正，-1反

    // if (BPtui_on_flag)
    // {
    // }
    /*必须时刻执行的，否则就要初始化*/
    UD_BPtui.updata(BP_tui_flag); // 检测退弹开始，退弹结束，这两时刻要做处理
    if (!en)                      /*功能初始化*/
    {
        BP_tui_flag = 0;
        kdan_tim = 0;
        BP_tui_tim = set_tui_tim;
        UD_BPtui.UD_data = UpDown_check_nothing;
        return;
    }

    /*累计卡弹时间*/
    if (fabs(BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].OUT_PID) > if_kadan_out_th && BP_M.sp < if_kadan_sp_th && BP_M.sp > -50 && !BP_tui_flag) // 输出阈值判断
    {
        kdan_tim++;
    }
    else
    {
        kdan_tim = 0;
    }
    /*判断卡弹时间阈值*/
    if (kdan_tim > if_kadan_tim_th)
    {
        /*判断卡弹的前后方向*/
        if (BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].OUT_PID > if_kadan_out_th)
        {
            kadan_forward = 1;
        }
        else if (BP_inpid[BP_inctrl_index.param_type][MOTOR_TYPE].OUT_PID < -if_kadan_out_th)
        {
            kadan_forward = -1;
        }
        kdan_tim = 0;
        BP_tui_flag = 1;
        kadan_cnt++;
    }
    /*执行卡弹处理*/
    /*退一格弹，并等待执行时间*/
    if (UD_BPtui.UD_data == UpDown_check_rising)
    {
        // 先定原点，再退弹
        BP_mang_slow.targe = BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref;
        BP_mang_slow.s_targe = BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref;
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Reset();

        /*朝反方向退一格*/
        BP_mang_slow.targe -= (one_shoot_mang * kadan_forward * 1);
        BP_mang_slow.s_targe -= (one_shoot_mang * kadan_forward * 1);
    }
    else if (UD_BPtui.UD_data == UpDown_check_falling) /*结束退弹，初始目标值，防止退弹也卡*/
    {
        BP_mang_slow.targe = BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref;
        BP_mang_slow.s_targe = BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Ref;
        BP_outpid[BP_outctrl_index.param_type][MOTOR_TYPE].Reset();
    }
    /*等待退弹执行完*/
    if (BP_tui_flag)
    {
        BP_tui_tim--;
        if (BP_tui_tim <= 0)
        {
            kadan_forward = 0;
            BP_tui_flag = 0;          // 退出退弹步骤
            BP_tui_tim = set_tui_tim; // 初始化执行时间
        }
    }
}

#pragma endregion

void CAN1_ctrl(void)
{
    if (BP_M.update() == HAL_OK)
    {
        // MCL_4_motorflag |= 0x4;
        BP_M.NSQD_8192mang_inf();
        BP_pid();
    }
    else if (MCL_R.update() == HAL_OK) // 1khz
    {
        // MCL_4_motorflag |= 0x1;
        MCL_R_sp_pid[MCL_ctrl_Index.param_type].Ref = MCL_R.sp;
        if (MCL_R_sp_pid[MCL_ctrl_Index.param_type].enable_flag)
        {
            mcl_sp_s.F_slow();
            MCL_R_sp_pid[MCL_ctrl_Index.param_type].Goal = -mcl_sp_s.s_targe;
            MCL_R_sp_pid[MCL_ctrl_Index.param_type].PID_update_void();
        }
        MCL_R.out = MCL_R_sp_pid[MCL_ctrl_Index.param_type].OUT_PID;
    }
    else if (MCL_L.update() == HAL_OK) // 1khz
    {
        // MCL_4_motorflag |= 0x2;

        MCL_L_sp_pid[MCL_ctrl_Index.param_type].Ref = MCL_L.sp;
        if (MCL_L_sp_pid[MCL_ctrl_Index.param_type].enable_flag)
        {
            MCL_L_sp_pid[MCL_ctrl_Index.param_type].Goal = mcl_sp_s.s_targe;
            MCL_L_sp_pid[MCL_ctrl_Index.param_type].PID_update_void();
        }

        MCL_L.out = MCL_L_sp_pid[MCL_ctrl_Index.param_type].OUT_PID;
    }
    else if (MCLDM_L.DM_update() == HAL_OK)
    {
    }
    else if (MCLDM_R.DM_update() == HAL_OK)
    {
    }
}

void watchdog_run_Task(void)
{
    YK.DT16_watchdog_run();
    YK.VT13_watchdog_run();
    CP_watchdog_run();
    MCL_L.watchdog_run();
    MCL_R.watchdog_run();
    BP_M.watchdog_run();
    YAW_M.watchdog_run();
    PITCH_M.watchdog_run();
    if (PITCH_M.ERR != 1 || !PITCH_M.online) /*自动使能*/
    {
        P_ctrl_index.protect_flag = 1;
        PITCH_M.DM_Start();
    }

    YK.lp_x_k = YK.get_cutoff_freq(YK.x_cutoff, 1000.0f);
    YK.lp_y_k = YK.get_cutoff_freq(YK.y_cutoff, 1000.0f);
    ZM_RXY.Set_LPK(ZM_RXY.get_cutoff_freq(ZM_RXY.Cutoff_freq));
    P_DY_LP.Set_LPK(P_DY_LP.get_cutoff_freq(P_DY_LP.Cutoff_freq));

    // gyroNotch_P136.NF_update();
    //    gyroNotch_P68.NF_update();
    //    gyroNotch_P46.NF_update();
    //    gyroNotch_P24.NF_update();

    PITCH_Slow.set_real();
    YAW_Slow.set_real();

    // gyroNotch_R.NF_update();
}

/*串口DMA发送可关传输半完成中断*/
#pragma region /*自瞄*/
Vision_process_t Vision_process;
Kf kalman_speedYaw1, kalman_accel1, kalman_distend1; // 卡尔曼滤波
Anti_top_Data TOP_Data;

#pragma region /*NO USE*/

uint8_t Anti_Time_Flag = 2, Anti_Time_Flag_UD, Top_Dir = 0, Shot_Flag = 1, Shot_Flag_Time_1 = 0, Shot_Flag_Time_2 = 0, UD_TOP_DIR_Flag = 0, Anti_MoveTop_Flag = 0;
int16_t Anti_Time_1 = 0, Anti_Time_2 = 0, TOP_1, TOP_2;
uint8_t ZM_Status = 0;
// int16_t ZM_Fire_delay = 834; // 开火延时
// uint8_t zm_test_en=0;
void ZM_tim_LPandSend_deal(void)
{
    // request.yaw_mang.f = M6020_CAN2_Yaw_Motor.mang;
    // request.pitch_mang.f = M6020_CAN1_Pitch_Motor.mang;
    // request.Yaw_Angle.f = V_Yaw.Vision_Low_Pass_Filter(IMU.realAngle.yaw);
    // request.Pitch_Angle.f = V_Pitch.Vision_Low_Pass_Filter(IMU.realAngle.pitch);
    request.Yaw_Angle.f = IMU.realAngle.yaw;
    request.Pitch_Angle.f = IMU.realAngle.pitch;

    request.Yaw_Anglespeed.f = IMU.Anglespeed.Deal_yaw; // 最应该LP的应该是角速度
    Mini_PC_SendData();
}
void ZM_anti_tim_deal(void)
{
    if (Anti_Time_Flag == 1)
    {
        Anti_Time_1++;
        Anti_Time_2 = 0;
    }
    if (Anti_Time_Flag == 0)
    {
        Anti_Time_2++;
        Anti_Time_1 = 0;
    }
}
void ZM_rx_Deal(void)
{

    uint32_t temp;
    if ((__HAL_UART_GET_FLAG(&MINI_PC_USART_HANDLE, UART_FLAG_IDLE) != RESET)) // 检查是否产生UART空闲线路中断（表示一帧数据传输完成）
    {
        __HAL_UART_CLEAR_IDLEFLAG(&MINI_PC_USART_HANDLE); // 清除空闲中断标志
        temp = MINI_PC_USART_HANDLE.Instance->SR;
        temp = MINI_PC_USART_HANDLE.Instance->DR;
        HAL_UART_DMAStop(&MINI_PC_USART_HANDLE); // 停止DMA传输（安全操作）
        getReceiveData(Mini_PC_rx_buf);          // 处理接收到的数据（Mini_PC_rx_buf是接收缓冲区）

        HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE, (uint8_t *)Mini_PC_rx_buf, sizeof(Mini_PC_rx_buf)); // 重新启动DMA接收，准备接收下一帧数据（128字节长度）

        /*************** Your code *****************/

        if (request.zimiao_status)
        {
            // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
            // P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        }
        // if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
        // {
        //     if (request.zimiao_status)
        //     {
        //         aa++;
        //         // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
        //         P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        //     }
        // }
    }
}
void ZM_2rx_Deal(DMA_HandleTypeDef *hdma)
{
    uint32_t temp;
    if ((__HAL_UART_GET_FLAG(&MINI_PC_USART_HANDLE, UART_FLAG_IDLE) != RESET)) // 检查是否产生UART空闲线路中断（表示一帧数据传输完成）
    {
        __HAL_UART_CLEAR_IDLEFLAG(&MINI_PC_USART_HANDLE); // 清除空闲中断标志
        temp = MINI_PC_USART_HANDLE.Instance->SR;
        temp = MINI_PC_USART_HANDLE.Instance->DR;
        HAL_UART_DMAStop(&MINI_PC_USART_HANDLE); // 停止DMA传输（安全操作）
        temp = hdma->Instance->NDTR;
        uint8_t PC_rx_len = MINI_PC_RXBUF_SIZE - temp;
        PC_FIFO = !PC_FIFO;

        HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE, (uint8_t *)Mini_PC_2rx_buf[PC_FIFO], sizeof(Mini_PC_2rx_buf[0])); // 重新启动DMA接收，准备接收下一帧数据（128字节长度）

        getReceiveData((uint8_t *)Mini_PC_2rx_buf[!PC_FIFO]); // 处理接收到的数据（Mini_PC_rx_buf是接收缓冲区）

        /*************** Your code *****************/
        if (request.zimiao_status)
        {
            // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
            // P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        }
        // if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
        // {
        //     if (request.zimiao_status)
        //     {
        //         aa++;
        //         // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
        //         P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        //     }
        // }
    }
}
#pragma endregion

void ZM_Task(void)
{
    /************************** 检测自己是红方还是蓝方*****************************/
    if (CP.ext_game_robot_status_t.robot_id < 10)
        request.mine = 0; // red
    else
        request.mine = 1;
    /*此项为主动开启自瞄条件，只有视觉反馈自瞄可用&主动开启才可开启自瞄*/
    if ((YKmode == ZHAN_DOU_MODE && YK.shubiao.press_r) ||
        (YKmode == SHANG_XIA_MODE && (YK.yaogan.ch1 > 600 || YK.shubiao.press_r)))
    {

        if ((YK.shubiao.press_r && YK.shubiao.press_l) || YK.yaogan.ch0 > 550)
        {
            request.zimiao_status = 2;
        }
        else
        {
            request.zimiao_status = 1;
        }

        // if (YK.shubiao.press_r) //||(YKmode==SHANG_XIA_MODE&&YK.yaogan.ch0>500)
        // {
        //     request.zimiao_status = 1;
        //     // if (UD_ZM_Fire.updata(response.Fire_Flag) == UpDown_check_rising && !ZM_Fire_delay)
        //     // {
        //     //     Shoot_flag = 1;
        //     //     ZM_Fire_delay = 834;
        //     // }
        // }
        // else
        // {
        //     request.zimiao_status = 0;
        // }
    }
    else
    {
        request.zimiao_status = 0;
    }

    if (CP.ext_shoot_data_t.bullet_speed < 23)
    {
        Bullet_Speed = 24.6f;
    }
    else
    {
        Bullet_Speed = LIMIT(CP.ext_shoot_data_t.bullet_speed, 22, 26);
    }

    // /*火控高实时性应该放中断*/
    // if (request.zimiao_status == 2)
    // {
    //     one_shoot = SuperPower.Fire_Flag;
    // }
}

float ZM_Y_Time = 0.012f; // 给视觉做火控预测的时间(相位补偿)
float ZM_P_Time = 0.025f; // 给视觉做火控预测的时间(相位补偿)

// float ZM_AP, ZM_AY;
float ZM_LAP, ZM_LYP;
float ZM_ACCP, ZM_ACCY;

// this->E_DAngle.pitch = (this->realAngle.pitch - this->last_realAngle.pitch) / delta_T; // dt
// this->E_DAngle.yaw = (this->realAngle.yaw - this->last_realAngle.yaw) / delta_T;
// this->E_DAngle.roll = (this->realAngle.roll - this->last_realAngle.roll) / delta_T;
// float W_ACCP,W_ACCY;
void ZM_TJtim_LPandSend_deal(void)
{
    static uint16_t tfreq = 1000;
    static uint8_t scan_freq = 0;
    static uint8_t ffreq = 2;
    static uint32_t ccnntt = 0;
    float delta_T = 1.0f / ((float)tfreq / (float)ffreq); // 发送周期

    scan_freq++;
    if ((scan_freq / ffreq) >= 1)
    {
        // float ZM_Time2 = ZM_Time * ZM_Time * 0.5f;
        // ZM_ACCP = (IMU.realAngle.pitch - ZM_LAP) / delta_T; // dt
        // ZM_ACCY = (IMU.realAngle.yaw - ZM_LYP) / delta_T;   // dt
        ZM_ACCP = (P_DY_LP.out - ZM_LAP) / delta_T;      // dt
        ZM_ACCY = (gyroNotch_Y2.out - ZM_LYP) / delta_T; // dt

        AS.Q_info_0.f = IMU.Q_info.q0;
        AS.Q_info_1.f = IMU.Q_info.q1;
        AS.Q_info_2.f = IMU.Q_info.q2;
        AS.Q_info_3.f = IMU.Q_info.q3;
        AS.Bullet_Speed.f = Bullet_Speed;
        // AS.Pitch_Angle.f = IMU.realAngle.pitch + P_DY_LP.out * ZM_P_Time; //+ ZM_ACCP * ZM_P_Time * ZM_P_Time * 0.5f;
        AS.Pitch_Angle.f = IMU.realAngle.pitch + P_Gyro_ZM * ZM_P_Time + P_AGyro_ZM * ZM_P_Time * ZM_P_Time * 0.5f;

        AS.Yaw_Angle.f = IMU.realAngle.yaw + Y_Gyro_ZM * ZM_Y_Time + Y_AGyro_ZM * ZM_Y_Time * ZM_Y_Time * 0.5f;
        // AS.Yaw_Angle.f = IMU.realAngle.yaw + gyroNotch_Y2.out * ZM_Y_Time + ZM_ACCY * ZM_Y_Time * ZM_Y_Time * 0.5f;

        ZM_LAP = P_DY_LP.out;
        ZM_LYP = gyroNotch_Y2.out;
        Mini_PC_TJ_SendData(YK.shubiao.press_r);
        scan_freq = 0;
        ccnntt++;
    }
}

/*没识别到不会发*/
uint8_t ZM_Start = 0;
uint8_t Fire_Flag = 0; // 开火标志位

float ZM_Angle_Deal(float ZM_Angle, float Now_Angle, float Now_Current_Angle)
{
    float diff = ZM_Angle - Now_Angle;

    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    else if (diff < -180.0f)
    {
        diff += 360.0f;
    }

    return Now_Current_Angle + diff;
}

volatile float Y_Angle_ZM = 0;
volatile float Y_Gyro_ZM = 0;
volatile float Y_AGyro_ZM = 0;
volatile float P_Angle_ZM = 0;
volatile float P_Gyro_ZM = 0;
volatile float P_AGyro_ZM = 0;

float last_zm_Y_angle = 0;
void ZM_2rxTJ_Deal(DMA_HandleTypeDef *hdma)
{
    uint32_t temp;
    if ((__HAL_UART_GET_FLAG(&MINI_PC_USART_HANDLE, UART_FLAG_IDLE) != RESET)) // 检查是否产生UART空闲线路中断（表示一帧数据传输完成）
    {
        __HAL_UART_CLEAR_IDLEFLAG(&MINI_PC_USART_HANDLE); // 清除空闲中断标志
        temp = MINI_PC_USART_HANDLE.Instance->SR;
        temp = MINI_PC_USART_HANDLE.Instance->DR;
        HAL_UART_DMAStop(&MINI_PC_USART_HANDLE); // 停止DMA传输（安全操作）
        temp = hdma->Instance->NDTR;
        uint8_t PC_rx_len = MINI_PC_RXBUF_SIZE - temp;
        PC_FIFO = !PC_FIFO;

        HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE, (uint8_t *)Mini_PC_2rx_buf[PC_FIFO], sizeof(Mini_PC_2rx_buf[0])); // 重新启动DMA接收，准备接收下一帧数据（128字节长度）

        TJ_GetReceive_SP((uint8_t *)Mini_PC_2rx_buf[!PC_FIFO]); // 处理接收到的数据（Mini_PC_rx_buf是接收缓冲区）

        /*************** Your code *****************/
        if (Mini_PC_2rx_buf[!PC_FIFO][0] == 0x66 && Mini_PC_2rx_buf[!PC_FIFO][28] == 0x11) // 检查数据帧头是否为0x66（自定义协议标识）
        {
            /*弧度转角度*/

            Y_Angle_ZM = ZM_Angle_Deal((SuperPower.yaw.f * 180.0f / PI), IMU.eulerAngle.yaw, IMU.realAngle.yaw); // 视觉发送的是相对值，需要做过圈处理
            Y_Gyro_ZM = (SuperPower.yaw_vel.f * 180.0f / PI);
            Y_AGyro_ZM = (SuperPower.yaw_acc.f * 180.0f / PI);

            P_Angle_ZM = (SuperPower.pitch.f * 180.0f / PI);
            P_Gyro_ZM = (SuperPower.pitch_vel.f * 180.0f / PI);
            P_AGyro_ZM = (SuperPower.pitch_acc.f * 180.0f / PI);

            // if (Y_Angle_ZM != 0)
            // {
            //     if (fabs(Y_Angle_ZM - last_zm_Y_angle) > 100) // 防止甩头
            //     {
            //         Y_Angle_ZM = last_zm_Y_angle;
            //     }
            //     last_zm_Y_angle = Y_Angle_ZM;
            // }
            // else
            // {
            //     last_zm_Y_angle = IMU.realAngle.yaw;
            // }

            // ZM_Time_Number = 0;
            // if (YKmode == SHANG_XIA_MODE)
            // {
            //     if (SuperPower.Fire_Flag == 1 || SuperPower.Fire_Flag == 2)
            //         ZM_Start = 1;
            //     else
            //         ZM_Start = 0;

            //     if (SuperPower.Fire_Flag == 2)
            //         Fire_Flag = 1;
            //     else
            //         Fire_Flag = 0;
            // }

            // if (SuperPower.Fire_Flag && request.zimiao_status)
            // {
            //     P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = LIMIT(ZM_RXP.out, -85.0f, 85.0f);
            //     Yaw_goal = LIMIT(Y_Angle_ZM, -85.0f, 85.0f);
            // }
            //                ZM_Status = 1;
            //            else
            //                ZM_Status = 0;

            // if (SuperPower.Fire_Flag == 2)
            //     Fire_Flag = 1;
            // else
            //     Fire_Flag = 0;
        }

        // if (request.zimiao_status)
        // {
        //     Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
        //     P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        // }
        // if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
        // {
        //     if (request.zimiao_status)
        //     {
        //         aa++;
        //         // Y_outpid[Y_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.yaw - response.yaw.f;
        //         P_outpid[P_ctrl_index.param_type][IMU_TYPE].Goal = IMU.realAngle.roll + response.pitch.f;
        //     }
        // }
    }
}
#pragma endregion

#pragma region /*RGB*/
/*&htim1, TIM_CHANNEL_1底部左边*/
/*&htim4, TIM_CHANNEL_2底部4pin右边，上往下数第二个PB7,第三个GND*/
// RGB_UI RGB(&htim1, TIM_CHANNEL_1); // 顶部左边

RGB_UI RGB(&htim1, TIM_CHANNEL_4); // 顶部左边
/*160个LED,一帧5.1ms*/
uint8_t wsadRGB[3] = {0};
uint8_t stateRGB[3] = {0};

uint8_t Light_RGB = 50; /*调整亮度*/
void set_RGBBuf(uint8_t r, uint8_t g, uint8_t b, uint8_t *buf)
{
    buf[0] = r;
    buf[1] = g;
    buf[2] = b;
}
void RGB_Task(void)
{
    /*遥控调亮度*/
    if (YKmode == XIAO_TUO_LUO_MODE)
    {
        int16_t light_buf = Light_RGB;
        light_buf += YK.yaogan.ch1 / 329;
        Light_RGB = LIMIT(light_buf, 1, 255);
    }
    // RGB.R = 255;
    // RGB.G = 255;
    // RGB.B = 255;
    /*微调指令*/
    // WSAD(WS停)

    if (YK.Pressed_Check(KEY_PRESSED_W)) /*红*/
    {
        set_RGBBuf(1, 0, 0, wsadRGB);
    }
    else if (YK.Pressed_Check(KEY_PRESSED_S)) /*绿*/
    {
        set_RGBBuf(0, 1, 0, wsadRGB);
    }
    else if (YK.Pressed_Check(KEY_PRESSED_D)) /*白*/
    {
        set_RGBBuf(1, 1, 1, wsadRGB);
    }
    else if (YK.Pressed_Check(KEY_PRESSED_A)) /*黄*/
    {
        set_RGBBuf(1, 1, 0, wsadRGB);
    }
    else
    {
        set_RGBBuf(0, 0, 0, wsadRGB);
    }
    uint8_t wsadrgb_buf[3];
    wsadrgb_buf[0] = wsadRGB[0] * Light_RGB;
    wsadrgb_buf[1] = wsadRGB[1] * Light_RGB;
    wsadrgb_buf[2] = wsadRGB[2] * Light_RGB;
    /*微调灯光区域*/
    for (uint8_t i = 0; i < PIXEL_NUM / 2; i++)
    {
        RGB.WS281x_SetPixelRGB(i, wsadrgb_buf[0], wsadrgb_buf[1], wsadrgb_buf[2]);
    }

    /*点位&策略指令，每次空闲持续一端时间，UI显示*/
    // Q,F
    if (YK.Pressed_Check(KEY_PRESSED_Z)) // 红打前哨战
    {
        set_RGBBuf(1, 0, 0, stateRGB);
    }
    else if (YK.Pressed_Check(KEY_PRESSED_X)) // 黄进攻
    {
        set_RGBBuf(1, 1, 0, stateRGB);
    }
    else if (YK.Pressed_Check(KEY_PRESSED_C)) // 绿防守
    {
        set_RGBBuf(0, 1, 0, stateRGB);
    } /*降落开镖白色*/
    // else /*离线*/
    // {
    //     RGB.R = 0;
    //     RGB.G = 0;
    //     RGB.B = 0;
    // }
    uint8_t statergb_buf[3];
    statergb_buf[0] = stateRGB[0] * Light_RGB;
    statergb_buf[1] = stateRGB[1] * Light_RGB;
    statergb_buf[2] = stateRGB[2] * Light_RGB;

    /*策略灯光区域*/
    for (uint8_t i = PIXEL_NUM / 2; i < PIXEL_NUM; i++)
    {
        RGB.WS281x_SetPixelRGB(i, statergb_buf[0], statergb_buf[1], statergb_buf[2]);
    }

    /*灯光调试，强制显示，有问题显示，没问题不显示*/
    /*离线紫，异常红*/
    /*MCL，离线，堵转或者发射一发弹掉速红,正常绿，保护白*/
    uint8_t MCL_L_RGB[3];
    if (!MCL_L.online || !MCL_Task_en || !MCLBP_On) /*离线，关摩擦轮调试*/
    {
        MCL_L_RGB[0] = 1;
        MCL_L_RGB[1] = 0;
        MCL_L_RGB[2] = 1;
    }
    else if (!MCL_ctrl_Index.protect_flag && fabs(MCL_L_sp_pid[MCL_ctrl_Index.param_type].error) > 100) /*堵转MCL_Normal_Onflag,发射闪一下*/
    {
        MCL_L_RGB[0] = 1;
        MCL_L_RGB[1] = 0;
        MCL_L_RGB[2] = 0;
    }
    else if (MCL_ON_flag) /*常态稳定*/
    {
        MCL_L_RGB[0] = 0;
        MCL_L_RGB[1] = 1;
        MCL_L_RGB[2] = 0;
    }
    else
    {
        MCL_L_RGB[0] = 1;
        MCL_L_RGB[1] = 1;
        MCL_L_RGB[2] = 1;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 1, MCL_L_RGB[0] * Light_RGB, MCL_L_RGB[1] * Light_RGB, MCL_L_RGB[2] * Light_RGB);

    uint8_t MCL_R_RGB[3];
    if (!MCL_R.online || !MCL_Task_en || !MCLBP_On) /*离线，关摩擦轮调试*/
    {
        MCL_R_RGB[0] = 1;
        MCL_R_RGB[1] = 0;
        MCL_R_RGB[2] = 1;
    }
    else if (!MCL_ctrl_Index.protect_flag && fabs(MCL_R_sp_pid[MCL_ctrl_Index.param_type].error) > 100) /*堵转*/
    {
        MCL_R_RGB[0] = 1;
        MCL_R_RGB[1] = 0;
        MCL_R_RGB[2] = 0;
    }
    else if (MCL_ON_flag)
    {
        MCL_R_RGB[0] = 0;
        MCL_R_RGB[1] = 1;
        MCL_R_RGB[2] = 0;
    }
    else
    {
        MCL_R_RGB[0] = 1;
        MCL_R_RGB[1] = 1;
        MCL_R_RGB[2] = 1;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 2, MCL_R_RGB[0] * Light_RGB, MCL_R_RGB[1] * Light_RGB, MCL_R_RGB[2] * Light_RGB);

    /*BP,离线紫，卡弹白，弹频绿到红*/
    float BP_RGB[3];
    if (!BP_M.online || !MCLBP_On || !BP_ON) /*离线，关拨盘调试*/
    {
        BP_RGB[0] = 1;
        BP_RGB[1] = 0;
        BP_RGB[2] = 1;
    }
    else if (BP_tui_flag) /*卡弹*/
    {
        BP_RGB[0] = 1;
        BP_RGB[1] = 1;
        BP_RGB[2] = 1;
    }
    else /*显示弹频绿到红*/
    {
        float hz_percent = (bp_thz - 1) / 19.0f; // 1-20hz
        BP_RGB[0] = hz_percent;
        BP_RGB[1] = 1 - hz_percent;
        BP_RGB[2] = 0;
    }

    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 3, (uint8_t)(BP_RGB[0] * Light_RGB), (uint8_t)(BP_RGB[1] * Light_RGB), (uint8_t)(BP_RGB[2] * Light_RGB));
    /*CP显示热量*/
    /*离线紫,显示当前热量绿到红*/
    uint8_t CP_RGB[3];
    if (!CP_Online)
    {
        CP_RGB[0] = 1;
        CP_RGB[1] = 0;
        CP_RGB[2] = 1;
    }
    else
    {
        float heat_percent = LIMIT(now_heat / CP.ext_game_robot_status_t.shooter_id1_17mm_barrel_heat_limit, 0, 1);
        CP_RGB[0] = heat_percent;
        CP_RGB[1] = 1 - heat_percent;
        CP_RGB[2] = 0;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 4, (uint8_t)(CP_RGB[0] * Light_RGB), (uint8_t)(CP_RGB[1] * Light_RGB), (uint8_t)(CP_RGB[2] * Light_RGB));

    /*ZM，离线,无识别红，识别黄，火控绿*/
    uint8_t ZM_RGB[3];
    if (!PC_online)
    {
        ZM_RGB[0] = 1;
        ZM_RGB[1] = 0;
        ZM_RGB[2] = 1;
    }
    else if (SuperPower.Fire_Flag == 0)
    {
        ZM_RGB[0] = 1;
        ZM_RGB[1] = 0;
        ZM_RGB[2] = 0;
    }
    else if (SuperPower.Fire_Flag == 1)
    {
        ZM_RGB[0] = 1;
        ZM_RGB[1] = 1;
        ZM_RGB[2] = 0;
    }
    else if (SuperPower.Fire_Flag >= 2)
    {
        ZM_RGB[0] = 0;
        ZM_RGB[1] = 1;
        ZM_RGB[2] = 0;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 5, (uint8_t)(ZM_RGB[0] * Light_RGB), (uint8_t)(ZM_RGB[1] * Light_RGB), (uint8_t)(ZM_RGB[2] * Light_RGB));

    /*P，离线，红闪红，在线绿*/
    uint8_t P_RGB[3];
    if (!PITCH_M.online)
    {
        P_RGB[0] = 1;
        P_RGB[1] = 0;
        P_RGB[2] = 1;
    }
    else if (PITCH_M.ERR != 1)
    {
        P_RGB[0] = 1;
        P_RGB[1] = 0;
        P_RGB[2] = 0;
    }
    else if (!P_ctrl_index.protect_flag)
    {
        P_RGB[0] = 0;
        P_RGB[1] = 1;
        P_RGB[2] = 0;
    }
    else
    {
        P_RGB[0] = 1;
        P_RGB[1] = 1;
        P_RGB[2] = 1;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 6, (uint8_t)(P_RGB[0] * Light_RGB), (uint8_t)(P_RGB[1] * Light_RGB), (uint8_t)(P_RGB[2] * Light_RGB));

    /*Y离线,在线绿，保护白*/
    uint8_t Y_RGB[3];
    if (!YAW_M.online)
    {
        Y_RGB[0] = 1;
        Y_RGB[1] = 0;
        Y_RGB[2] = 1;
    }
    else if (!Y_ctrl_index.protect_flag)
    {
        Y_RGB[0] = 0;
        Y_RGB[1] = 1;
        Y_RGB[2] = 0;
    }
    else
    {
        Y_RGB[0] = 1;
        Y_RGB[1] = 1;
        Y_RGB[2] = 1;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 7, (uint8_t)(Y_RGB[0] * Light_RGB), (uint8_t)(Y_RGB[1] * Light_RGB), (uint8_t)(Y_RGB[2] * Light_RGB));

    uint8_t YK_RGB[3];
    /*离线紫，双控白，图传绿，DT7黄*/
    if (YK.dt16_signal_flag && YK.vt13yk_signal_flag)
    {
        YK_RGB[0] = 1;
        YK_RGB[1] = 1;
        YK_RGB[2] = 1;
    }
    else if (!YK.dt16_signal_flag && YK.vt13yk_signal_flag)
    {
        YK_RGB[0] = 0;
        YK_RGB[1] = 1;
        YK_RGB[2] = 0;
    }
    else if (YK.dt16_signal_flag && !YK.vt13yk_signal_flag)
    {
        YK_RGB[0] = 1;
        YK_RGB[1] = 1;
        YK_RGB[2] = 0;
    }
    else
    {
        YK_RGB[0] = 1;
        YK_RGB[1] = 0;
        YK_RGB[2] = 1;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 8, (uint8_t)(YK_RGB[0] * Light_RGB), (uint8_t)(YK_RGB[1] * Light_RGB), (uint8_t)(YK_RGB[2] * Light_RGB));

    uint8_t IMU_RGB[3];
    if (IMU.state == BMI088_OK)
    {
        IMU_RGB[0] = 1;
        IMU_RGB[1] = 1;
        IMU_RGB[2] = 1;
    }
    else if (IMU.state == BMI088_ACC_ID_ERROR)
    {
        IMU_RGB[0] = 1;
        IMU_RGB[1] = 0;
        IMU_RGB[2] = 0;
    }
    else if (IMU.state == BMI088_GYRO_ID_ERROR)
    {
        IMU_RGB[0] = 0;
        IMU_RGB[1] = 0;
        IMU_RGB[2] = 1;
    }
    else if (IMU.state == BMI088_ERROR)
    {
        IMU_RGB[0] = 1;
        IMU_RGB[1] = 1;
        IMU_RGB[2] = 0;
    }
    else if (IMU.state == BMI088_SELFTEXT_ERROR)
    {
        IMU_RGB[0] = 1;
        IMU_RGB[1] = 0;
        IMU_RGB[2] = 1;
    }
    else if (IMU.state == BMI088_SET_ERROR)
    {
        IMU_RGB[0] = 0;
        IMU_RGB[1] = 1;
        IMU_RGB[2] = 1;
    }
    else
    {
        IMU_RGB[0] = 1;
        IMU_RGB[1] = 0;
        IMU_RGB[2] = 0;
    }
    RGB.WS281x_SetPixelRGB(PIXEL_NUM - 9, (uint8_t)(IMU_RGB[0] * Light_RGB), (uint8_t)(IMU_RGB[1] * Light_RGB), (uint8_t)(IMU_RGB[2] * Light_RGB));

    /**/
    RGB.WS_Load();

    // RGB.WS_WriteAll_RGB(RGB.R, RGB.G, RGB.B);
    // RGB.WS_WriteAll_RGB(255, 255, 255);

    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 50); // 设置占空比为50%（假设ARR=2000）
}
#pragma endregion

#pragma region /*UI*/
// 起落架挡住云台PY
void Block(void)
{
    /*4个鸡jio*/
    if ((YAW_M.mang > 1 && YAW_M.mang < 3 && PITCH_M.mang > 0.1f && PITCH_M.mang < 0.2f) ||
        (YAW_M.mang > 4 && YAW_M.mang < 6 && PITCH_M.mang > 0.3f && PITCH_M.mang < 0.4f) ||
        (YAW_M.mang > 7 && YAW_M.mang < 9 && PITCH_M.mang > 0.5f && PITCH_M.mang < 0.6f) ||
        (YAW_M.mang > 10 && YAW_M.mang < 12 && PITCH_M.mang > 0.7f && PITCH_M.mang < 0.8f))

    {
    }
}
uint32_t ui_name1[7] = {1, 2, 3, 4, 5, 6, 7};
graphic_tpyedef graphic_tpye1[7] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line};
Color_tpyedef color1[7] = {Color_Yellow, Color_Cyan, Color_Pink, Color_Cyan, Color_Cyan, Color_Pink, Color_Cyan};
uint16_t d1[8][7] = {{0},
                     {0},
                     {1, 1, 1, 0, 0, 0, 0},               // 10, 10, 1}
                     {960, 950, 950, 950, 950, 950, 950}, // 中竖 落点横，25m落点横 地面三横start_x[7]，中0/5高6/2低8  前哨线 16m线
                     {525, 520, 500, 470, 430, 370, 215}, // start_y
                     {0, 0, 0, 0, 0, 0, 0},
                     {960, 970, 970, 970, 970, 970, 970},  // end_x
                     {515, 520, 500, 470, 430, 370, 215}}; // end_y

uint32_t ui_name2[7] = {11, 12, 13, 14, 15, 16, 17};
graphic_tpyedef graphic_tpye2[7] = {Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line, Graphic_Line};
Color_tpyedef color2[7] = {Color_Cyan, Color_Cyan, Color_Cyan, Color_Cyan, Color_Cyan, Color_Pink, Color_Cyan};
uint16_t d2[8][7] = {{0},
                     {0},
                     {0, 0, 0, 0, 0, 0, 0},               // 10, 10, 1}
                     {960, 950, 950, 950, 950, 950, 950}, // 中竖 环高基地 地面三横start_x[7]，中0/5高6/2低8  前哨线 16m线
                     {470, 400, 442, 470, 430, 370, 215}, // start_y
                     {0, 0, 0, 0, 0, 0, 0},
                     {960, 970, 970, 970, 970, 970, 970},  // end_x
                     {430, 400, 442, 470, 430, 370, 215}}; // end_y

Color_tpyedef ZM_color = Color_Black;
uint16_t ZM_size = 20;
uint16_t ZM_width = 3;

uint16_t ZM_x = 1200;
uint16_t ZM_y = 340;
uint8_t char_ZM[] = "ZM";

Color_tpyedef G_color = Color_Green;
uint16_t G_size = 20;
uint16_t G_width = 3;

uint16_t G_x = 960 - (1200 - 960);
uint16_t G_y = 340;

void UI_TASK(void)
{
    static uint8_t scan_tim_ui = 0;
    static uint8_t scan_inc_ui = 0;
    static uint8_t scan_mdf_ui = 0;

    if (scan_tim_ui % 30 != 0)
    {
        switch (scan_mdf_ui % 2)
        {
        case 0:
            if (SuperPower.Fire_Flag == 0)
            {
                ZM_color = Color_Black;
            }
            else if (SuperPower.Fire_Flag == 1)
            {
                ZM_color = Color_Green;
            }
            else if (SuperPower.Fire_Flag == 2)
            {
                ZM_color = Color_Pink;
            }

            // if (YK.shubiao.press_r)
            // {
            //     ZM_color = Color_Pink;
            // }
            // else if (YK.shubiao.press_l)
            // {
            //     ZM_color = Color_Green;
            // }
            // else if (SuperPower.Fire_Flag == 0)
            // {
            //     ZM_color = Color_Black;
            // }
            CP_DrawOrDelete_Char(40, Modify_Graphic, 0, ZM_color, ZM_size, ZM_width, ZM_x, ZM_y, (uint8_t *)"ZM"); // 最上面的准星横线数字，意义不明

            break;
        case 1:
            /* code */
            if (shoot_mode == 1)
            {
                G_color = Color_Green;
            }
            else if (shoot_mode == 2)
            {
                G_color = Color_Pink;
            }
            else
            {
                G_color = Color_Pink;
            }
            CP_DrawOrDelete_Char(41, Modify_Graphic, 0, G_color, G_size, G_width, G_x, G_y, (uint8_t *)"G"); // 最上面的准星横线数字，意义不明

            break;

        default:
            CP_DrawOrDelete_Char(40, Modify_Graphic, 0, ZM_color, ZM_size, ZM_width, ZM_x, ZM_y, (uint8_t *)"ZM"); // 最上面的准星横线数字，意义不明
            break;
        }

        // if (SuperPower.Fire_Flag == 0)
        // {
        //     ZM_color = Color_Black;
        // }
        // else if (SuperPower.Fire_Flag == 1)
        // {
        //     ZM_color = Color_Green;
        // }
        // else if (SuperPower.Fire_Flag == 2)
        // {
        //     ZM_color = Color_Pink;
        // }
        // if (YK.shubiao.press_r)
        // {
        //     ZM_color = Color_Pink;
        // }
        // else if (YK.shubiao.press_l)
        // {
        //     ZM_color = Color_Green;
        // }
        // else if (SuperPower.Fire_Flag == 0)
        // {
        //     ZM_color = Color_Black;
        // }
        // CP_DrawOrDelete_Char(40, Modify_Graphic, 0, ZM_color, ZM_size, ZM_width, ZM_x, ZM_y, (uint8_t *)"ZM"); // 最上面的准星横线数字，意义不明
        scan_mdf_ui++;
        scan_tim_ui++;
    }
    else
    {
        switch (scan_inc_ui % 3)
        {
        case 0:
            CP_DrawOrDelete_Seven_Graphic(ui_name1, Increase_Graphic, graphic_tpye1, 0, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]);
            break;
        case 1:
            if (shoot_mode == 1)
            {
                G_color = Color_Green;
            }
            else if (shoot_mode == 2)
            {
                G_color = Color_Pink;
            }
            else
            {
                G_color = Color_Pink;
            }
            CP_DrawOrDelete_Char(41, Increase_Graphic, 0, G_color, G_size, G_width, G_x, G_y, (uint8_t *)"G"); // 最上面的准星横线数字，意义不明
            // CP_DrawOrDelete_Seven_Graphic(ui_name2, Increase_Graphic, graphic_tpye2, 0, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
            break;
        case 2:
            if (SuperPower.Fire_Flag == 0)
            {
                ZM_color = Color_White;
            }
            else if (SuperPower.Fire_Flag == 1)
            {
                ZM_color = Color_Yellow;
            }
            else if (SuperPower.Fire_Flag == 2)
            {
                ZM_color = Color_Pink;
            }
            // CP_DrawOrDelete_Char(30, Increase_Graphic, 0, ZM_color, ZM_size, ZM_width, ZM_x, ZM_y, char_ZM);
            CP_DrawOrDelete_Char(40, Increase_Graphic, 0, ZM_color, ZM_size, ZM_width, ZM_x, ZM_y, (uint8_t *)"ZM"); // 最上面的准星横线数字，意义不明
                                                                                                                     // CP_DrawOrDelete_Char(40, Increase_Graphic, 0, Color_Yellow, 7, 2, 1000, 470, (uint8_t *)"6/2"); // 最上面的准星横线数字，意义不明

            break;

        default:
            break;
        }
        scan_inc_ui++;
        scan_tim_ui = 1;
        // if (scan_inc_ui % 2 == 0)
        // {
        //     CP_DrawOrDelete_Seven_Graphic(ui_name1, Increase_Graphic, graphic_tpye1, 0, color1, d1[0], d1[1], d1[2], d1[3], d1[4], d1[5], d1[6], d1[7]);
        // }
        // else
        // {
        //     CP_DrawOrDelete_Seven_Graphic(ui_name2, Increase_Graphic, graphic_tpye2, 0, color2, d2[0], d2[1], d2[2], d2[3], d2[4], d2[5], d2[6], d2[7]);
        // }
    }
}
#pragma endregion

#pragma region /*时间片调度器*/
typedef struct
{
    uint8_t run; // 调度标志
    uint16_t TimCount;
    uint16_t TimRload;       // 重载值
    void (*pTaskFunc)(void); // 函数指针，保存任务函数地址
} TaskComps_t;

static TaskComps_t TaskComps[] = // 160hz
    {
        {0, 2, 2, MYMODE_Task},
        {0, 1, 1, BP_Task},
        {0, 1, 1, MCL_task},
        {0, 2, 2, ZM_Task},
        {0, 1, 1, Dynamic_Notach_Param_Task},
        {0, 1, 1, YAW_Task},
        {0, 1, 1, PITCH_Task},
        {0, 16, 16, watchdog_run_Task},
        {0, 2, 2, RGB_Task},
        {0, 5, 5, UI_TASK} // 32hz

};
uint8_t TASK_NUM_MAX = sizeof(TaskComps) / sizeof(TaskComps[0]);
void TaskHandler(void) // while
{
    for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
    {
        if (TaskComps[i].run)
        {
            TaskComps[i].run = 0;
            TaskComps[i].pTaskFunc(); // 执行调度任务
        }
    }
}

void TaskScheduler(void) // 定时器
{
    for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
    {
        if (TaskComps[i].TimCount)
        {
            TaskComps[i].TimCount--;
            if (TaskComps[i].TimCount == 0)
            {
                TaskComps[i].TimCount = TaskComps[i].TimRload;
                TaskComps[i].run = 1;
            }
        }
    }
}
#pragma endregion

#pragma region /*debug*/
uint8_t YAW_M_en = 0;
uint8_t PITCH_M_en = 0;
void motor_enable(void)
{
    YAW_M_en = 1;
    PITCH_M_en = 1;
    MCL_L.en = 1;
    MCL_R.en = 1;
    BP_M.en = 1;

    // /*发射机构*/
    // YAW_M.en = 0;
    // PITCH_M.en = 0;
    // MCL_L.en = 1;
    // MCL_R.en = 1;
    // BP_M.en = 1;

    // /*拨盘测试*/
    // YAW_M.en = 0;
    // PITCH_M.en = 0;
    // MCL_L.en = 0;
    // MCL_R.en = 0;
    // BP_M.en = 1;

    /*全保护*/
    // YAW_M.en = 0;
    // PITCH_M.en = 0;
    // MCL_L.en = 0;
    // MCL_R.en = 0;
    // BP_M.en = 0;
}

float dwt_timeA = 0;
float dwt_timeB = 0;
float dwt_timeSum = 0;
// dwt_timeA = IMU.DWT_Get_time();
// dwt_timeSum = dwt_timeA + dwt_timeB;
// dwt_timeB = IMU.DWT_Get_time();

#pragma endregion
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#pragma region /*main*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
#ifdef DM3507
    PITCH_M.T_MAX = 5;
    PITCH_M.T_MIN = -5;
    PITCH_M.P_MAX = 12.556f;
    PITCH_M.P_MIN = -12.556f;
    PITCH_M.V_MAX = 50;
    PITCH_M.V_MIN = -50;
#endif // DEBUG
    // user_init();
    // mcl_sp_s.set_real();      // 设置单位化速度
    // BPin_tim_slow.set_real(); // 每次修改完调用set_real才生效
    // BPintui_tim_slow.set_real();
    // BPwait_tim_slow.set_real();
    motor_enable();

    HAL_Delay(500);
    HAL_Delay(5);

    CAN_1.Init(0, 0);
    CAN_2.Init(1, 1);
    /*初始化后要等待一会在发送*/
    HAL_Delay(600);
    PITCH_M.DM_Start();
    // PITCH_M.mit_kd = 0.01f;
    // HAL_Delay(1);
    // MCLDM_L.DM_Start();
    // HAL_Delay(1);
    // MCLDM_R.DM_Start();
    // HAL_Delay(1);

    HAL_Delay(1);

    /*裁判系统*/
    CP_System_Init();
    HAL_Delay(0);
    IMU.state = IMU.Init(1);
    HAL_Delay(1);

    // YK.VT13_Init();
    YK.VT13_2Init();

    HAL_Delay(0);

    // YK.DT16_Init();
    YK.DT16_2Init();
    HAL_Delay(0);

    // Mini_PC_Init();
    Mini_PC_UART_Init();
    // HAL_TIM_Base_Start_IT(&htim1); // 逻辑保险
    HAL_Delay(0);
    // HAL_TIM_Base_Start_IT(&htim6); // 遥控器看门狗 10hz
    // HAL_Delay(0);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // RGB_PWM
    RGB.RGB_UI_Init();

    HAL_TIM_Base_Start_IT(&htim7); //  160hz启动任务
    HAL_Delay(0);
    HAL_TIM_Base_Start_IT(&htim12); // minipc

    // HAL_TIM_Base_Start_IT(&htim8); // 40hz 0x13id板间通信
    // HAL_Delay(0);
    // HAL_TIM_Base_Start_IT(&htim9); // 2khz领控电机,can2发送
    // imu 1khz htim12
    // RGB_UI.RGB_UI_Init();

    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //    SERVO_Init; // 舵机启动
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // Mini_PC_UART_Init();
    // Laser_deal(GPIO_PIN_SET); // 激光启动
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        /*板间通信底盘状态标志*/
        // YT_Tx_static_Flag = (bool)deploy_flag ? (YT_Tx_static_Flag | 0x0001) : (YT_Tx_static_Flag & (uint16_t)~1);
        // YT_Tx_static_Flag = XTL_flag ? (YT_Tx_static_Flag | 0x0002) : (YT_Tx_static_Flag & (uint16_t)~2);

        // if (YK.Pressed_Check(KEY_PRESSED_Z) && YK.Pressed_Check(KEY_PRESSED_CTRL)) // 按下z和ctrl，进行软复位
        // {
        //     stm32_reset();
        // }

        TaskHandler();
        /*wait for interrupt*/
        // __WFI();
        // HAL_Delay(5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#pragma endregion
/*********************************************            can1 ： 6摩擦轮 拨盘，Y轴   通信         ********************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 8khz
{
    if (CAN_1.Receive(CAN_RX_FIFO0) == HAL_OK) // 如果can1可以用// 摩擦轮闭环状态的时候再更新电机数据并算pid
    {
        // BP_17mm_task();
        CAN1_ctrl();
    }
}

/*********************************************            can2  拨盘 yaw pitch minipitch 板间tx  摩擦轮     小pitch轴         ********************************************/
// 云台板间通信can接收的优先级应该比发送要低，对信息实时性要求不高
// uint32_t yaw_cnt = 0;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 4khz
{
    if (CAN_2.Receive(CAN_RX_FIFO1) == HAL_OK)
    {
        if (PITCH_M.DM_update() == HAL_OK)
        {
            PITCH_M.update_xPI_mang_inf_basic_zeromang(4.0f);
            PITCH_motor_pid();
        }
        else if (YAW_M.update() == HAL_OK)
        {
            YAW_M.NSQD_8192mang_inf();
            // Test_J();

            YAW_motor_pid();
        }
        // yaw_cnt++;
    }
}
volatile uint32_t tim1cnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 2250hz
{
    // cnt3++;
    if (htim == &htim12) // 1khz
    {

        ZM_TJtim_LPandSend_deal();
#ifdef DEBUG
        test_pid_set_controler_imu();
        /*法4反馈传入
        格式可以和法1一样*/

        test_set2param_it();
#endif // DEBUG

        // ZM_anti_tim_deal();
    }
    /*瓴控电机*/
    /*对于先发后收的电机，一个定时器处理所有电机最好*/
    else if (htim == &htim9) // 2000hz,CAN2发送，瓴控电机,中断优先级最高，防止被打断
    {
        static uint8_t can_scan = 1;
        if (can_scan) // 2KHZ定时器频率2分频1k
        {
            IMU.analyse(); // 51.5us
            // IMU_NF();

            // PITCH_imu_pid(); // 1.15us
            PITCH_imuCtrl();
            PITCH_M.if_en();

            PITCH_M.DM_MIT(PITCH_M.mit_pos, PITCH_M.mit_vel, PITCH_M.mit_kp, PITCH_M.mit_kd, (PITCH_M_en ? PITCH_M.mit_torq : 0));
            // MCLDM_R.DM_MIT(MCLDM_L.mit_pos, MCLDM_L.mit_vel, MCLDM_L.mit_kp, MCLDM_L.mit_kd, MCLDM_L.mit_torq);
            // PITCH_M.DM_MIT(PITCH_M.mit_pos, PITCH_M.mit_vel, PITCH_M.mit_kp, PITCH_M.mit_kd, 0);
            Yaw_imuCtrl();

            /*定时完直接发当然最快了，接收完再算pid咯*/
            can_scan = 0;
        }
        else
        {
            YAW_M.if_en();
            // CAN_2.Send_RM(0x1FF, 0, 0, 0, 0); // 0.7us
            CAN_2.Send_RM(0x1FF - YAW_SMC_or_PID, 0, (YAW_M_en ? YAW_M.out : 0), 0, 0); // 0.7us

            BP_M.if_en();
            MCL_R.if_en();
            MCL_L.if_en();
            CAN_1.Send_RM(0x1FF, (int16_t)BP_M.out, 0,
                          (int16_t)MCL_R.out,
                          (int16_t)MCL_L.out); // 1khz
            // CAN_1.Send_RM(0x1FF, (int16_t)BP_M.out, 0,
            //               0,
            //               0); // 1khz

            can_scan = 1;
        }
    }

    else if (htim == &htim7) // 160hz,发射遥控器
    {
        TaskScheduler();
        // BP_loss_tim_deal();
        // deploy_mode_tim_deal();
    }
    else if (htim == &htim1) // 50hz  决策保险   改摩擦轮转速  py轴
    {
        tim1cnt++;
        // protect_tim_deal();
    }
    else if (htim == &htim6) // 10hz 看门狗
    {
        // YK.DT16_watchdog_run();
        // YK.VT13_watchdog_run();
    }
    else if (htim == &htim8) // 40HZ
    {

        // can2bjtx013 = 1;
    }
}
#pragma region /*NO USE*/
// 15hz 9216
// uint8_t ctrl_state = 1; // 控制模式
// int16_t bp_output = 0;  // 电机实际输出
// int16_t force = 0;      // 前馈值

// int16_t force_f = 0; // 600;
// int16_t force_b = 0; //-400;

// PID_class M2006_mang_pid(1, 0, 0, 14000, 0, 0, 14000);
// PID_class M2006_sp_pid(2, 0, 0, 10000, 0, 0, 10000);
// float M2006_tsp_slow = 0; // pid斜坡速度
// float M2006_targe_sp = 0; //(20.0f) * (8192.0f * 36.0f / 8.0f) / 60.0f; // 614.4//pid闭环目标速度
// int32_t M2006_targe_mang = 0;
// void BP_17mm_task(void)
// {
//     BP_M.NSQD_8192mang_inf();
//     if (ctrl_state == 1) // 有卡弹处理
//     {
//         if (!BP_tui_flag)
//         {
//             M2006_targe_sp = (bp_hz) * (8192.0f * 36.0f / 8.0f) / 60.0f;
//             force_en_flag = 1;
//         }
//         else
//         {
//             M2006_targe_sp = BP_tui_sp;
//             force_en_flag = -1;
//             BP_tui_tim--;
//             if (BP_tui_tim <= 0)
//             {
//                 BP_tui_flag = 0;
//                 BP_tui_tim = 100;
//             }
//         }
//         // F_slow(&M2006_tsp_slow, M2006_targe_sp, 5, 5, 50);
//         M2006_sp_pid.PID_update_LP(M2006_targe_sp, BP_M.sp);
//         BP_M.out = M2006_sp_pid.OUT_PID + force;
//     }
//     else if (ctrl_state == 2) // 无卡单处理
//     {
//         M2006_targe_sp = (bp_hz) * (8192.0f * 36.0f / 8.0f) / 60.0f;
//         force_en_flag = 1;
//         M2006_sp_pid.PID_update_LP(M2006_targe_sp, BP_M.sp);
//         BP_M.out = M2006_sp_pid.OUT_PID + force;
//         // M2006_mang_pid.PID_update_LP(BP_targe, BP_M.mang, 1);
//         // M2006_sp_pid.PID_update_LP(M2006_targe_sp, BP_M.sp, 1);
//     }
//     else if (ctrl_state == 4)
//     {
//         M2006_targe_sp = YK.yaogan.ch0 * 10;
//         M2006_sp_pid.PID_update_LP(M2006_targe_sp, BP_M.sp);
//         BP_M.out = M2006_sp_pid.OUT_PID;
//     }
//     else if (ctrl_state == 3)
//     {
//         BP_M.out = 0;
//     }
//     else
//     {
//         BP_M.out = 0;
//     }
// }

// void force_enable(int8_t en)
// {

//     if (en > 0)
//     {
//         force = 230 + bp_hz * 20; //+force_f
//     }
//     else if (en < 0)
//     {
//         force = -230 + (BP_tui_sp / 614.4f) * 20; //+force_b
//     }
//     else
//     {
//         force = 0;
//     }
// }
// void BP_while_deal(void)
// {

//     // 遥控调速
//     if (YKmode == DAN_YUN_TAI_MODE || YKmode == SHANG_XIA_MODE)
//     {
//         int16_t ifud_BP_up = 0;
//         int16_t ifud_BP_down = 0;

//         if (YK.yaogan.ch0 > 600)
//         {
//             ifud_BP_up = 1;
//         }
//         else
//         {
//             ifud_BP_up = 0;
//         }

//         if (YK.yaogan.ch0 < -600)
//         {
//             ifud_BP_down = 1;
//         }
//         else
//         {
//             ifud_BP_down = 0;
//         }

//         UD_BPsp_up.updata(ifud_BP_up);
//         UD_BPsp_down.updata(ifud_BP_down);
//         if (UD_BPsp_up.UD_data == UpDown_check_rising && bp_hz < 20)
//         {
//             bp_hz++;
//         }
//         if (UD_BPsp_down.UD_data == UpDown_check_rising && bp_hz > 0)
//         {
//             bp_hz--;
//         }
//     }

//     if (YK.dt16_signal_flag == 0) // 没接遥控
//     {
//     }
//     else
//     {
//         if (YKmode == DAN_YUN_TAI_MODE)
//         {
//             ctrl_state = 1; // 0
//         }
//         else if (YKmode == SHANG_XIA_MODE)
//         {
//             ctrl_state = 2; // 1
//         }
//         else if (YKmode == SHUANG_ZHONG_MODE || YKmode == DIAO_SHE_MODE)
//         {
//             ctrl_state = 4; // 2
//         }
//         else
//         {
//             ctrl_state = 3; // 3
//         }
//     }

//     force_enable(force_en_flag);
// }
#pragma endregion
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
