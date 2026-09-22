/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
    /*ö��ֻ�ǡ�������*/
    typedef enum
    {
        GYRO_TYPE = 1,
        M_SP_TYPE = 0,
        ANGLE_TYPE = 1,
        MANG_TYPE = 0,
        IMU_TYPE = 1,
        MOTOR_TYPE = 0
    } PID_Ctrl_Ref_e; /*imu���п��ޣ����ǵ�����������У����Ե������Ϊ0��imuΪ1������forѭ���ᵼ������Խ�磡*/
    /* ����/�����л�&�Զ���ʼ������ṹ��?
    �����÷�����Ϲ��ܺ���ʹ��?
    pid�����ʽ����������û����������ֻ����ģ�黯���Լ���ض����
    ��֪����ô����Բο�����
     */
    typedef struct
    {
        uint8_t param_type;      // ������������
        uint8_t ref_type;        // ������������
        const uint8_t param_num; // ��������/��������
        const uint8_t ref_num;   // ������Դ����

        uint8_t protect_flag; // �ܿ���
        /*��3ȱʡ��Ĭ��0*/
        uint8_t log_err;                   // ���β�������־��Ϣ
        uint32_t param_index_overstep_log; // ��������Խ����־
        uint32_t ref_index_overstep_log;   // ��������Խ����־

    } PID_Ctrl_Index;

    extern void ZM_rx_Deal(void);
    extern void ZM_2rx_Deal(DMA_HandleTypeDef *hdma);
    extern void ZM_2rxTJ_Deal(DMA_HandleTypeDef *hdma);
    // extern uint8_t ZM_Status;
    extern volatile float Y_Angle_ZM;
    extern volatile float Y_Gyro_ZM;
    extern volatile float Y_AGyro_ZM;
    extern volatile float P_Angle_ZM;
    extern volatile float P_Gyro_ZM;
    extern volatile float P_AGyro_ZM;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define YK_Pin GPIO_PIN_6
#define YK_GPIO_Port GPIOC
#define YKC7_Pin GPIO_PIN_7
#define YKC7_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
