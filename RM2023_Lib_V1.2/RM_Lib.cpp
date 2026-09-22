/**
  ******************************************************************************
  * File Name				: RM_Lib.cpp
  * Description			: RM c++库，包含USART，CAN，麦轮底盘算法，PID，遥控器，
  陀螺仪，snail电调，裁判系统交互，达妙4310，瓴控6010等库函数
  * Version					: v1.4
  * Creation Date		: 2025.4.12
  ******************************************************************************
  */
#include "RM_Lib.hpp"
// #include "stdarg.h"
#include "string.h"
/**************************************** USART **********************************************************/
uint8_t info_ubuf[USART_BUF_SIZE];
// uint8_t INFO_DMA(const char *fmt, ...)
// {
//     uint8_t err_flag = HAL_OK;
//     static uint8_t tx_buf[256] = {0};
//     static va_list ap;
//     static uint16_t len;
//     va_start(ap, fmt);

//     // return length of string
//     // 返回字符串长度
//     len = vsprintf((char *)tx_buf, fmt, ap);

//     va_end(ap);
//     err_flag = HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
//     return err_flag;
// }
float Rad2Angle(float Rad)
{
    return Rad * 180.0f / m_pi;
}

float Angle2Rad(float Angle)
{
    return Angle * m_pi / 180.0f;
}
#pragma region /*CAN*/
#ifdef __CAN_H__
/**************************************** C A N **********************************************************/
void USER_CAN::Init(uint16_t t, uint16_t x)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    this->FreeTxNum = 0;
    this->TxHeader.DLC = 8;
    this->TxHeader.IDE = CAN_ID_STD;
    this->TxHeader.RTR = CAN_RTR_DATA;

    sFilterConfig.FilterBank = t;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    if (this->FIFO == 0)
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    else
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;

    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = x;

    if (HAL_CAN_ConfigFilter(this->hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_Start(this->hcan) != HAL_OK)
        Error_Handler();

    if (this->FIFO == 0)
    {
        if (HAL_CAN_ActivateNotification(this->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
            Error_Handler();
    }
    else
    {
        if (HAL_CAN_ActivateNotification(this->hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
            Error_Handler();
    }
    can_send_error_state = HAL_OK; // 初始化没问题标志
}
HAL_StatusTypeDef USER_CAN::STD_ID_Send(uint16_t Id, uint8_t *pData)
{
    if (Id > 0x7FF) // 11位ID
    {
        can_user_error_cnt++;
        return HAL_ERROR;
    }
    this->TxHeader.StdId = Id;
    this->TxHeader.IDE = CAN_ID_STD; // 如果此参数不赋值，无法判断是标准ID还是扩展ID
    // uint8_t cntbuf = 0;
    // do
    // {

    //     cntbuf++;
    //     if (cntbuf == 2)
    //     {
    //         this->test_busy_cnt++;
    //     }
    this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);
    /*正常情况下不会出现3个发送邮箱都占满的情况，
    优化写法是可以做到基本都有2个空闲邮箱的
    对于can分配要多加细心，如果发送邮箱满了
    可能总线负载过大，挂载电机过多，用can分析仪找找问题
    可能can发送分配有问题，同时多次调用了can发送函数，比如3次以上
    可能can外设发送失败，一般是电路吧canH，canL不小心反接，短路，can电阻不对
    */
    if (this->FreeTxNum == 0)
    {
        // HAL_CAN_ActivateNotification(this->hcan, CAN_IT_TX_MAILBOX_EMPTY); // 发送完毕中断
        can_send_busy_cnt++; // 邮箱满计次
        // return HAL_BUSY;
    }
    // } while (this->FreeTxNum == 0);
    can_send_error_state = HAL_CAN_AddTxMessage(this->hcan, &this->TxHeader, pData, &this->TxMailbox);
    if (can_send_error_state != HAL_OK)
    {
        can_send_error_cnt++; // 发送错误计次
    }
    return can_send_error_state;
}

// 第一个参数，标准帧0，扩展帧4
HAL_StatusTypeDef USER_CAN::Send8Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint8_t uint8_d1, uint8_t uint8_d2, uint8_t uint8_d3, uint8_t uint8_d4, uint8_t uint8_d5, uint8_t uint8_d6, uint8_t uint8_d7, uint8_t uint8_d8)
{
    uint8_t pData[8];
    pData[0] = uint8_d1;
    pData[1] = uint8_d2;
    pData[2] = uint8_d3;
    pData[3] = uint8_d4;
    pData[4] = uint8_d5;
    pData[5] = uint8_d6;
    pData[6] = uint8_d7;
    pData[7] = uint8_d8;
    if (CAN_ID_TYPE == CAN_ID_STD)
    {
        motor_send_error_state = this->STD_ID_Send((uint16_t)Id, pData);
    }
    else if (CAN_ID_TYPE == CAN_ID_EXT)
    {
        motor_send_error_state = this->EXT_ID_Send(Id, pData);
    }
    else
    {
        can_user_error_cnt++; // 不是标准帧和扩展帧
        return HAL_ERROR;
    }
    if (motor_send_error_state != HAL_OK)
    {
        can_send_Xbit_error_cnt++;
    }

    return motor_send_error_state;
}

HAL_StatusTypeDef USER_CAN::Send16Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint16_t uint16_d1, uint16_t uint16_d2, uint16_t uint16_d3, uint16_t uint16_d4)
{
    uint8_t pData[8];
    pData[0] = (uint8_t)(uint16_d1 >> 8); // 高字节在前，低字节在后，大端模式
    pData[1] = (uint8_t)uint16_d1;
    pData[2] = (uint8_t)(uint16_d2 >> 8);
    pData[3] = (uint8_t)uint16_d2;
    pData[4] = (uint8_t)(uint16_d3 >> 8);
    pData[5] = (uint8_t)uint16_d3;
    pData[6] = (uint8_t)(uint16_d4 >> 8);
    pData[7] = (uint8_t)uint16_d4;
    if (CAN_ID_TYPE == CAN_ID_STD)
    {
        motor_send_error_state = this->STD_ID_Send((uint16_t)Id, pData);
    }
    else if (CAN_ID_TYPE == CAN_ID_EXT)
    {
        motor_send_error_state = this->EXT_ID_Send(Id, pData);
    }
    else
    {
        can_user_error_cnt++; // 不是标准帧和扩展帧
        return HAL_ERROR;
    }
    if (motor_send_error_state != HAL_OK)
    {
        can_send_Xbit_error_cnt++;
    }

    return motor_send_error_state;
}
HAL_StatusTypeDef USER_CAN::Send32Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint32_t uint32_d1, uint32_t uint32_d2)
{
    uint8_t pData[8];
    pData[0] = (uint8_t)(uint32_d1 >> 24); // 高字节在前，低字节在后，大端模式
    pData[1] = (uint8_t)(uint32_d1 >> 16);
    pData[2] = (uint8_t)(uint32_d1 >> 8);
    pData[3] = (uint8_t)uint32_d1;
    pData[4] = (uint8_t)(uint32_d2 >> 24);
    pData[5] = (uint8_t)(uint32_d2 >> 16);
    pData[6] = (uint8_t)(uint32_d2 >> 8);
    pData[7] = (uint8_t)uint32_d2;
    if (CAN_ID_TYPE == CAN_ID_STD)
    {
        motor_send_error_state = this->STD_ID_Send((uint16_t)Id, pData);
    }
    else if (CAN_ID_TYPE == CAN_ID_EXT)
    {
        motor_send_error_state = this->EXT_ID_Send(Id, pData);
    }
    else
    {
        can_user_error_cnt++; // 不是标准帧和扩展帧
        return HAL_ERROR;
    }
    if (motor_send_error_state != HAL_OK)
    {
        can_send_Xbit_error_cnt++;
    }

    return motor_send_error_state;
}
HAL_StatusTypeDef USER_CAN::Send64Bit(uint32_t CAN_ID_TYPE, uint32_t Id, uint64_t uint64_data)
{
    uint8_t pData[8];
    pData[0] = (uint8_t)(uint64_data >> 56); // 高字节在前，低字节在后，大端模式
    pData[1] = (uint8_t)(uint64_data >> 48);
    pData[2] = (uint8_t)(uint64_data >> 40);
    pData[3] = (uint8_t)(uint64_data >> 32);
    pData[4] = (uint8_t)(uint64_data >> 24);
    pData[5] = (uint8_t)(uint64_data >> 16);
    pData[6] = (uint8_t)(uint64_data >> 8);
    pData[7] = (uint8_t)uint64_data;
    if (CAN_ID_TYPE == CAN_ID_STD)
    {
        motor_send_error_state = this->STD_ID_Send((uint16_t)Id, pData);
    }
    else if (CAN_ID_TYPE == CAN_ID_EXT)
    {
        motor_send_error_state = this->EXT_ID_Send(Id, pData);
    }
    else
    {
        can_user_error_cnt++; // 不是标准帧和扩展帧
        return HAL_ERROR;
    }
    if (motor_send_error_state != HAL_OK)
    {
        can_send_Xbit_error_cnt++;
    }

    return motor_send_error_state;
}
HAL_StatusTypeDef USER_CAN::Send_RM(uint16_t Id, int16_t M_201, int16_t M_202, int16_t M_203, int16_t M_204)
{
    uint8_t pData[8];
    pData[0] = (uint8_t)(M_201 >> 8); // 高字节在前，低字节在后，大端模式
    pData[1] = (uint8_t)M_201;
    pData[2] = (uint8_t)(M_202 >> 8);
    pData[3] = (uint8_t)M_202;
    pData[4] = (uint8_t)(M_203 >> 8);
    pData[5] = (uint8_t)M_203;
    pData[6] = (uint8_t)(M_204 >> 8);
    pData[7] = (uint8_t)M_204;

    motor_send_error_state = this->STD_ID_Send(Id, pData);
    if (motor_send_error_state != HAL_OK)
    {
        can_send_RM_error_cnt++;
    }
    return can_send_error_state;
}

HAL_StatusTypeDef USER_CAN::Receive(uint32_t fifo)
{
    // if (fifo != FIFO)
    // {
    //     can_user_error_cnt++;
    //     return HAL_ERROR; // 防止应人疏忽，放错回调函数，造成意想不到的后果
    // }

    can_getRxMessage_error_state = HAL_CAN_GetRxMessage(this->hcan, FIFO, &this->RxHeader, this->rx_buf);
    if (can_getRxMessage_error_state != HAL_OK)
    {
        can_getRxMessage_error_cnt++; // 接收错误计次
    }
    return can_getRxMessage_error_state;
}

HAL_StatusTypeDef USER_CAN::EXT_ID_Send(uint32_t Id, uint8_t *pData)
{
    if (Id > 0x1FFFFFFF) // 11位ID
    {
        can_user_error_cnt++;
        return HAL_ERROR;
    }
    this->TxHeader.IDE = CAN_ID_EXT;
    // this->can_rev->TxHeader.RTR = CAN_RTR_DATA;
    this->TxHeader.ExtId = Id; // 取低29位
    this->TxHeader.DLC = 8;
    // this->TxHeader.IDE = CAN_ID_EXT;
    // this->TxHeader.ExtId = Id;
    // this->TxHeader.DLC = 8;

    // uint32_t timeout = HAL_GetTick() + 5; // Set a timeout of 10ms
    // do
    // {
    //     this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);
    //     if (HAL_GetTick() > timeout)
    //     {
    //         this->can_send_busy_cnt++;
    //         return HAL_TIMEOUT;
    //     }
    // } while (this->FreeTxNum == 0);

    this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);
    /*正常情况下不会出现3个发送邮箱都占满的情况，
    优化写法是可以做到基本都有2个空闲邮箱的
    对于can分配要多加细心，如果发送邮箱满了
    可能总线负载过大，挂载电机过多，用can分析仪找找问题
    可能can发送分配有问题，同时多次调用了can发送函数，比如3次以上
    可能can外设发送失败，一般是电路吧canH，canL不小心反接，短路，can电阻不对
    */
    if (this->FreeTxNum == 0)
    {
        // HAL_CAN_ActivateNotification(this->hcan, CAN_IT_TX_MAILBOX_EMPTY); // 发送完毕中断
        can_send_busy_cnt++; // 邮箱满计次
        // return HAL_BUSY;
    }

    can_send_error_state = HAL_CAN_AddTxMessage(this->hcan, &this->TxHeader, pData, &this->TxMailbox);
    if (can_send_error_state != HAL_OK)
    {
        can_send_error_cnt++;
    }
    return can_send_error_state;
}
#endif
#pragma endregion

#pragma region /*RM_M*/
#ifdef __CAN_H__
HAL_StatusTypeDef MOTOR_RM::update(void)
{
    if (this->can_rev->RxHeader.StdId != this->ID)
    {
        return HAL_ERROR;
    }
    this->mang = (int16_t)((this->can_rev->rx_buf[0] << 8) | this->can_rev->rx_buf[1]);
    this->sp = (int16_t)((this->can_rev->rx_buf[2] << 8) | this->can_rev->rx_buf[3]);
    this->AT_current = (int16_t)((this->can_rev->rx_buf[4] << 8) | this->can_rev->rx_buf[5]);
    this->temp = this->can_rev->rx_buf[6];
    loss_time = 0;
    online = 1;

    // this->update_mang_inf();//不要写函数里面，有些电机不需要位置信息，需要的再加上
    return HAL_OK;
}

void MOTOR_RM::update_mang_inf(void)
{
    if (this->first == 0) // 初次上电或者需要重新标点
    {
        this->mang_inf = 0;
        this->Last_mang = this->mang;
        this->first = 1;
    }

    if ((this->mang - this->Last_mang) < -5000)
    {
        this->mang_inf += 8191;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 5000)
    {
        this->mang_inf -= 8191;
        this->motor_number--;
    }
    this->mang_inf -= this->Last_mang;
    this->mang_inf += this->mang;
    this->Last_mang = this->mang;
}
void MOTOR_RM::First(void)
{
    if (this->first == 0) // 如果是第一次，或者first=0
    {
        this->mang_inf = 0;
        this->nsqd_8192xCnt_mang = 0;
        this->first_mang_inc = this->mang; // 记录第一次增量
        this->Last_mang = this->mang;
        this->first = 1;
    }
}

void MOTOR_RM::NSQD_8192mang_inf(void)
{
    First();

    if ((this->mang - this->Last_mang) < -5000)
    {
        this->nsqd_8192xCnt_mang += 8192;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 5000)
    {
        this->nsqd_8192xCnt_mang -= 8191; // 可能是8192
        this->motor_number--;
    }
    this->mang_inf = nsqd_8192xCnt_mang + this->mang - this->first_mang_inc;
    this->Last_mang = this->mang;
}
#endif
#pragma endregion

#pragma region /*LK_M*/
#ifdef __CAN_H__
/*************************************  瓴控电机  ************************************************/
/*
tips：使用瓴控电机的时候记得发送信号给电机，不然他不会反馈信号给你
使用广播模式或者单电机模式需要在上位机上设置，使用广播模式不能使用单电机的命令，还有他不能在can中断里面发送数据
读取数据的时候一定需要这个函数Motor_LK6010.LK_Broadcast_update()，不然会没有数据
*/

HAL_StatusTypeDef MOTOR_LK::LK_Close(uint16_t Id) // 电机关闭命令
{
    uint8_t pData[8];
    pData[0] = 0x80;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;

    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_LK::LK_Start(uint16_t Id) // 电机运行命令,
{
    uint8_t pData[8];
    pData[0] = 0x88;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_LK::LK_Stop(uint16_t Id) // 电机停止命令
{
    uint8_t pData[8];
    pData[0] = 0x81;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_MIT_ClossControl(uint16_t Id) // 转矩闭环控制命令 数值范围-2048~ 2048  对应MG电机实际转矩电流范围-33A~33A  电机在收到命令后回复主机
{
    uint8_t pData[8];

    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_LK::LK_SP_ClossControl(uint16_t Id) // 速度闭环控制命令
{
    uint8_t pData[8];
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_LK::LK_More_Mang_ClossControl_1(uint16_t Id) // 多圈位置闭环控制命令 1
{
    uint8_t pData[8];
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_More_Mang_ClossControl_2(uint16_t Id) // 多圈位置闭环控制命令 2
{
    uint8_t pData[8];
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Alone_Mang_ClossControl_1(uint16_t Id) // 单圈位置闭环控制命令 1
{
    uint8_t pData[8];
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Alone_Mang_ClossControl_2(uint16_t Id) // 单圈位置闭环控制命令 2
{
    uint8_t pData[8];
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_PID(uint16_t Id) // 读取电机PID
{
    uint8_t pData[8];
    pData[0] = 0x30;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_Acc(uint16_t Id) // 读取电机的加速度
{
    uint8_t pData[8];
    pData[0] = 0x33;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_Encoder(uint16_t Id) // 读取电机的编码器
{
    uint8_t pData[8];
    pData[0] = 0x90;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_More_Mang(uint16_t Id) // 读取电机多圈角度
{
    uint8_t pData[8];
    pData[0] = 0x92;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_Alone_Mang(uint16_t Id) // 读取电机单圈角度
{
    uint8_t pData[8];
    pData[0] = 0x94;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_MotorState_1(uint16_t Id) // 读取电机状态1，该命令读取当前电机的温度、电压和错误状态标志
{
    uint8_t pData[8];
    pData[0] = 0x9A;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Clear_MotorMismark(uint16_t Id) // 该命令清除当前电机的错误状态，电机收到后返回
{
    uint8_t pData[8];
    pData[0] = 0x9B;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_MotorState_2(uint16_t Id) // 读取电机状态2，该命令读取当前电机的温度、电压、转速、编码器位置。
{
    uint8_t pData[8];
    pData[0] = 0x9C;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_LK::LK_Read_MotorState_3(uint16_t Id) // 读取电机状态3，该命令读取当前电机的温度和相电流数据。
{
    uint8_t pData[8];
    pData[0] = 0x9D;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
     这个是广播模式控制，可以一条命令同时控制最多 4 个电机

    .需要在上位机中开启广播模式，并将 CAN 总线波特率设置为 500Kbps 以上
    .同时包含了 4 个电机的力矩电流控制值。控制量为 16bit 整形数据。数据范围-2000 ~ +2000,对应实际转矩电流范围-32A~32A
    .例如，主控向#1 电机发送力矩电流 100、向#3 电机发送力矩电流-100 的命令数据如下：
       64 00 00 00 9C FF 00 00
    .驱动回复：和单电机力矩控制命令的回复相同，各个电机回复命令的报文格式相同，各个电机根据 ID 从小到大依次回复反馈的ID是0x140 + ID(1~4)
    发送的ID是0x208

*/

HAL_StatusTypeDef USER_CAN::Broadcast_Send_LK(int16_t M_201, int16_t M_202, int16_t M_203, int16_t M_204) // 输入的数不要超过+-2000
{
    uint8_t pData[8];
    pData[0] = *(uint8_t *)(&M_201); // 低字节在前，高字节在后,小端模式
    pData[1] = *((uint8_t *)(&M_201) + 1);
    pData[2] = *(uint8_t *)(&M_202);
    pData[3] = *((uint8_t *)(&M_202) + 1);
    pData[4] = *(uint8_t *)(&M_203);
    pData[5] = *((uint8_t *)(&M_203) + 1);
    pData[6] = *(uint8_t *)(&M_204);
    pData[7] = *((uint8_t *)(&M_204) + 1);

    motor_send_error_state = this->STD_ID_Send(0x280, pData);
    if (motor_send_error_state != HAL_OK)
    {
        can_send_LK_error_cnt++; // 领控电机发送错误计次
    }
    return motor_send_error_state;
}

HAL_StatusTypeDef MOTOR_LK::LK_Broadcast_update(void)
{
    if (this->can_rev->RxHeader.StdId != this->ID) // 电机id不同就改这个电机2就改142
    {
        return HAL_ERROR;
    }
    order = (this->can_rev->rx_buf[0]);                                    // 命令字节
    temp = (this->can_rev->rx_buf[1]);                                     // 电机温度
    iq = ((this->can_rev->rx_buf[3]) << 8) + (this->can_rev->rx_buf[2]);   // 电流
    sp = ((this->can_rev->rx_buf[5]) << 8) + (this->can_rev->rx_buf[4]);   //
    mang = ((this->can_rev->rx_buf[7]) << 8) + (this->can_rev->rx_buf[6]); //
    // this->update_65535mang_inf_free();//需要选择合适的过圈检测类型不在此做 // 得到多圈角度
    return HAL_OK;
}
/*
角度累加原理：
如果前一秒的的角度与这次角度相差大于+-5000，说明过圈，于是在原来的基础上相应的加减一圈的角度就好
*/
void MOTOR_LK::update_65535mang_inf_free(void)
{
    if (this->first == 0)
    {
        this->first = 1;
        this->Last_mang = this->mang;
    }

    if ((this->mang - this->Last_mang) < -5000)
    {
        this->mang_inf += 65535;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 5000)
    {
        this->mang_inf -= 65535;
        this->motor_number--;
    }
    this->mang_inf -= this->Last_mang;
    this->mang_inf += this->mang;
    this->Last_mang = this->mang;
}
void MOTOR_LK::update_65535mang_inf_basic_zeromang(void)
{
    if (this->first == 0)
    {
        this->Last_mang = this->mang;
        this->first = 1;
    }

    if ((this->mang - this->Last_mang) < -32768)
    {
        this->nsqd_65535xCnt_mang += 65535;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 32768)
    {
        this->nsqd_65535xCnt_mang -= 65535;
        this->motor_number--;
    }
    // this->mang_inf -= this->Last_mang;
    this->mang_inf = this->nsqd_65535xCnt_mang + this->mang;
    this->Last_mang = this->mang;
}
/*************************************  以上是瓴控电机  ************************************************/
#endif
#pragma endregion

#pragma region /*DM_M*/
#ifdef __CAN_H__
/*************************************  以下是达妙电机  ************************************************/

/*
tips:   这个达妙电机有时候重新烧录代码后的1s之内不能动电机，不然会死机，尽量不要频繁发送电机启动命令,该达妙电机的库可能存在问题需要解决，程序的写法尽量和官方的技术手册一样
使用DM4310的时候要在main函数设置电机原点然后启动电机，达妙电机会一直发送报文给我们
第一个数据是err，内容代表如下：
*/
HAL_StatusTypeDef MOTOR_DM::DM_Start(void) // 进入电机  发送指令
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFC;
    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
void MOTOR_DM::DM_Param_Set(float p_max, float v_max, float t_max)
{
    P_MAX = p_max;
    P_MIN = -p_max;
    V_MAX = v_max;
    V_MIN = -v_max;
    T_MAX = t_max;
    T_MIN = -t_max;
}

HAL_StatusTypeDef MOTOR_DM::DM_End(void) // 退出电机   发送指令
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFD;
    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_DM::DM_Savezero(void) // 保存位置零点   发送指令
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFE;
    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief  MIT模式控下控制帧,kp=0,kd不为0（kd给0会震荡）
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
HAL_StatusTypeDef MOTOR_DM::DM_MIT(float _pos, float _vel, float _KP, float _KD, float _torq) // MIT 模式
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint8_t pData[8];
    pData[0] = (pos_tmp >> 8);
    pData[1] = pos_tmp;
    pData[2] = (vel_tmp >> 4);
    pData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    pData[4] = kp_tmp;
    pData[5] = (kd_tmp >> 4);
    pData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    pData[7] = tor_tmp;
    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
/**
 * @brief帧 ID 为设定的 CAN ID 值加上 0x100 的偏移
 * @param _pos：位置给定，浮点型，低位在前，高位在后
 * @param _vel：速度给定，浮点型，低位在前，高位在后
 * @param
 * @param 此处发送命令的 CAN ID 是 0x100+ID。速度给定是梯形加速度运行下最高速度的，即为匀速段的速度值。
 */
HAL_StatusTypeDef MOTOR_DM::DM_POS(float _pos, float _vel) // 位置速度模式
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    uint8_t pData[8];
    pData[0] = *pbuf;
    pData[1] = *(pbuf + 1);
    pData[2] = *(pbuf + 2);
    pData[3] = *(pbuf + 3);
    pData[4] = *vbuf;
    pData[5] = *(vbuf + 1);
    pData[6] = *(vbuf + 2);
    pData[7] = *(vbuf + 3);
    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */

HAL_StatusTypeDef MOTOR_DM::DM_VEL(float _vel) // 速度模式
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&_vel;

    uint8_t pData[4];
    pData[0] = *vbuf;
    pData[1] = *(vbuf + 1);
    pData[2] = *(vbuf + 2);
    pData[3] = *(vbuf + 3);

    motor_send_state = this->can_rev->STD_ID_Send(CAN_ID, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
void MOTOR_DM::update_xPI_mang_inf_basic_zeromang(float x_pi)
{
    if (this->first == 0)
    {
        this->Last_mang = this->mang;
        this->first = 1;
    }

    if ((this->mang - this->Last_mang) < -x_pi * PI)
    {
        this->nsqd_8PI_Cnt_mang += 8 * PI;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > x_pi * PI)
    {
        this->nsqd_8PI_Cnt_mang -= 8 * PI;
        this->motor_number--;
    }
    this->mang_inf = this->nsqd_8PI_Cnt_mang + this->mang;
    this->Last_mang = this->mang;
}
HAL_StatusTypeDef MOTOR_DM::DM_update(void) // 得到数据
{
    if (this->can_rev->RxHeader.StdId != MASTER_ID)
    {
        return HAL_ERROR;
    }
    state_id = (this->can_rev->rx_buf[0]) & 0x0F;
    ERR = (this->can_rev->rx_buf[0]) >> 4;
    p_int = ((this->can_rev->rx_buf[1] << 8) | this->can_rev->rx_buf[2]);
    v_int = (this->can_rev->rx_buf[3] << 4) | (this->can_rev->rx_buf[4] >> 4) % 16384;
    t_int = ((this->can_rev->rx_buf[4] & 0xF) << 8) | this->can_rev->rx_buf[5];
    mang = uint_to_float(p_int, P_MIN, P_MAX, 16);   // (-12.5,12.5)
    sp = uint_to_float(v_int, V_MIN, V_MAX, 12);     // (-45.0,45.0)
    Torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    T_Rotor = (float)(this->can_rev->rx_buf[6]);
    T_MOS = (float)(this->can_rev->rx_buf[7]);
    online = 1;
    loss_time = 0;

    return HAL_OK;
}
// #define My_id (0xFE)
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#endif /*__CAN_H__*/
#pragma endregion

#pragma region /*Cyber_Gear*/
#ifdef __CAN_H__

HAL_StatusTypeDef Cyber_Gear::update(void)
{
    if (Motor_Id_Get(this->can_rev->RxHeader.ExtId) != this->MOTOR_ID)
    {
        return HAL_ERROR;
    }
    this->mang = uint_to_float(this->can_rev->rx_buf[0] << 8 | this->can_rev->rx_buf[1], -4 * PI, 4 * PI, 16);
    this->sp = uint_to_float(this->can_rev->rx_buf[2] << 8 | this->can_rev->rx_buf[3], -30, 30, 16);
    this->torque = uint_to_float(this->can_rev->rx_buf[4] << 8 | this->can_rev->rx_buf[5], -12, 12, 16);
    this->temp = 1.0f * (this->can_rev->rx_buf[6] << 8 | this->can_rev->rx_buf[7]) / 10;
    return HAL_OK;
}
HAL_StatusTypeDef Cyber_Gear::torque_Send(uint8_t motor_id, float torque)
{
    uint8_t Data[8] = {0X7F, 0XFF, 0X7F, 0XFF, 0, 0, 0, 0};
    motor_send_state = this->can_rev->EXT_ID_Send(EXTID_SET(1, motor_id, float_to_uint(torque, T_MIN, T_MAX, 16)), Data);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef Cyber_Gear::Stop(uint8_t motor_id)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    motor_send_state = this->can_rev->EXT_ID_Send(EXTID_SET(4, motor_id, MY_Master_ID), data);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef Cyber_Gear::Enable(uint8_t motor_id)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    motor_send_state = this->can_rev->EXT_ID_Send(EXTID_SET(3, motor_id, MY_Master_ID), data);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

uint32_t Cyber_Gear::EXTID_SET(uint8_t mode, uint8_t Motor_id, uint16_t data) // 把电机数据合成EXTID
{
    // if (mode == 0 || mode == 3 || mode == 4 || mode == 17 || mode == 6)
    // {
    //     return ((uint32_t)mode << 24 | (uint32_t)MY_Master_ID << 8 | Motor_id) & 0x1FFFFFFF;
    // }
    // else
    return ((uint32_t)mode << 24 | (uint32_t)data << 8 | Motor_id) & 0x1FFFFFFF;
}
uint8_t Cyber_Gear::Motor_Id_Get(uint32_t EXTID)
{
    return (EXTID >> 8) & 0xFF;
}
void Cyber_Gear::update_4PI_mang_inf_basic_zeromang(void)
{
    if (this->first == 0)
    {
        this->Last_mang = this->mang;
        this->first = 1;
    }

    if ((this->mang - this->Last_mang) < -4 * PI)
    {
        this->nsqd_8PI_Cnt_mang += 8 * PI;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 4 * PI)
    {
        this->nsqd_8PI_Cnt_mang -= 8 * PI;
        this->motor_number--;
    }
    this->mang_inf = this->nsqd_8PI_Cnt_mang + this->mang;
    this->Last_mang = this->mang;
}
#endif /*__CAN_H__*/

#pragma endregion

#pragma region /*RMD_M*/
#ifdef __CAN_H__

/*************************************  RMD  ************************************************/

/*
Order_Id :  0x30, 读取当前电机的PID
            0x33，读取加速度参数
            0x90,读取编码器的当前位置
            0x19，写入当前位置到ROM作为电机零点，多次使用会影响芯片寿命
                        0x92,读取多圈角度
                        0x94,读取单圈角度
                        0x95，清楚电机角度
                        0x9A,读取电机状态1和错误标志
            0x9B,清除电机错误标志
                        0x9C,读取电机状态2
                        0x9D,读取电机状态3
            0x80,电机关闭
                        0x81,电机停止
            0x88,电机运行
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Read_and_Write_Things(uint16_t Id, uint16_t Order_Id)
{
    uint8_t pData[8];
    pData[0] = Order_Id;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
Order_Id: 0x31,写入PID参数到RAM，断电后参数失效
          0x32,写入PID参数到ROM，断电后仍然有效
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Write_PID(uint16_t Id, uint16_t Order_Id, uint16_t anglePidKp, uint16_t anglePidKi, uint16_t speedPidKp, uint16_t speedPidKi, uint16_t iqPidKp, uint16_t iqPidKi)
{
    uint8_t pData[8];
    pData[0] = Order_Id;
    pData[1] = 0x00;
    pData[2] = anglePidKp;
    pData[3] = anglePidKi;
    pData[4] = speedPidKp;
    pData[5] = speedPidKi;
    pData[6] = iqPidKp;
    pData[7] = iqPidKi;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
/*
0x34,写入加速度到RAM
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Write_ACCLE_to_RAM(uint16_t Id, int32_t Accel)
{
    uint8_t pData[8];
    pData[0] = 0x34;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = Accel;
    pData[5] = (Accel >> 8);
    pData[6] = (Accel >> 16);
    pData[7] = (Accel >> 24);
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
0x91，写入编码器值到ROM作为电机零点
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Write_EncoderOffset_to_ROM(uint16_t Id, uint16_t EncoderOffset)
{
    uint8_t pData[8];
    pData[0] = 0x91;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = 0x00;
    pData[5] = 0x00;
    pData[6] = EncoderOffset;
    pData[7] = EncoderOffset >> 8;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
0xA1,转矩闭环控制
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Iqcontrol_Motor(uint16_t Id, int16_t iqControl)
{
    uint8_t pData[8];
    pData[0] = 0XA1;
    pData[1] = 0x00;
    pData[2] = 0x00;
    pData[3] = 0x00;
    pData[4] = iqControl;
    pData[5] = iqControl >> 8;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
Order_Id： 0xA2,速度闭环控制
                      0xA3,位置闭环控制命令1
                        0xA4,位置闭环控制命令2
                        0xA7,位置闭环控制命令5
                        0xA8,位置闭环控制命令6
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Speedcontrol_Motor(uint16_t Id, uint16_t Order_Id, uint16_t maxSpeed, int32_t angleControl)
{
    uint8_t pData[8];
    pData[0] = Order_Id;
    pData[1] = 0x00;
    pData[2] = maxSpeed;
    pData[3] = maxSpeed >> 8;
    pData[4] = angleControl;
    pData[5] = angleControl >> 8;
    pData[6] = angleControl >> 16;
    pData[7] = angleControl >> 24;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/*
Order_Id：0xA5,位置闭环控制命令3，spinDirection 0x00代表顺时针，0x01代表逆时针
                    0xA6,位置闭环控制命令4
*/
HAL_StatusTypeDef MOTOR_RMD::RMD_Anglecontrol_Motor(uint16_t Id, uint16_t Order_Id, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl)
{
    uint8_t pData[8];
    pData[0] = Order_Id;
    pData[1] = spinDirection;
    pData[2] = maxSpeed;
    pData[3] = maxSpeed >> 8;
    pData[4] = angleControl;
    pData[5] = angleControl >> 8;
    pData[6] = 0x00;
    pData[7] = 0x00;
    motor_send_state = this->can_rev->STD_ID_Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_RMD::RMD_update(void)
{
    if (this->can_rev->RxHeader.StdId != this->ID)
    {
        return HAL_ERROR;
    }
    switch (this->can_rev->rx_buf[0])
    {
    case 0x30:
    {
        this->RMD_X.anglePidKp = this->can_rev->rx_buf[2];
        this->RMD_X.anglePidKi = this->can_rev->rx_buf[3];
        this->RMD_X.speedPidKp = this->can_rev->rx_buf[4];
        this->RMD_X.speedPidKi = this->can_rev->rx_buf[5];
        this->RMD_X.iqPidKp = this->can_rev->rx_buf[6];
        this->RMD_X.iqPidKi = this->can_rev->rx_buf[7];
        break;
    }
    case 0x33:
    {
        this->RMD_X.Accel = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8) | (this->can_rev->rx_buf[6] << 16) | (this->can_rev->rx_buf[7] << 24));
        break;
    }
    case 0x90:
    {
        this->RMD_X.encoder = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        this->RMD_X.encoderRaw = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0x19:
    {
        this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0x92:
    {
        this->RMD_X.motorAngle = (this->can_rev->rx_buf[1] | this->can_rev->rx_buf[2] << 8 | (this->can_rev->rx_buf[3] << 16) | (this->can_rev->rx_buf[4] << 24)); // 没完整
        break;
    }
    case 0x94:
    {
        this->RMD_X.circleAngle = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0x9A:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.voltage = (this->can_rev->rx_buf[3] | (this->can_rev->rx_buf[4] << 8));
        this->RMD_X.eerorState = this->can_rev->rx_buf[7] & 0x09; // 1 低压保护，8 过温保护 ，9低压保护，过温保护
        break;
    }
    case 0x9C:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0x9D:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iA = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.iB = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.iC = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA1:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA2:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA3:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA4:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA5:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA6:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA7:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    case 0xA8:
    {
        this->RMD_X.temperature = this->can_rev->rx_buf[1];
        this->RMD_X.iq = (this->can_rev->rx_buf[2] | (this->can_rev->rx_buf[3] << 8));
        ;
        this->RMD_X.speed = (this->can_rev->rx_buf[4] | (this->can_rev->rx_buf[5] << 8));
        ;
        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6] | (this->can_rev->rx_buf[7] << 8));
        break;
    }
    }
    return HAL_OK;
}
#endif /*__CAN_H__*/

#pragma endregion

#pragma region /*ML*/
/*************************************************************************************************************/

MOTOR_DiPan::MOTOR_DiPan(void)
{
    this->ML.qz = 0;
    this->ML.qy = 0;
    this->ML.hz = 0;
    this->ML.hy = 0;
}

void MOTOR_DiPan::ML_Data_Deal(float lx, float ly, float lp, int MAX_rate)
{
    float qy_in, qz_in, hy_in, hz_in;
    float sqy_in, sqz_in, shy_in, shz_in, sx, sp, sy;
    float sqy_per, sqz_per, shz_per, shy_per;
    float Y_max, M_max, P_max;
    float qy_per, qz_per, hz_per, hy_per, Y_per, P_per;

    if (ly > 0)
        sy = ly;
    else
        sy = -ly;
    if (lp > 0)
        sp = lp;
    else
        sp = -lp;
    if (lx > 0)
        sx = lx;
    else
        sx = -lx;

    Y_max = sy;

    if (Y_max < sx)
        Y_max = sx;
    if (Y_max < sp)
        Y_max = sp;

    Y_per = Y_max / 660;

    qy_in = lx + ly - lp;
    qz_in = lx - ly + lp;
    hy_in = lx - ly - lp;
    hz_in = lx + ly + lp;

    if (qy_in > 0)
        sqy_in = qy_in;
    else
        sqy_in = -qy_in;
    if (qz_in > 0)
        sqz_in = qz_in;
    else
        sqz_in = -qz_in;
    if (hy_in > 0)
        shy_in = hy_in;
    else
        shy_in = -hy_in;
    if (hz_in > 0)
        shz_in = hz_in;
    else
        shz_in = -hz_in;

    M_max = sqy_in;
    if (M_max < sqz_in)
        M_max = sqz_in;
    if (M_max < shy_in)
        M_max = shy_in;
    if (M_max < shz_in)
        M_max = shz_in;

    if (M_max == 0)
    {
        qy_per = 0;
        qz_per = 0;
        hy_per = 0;
        hz_per = 0;
    }
    else
    {
        qy_per = qy_in / M_max;
        qz_per = qz_in / M_max;
        hy_per = hy_in / M_max;
        hz_per = hz_in / M_max;
    }

    if (qy_per > 0)
        sqy_per = qy_per;
    else
        sqy_per = -qy_per;
    if (qz_per > 0)
        sqz_per = qz_per;
    else
        sqz_per = -qz_per;
    if (hy_per > 0)
        shy_per = hy_per;
    else
        shy_per = -hy_per;
    if (hz_per > 0)
        shz_per = hz_per;
    else
        shz_per = -hz_per;

    P_max = sqy_per;
    if (P_max < sqz_per)
        P_max = sqz_per;
    if (P_max < shy_per)
        P_max = shy_per;
    if (P_max < shz_per)
        P_max = shz_per;

    P_per = (P_max * 4 - (sqy_per + sqz_per + shy_per + shz_per)) / 3.1415926f + 1;
    //		P_per=1;

    this->ML.qy = -qy_per * Y_per * MAX_rate * P_per;
    this->ML.qz = qz_per * Y_per * MAX_rate * P_per;
    this->ML.hy = -hy_per * Y_per * MAX_rate * P_per;
    this->ML.hz = hz_per * Y_per * MAX_rate * P_per;
}
#pragma endregion

#pragma region /*舵轮*/
/***************************************舵轮************************************************************************/

float RUDDER_DiPan::Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1)
{
    if (CH0 == 0)
        return abs(CH1);
    if (CH1 == 0)
        return abs(CH0);
    return sqrtf(CH0 * CH0 + CH1 * CH1);
}
float RUDDER_DiPan::Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2)
{
    float ch2_sqrt2 = 1.4142135623730950488016887242097f * abs(CH2);
    float ch01_max = Get_Ch0_Ch1_Vector_Speed(CH0, CH1);
    return ch2_sqrt2 > ch01_max ? ch2_sqrt2 : ch01_max;
}
float RUDDER_DiPan::absf(float d0)
{
    return d0 >= 0 ? d0 : -d0;
}
float RUDDER_DiPan::Get_Max_float(float d0, float d1)
{
    d0 = absf(d0);
    return d0 > d1 ? d0 : d1;
}

int16_t RUDDER_DiPan::Get_Max_int16(int16_t d0, int16_t d1)
{
    d0 = abs(d0);
    d1 = abs(d1);
    return d0 > d1 ? d0 : d1;
}

void RUDDER_DiPan::Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2)
{
    if (CH0 == 0 && CH1 == 0 && CH2 == 0)
    {
        g_wheel_3508[QZ].yaogan_speed = 0;
        g_wheel_3508[HZ].yaogan_speed = 0;
        g_wheel_3508[HY].yaogan_speed = 0;
        g_wheel_3508[QY].yaogan_speed = 0;
        return;
    }

    float CH2_COS45 = COS_45 * CH2;
    QZ_xy.x = CH0 + CH2_COS45, QZ_xy.y = CH1 + CH2_COS45;
    HZ_xy.x = CH0 - CH2_COS45, HZ_xy.y = CH1 + CH2_COS45;
    HY_xy.x = CH0 - CH2_COS45, HY_xy.y = CH1 - CH2_COS45;
    QY_xy.x = CH0 + CH2_COS45, QY_xy.y = CH1 - CH2_COS45;
    g_wheel_3508[QZ].yaogan_speed = sqrtf(QZ_xy.x * QZ_xy.x + QZ_xy.y * QZ_xy.y);
    g_wheel_3508[HZ].yaogan_speed = sqrtf(HZ_xy.x * HZ_xy.x + HZ_xy.y * HZ_xy.y);
    g_wheel_3508[HY].yaogan_speed = sqrtf(HY_xy.x * HY_xy.x + HY_xy.y * HY_xy.y);
    g_wheel_3508[QY].yaogan_speed = sqrtf(QY_xy.x * QY_xy.x + QY_xy.y * QY_xy.y);
    float max_speed_nor = Get_Max_float(
        Get_Max_float(
            Get_Max_float(g_wheel_3508[QZ].yaogan_speed, g_wheel_3508[HZ].yaogan_speed), g_wheel_3508[HY].yaogan_speed),
        g_wheel_3508[QY].yaogan_speed);
    float max_speed_yaogan = Get_Max_int16(Get_Max_int16(Get_Max_int16(CH0, CH1), CH2), CH2);
    g_wheel_3508[QZ].yaogan_speed = g_wheel_3508[QZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
    g_wheel_3508[HZ].yaogan_speed = g_wheel_3508[HZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
    g_wheel_3508[HY].yaogan_speed = g_wheel_3508[HY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
    g_wheel_3508[QY].yaogan_speed = g_wheel_3508[QY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
    g_angle_6020[QZ] = atan2(QZ_xy.x, QZ_xy.y) * RAD2MANG + ZERO[QZ];
    g_angle_6020[HZ] = atan2(HZ_xy.x, HZ_xy.y) * RAD2MANG + ZERO[HZ];
    g_angle_6020[HY] = atan2(HY_xy.x, HY_xy.y) * RAD2MANG + ZERO[HY];
    g_angle_6020[QY] = atan2(QY_xy.x, QY_xy.y) * RAD2MANG + ZERO[QY];
}

void RUDDER_DiPan::Xiaotuoluo_jie_Suan(uint16_t Mang_yaw, int16_t CH0, int16_t CH1, int16_t CH2)
{
    int16_t mang_yaw_int16 = Mang_yaw << 3;
    mang_yaw_int16 /= 8;
    float yaw_angle = -mang_yaw_int16 / RAD2MANG;
    float yaokong_angle = atan2(-CH0, -CH1);
    float max_speed_yaogan = Get_Max_int16(CH0, CH1);
    float vector_x = sinf(yaw_angle + yaokong_angle) * max_speed_yaogan, vector_y = cosf(yaw_angle + yaokong_angle) * max_speed_yaogan;
    Not_Xiaotuoluo_Jie_Suan(vector_x, vector_y, CH2);
}

/************************************************************************************************************/
#pragma endregion

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}
#pragma region /*PID*/
/**************************************** P I D ***************************************************************************/
/*********************************************  D  L  *******************************************************/
bool PID_class::Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now)
{
    return (abs(goal - now) > 2048);
}
void PID_class::PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight)
{
    int16_t deal_3508_g = goal << 3, deal_3508_n = now << 3;
    int now_error;
    goal &= 0x1fff;
    bool ret = Get_6020mang_need_turn_direction_is(goal, now);
    int16_t res;
    int16_t error;

    if (ret)
    {
        goal = (goal + (8192 / 2)) & 0x1fff;
    }

    res = (goal - now);
    if (res > 8192 / 2)
    {
        error = (res - 8192);
    }
    else if (res < -8192 / 2)
    {
        error = (res + 8192);
    }
    else
    {
        error = res;
    }

    now_error = error;

    wheel_dir_and_weight->speed = (float)((2048 - abs(now_error)) / (float)(2048)) * wheel_dir_and_weight->yaogan_speed;

    if (((deal_3508_g - (2048 << 3)) < deal_3508_n) && ((deal_3508_g + (2048 << 3)) > deal_3508_n))
    {
        wheel_dir_and_weight->dir = 0;
    }
    else
    {
        wheel_dir_and_weight->dir = 1;
        wheel_dir_and_weight->speed = -wheel_dir_and_weight->speed;
    }

    this->OUT_P = this->KP * now_error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);

    this->OUT_I += this->KI * (now_error);
    this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);

    this->OUT_D = this->KD * (now_error - this->LAST_Error);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = now_error;
}
void PID_class::PID_new_update(float goal, float now)
{

    float now_error;

    now_error = goal - now;

    this->OUT_P = this->KP * now_error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);

    if (abs(now_error) < this->Separate) // 积分分离  当误差大于设定值不加入积分防止积分超调
    {
        this->OUT_I += this->KI * (now_error);
        this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);
    }
    else
    {
        this->OUT_I = 0;
    }
    this->OUT_D = this->KD * (now_error - this->LAST_Error);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = now_error;
}
void PID_class::PID_update(float goal, float now)
{
    // float now_error;

    this->error = goal - now;

    this->OUT_P = this->KP * this->error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);
    this->OUT_I += this->KI * (this->error);
    this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);
    this->OUT_D = this->KD * (this->error - this->LAST_Error);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = this->error;
}
void PID_class::PID_update_void(void)
{
    if (!enable_flag) // 自己判断初始化
    {
        Reset();
        return;
    }
    this->error = Goal - Ref;

    this->OUT_P = this->KP * this->error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);
    // 先判断超不超误差。
    // 超了，累积趋势，输出保持不动
    // 超了，非累积趋势，取消积分作用
    // 没超，累积趋势，累积
    // 没超，非累积趋势，累积
    if (fabs(this->error) > this->Deadzoom) // 如果大于死区
    {
        if (fabs(this->error) < this->Separate)
        {
            this->OUT_I += this->KI * this->error;
            this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);
        }
        else if (this->error * this->OUT_I < 0)
        {
            this->OUT_I = 0;
        }
    }

    this->OUT_D = this->KD * low_pass_filter(this->error - this->LAST_Error, D_LP_value);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = this->error;
}
void PID_class::PID_update_LP(float goal, float now) // 推荐此函数
{
    if (!enable_flag)
    {
        Reset();
        return;
    }
    this->error = goal - now;

    this->OUT_P = this->KP * this->error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);
    // 先判断超不超误差。
    // 超了，累积趋势，输出保持不动
    // 超了，非累积趋势，取消积分作用
    // 没超，累积趋势，累积
    // 没超，非累积趋势，累积
    if (fabs(this->error) > this->Deadzoom) // 如果大于死区
    {
        if (fabs(this->error) < this->Separate)
        {
            this->OUT_I += this->KI * this->error;
            this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);
        }
        else if (this->error * this->OUT_I < 0)
        {
            this->OUT_I = 0;
        }
    }

    this->OUT_D = this->KD * low_pass_filter(this->error - this->LAST_Error, D_LP_value);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = this->error;
}
float PID_class::low_pass_filter(float value, float k_value)
{

    float out;

    /***************** 如果第一次进入，则给 out_last 赋值 ******************/
    //    static char fisrt_flag = 1;
    //    if (fisrt_flag == 1)
    //    {
    //        fisrt_flag = 0;
    //        LP_out_last = value;
    //    }

    /*************************** 一阶滤波 *********************************/
    out = LP_out_last + k_value * (value - LP_out_last);
    LP_out_last = out;

    return out;
}
#pragma endregion

#pragma region /*Fuzzy PID*/
void PID_class::PID_Inc_update(float goal, float now)
{
    float now_error;

    now_error = goal - now;

    this->OUT_P = this->KP * (now_error - this->inc_last_error);
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);

    this->OUT_I = this->KI * now_error;
    this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);

    this->OUT_D = this->KD * (now_error - 2 * this->inc_last_error + this->inc_lastlast_error);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->delta_OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->delta_OUT_PID = LIMIT(this->delta_OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->OUT_PID += this->delta_OUT_PID;

    this->inc_lastlast_error = this->inc_last_error; ////更新上上一次的误差
    this->inc_last_error = now_error;                // 更新上一次的误差
}
void PID_Fuzzy_class::fuzzy(float goal, float now)
{
    this->error = goal - now;                              // 得出误差
    float e = this->error / this->stair;                   // 误差除阶梯值
    float ec = (this->Out - this->Out_last) / this->stair; // 这次输出和上次输出的差值除阶梯值，就是输出的斜率除阶梯值
    short etemp, ectemp;
    float eLefttemp, ecLefttemp; // 隶属度，隶属度也为概率
    float eRighttemp, ecRighttemp;

    short eLeftIndex, ecLeftIndex; // 标签
    short eRightIndex, ecRightIndex;

    // 模糊化
    if (e >= PL)    // 误差除阶梯值大于等于正大，开始比较误差，看误差最小在哪里
        etemp = PL; // 超出范围
    else if (e >= PM)
        etemp = PM;
    else if (e >= PS)
        etemp = PS;
    else if (e >= ZE)
        etemp = ZE;
    else if (e >= NS)
        etemp = NS;
    else if (e >= NM)
        etemp = NM;
    else if (e >= NL)
        etemp = NL;
    else
        etemp = 2 * NL; // 都不在就是等于两倍的最小

    if (etemp == PL) // 如果模糊化的误差大于最大论域
    {
        // 计算E隶属度
        eRighttemp = 0; // 右溢出：最大溢出
        eLefttemp = 1;

        // 计算标签
        eLeftIndex = 6;
        eRightIndex = 6;
    }
    else if (etemp == 2 * NL) // 如果模糊化的误差小于最小论域
    {
        // 计算E隶属度
        eRighttemp = 1; // 左溢出
        eLefttemp = 0;

        // 计算标签
        eLeftIndex = 0;
        eRightIndex = 0;
    }
    else // 如果在论域内
    {
        // 计算E隶属度
        eRighttemp = (e - etemp); // 误差减最小模糊， 线性函数作为隶属函数，计算所占比例
        eLefttemp = (1 - eRighttemp);

        // 计算标签
        eLeftIndex = (short)(etemp - NL); // 例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
        eRightIndex = (short)(eLeftIndex + 1);
    }

    if (ec >= PL)
        ectemp = PL;
    else if (ec >= PM)
        ectemp = PM;
    else if (ec >= PS)
        ectemp = PS;
    else if (ec >= ZE)
        ectemp = ZE;
    else if (ec >= NS)
        ectemp = NS;
    else if (ec >= NM)
        ectemp = NM;
    else if (ec >= NL)
        ectemp = NL;
    else
        ectemp = 2 * NL;

    if (ectemp == PL)
    {
        // 计算EC隶属度
        ecRighttemp = 0; // 右溢出
        ecLefttemp = 1;

        ecLeftIndex = 6;
        ecRightIndex = 6;
    }
    else if (ectemp == 2 * NL)
    {
        // 计算EC隶属度
        ecRighttemp = 1;
        ecLefttemp = 0;

        ecLeftIndex = 0;
        ecRightIndex = 0;
    }
    else
    {
        // 计算EC隶属度
        ecRighttemp = (ec - ectemp);
        ecLefttemp = (1 - ecRighttemp);

        ecLeftIndex = (short)(ectemp - NL);
        ecRightIndex = (short)(eLeftIndex + 1);
    }
    this->dKp = this->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);

    this->dKi = this->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

    this->dKd = this->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
}
void PID_Fuzzy_class::FuzzyPID_update(float goal, float now)
{
    this->LastError = this->error;
    this->error = goal - now;

    fuzzy(goal, now); // 模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值

    float Kp = this->Kp0 + this->dKp, Ki = this->Ki0 + this->dKi, Kd = this->Kd0 + this->dKd; // PID均模糊
    //	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //仅PD均模糊
    //	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //仅P均模糊

    if (fabs(this->error) < this->I_L)
    {
        this->SumError += this->error / 2;
        this->SumError = LIMIT(this->SumError, -this->IMax, this->IMax);
    }

    this->POut = Kp * this->error;
    this->IOut = Ki * this->SumError;
    this->DOut = Kd * (this->error - this->LastError);

    this->Out_last = this->Out;
    this->Out = LIMIT(this->POut + this->IOut + this->DOut, this->OutMax, -this->OutMax);
}
#pragma endregion

#pragma region /*滑膜控制 SMC*/
/*************************************  滑膜控制  ************************************************/
void SMC::SMC_Tick(float Target_angle, float angle, float angle_vel)
{
    // 1. 误差定义（期望 - 实际）
    error = Target_angle - angle;

    // 2. 目标速度/加速度
    d_tge = (Target_angle - Last_Target_angle);
    // dd_tge = (d_tge - Last_d_tge);//加速度前馈项，直接微分噪声大，建议舍去
    dd_tge = 0;

    // 3. 速度误差（期望速度 - 实际速度）
    d_error = d_tge - angle_vel;
    // 4. 滑模面
    s = C * error + d_error;

    // 5. 控制律（标准形式：所有项都是正号）
    u = J * (dd_tge + C * d_error + K * s + epsilon * Sat_v2(s));

    // 6. 限幅
    u = LIMIT(u, -u_max, u_max);
    // 7. 更新
    // Last_d_tge = d_tge;
    Last_Target_angle = Target_angle;
}

void SMC::SMC_Tick2(float Target_angle, float angle, float angle_vel)
{
    // 1. 误差定义修正（期望 - 实际）
    error = angle - Target_angle;

    // 2. 数值微分修正
    d_tge = (Target_angle - Last_Target_angle);
    // dd_tge = (d_tge - Last_d_tge);
    dd_tge = 0; // 加速度前馈项，直接微分噪声大，建议舍去

    // 3. 滑模面计算（ė = θ̇d - θ̇）
    d_error = angle_vel - d_tge;

    // smc surface
    s = C * error + d_error;
    // u = J * (dd_tge - C * d_error - K * s - epsilon * Sat(s));
    u = J * (dd_tge - C * d_error - K * s - epsilon * Sat_v2(s));

    // 控制量限幅
    u = LIMIT(u, -u_max, u_max);

    // 参数更新
    // Last_d_tge = d_tge;
    Last_Target_angle = Target_angle;
}
void SMC::SMC_Tick3(float Target_angle, float angle, float angle_vel) // 测试失败，用不了
{
    // 1. 误差定义修正（期望 - 实际）
    error = angle - Target_angle;

    // 2. 数值微分修正
    d_tge = (Target_angle - Last_Target_angle) / dt;
    dd_tge = (d_tge - Last_d_tge) / dt;

    // 3. 滑模面计算（ė = θ̇d - θ̇）
    d_error = angle_vel - d_tge;

    // 误差下限处理
    // if (fabs(error) < error_eps)
    // {
    //     u = 0;
    //     return;
    // }

    // smc surface
    s = C * error + d_error;
    // u = J * (dd_tge - C * d_error - K * s - epsilon * Sat(s));
    u = J * (dd_tge - C * d_error - K * s - epsilon * Sat_v2(s));

    // 控制量限幅
    u = LIMIT(u, -u_max, u_max);

    // 参数更新
    Last_d_tge = d_tge;
    Last_Target_angle = Target_angle;
}

void SMC::SMC_Tick_ZM(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel)
{
    // 1. 误差定义
    error = angle - Target_angle;

    // 2. 速度误差
    d_error = angle_vel - Target_angle_vel;

    // 3. 滑模面
    s = C * error + d_error;

    // 4. 控制律
    u = J * (Target_angle_acc - C * d_error - K * s - epsilon * Sat_v2(s));

    // 5. 限幅
    u = LIMIT(u, -u_max, u_max);
}

/*正项式*/
// void SMC::SMC_Tick_I(float Target_angle, float angle, float angle_vel)
// {
//     // 1. 误差定义（期望 - 实际）
//     error = Target_angle - angle;
//     error_integral += error;
//     // 2. 目标速度/加速度
//     d_tge = (Target_angle - Last_Target_angle);
//     dd_tge = (d_tge - Last_d_tge);
//     // 3. 速度误差（期望速度 - 实际速度）
//     d_error = d_tge - angle_vel;
//     // 4. 滑模面
//     s = C * error + d_error + C2 * error_integral;
//     // 5. 控制律（标准形式：所有项都是正号）
//     u = J * (dd_tge + C * d_error + C2 * error + K * s + epsilon * Sat(s));
//     // 6. 限幅
//     u = LIMIT(u, -u_max, u_max);
//     // 7. 更新
//     Last_d_tge = d_tge;
//     Last_Target_angle = Target_angle;
// }

void SMC::SMC_Tick_I(float Target_angle, float angle, float angle_vel)
{
    // 1. 误差定义修正（期望 - 实际）
    error = angle - Target_angle;

    // 2. 数值微分修正
    d_tge = (Target_angle - Last_Target_angle);
    // dd_tge = (d_tge - Last_d_tge);
    dd_tge = 0; // 加速度前馈项，直接微分噪声大，建议舍去

    // 3. 滑模面计算（ė = θ̇d - θ̇）
    d_error = angle_vel - d_tge;
    error_integral += error;

    // smc surface
    s = C * error + d_error + C2 * error_integral;
    // u = J * (dd_tge - C * d_error - K * s - epsilon * Sat(s));
    u = J * (dd_tge - C * d_error - C2 * error - K * s - epsilon * Sat_v2(s));

    // 控制量限幅
    u = LIMIT(u, -u_max, u_max);

    // 参数更新
    // Last_d_tge = d_tge;
    Last_Target_angle = Target_angle;
}

void SMC::SMC_Tick_ZM_I(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel)
{
    // 1. 误差定义
    error = angle - Target_angle;

    // 2. 速度误差
    d_error = angle_vel - Target_angle_vel;

    error_integral += error;

    // 3. 滑模面
    s = C * error + d_error + C2 * error_integral;

    // 4. 控制律
    u = J * (Target_angle_acc - C * d_error - C2 * error - K * s - epsilon * Sat_v2(s));

    // 5. 限幅
    u = LIMIT(u, -u_max, u_max);
}

#pragma endregion

/****************************************** D B U S **********************************************************/
#pragma region /*DBUS*/

#ifdef __USART_H__

void DBUS::watchdog_run(void)
{
    this->time_100ms++;
    if (this->time_100ms > 5)
    {
        this->set_zero();
        for (uint8_t i = 0; i < 25; i++)
        {
            dbus_rx_buffer[i] = 0; // 缓冲区不清0，底盘就会受到错误数据，比如关闭遥控器电源时，仍然会发关闭前的遥控器数据
        }
        this->feed_watchdog();
        if (HAL_UART_GetState(this->huart) == HAL_UART_STATE_ERROR)
        {
            HAL_UART_Abort_IT(this->huart);
            this->Init();
        }
        this->dog = true;
    }
    else
    {
        this->dog = false;
    }
}
void DBUS::Init(void)
{
    this->set_zero();
    __HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(this->huart, dbus_rx_buffer, sizeof(dbus_rx_buffer));
}
void DBUS::DBUS_RxCplt_IRQHandler(void)
{
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag = __HAL_UART_GET_FLAG(this->huart, UART_FLAG_IDLE);
    if (tmp_flag != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(this->huart);
        temp = this->huart->Instance->SR;
        temp = this->huart->Instance->DR;
        HAL_UART_DMAStop(this->huart);
        if (check_and_deal() == HAL_OK)
        {
            this->data_deal();
            this->feed_watchdog();
        }
        HAL_UART_Receive_DMA(this->huart, dbus_rx_buffer, sizeof(dbus_rx_buffer));
    }
}
HAL_StatusTypeDef DBUS::check_and_deal(void)
{
    if (this->shubiao.z != 0 ||
        dbus_rx_buffer[18] != 0 || dbus_rx_buffer[19] != 0 ||
        this->shubiao.press_r >= 2 || this->shubiao.press_l >= 2)
    {
        this->set_zero();
        for (uint8_t i = 0; i < 25; i++)
        {
            dbus_rx_buffer[i] = 0; // 缓冲区不清0，底盘就会受到错误数据，比如关闭遥控器电源时，仍然会发关闭前的遥控器数据
        }
        rx_error_cnt++;
        return HAL_ERROR;
    }
    else
        return HAL_OK;
}
void DBUS::set_zero(void)
{
    this->yaogan.ch0 = 0;
    this->yaogan.ch1 = 0;
    this->yaogan.ch2 = 0;
    this->yaogan.ch3 = 0;
    this->yaogan.s1 = YK_SW_UP;
    this->yaogan.s2 = YK_SW_UP;
    this->shubiao.press_l = 0;
    this->shubiao.press_r = 0;
    this->shubiao.x = 0;
    this->shubiao.y = 0;
    this->shubiao.z = 0;
    this->jianpan = 0;
}
// void DBUS::jianpan_deal(void) // 无用
// {
//     if (this->last_jianpan > this->jianpan)
//     {
//         this->index = (uint16_t)(log(this->last_jianpan - this->jianpan) / log(2));
//         if (this->delaycount[this->index] < 10)
//         {
//             this->delaycount[this->index]++;
//             this->jianpan |= 0x01 << this->index;
//         }
//         else
//         {
//             last_jianpan &= ~(0x01 << this->index);
//             this->delaycount[this->index] = 0;
//         }
//     }
//     else
//     {
//         this->last_jianpan = this->jianpan;
//         memset(this->delaycount, 0, 16);
//     }
// }
void DBUS::data_deal(void)
{
    this->yaogan.ch0 = ((dbus_rx_buffer[0] | (dbus_rx_buffer[1] << 8)) & 0x07ff) - 1024;        //!< Channel 0
    this->yaogan.ch1 = (((dbus_rx_buffer[1] >> 3) | (dbus_rx_buffer[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
    this->yaogan.ch2 = (((dbus_rx_buffer[2] >> 6) | (dbus_rx_buffer[3] << 2) |                  //!< Channel 2
                         (dbus_rx_buffer[4] << 10)) &
                        0x07ff) -
                       1024;
    this->yaogan.ch3 = (((dbus_rx_buffer[4] >> 1) | (dbus_rx_buffer[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
    this->yaogan.s1 = ((dbus_rx_buffer[5] >> 4) & 0x000C) >> 2;                                 //!< Switch left
    this->yaogan.s2 = ((dbus_rx_buffer[5] >> 4) & 0x0003);                                      //!< Switch right
    this->shubiao.x = (int16_t)(dbus_rx_buffer[6] | (dbus_rx_buffer[7] << 8));                  //!< Mouse X axis
    this->shubiao.y = (int16_t)(dbus_rx_buffer[8] | (dbus_rx_buffer[9] << 8));                  //!< Mouse Y axis
    this->shubiao.z = (int16_t)(dbus_rx_buffer[10] | (dbus_rx_buffer[11] << 8));                //!< Mouse Z axis
    this->shubiao.press_l = dbus_rx_buffer[12];                                                 //!< Mouse Left Is Press ?
    this->shubiao.press_r = dbus_rx_buffer[13];                                                 //!< Mouse Right Is Press ?
    this->jianpan = dbus_rx_buffer[14] | (dbus_rx_buffer[15] << 8);
    // this->yaogan.v = (int16_t)(((dbus_rx_buffer[16] | (dbus_rx_buffer[17] << 8)) & 0x07ff) - 1024);

    // this->jianpan_deal();
}

void DBUS::can_receive_data_deal(uint8_t num, uint8_t *buf)
{
    if (this->yaogan.ch0 == -1024 || this->yaogan.ch1 == -1024 ||
        this->yaogan.ch2 == -1024 || this->yaogan.ch3 == -1024 ||
        this->yaogan.s1 > 3 || this->yaogan.s2 > 3 ||
        this->yaogan.s1 == 0 || this->yaogan.s2 == 0)
    {
        this->set_zero();
        return;
    }
    if (num == 1)
    {
        this->yaogan.ch0 = ((buf[0] | (buf[1] << 8)) & 0x07ff) - 1024;        //!< Channel 0
        this->yaogan.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
        this->yaogan.ch2 = (((buf[2] >> 6) | (buf[3] << 2) |                  //!< Channel 2
                             (buf[4] << 10)) &
                            0x07ff) -
                           1024;
        this->yaogan.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
        this->yaogan.s1 = ((buf[5] >> 4) & 0x000C) >> 2;                      //!< Switch left
        this->yaogan.s2 = ((buf[5] >> 4) & 0x0003);                           //!< Switch right
        this->shubiao.x = (int16_t)(buf[6] | (buf[7] << 8));                  //!< Mouse X axis
    }
    else if (num == 2)
    {
        this->shubiao.y = (int16_t)(buf[0] | (buf[1] << 8)); //!< Mouse Y axis
        this->shubiao.z = (int16_t)(buf[2] | (buf[3] << 8)); //!< Mouse Z axis
        this->shubiao.press_l = buf[4];                      //!< Mouse Left Is Press?
        this->shubiao.press_r = buf[5];                      //!< Mouse Right Is Press?
        this->jianpan = buf[6] | (buf[7] << 8);              //!< KeyBoard value
                                                             //
                                                             //		this->jianpan_deal();
    }
}

uint8_t DBUS::Pressed_Check(uint16_t keyvalue)
{
    if (this->jianpan & keyvalue)
        return 1;
    else
        return 0;
}
#endif // __USART_H__

#pragma endregion

#pragma region /*RC*/
#ifdef __USART_H__

void RC::DT16_watchdog_run(void)
{
    this->DT16_time_100ms++;
    if (this->DT16_time_100ms > 7)
    {
        dt16_signal_flag = 0;
        this->DT16_set_zero();
        this->DT16_feed_watchdog();
        HAL_UART_Abort_IT(this->DT16_huart);
        this->DT16_Init();
        YK_ctrl();
    }
}
void RC::DT16_2watchdog_run(void)
{
    this->DT16_time_100ms++;
    if (this->DT16_time_100ms > 7)
    {
        dt16_signal_flag = 0;
        this->DT16_set_zero();
        this->DT16_feed_watchdog();
        HAL_UART_Abort_IT(this->DT16_huart);
        this->DT16_2Init();
        YK_ctrl();
    }
}
void RC::VT13_watchdog_run(void)
{
    this->VT13_time_100ms++;
    if (this->VT13_time_100ms > 7)
    {
        vt13yk_signal_flag = 0;
        this->VT13_YK_set_zero();
        this->VT13_feed_watchdog();
        HAL_UART_Abort_IT(this->TC_huart);
        this->VT13_Init();
        YK_ctrl();
    }
}
void RC::VT13_2watchdog_run(void)
{
    this->VT13_time_100ms++;
    if (this->VT13_time_100ms > 7)
    {
        vt13yk_signal_flag = 0;
        this->VT13_YK_set_zero();
        this->VT13_feed_watchdog();
        HAL_UART_Abort_IT(this->TC_huart);
        this->VT13_2Init();
        YK_ctrl();
    }
}
void RC::DT16_Init(void)
{
    this->DT16_set_zero();
    // this->memset_dr16_rx_buffer();
    memset(&DR16_rx_buffer, 0, sizeof(DR16_rx_buffer));
    if (this->DT16_huart->Init.BaudRate != 100000)
    {
        // INFO("DEBUSif Init Error\n");
        // HAL_Delay(10);
        return;
    }

    __HAL_UART_ENABLE_IT(this->DT16_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(this->DT16_huart, DR16_rx_buffer, sizeof(DR16_rx_buffer));
}
void RC::DT16_2Init(void)
{
    this->DT16_set_zero();
    memset(&DR16_2rx_buffer, 0, sizeof(DR16_2rx_buffer));
    if (this->DT16_huart->Init.BaudRate != 100000)
    {
        // INFO("DEBUSif Init Error\n");
        // HAL_Delay(10);
        return;
    }

    __HAL_UART_ENABLE_IT(this->DT16_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(this->DT16_huart, DR16_2rx_buffer[dr16_fifo_index], sizeof(DR16_2rx_buffer[0]));
}

void RC::VT13_Init(void)
{
    this->VT13_YK_set_zero();
    memset(&VT13_rx_buffer, 0, sizeof(VT13_rx_buffer));

    if (this->TC_huart->Init.BaudRate != 921600)
    {
        return;
    }

    __HAL_UART_ENABLE_IT(this->TC_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(this->TC_huart, VT13_rx_buffer, sizeof(VT13_rx_buffer));
}
void RC::VT13_2Init(void)
{
    this->VT13_YK_set_zero();
    memset(&VT13_2rx_buffer, 0, sizeof(VT13_2rx_buffer));
    if (this->TC_huart->Init.BaudRate != 921600)
    {
        return;
    }

    __HAL_UART_ENABLE_IT(this->TC_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(this->TC_huart, VT13_2rx_buffer[vt13_fifo_index], sizeof(VT13_2rx_buffer[0]));
}
void RC::DT16_RxCplt_IRQHandler(void)
{
    uint32_t temp;
    if (__HAL_UART_GET_FLAG(this->DT16_huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(this->DT16_huart);
        temp = this->DT16_huart->Instance->SR;
        temp = this->DT16_huart->Instance->DR;
        HAL_UART_DMAStop(this->DT16_huart);
        this->DT16_data_deal(); /*先解析数据，再做判断，再传值*/
        if (DT16_check_and_deal() == HAL_OK)
        {

            this->DT16_feed_watchdog();
            dt16_signal_flag = 1;
            YK_ctrl();
            // this->fill_data();
        }
        HAL_UART_Receive_DMA(this->DT16_huart, DR16_rx_buffer, sizeof(DR16_rx_buffer));
    }
}
void RC::DT16_2RxCplt_IRQHandler(DMA_HandleTypeDef *hdma)
{
    uint32_t temp;
    if (__HAL_UART_GET_FLAG(this->DT16_huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(this->DT16_huart);
        temp = this->DT16_huart->Instance->SR;
        temp = this->DT16_huart->Instance->DR;
        HAL_UART_DMAStop(this->DT16_huart);
        temp = hdma->Instance->NDTR;
        uint8_t dr16_rx_len = sizeof(DR16_2rx_buffer[0]) - temp;
        dr16_fifo_index = !dr16_fifo_index;
        HAL_UART_Receive_DMA(this->DT16_huart, DR16_2rx_buffer[dr16_fifo_index], sizeof(DR16_2rx_buffer[0]));

        this->DT16_2data_deal(DR16_2rx_buffer[!dr16_fifo_index]); /*先解析数据，再做判断，再传值*/
        if (DT16_2check_and_deal(DR16_2rx_buffer[!dr16_fifo_index]) == HAL_OK)
        {
            this->DT16_feed_watchdog();
            dt16_signal_flag = 1;
            YK_ctrl();
            // this->fill_data();
        }
    }
}
void RC::DT16_v_deal(void)
{
    if (first == 0) // 如果是第一次，或者first=0
    {
        v_inf_buf = 0;
        nsqd_2048xCnt_mang = 0;
        first_v_inc = yaogan.v; // 记录第一次增量
        last_v = yaogan.v;
        first = 1;
    }

    if (((yaogan.v - last_v) < -1024) && last_v == 660) /*非常特殊的特判没有意义*/
    {
        nsqd_2048xCnt_mang = 0;
    }
    else if (((yaogan.v - last_v) > 1024) && yaogan.v == 660)
    {
        nsqd_2048xCnt_mang = -2048;
    }

    if (yaogan.v > 1030)
    {
        nsqd_2048xCnt_mang = 0;
    }
    v_inf_buf = nsqd_2048xCnt_mang + yaogan.v - first_v_inc;
    if (v_inf_buf > 6000) /*过圈频率太低，只能缓解*/
    {
        v_inf_buf = 6000;
        // nsqd_2048xCnt_mang = 6144;
    }
    else if (v_inf_buf < -1024)
    {
        v_inf_buf = -1024;
        // nsqd_2048xCnt_mang = -2048;
    }
    v_inf = v_inf_buf; /*最终值传递到外部*/
    last_v = yaogan.v;
}
uint8_t RC::VT13_RxCplt_IRQHandler(void)
{
    uint8_t ifdeal = 0;
    uint32_t temp;
    if (__HAL_UART_GET_FLAG(this->TC_huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(this->TC_huart);
        temp = this->TC_huart->Instance->SR;
        temp = this->TC_huart->Instance->DR;
        HAL_UART_DMAStop(this->TC_huart);
        VT13_YK_deal(); // 判断遥控帧
        VT13_self_ctrl_deal();
        HAL_UART_Receive_DMA(this->TC_huart, VT13_rx_buffer, sizeof(VT13_rx_buffer));
        ifdeal = 1;
    }

    return ifdeal;
}
uint8_t RC::VT13_2RxCplt_IRQHandler(DMA_HandleTypeDef *hdma)
{
    uint8_t ifdeal = 0;
    uint32_t temp;
    if (__HAL_UART_GET_FLAG(this->TC_huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(this->TC_huart);
        temp = this->TC_huart->Instance->SR;
        temp = this->TC_huart->Instance->DR;
        HAL_UART_DMAStop(this->TC_huart);

        temp = hdma->Instance->NDTR;
        uint8_t TC_rx_len = sizeof(VT13_2rx_buffer[0]) - temp;
        vt13_fifo_index = !vt13_fifo_index;
        HAL_UART_Receive_DMA(this->TC_huart, VT13_2rx_buffer[vt13_fifo_index], sizeof(VT13_2rx_buffer[0]));

        VT13_2YK_deal(VT13_2rx_buffer[!vt13_fifo_index]); // 判断遥控帧
        VT13_2self_ctrl_deal(VT13_2rx_buffer[!vt13_fifo_index]);
        ifdeal = 1;
    }

    return ifdeal;
}
void RC::VT13_2YK_deal(uint8_t *rxbuf)
{
    if (VT13_2check_and_deal(rxbuf) == HAL_OK) // 检查帧头
    {
        this->VT13_2data_deal(rxbuf);
        this->VT13_feed_watchdog();
        vt_yk_cnt++;
        vt13yk_signal_flag = 1;
        YK_ctrl();
        // this->fill_data();
    }
}

void RC::VT13_YK_deal(void)
{
    if (VT13_check_and_deal() == HAL_OK) // 检查帧头
    {
        this->VT13_data_deal();
        this->VT13_feed_watchdog();
        vt_yk_cnt++;
        vt13yk_signal_flag = 1;
        YK_ctrl();
        // this->fill_data();
    }
}

HAL_StatusTypeDef RC::DT16_check_and_deal(void)
{
    /*要加对每个摇杆的判断，否则初始化的前几帧摇杆会到-1024*/
    if (abs(DT16_yaogan.ch0) > 660 || abs(DT16_yaogan.ch1) > 660 || abs(DT16_yaogan.ch2) > 660 || abs(DT16_yaogan.ch3) > 660 || DT16_shubiao.z != 0 ||
        DR16_rx_buffer[18] != 0 || DR16_rx_buffer[19] != 0 ||
        DT16_shubiao.press_r >= 2 || DT16_shubiao.press_l >= 2) //
    {
        err_cnt++;
        this->DT16_yaogan.ch0 = 0; /*YK_ctrl()会进行数据传值，看门狗定时调用了YK_ctrl()，所以拨杆不要复位，保持上一次的有效值*/
        this->DT16_yaogan.ch1 = 0;
        this->DT16_yaogan.ch2 = 0;
        this->DT16_yaogan.ch3 = 0;
        this->DT16_shubiao.press_l = 0;
        this->DT16_shubiao.press_r = 0;
        this->DT16_shubiao.x = 0;
        this->DT16_shubiao.y = 0;
        this->DT16_shubiao.z = 0;
        this->DT16_yaogan.v = 0;
        this->DT16_jianpan = 0;
        if (DT16_yaogan.s2 > 3 || DT16_yaogan.s1 > 3) /*真的数据异常再复位*/
        {
            this->DT16_yaogan.s1 = YK_SW_UP;
            this->DT16_yaogan.s2 = YK_SW_UP;
        }

        // this->DT16_set_zero();
        return HAL_ERROR;
    }
    else
        return HAL_OK;
}
HAL_StatusTypeDef RC::DT16_2check_and_deal(uint8_t *rxbuf)
{
    /*要加对每个摇杆的判断，否则初始化的前几帧摇杆会到-1024*/
    if (abs(DT16_yaogan.ch0) > 660 || abs(DT16_yaogan.ch1) > 660 || abs(DT16_yaogan.ch2) > 660 || abs(DT16_yaogan.ch3) > 660 || DT16_shubiao.z != 0 ||
        rxbuf[18] != 0 || rxbuf[19] != 0 ||
        DT16_shubiao.press_r >= 2 || DT16_shubiao.press_l >= 2) //
    {
        err_cnt++;
        this->DT16_yaogan.ch0 = 0; /*YK_ctrl()会进行数据传值，看门狗定时调用了YK_ctrl()，所以拨杆不要复位，保持上一次的有效值*/
        this->DT16_yaogan.ch1 = 0;
        this->DT16_yaogan.ch2 = 0;
        this->DT16_yaogan.ch3 = 0;
        this->DT16_shubiao.press_l = 0;
        this->DT16_shubiao.press_r = 0;
        this->DT16_shubiao.x = 0;
        this->DT16_shubiao.y = 0;
        this->DT16_shubiao.z = 0;
        this->DT16_yaogan.v = 0;
        this->DT16_jianpan = 0;
        if (DT16_yaogan.s2 > 3 || DT16_yaogan.s1 > 3) /*真的数据异常再复位*/
        {
            this->DT16_yaogan.s1 = YK_SW_UP;
            this->DT16_yaogan.s2 = YK_SW_UP;
        }

        // this->DT16_set_zero();
        return HAL_ERROR;
    }
    else
        return HAL_OK;
}
HAL_StatusTypeDef RC::VT13_check_and_deal(void)
{
    if (VT13_rx_buffer[0] == 0xA9 && VT13_rx_buffer[1] == 0x53)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}
HAL_StatusTypeDef RC::VT13_2check_and_deal(uint8_t *rxbuf)
{
    if (rxbuf[0] != 0xA9 || rxbuf[1] != 0x53)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
void RC::DT16_set_zero(void)
{
    this->DT16_yaogan.ch0 = 0;
    this->DT16_yaogan.ch1 = 0;
    this->DT16_yaogan.ch2 = 0;
    this->DT16_yaogan.ch3 = 0;
    this->DT16_yaogan.s1 = YK_SW_UP;
    this->DT16_yaogan.s2 = YK_SW_UP;
    this->DT16_shubiao.press_l = 0;
    this->DT16_shubiao.press_r = 0;
    this->DT16_shubiao.x = 0;
    this->DT16_shubiao.y = 0;
    this->DT16_shubiao.z = 0;
    this->DT16_yaogan.v = 0;
    this->DT16_jianpan = 0;

    // for (int i = 1; i <= 25; i++)
    // {
    //     DR16_rx_buffer[i - 1] = 0;
    //     if (i == 6)
    //         DR16_rx_buffer[i - 1] = (DR16_rx_buffer[i - 1] & 0x0F) | (0x5 << 4); // 清零时双上
    // }
    // if (!this->VT13_Data.mode_sw)
    //     this->fill_data();
}
void RC::VT13_YK_set_zero(void)
{
    this->VT13_Data.ch_0 = 0;
    this->VT13_Data.ch_1 = 0;
    this->VT13_Data.ch_2 = 0;
    this->VT13_Data.ch_3 = 0;
    this->VT13_Data.mode_sw = 0;
    this->VT13_Data.pause = 0;
    this->VT13_Data.fn_1 = 0;
    this->VT13_Data.fn_2 = 0;
    this->VT13_Data.wheel = 0;
    this->VT13_Data.trigger = 0;
    this->VT13_Data.mouse_x = 0;
    this->VT13_Data.mouse_y = 0;
    this->VT13_Data.mouse_z = 0;
    this->VT13_Data.mouse_left = 0;
    this->VT13_Data.mouse_right = 0;
    this->VT13_Data.mouse_middle = 0;
    this->VT13_Data.key = 0;
    this->VT13_Data.crc16 = 0;
}
void RC::DT16_data_deal(void)
{
    this->DT16_yaogan.ch0 = ((DR16_rx_buffer[0] | (DR16_rx_buffer[1] << 8)) & 0x07ff) - 1024;        //!< Channel 0
    this->DT16_yaogan.ch1 = (((DR16_rx_buffer[1] >> 3) | (DR16_rx_buffer[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
    this->DT16_yaogan.ch2 = (((DR16_rx_buffer[2] >> 6) | (DR16_rx_buffer[3] << 2) |                  //!< Channel 2
                              (DR16_rx_buffer[4] << 10)) &
                             0x07ff) -
                            1024;
    this->DT16_yaogan.ch3 = (((DR16_rx_buffer[4] >> 1) | (DR16_rx_buffer[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
    this->DT16_yaogan.s1 = ((DR16_rx_buffer[5] >> 4) & 0x000C) >> 2;                                 //!< Switch left
    this->DT16_yaogan.s2 = ((DR16_rx_buffer[5] >> 4) & 0x0003);                                      //!< Switch right
    this->DT16_shubiao.x = (int16_t)(DR16_rx_buffer[6] | (DR16_rx_buffer[7] << 8));                  //!< Mouse X axis
    this->DT16_shubiao.y = (int16_t)(DR16_rx_buffer[8] | (DR16_rx_buffer[9] << 8));                  //!< Mouse Y axis
    this->DT16_shubiao.z = (int16_t)(DR16_rx_buffer[10] | (DR16_rx_buffer[11] << 8));                //!< Mouse Z axis
    this->DT16_shubiao.press_l = DR16_rx_buffer[12];                                                 //!< Mouse Left Is Press ?
    this->DT16_shubiao.press_r = DR16_rx_buffer[13];                                                 //!< Mouse Right Is Press ?
    this->DT16_jianpan = DR16_rx_buffer[14] | (DR16_rx_buffer[15] << 8);
    this->DT16_yaogan.v = (int16_t)(DR16_rx_buffer[16] | (DR16_rx_buffer[17] << 8)) - 1024;
    rx_cnt++;
}
void RC::DT16_2data_deal(uint8_t *rxbuf)
{
    this->DT16_yaogan.ch0 = ((rxbuf[0] | (rxbuf[1] << 8)) & 0x07ff) - 1024;        //!< Channel 0
    this->DT16_yaogan.ch1 = (((rxbuf[1] >> 3) | (rxbuf[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
    this->DT16_yaogan.ch2 = (((rxbuf[2] >> 6) | (rxbuf[3] << 2) |                  //!< Channel 2
                              (rxbuf[4] << 10)) &
                             0x07ff) -
                            1024;
    this->DT16_yaogan.ch3 = (((rxbuf[4] >> 1) | (rxbuf[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
    this->DT16_yaogan.s1 = ((rxbuf[5] >> 4) & 0x000C) >> 2;                        //!< Switch left
    this->DT16_yaogan.s2 = ((rxbuf[5] >> 4) & 0x0003);                             //!< Switch right
    this->DT16_shubiao.x = (int16_t)(rxbuf[6] | (rxbuf[7] << 8));                  //!< Mouse X axis
    this->DT16_shubiao.y = (int16_t)(rxbuf[8] | (rxbuf[9] << 8));                  //!< Mouse Y axis
    this->DT16_shubiao.z = (int16_t)(rxbuf[10] | (rxbuf[11] << 8));                //!< Mouse Z axis
    this->DT16_shubiao.press_l = rxbuf[12];                                        //!< Mouse Left Is Press ?
    this->DT16_shubiao.press_r = rxbuf[13];                                        //!< Mouse Right Is Press ?
    this->DT16_jianpan = rxbuf[14] | (rxbuf[15] << 8);
    this->DT16_yaogan.v = (int16_t)(rxbuf[16] | (rxbuf[17] << 8)) - 1024;
    rx_cnt++;
}

void RC::VT13_data_deal(void)
{
    this->VT13_Data.ch_0 = ((VT13_rx_buffer[2] | (VT13_rx_buffer[3] << 8)) & 0x07ff) - 1024;
    this->VT13_Data.ch_1 = (((VT13_rx_buffer[3] >> 3) | (VT13_rx_buffer[4] << 5)) & 0x07ff) - 1024;
    this->VT13_Data.ch_2 = (((VT13_rx_buffer[4] >> 6) | (VT13_rx_buffer[5] << 2) |
                             (VT13_rx_buffer[6] << 10)) &
                            0x07ff) -
                           1024;
    this->VT13_Data.ch_3 = (((VT13_rx_buffer[6] >> 1) | (VT13_rx_buffer[7] << 7)) & 0x07ff) - 1024;
    this->VT13_Data.mode_sw = ((VT13_rx_buffer[7] >> 4) & 0x0003);
    this->VT13_Data.pause = ((VT13_rx_buffer[7] >> 6) & 0x01);
    this->VT13_Data.fn_1 = ((VT13_rx_buffer[7] >> 7) & 0x01);
    this->VT13_Data.fn_2 = ((VT13_rx_buffer[8] >> 0) & 0x01);
    this->VT13_Data.wheel = (((VT13_rx_buffer[8] >> 1) | (VT13_rx_buffer[9] << 7)) & 0x07FF) - 1024;
    this->VT13_Data.trigger = (VT13_rx_buffer[9] >> 4) & 0x01;

    this->VT13_Data.mouse_x = (VT13_rx_buffer[10] | (VT13_rx_buffer[11] << 8));
    this->VT13_Data.mouse_y = (VT13_rx_buffer[12] | (VT13_rx_buffer[13] << 8));
    this->VT13_Data.mouse_z = (VT13_rx_buffer[14] | (VT13_rx_buffer[15] << 8));
    this->VT13_Data.mouse_left = (VT13_rx_buffer[16] >> 0) & 0x03;
    this->VT13_Data.mouse_right = (VT13_rx_buffer[16] >> 2) & 0x03;
    this->VT13_Data.mouse_middle = (VT13_rx_buffer[16] >> 4) & 0x03;
    this->VT13_Data.key = (VT13_rx_buffer[17] | (VT13_rx_buffer[18] << 8));
    this->VT13_Data.crc16 = (VT13_rx_buffer[19] | (VT13_rx_buffer[20] << 8)); // 不建议使用

    // this->VT13_Data.ch_0 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_1 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_2 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_3 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.wheel -= RC_CH_VALUE_OFFSET; //-1024
}
void RC::VT13_2data_deal(uint8_t *rxbuf)
{
    this->VT13_Data.ch_0 = ((rxbuf[2] | (rxbuf[3] << 8)) & 0x07ff) - 1024;
    this->VT13_Data.ch_1 = (((rxbuf[3] >> 3) | (rxbuf[4] << 5)) & 0x07ff) - 1024;
    this->VT13_Data.ch_2 = (((rxbuf[4] >> 6) | (rxbuf[5] << 2) |
                             (rxbuf[6] << 10)) &
                            0x07ff) -
                           1024;
    this->VT13_Data.ch_3 = (((rxbuf[6] >> 1) | (rxbuf[7] << 7)) & 0x07ff) - 1024;
    this->VT13_Data.mode_sw = ((rxbuf[7] >> 4) & 0x0003);
    this->VT13_Data.pause = ((rxbuf[7] >> 6) & 0x01);
    this->VT13_Data.fn_1 = ((rxbuf[7] >> 7) & 0x01);
    this->VT13_Data.fn_2 = ((rxbuf[8] >> 0) & 0x01);
    this->VT13_Data.wheel = (((rxbuf[8] >> 1) | (rxbuf[9] << 7)) & 0x07FF) - 1024;
    this->VT13_Data.trigger = (rxbuf[9] >> 4) & 0x01;
    this->VT13_Data.mouse_x = (rxbuf[10] | (rxbuf[11] << 8));
    this->VT13_Data.mouse_y = (rxbuf[12] | (rxbuf[13] << 8));
    this->VT13_Data.mouse_z = (rxbuf[14] | (rxbuf[15] << 8));
    this->VT13_Data.mouse_left = (rxbuf[16] >> 0) & 0x03;
    this->VT13_Data.mouse_right = (rxbuf[16] >> 2) & 0x03;
    this->VT13_Data.mouse_middle = (rxbuf[16] >> 4) & 0x03;
    this->VT13_Data.key = (rxbuf[17] | (rxbuf[18] << 8));
    this->VT13_Data.crc16 = (rxbuf[19] | (rxbuf[20] << 8)); // 不建议使用

    // this->VT13_Data.ch_0 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_1 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_2 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.ch_3 -= RC_CH_VALUE_OFFSET;
    // this->VT13_Data.wheel -= RC_CH_VALUE_OFFSET; //-1024
}
void RC::VT13_self_ctrl_deal(void)
{
    if (this->VT13_rx_buffer[0] == 0XA5 && this->VT13_rx_buffer[6] == 0x03) // 不是就不是
    {
        custom_controller_data_t poly_Custom_Controller_data;
        // if (this->tuchuan_rx_buffer[5] == 0X04) // 0X304,键鼠数据
        // {
        //     shubiao.x = tuchuan_rx_buffer[6 + 1] | tuchuan_rx_buffer[6 + 2] << 8;
        //     shubiao.y = tuchuan_rx_buffer[6 + 2 + 1] | tuchuan_rx_buffer[6 + 2 + 2] << 8;
        //     shubiao.z = tuchuan_rx_buffer[6 + 2 + 2 + 1] | tuchuan_rx_buffer[6 + 2 + 2 + 2] << 8;
        //     shubiao.press_l = tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1];
        //     shubiao.press_r = tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1 + 1];
        //     memcpy(&jianpan, &tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1 + 1 + 1], sizeof(jianpan));
        //     rx_304id_cnt++;
        // }
        // else
        if (VT13_rx_buffer[5] == 0X02) // 0x302,自定义控制器
        {
            memcpy(&poly_Custom_Controller_data.j0.c[0], &VT13_rx_buffer[7], 4);
            memcpy(&poly_Custom_Controller_data.j1.c[0], &VT13_rx_buffer[7 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j2.c[0], &VT13_rx_buffer[7 + 4 + 4], 4);

            memcpy(&poly_Custom_Controller_data.j3.c[0], &VT13_rx_buffer[7 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j4.c[0], &VT13_rx_buffer[7 + 4 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j5.c[0], &VT13_rx_buffer[7 + 4 + 4 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j6.c[0], &VT13_rx_buffer[7 + 4 + 4 + 4 + 4 + 4 + 4], 4);

            poly_Custom_Controller_data.y = VT13_rx_buffer[7 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1];
            poly_Custom_Controller_data.z = VT13_rx_buffer[7 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 2];
            if (isnan(poly_Custom_Controller_data.j0.f) ||
                isnan(poly_Custom_Controller_data.j1.f) ||
                isnan(poly_Custom_Controller_data.j2.f) ||
                isnan(poly_Custom_Controller_data.j3.f) ||
                isnan(poly_Custom_Controller_data.j4.f) ||
                isnan(poly_Custom_Controller_data.j5.f) ||
                isnan(poly_Custom_Controller_data.j6.f))
            {
                self_err_cnt++;
            }
            else
            {
                Custom_Controller_data.j0.f = poly_Custom_Controller_data.j0.f;
                Custom_Controller_data.j1.f = poly_Custom_Controller_data.j1.f;
                Custom_Controller_data.j2.f = poly_Custom_Controller_data.j2.f;
                Custom_Controller_data.j3.f = poly_Custom_Controller_data.j3.f;
                Custom_Controller_data.j4.f = poly_Custom_Controller_data.j4.f;
                Custom_Controller_data.j5.f = poly_Custom_Controller_data.j5.f;
                Custom_Controller_data.j6.f = poly_Custom_Controller_data.j6.f;
                self_ctrl_enable_flag = 1;
                selfctrl_cnt++;
            }
        }
        // else if (tuchuan_rx_buffer[5] == 0x09) // 机器人发自定义控制器
        // {
        //     memcpy(&passback_data, &tuchuan_rx_buffer[7], sizeof(passback_data));
        //     rx_309id_cnt++;
        // }
        // else
        // this->set_zero();
        // this->Data_packaging();
    }
}

void RC::VT13_2self_ctrl_deal(uint8_t *rxbuf)
{
    if (rxbuf[0] == 0XA5 && rxbuf[6] == 0x03) // 不是就不是
    {
        custom_controller_data_t poly_Custom_Controller_data;
        // if (this->tuchuan_rx_buffer[5] == 0X04) // 0X304,键鼠数据
        // {
        //     shubiao.x = tuchuan_rx_buffer[6 + 1] | tuchuan_rx_buffer[6 + 2] << 8;
        //     shubiao.y = tuchuan_rx_buffer[6 + 2 + 1] | tuchuan_rx_buffer[6 + 2 + 2] << 8;
        //     shubiao.z = tuchuan_rx_buffer[6 + 2 + 2 + 1] | tuchuan_rx_buffer[6 + 2 + 2 + 2] << 8;
        //     shubiao.press_l = tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1];
        //     shubiao.press_r = tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1 + 1];
        //     memcpy(&jianpan, &tuchuan_rx_buffer[6 + 2 + 2 + 2 + 1 + 1 + 1], sizeof(jianpan));
        //     rx_304id_cnt++;
        // }
        // else
        if (rxbuf[5] == 0X02) // 0x302,自定义控制器
        {
            memcpy(&poly_Custom_Controller_data.j0.c[0], &rxbuf[7], 4);
            memcpy(&poly_Custom_Controller_data.j1.c[0], &rxbuf[7 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j2.c[0], &rxbuf[7 + 4 + 4], 4);

            memcpy(&poly_Custom_Controller_data.j3.c[0], &rxbuf[7 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j4.c[0], &rxbuf[7 + 4 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j5.c[0], &rxbuf[7 + 4 + 4 + 4 + 4 + 4], 4);
            memcpy(&poly_Custom_Controller_data.j6.c[0], &rxbuf[7 + 4 + 4 + 4 + 4 + 4 + 4], 4);

            poly_Custom_Controller_data.y = rxbuf[7 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1];
            poly_Custom_Controller_data.z = rxbuf[7 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 2];
            if (isnan(poly_Custom_Controller_data.j0.f) ||
                isnan(poly_Custom_Controller_data.j1.f) ||
                isnan(poly_Custom_Controller_data.j2.f) ||
                isnan(poly_Custom_Controller_data.j3.f) ||
                isnan(poly_Custom_Controller_data.j4.f) ||
                isnan(poly_Custom_Controller_data.j5.f) ||
                isnan(poly_Custom_Controller_data.j6.f))
            {
                self_err_cnt++;
            }
            else
            {
                Custom_Controller_data.j0.f = poly_Custom_Controller_data.j0.f;
                Custom_Controller_data.j1.f = poly_Custom_Controller_data.j1.f;
                Custom_Controller_data.j2.f = poly_Custom_Controller_data.j2.f;
                Custom_Controller_data.j3.f = poly_Custom_Controller_data.j3.f;
                Custom_Controller_data.j4.f = poly_Custom_Controller_data.j4.f;
                Custom_Controller_data.j5.f = poly_Custom_Controller_data.j5.f;
                Custom_Controller_data.j6.f = poly_Custom_Controller_data.j6.f;
                self_ctrl_enable_flag = 1;
                selfctrl_cnt++;
            }
        }
        // else if (tuchuan_rx_buffer[5] == 0x09) // 机器人发自定义控制器
        // {
        //     memcpy(&passback_data, &tuchuan_rx_buffer[7], sizeof(passback_data));
        //     rx_309id_cnt++;
        // }
        // else
        // this->set_zero();
        // this->Data_packaging();
    }
}

void RC::can_receive_data_deal(uint8_t *buf)
{
    this->yaogan.ch0 = ((buf[0] | (buf[1] << 8)) & 0x07ff) - 1024;        //!< Channel 0
    this->yaogan.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
    this->yaogan.ch2 = (((buf[2] >> 6) | (buf[3] << 2) |                  //!< Channel 2
                         (buf[4] << 10)) &
                        0x07ff) -
                       1024;
    this->yaogan.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
    this->yaogan.s1 = ((buf[5] >> 4) & 0x000C) >> 2;                      //!< Switch left
    this->yaogan.s2 = ((buf[5] >> 4) & 0x0003);                           //!< Switch right
    this->jianpan = buf[6] | (buf[7] << 8);                               //!< KeyBoard value

    if (this->yaogan.ch0 == -1024 || this->yaogan.ch1 == -1024 ||
        this->yaogan.ch2 == -1024 || this->yaogan.ch3 == -1024 ||
        this->yaogan.s1 > 3 || this->yaogan.s2 > 3 ||
        this->yaogan.s1 == 0 || this->yaogan.s2 == 0)
    {
        this->DT16_set_zero();
    }
}

uint8_t RC::Pressed_Check(uint16_t keyvalue)
{
    if (this->jianpan & keyvalue)
        return 1;
    else
        return 0;
}

void RC::fill_data(void) /*选择链路*/ // 无用
{
    if (this->VT13_Data.mode_sw) // C:0   N:1   S:2
    {
        this->yaogan.ch0 = this->VT13_Data.ch_0;
        this->yaogan.ch1 = this->VT13_Data.ch_1;
        this->yaogan.ch2 = this->VT13_Data.ch_2;
        this->yaogan.ch3 = this->VT13_Data.ch_3;

        this->jianpan = this->VT13_Data.key;

        this->yaogan.s1 = YK_SW_DOWN;
        this->yaogan.s2 = YK_SW_DOWN; // this->VT13_Data.mouse_x

        this->shubiao.x = this->VT13_Data.mouse_x;
        this->shubiao.y = this->VT13_Data.mouse_y;
        this->shubiao.z = this->VT13_Data.mouse_z;
        this->shubiao.press_l = this->VT13_Data.mouse_left;
        this->shubiao.press_r = this->VT13_Data.mouse_right;
    }
    else
    {
        this->yaogan.ch0 = this->DT16_yaogan.ch0;
        this->yaogan.ch1 = this->DT16_yaogan.ch1;
        this->yaogan.ch2 = this->DT16_yaogan.ch2;
        this->yaogan.ch3 = this->DT16_yaogan.ch3;
        this->yaogan.s1 = DT16_yaogan.s1;
        this->yaogan.s2 = DT16_yaogan.s2;
        this->shubiao.x = this->DT16_shubiao.x;
        this->shubiao.y = this->DT16_shubiao.y;
        this->shubiao.z = this->DT16_shubiao.z;
        this->shubiao.press_l = this->DT16_shubiao.press_l;
        this->shubiao.press_r = this->DT16_shubiao.press_r;
        this->jianpan = this->DT16_jianpan;
        this->yaogan.v = this->DT16_yaogan.v;
    }
}

// float RC::SF(float t, float *slopeFilter, float res)
// {
//     for (int i = SF_LENGTH - 1; i > 0; i--)
//     {
//         slopeFilter[i] = slopeFilter[i - 1];
//     }
//     slopeFilter[0] = t;
//     for (int i = 0; i < SF_LENGTH; i++)
//     {
//         res += slopeFilter[i];
//     }
//     return (res / SF_LENGTH);
// }

// float RC::Mouse_X_Speed(float Xmax)
// {
//     int16_t res = 0;
//     // if (fabs(this->shubiao.x) > Xmax)
//     //     res = 0;
//     // else
//     //     res = this->SF(KalmanFilter(&KF_Mouse_X_Speed, (float)this->shubiao.x),
//     //                    this->shubiao.SFX, 0);
//     return (float)res;
// }

// float RC::Mouse_Y_Speed(float Ymax)
// {
//     int16_t res = 0;
//     // if (fabs(this->shubiao.y) > Ymax)
//     //     res = 0;
//     // else
//     //     res = this->SF(KalmanFilter(&KF_Mouse_Y_Speed, (float)this->shubiao.y),
//     //                    this->shubiao.SFY, 0);
//     return (float)res;
// }

bool RC::verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if ((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = this->get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

uint16_t RC::get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if (p_msg == NULL)
    {
        return 0xffff;
    }

    while (len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}
#endif // __USART_H__

#pragma endregion

#pragma region /*ADXRS290*/
/*******************************************************ADXRS290*******************************************************************/
#ifdef __SPI_H__
ADXRS290_StatusTypeDef ADXRS290::adxrs290_writeByte(uint8_t subAddress, uint8_t data)
{
    uint8_t t_buf[] = {subAddress, data};
    this->ADXRS290_SPI_ON();
    HAL_SPI_Transmit(hspi, (uint8_t *)t_buf, 2, 999);
    this->ADXRS290_SPI_OFF();
    HAL_Delay(1);
    return ADXRS290_OK;
}

void ADXRS290::adxrs290_readBytes(uint8_t subAddress, uint8_t count, uint8_t *spi_rev_buf)
{
    uint8_t t_buf[] = {(uint8_t)(subAddress | 0x80)};
    this->ADXRS290_SPI_ON();
    HAL_SPI_Transmit(hspi, (uint8_t *)t_buf, 1, 999);
    HAL_SPI_Receive(hspi, (uint8_t *)spi_rev_buf, count, 999);
    this->ADXRS290_SPI_OFF();
}

uint8_t ADXRS290::adxrs290_readByte(uint8_t subAddress)
{
    uint8_t data, t_buf[] = {(uint8_t)(subAddress | 0x80)};
    this->ADXRS290_SPI_ON();
    HAL_SPI_Transmit(hspi, (uint8_t *)t_buf, 1, 999);
    HAL_SPI_Receive(hspi, (uint8_t *)&data, 1, 999);
    this->ADXRS290_SPI_OFF();
    return data;
}

ADXRS290_StatusTypeDef ADXRS290::Init(uint8_t hpf_corner, uint8_t odr_lpf)
{
    uint8_t ADXRS290_check[32];

    while (strcmp("put the Update function in EXTI Rising interrupt", string_check_290) != 0)
        ;

    HAL_Delay(10);
    adxrs290_writeByte(ADXRS290_POWER_CTL, 0X00);
    adxrs290_writeByte(ADXRS290_DATA_READY, 0X01);
    adxrs290_writeByte(ADXRS290_Filter, hpf_corner << 4 | odr_lpf);
    adxrs290_writeByte(ADXRS290_POWER_CTL, 0x02);

    adxrs290_readBytes(ADXRS290_ADI_ID, 4, ADXRS290_check);
    if (!((ADXRS290_check[0] == 0XAD) & (ADXRS290_check[1] == 0X1D) & (ADXRS290_check[2] == 0x92) & (ADXRS290_check[3] == 0x09)))
        return ADXRS290_ID_ERROR;

    adxrs290_readBytes(ADXRS290_POWER_CTL, 3, ADXRS290_check);
    if (!((ADXRS290_check[0] == 0X02) & (ADXRS290_check[1] == ((hpf_corner << 4) | (odr_lpf << 0))) & (ADXRS290_check[2] == 0X01)))
        return ADXRS290_SET_ERROR;

    HAL_Delay(1);
    adxrs290_readBytes(ADXRS290_DATAX0, 4, ADXRS290_check);
    return ADXRS290_OK;
}

void ADXRS290::adxrs290_update(void)
{
    uint8_t ADXRS290_rev_buf[32];
    this->ADXRS290_SPI_ON();
    adxrs290_readBytes(ADXRS290_DATAX0, 4, ADXRS290_rev_buf);
    this->ADXRS290_SPI_OFF();
    int16_t temp_x = (int16_t)(ADXRS290_rev_buf[1] << 8 | ADXRS290_rev_buf[0] << 0);
    int16_t temp_y = (int16_t)(ADXRS290_rev_buf[3] << 8 | ADXRS290_rev_buf[2] << 0);

    if (this->sensor_data_X.dev_count <= SELF_TEST_NUM_290)
    {
        this->sensor_data_X.bias = (this->sensor_data_X.dev_count * this->sensor_data_X.bias + temp_x) / (this->sensor_data_X.dev_count + 1);
        this->sensor_data_X.v = ((float)temp_x - this->sensor_data_X.bias) / (((uint16_t)0x7fff) / 200);
        this->sensor_data_X.v_nonoise = this->sensor_data_X.v;
        this->sensor_data_X.dev_count++;
    }
    else
    {
        if (this->sensor_data_X.bias > 10 || this->sensor_data_X.bias < -10)
            this->sensor_data_X.bias = 0;
        this->sensor_data_X.v = ((float)temp_x - this->sensor_data_X.bias) / (((uint16_t)0x7fff) / 200);
        if (this->sensor_data_X.v < -DEAD_ZONE_290 || this->sensor_data_X.v > DEAD_ZONE_290)
            this->sensor_data_X.v_nonoise = this->sensor_data_X.v;
        else
            this->sensor_data_X.v_nonoise = 0;
        this->sensor_data_X.theta_euler += this->sensor_data_X.v_nonoise * 0.0020202f;
    }

    if (this->sensor_data_Y.dev_count <= SELF_TEST_NUM_290)
    {
        this->sensor_data_Y.bias = (this->sensor_data_Y.dev_count * this->sensor_data_Y.bias + temp_y) / (this->sensor_data_Y.dev_count + 1);
        this->sensor_data_Y.v = ((float)temp_y - this->sensor_data_Y.bias) / (((uint16_t)0x7fff) / 200);
        this->sensor_data_Y.v_nonoise = this->sensor_data_Y.v;
        this->sensor_data_Y.dev_count++;
    }
    else
    {
        if (this->sensor_data_Y.bias > 10 || this->sensor_data_Y.bias < -10)
            this->sensor_data_Y.bias = 0;
        this->sensor_data_Y.v = ((float)temp_y - this->sensor_data_Y.bias) / (((uint16_t)0x7fff) / 200);
        if (this->sensor_data_Y.v < -DEAD_ZONE_290 || this->sensor_data_Y.v > DEAD_ZONE_290)
            this->sensor_data_Y.v_nonoise = this->sensor_data_Y.v;
        else
            this->sensor_data_Y.v_nonoise = 0;
        this->sensor_data_Y.theta_euler += this->sensor_data_Y.v_nonoise * 0.0020202f;
    }
}
#endif
#pragma endregion

#pragma region /*ADXRS453*/
/*******************************************************ADXRS453*******************************************************************/
#ifdef __SPI_H__

ADXRS453_StatusTypeDef ADXRS453::Init()
{
    int16_t ADXRS453_RATE;
    ADXRS453_StatusTypeDef error;

    while (strcmp("put the Update function in 485hz interrupt", string_check) != 0)
        ;

    HAL_Delay(100);
    //	error=this->sensor(1,(int16_t*)&ADXRS453_RATE);
    ////	if(error!=ADXRS453_OK)return error;
    //	HAL_Delay(60);
    //	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
    ////	if(error!=ADXRS453_OK)return error;
    //	HAL_Delay(60);
    //	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
    //	if(error!=ADXRS453_OK)return error;
    HAL_Delay(60);
    error = this->sensor(0, (int16_t *)&ADXRS453_RATE); // erro1
    //	if(error!=ADXRS453_OK)return error;
    HAL_Delay(60);
    error = this->sensor(0, (int16_t *)&ADXRS453_RATE); // erro13
    if (error != ADXRS453_OK)
        return error;
    HAL_Delay(60);
    HAL_TIM_Base_Start_IT(this->htim);
    return error;
}

ADXRS453_StatusTypeDef ADXRS453::adxrs453_update(void)
{
    int16_t ADXRS453_RATE;
    ADXRS453_StatusTypeDef error;

    if (this->sensor_data.dev_count < this->SELF_TEST_NUM)
    {
        error = this->sensor(0, (int16_t *)&ADXRS453_RATE);
        if (error == ADXRS453_OK)
        {
            if (this->sensor_data.bias > this->sensor_data.offset_max)
                this->sensor_data.offset_max = this->sensor_data.bias;
            if (this->sensor_data.bias < this->sensor_data.offset_min)
                this->sensor_data.offset_min = this->sensor_data.bias;

            if (this->sensor_data.offset_max - this->sensor_data.offset_min > 300)
            {
                this->sensor_data.dev_count = 0;
            }
            if (this->sensor_data.dev_count == 0)
            {
                this->sensor_data.bias = 0;
                this->sensor_data.offset_max = 0;
                this->sensor_data.offset_min = 0;
            }
            this->sensor_data.bias = (this->sensor_data.dev_count * this->sensor_data.bias + ADXRS453_RATE) / (this->sensor_data.dev_count + 1);
            this->sensor_data.v = ((float)ADXRS453_RATE - this->sensor_data.bias) * 0.0125f;
            this->sensor_data.v_nonoise = this->sensor_data.v;
            this->sensor_data.dev_count++;
        }
    }
    else
    {
        if (this->sensor_data.bias > 300 || this->sensor_data.bias < -300)
            this->sensor_data.bias = 0;
        error = this->sensor(0, (int16_t *)&ADXRS453_RATE);
        if (error == ADXRS453_OK)
        {
            this->sensor_data.v = ((float)ADXRS453_RATE - this->sensor_data.bias) * 0.0125f;
            if (this->sensor_data.v < -this->DEAD_ZONE || this->sensor_data.v > this->DEAD_ZONE)
                this->sensor_data.v_nonoise = this->sensor_data.v;
            else
                this->sensor_data.v_nonoise = 0;
            this->sensor_data.theta_euler += this->sensor_data.v_nonoise * 0.0020202f;
            this->sensor_data.calibration = 1;
        }
    }
    return error;
}

ADXRS453_StatusTypeDef ADXRS453::sensor(bool CHK, int16_t *date)
{
    uint32_t temp = 0x20000000 | (CHK << 1);

    temp = this->TransmitReceive(temp);

    if ((temp & (0x01 << 28)) != (this->odd_check(temp & 0xEFFF0000) << 28))
        return ADXRS453_P0_ERROR;
    else if ((temp & 0x00000001) != (this->odd_check(temp & 0xFFFFFFFE) << 0))
        return ADXRS453_P1_ERROR;
    else if ((temp & (0x01 << 1)) != 0)
        return ADXRS453_CHK_ERROR;
    else if ((temp & (0x01 << 7)) != 0)
        return ADXRS453_PLL_ERROR;
    else if ((temp & (0x01 << 6)) != 0)
        return ADXRS453_Q_ERROR;
    else if ((temp & (0x01 << 5)) != 0)
        return ADXRS453_NVM_ERROR;
    else if ((temp & (0x01 << 4)) != 0)
        return ADXRS453_POR_ERROR;
    else if ((temp & (0x01 << 3)) != 0)
        return ADXRS453_PWR_ERROR;
    else if ((temp & (0x01 << 2)) != 0)
        return ADXRS453_CST_ERROR;

    else if ((temp & 0xEC000000) != 0x4000000)
    {
        if ((temp & 0xEFF80000) == 0x0E000000)
            return ADXRS453_RW_ERROR;
        else if ((temp & (0x01 << 18)) != 0)
            return ADXRS453_SPI_ERROR;
        else if ((temp & (0x01 << 17)) != 0)
            return ADXRS453_RE_ERROR;
        else if ((temp & (0x01 << 16)) != 0)
            return ADXRS453_DU_ERROR;
        else
            return ADXRS453_RW_ERROR;
    }

    *date = temp >> 10;
    return ADXRS453_OK;
}
ADXRS453_StatusTypeDef ADXRS453::addread(uint8_t address, int16_t *date)
{
    uint32_t temp = 0x80000000 | (address << 17);
    temp = this->TransmitReceive(temp);

    if ((temp & (0x01 << 28)) != (this->odd_check(temp & 0xEFFF0000) << 28))
        return ADXRS453_P0_ERROR;
    if ((temp & 0x00000001) != (this->odd_check(temp & 0xFFFFFFFE) << 0))
        return ADXRS453_P1_ERROR;

    if ((temp & 0xEF000000) != 0x4E000000)
    {
        if ((temp & 0xEF000000) == 0x0E000000)
            return ADXRS453_RW_ERROR;
        else if ((temp & (0x01 << 18)) != 0)
            return ADXRS453_SPI_ERROR;
        else if ((temp & (0x01 << 17)) != 0)
            return ADXRS453_RE_ERROR;
        else if ((temp & (0x01 << 16)) != 0)
            return ADXRS453_DU_ERROR;
        else
            return ADXRS453_RW_ERROR;
    }

    *date = temp >> 5;
    return ADXRS453_OK;
}
uint32_t ADXRS453::TransmitReceive(uint32_t address)
{
    uint32_t temp = (address >> 16) | (address << 16), result;
    temp |= this->odd_check(temp) << 16;

    this->SPI_ON();
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)&temp, (uint8_t *)&result, 2, 999);
    this->SPI_OFF();
    result = (result >> 16) | (result << 16);
    return result;
}
bool ADXRS453::odd_check(uint32_t date)
{
    static const bool ParityTable256[256] =
        {
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
            P6(0), P6(1), P6(1), P6(0)};

    uint32_t check = date;
    check ^= check >> 16;
    check ^= check >> 8;
    return !ParityTable256[check & 0xff];
}

#endif
#pragma endregion

#pragma region /*BMI088*/
/*******************************************************BMI088*******************************************************************/
#ifdef __SPI_H__

void BMI088::BMI088_writeByte(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t subAddress, uint8_t data)
{
    uint8_t t_buf[] = {(uint8_t)(subAddress & 0x7F)};
    this->BMI088_SPI_ON(GPIOx, GPIO_Pin);
    HAL_SPI_Transmit(hspi, (uint8_t *)t_buf, 1, 999);

    //	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
    //	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
    //	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

    HAL_SPI_Transmit(hspi, &data, 1, 999);
    this->BMI088_SPI_OFF(GPIOx, GPIO_Pin);
}

void BMI088::BMI088_readBytes(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf)
{
    uint8_t t_buf[] = {(uint8_t)(subAddress | 0x80)};
    this->BMI088_SPI_ON(GPIOx, GPIO_Pin);
    HAL_SPI_Transmit(hspi, (uint8_t *)t_buf, 1, 999);
    HAL_SPI_Receive(hspi, (uint8_t *)spi_rev_buf, len, 999);
    this->BMI088_SPI_OFF(GPIOx, GPIO_Pin);
}

void BMI088::BMI088_write_Acc(uint8_t subAddress, uint8_t data)
{
    BMI088_writeByte(this->CSB1_GPIOx, this->CSB1_GPIO_Pin, subAddress, data);
}
void BMI088::BMI088_write_Gyro(uint8_t subAddress, uint8_t data)
{
    BMI088_writeByte(this->CSB2_GPIOx, this->CSB2_GPIO_Pin, subAddress, data);
}

void BMI088::BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf)
{
    uint8_t t_buf[64], temp;
    this->BMI088_readBytes(this->CSB1_GPIOx, this->CSB1_GPIO_Pin, subAddress, len + 1, t_buf);
    for (temp = 0; temp < len; temp++)
        spi_rev_buf[temp] = t_buf[temp + 1];
}

void BMI088::BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t *spi_rev_buf)
{
    this->BMI088_readBytes(this->CSB2_GPIOx, this->CSB2_GPIO_Pin, subAddress, len, spi_rev_buf);
}

void BMI088::set_zero(void)
{
    this->sensor_data = {0};
}

float inVSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

BMI088_StatusTypeDef BMI088::Init(uint8_t dwt_en)
{
    uint8_t BMI088_rev_buf[32] = {0}, loop_break = 1, loop_count = 0;

    // while (strcmp("put the Update function in 400Hz interrupt", string_check_088) != 0)
    // {
    //     INFO("BMI088 Init Error\n");
    //     HAL_Delay(10);
    // }

    // enable acc
    BMI088_write_Acc(ACC_SOFTRESET, 0xB6); // 复位加速度计
    HAL_Delay(1);

    BMI088_read_Acc(ACC_CHIP_ID, 1, BMI088_rev_buf); // 初始化为SPI模式
    HAL_Delay(5);
    BMI088_read_Acc(ACC_CHIP_ID, 1, BMI088_rev_buf); // 确定ID无误
    if (BMI088_rev_buf[0] != 0x1E)
        return BMI088_ACC_ID_ERROR;

    BMI088_write_Acc(ACC_PWR_CONF, 0x00); // 设置加速度计为正常模式
    HAL_Delay(5);
    BMI088_write_Acc(ACC_PWR_CTRL, 0x04); // 使能加速度计和温度计
    HAL_Delay(10);                        // 加速度计启动延时

    BMI088_write_Acc(ACC_CONF, 0x0A); // 设置加速度计输出速率为 400Hz

    BMI088_write_Acc(ACC_RANGE, AccRange); // 设置量程为 ±6g

    HAL_Delay(2);

    BMI088_write_Gyro(GYRO_SOFTRESET, 0xB6);           // 复位陀螺仪
    HAL_Delay(30);                                     // 陀螺仪启动延时
    BMI088_read_Gyro(GYRO_CHIP_ID, 1, BMI088_rev_buf); // 确定ID无误
    if (BMI088_rev_buf[0] != 0x0F)
        return BMI088_GYRO_ID_ERROR;

    BMI088_write_Gyro(GYRO_BANDWIDTH, 0x02); // 设置陀螺仪输出速率为 1000Hz，过滤器带宽为???Hz
    BMI088_write_Gyro(GYRO_RANGE, GyroRange);

    BMI088_write_Gyro(GYRO_SELF_TEST, 0x01); // 陀螺仪自检

    while (loop_break)
    {
        BMI088_read_Gyro(GYRO_SELF_TEST, 1, BMI088_rev_buf); // 自检
        if ((BMI088_rev_buf[0] & 0x02) == 0x02)
        {
            loop_break = 0;
        }
        loop_count++;
        if (loop_count >= 40) // 自检超时，自检失败
        {
            this->set_zero();
            return BMI088_SELFTEXT_ERROR;
        }
        HAL_Delay(10);
    }

    HAL_Delay(1);
    this->BMI088_read_Gyro(GYRO_SELF_TEST, 1, BMI088_rev_buf);
    if ((BMI088_rev_buf[0] & 0x04) != 0)
        return BMI088_SELFTEXT_ERROR;

    if (this->AccRange == BMI088_ACC_RANGE_3)
        this->AccRangsetting = 0.0008974358974f; // 1/1114.285
    else if (this->AccRange == BMI088_ACC_RANGE_6)
        this->AccRangsetting = 0.00179443359375f; // 1/557.2789
    else if (this->AccRange == BMI088_ACC_RANGE_12)
        this->AccRangsetting = 0.0035888671875f; // 1/278.6395
    else if (this->AccRange == BMI088_ACC_RANGE_24)
        this->AccRangsetting = 0.007177734375f; // 1/139.3197

    if (this->GyroRange == BMI088_GYRO_RANGE_2000)
        this->GyroResolution = 16.384f;
    else if (this->GyroRange == BMI088_GYRO_RANGE_1000)
        this->GyroResolution = 32.768f;
    else if (this->GyroRange == BMI088_GYRO_RANGE_500)
        this->GyroResolution = 65.536f;
    else if (this->GyroRange == BMI088_GYRO_RANGE_250)
        this->GyroResolution = 131.072f;
    else if (this->GyroRange == BMI088_GYRO_RANGE_125)
        this->GyroResolution = 262.144f;

    this->set_zero();
    DWT_Init();
    if (dwt_en)
    {
        enable_dwt = 1;
    }
    else
    {
        enable_dwt = 0;
        delta_T = 1.0f / freq;
    }
    // this->low_pass_filter_init();
    /*初始化滤波器*/
    lpf_gyro_param = get_cutoff_freq(lpf_gyro_cutoff_freq, freq);
    lpf_acc_param = get_cutoff_freq(lpf_acc_cutoff_freq, freq);
    HAL_TIM_Base_Start_IT(this->htim);

    return BMI088_OK;
}
// 目前不用这个函数
void BMI088::BMI088_update(void)
{
    // uint8_t BMI088_rev_buf[32];
    // int16_t temp = 0;

    // // enable acc
    // BMI088_read_Acc(TEMP_MSB, 2, BMI088_rev_buf);
    // temp = (int16_t)((BMI088_rev_buf[1] & 0xE0) | BMI088_rev_buf[0] << 8);
    // temp >>= 5;
    // this->sensor_data.temperature = temp * 0.125 + 23;

    // if (abs(this->sensor_data.temperature - this->last_temperature) > 2 && this->filter_count_temperature < 10)
    // {
    //     this->sensor_data.temperature = this->last_temperature;
    //     this->filter_count_temperature++;
    // }
    // else
    // {
    //     this->Acc_Temperature_Offset = (this->sensor_data.temperature - 25) * 0.2f;
    //     this->Gyro_Temperature_Offset = (this->sensor_data.temperature - 25) * 0.015;
    //     this->last_temperature = this->sensor_data.temperature;
    //     this->filter_count_temperature = 0;
    // }
    // BMI088_read_Acc(ACC_X_LSB, 6, BMI088_rev_buf); // 单位 mg
    // this->sensor_data.acc.x = ((int16_t)(BMI088_rev_buf[1] << 8 | BMI088_rev_buf[0])) * this->AccRangsetting;
    // this->sensor_data.acc.y = ((int16_t)(BMI088_rev_buf[3] << 8 | BMI088_rev_buf[2])) * this->AccRangsetting;
    // this->sensor_data.acc.z = ((int16_t)(BMI088_rev_buf[5] << 8 | BMI088_rev_buf[4])) * this->AccRangsetting;

    // BMI088_read_Gyro(GYRO_SELF_TEST, 1, BMI088_rev_buf);
    // if ((BMI088_rev_buf[0] & 0x10) == 0x10)
    // {
    //     BMI088_read_Gyro(RATE_X_LSB, 6, BMI088_rev_buf); // 单位 °/s
    //     this->sensor_data.gyro.origin.x = ((int16_t)(BMI088_rev_buf[1] << 8 | BMI088_rev_buf[0]));
    //     this->sensor_data.gyro.origin.y = ((int16_t)(BMI088_rev_buf[3] << 8 | BMI088_rev_buf[2]));
    //     this->sensor_data.gyro.origin.z = ((int16_t)(BMI088_rev_buf[5] << 8 | BMI088_rev_buf[4]));

    //     this->sensor_data.gyro.LPF.x = this->low_pass_filter(this->sensor_data.gyro.origin.x);
    //     this->sensor_data.gyro.LPF.y = this->low_pass_filter(this->sensor_data.gyro.origin.y);
    //     this->sensor_data.gyro.LPF.z = this->low_pass_filter(this->sensor_data.gyro.origin.z);

    //     if (this->sensor_data.calibration)
    //     {
    //         this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x - this->sensor_data.gyro.offset.x + this->Acc_Temperature_Offset;
    //         this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y - this->sensor_data.gyro.offset.y + this->Acc_Temperature_Offset;
    //         this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z - this->sensor_data.gyro.offset.z + this->Acc_Temperature_Offset;
    //     }
    //     else
    //     {
    //         this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x + this->Acc_Temperature_Offset;
    //         this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y + this->Acc_Temperature_Offset;
    //         this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z + this->Acc_Temperature_Offset;
    //     }

    //     if (this->sensor_data.runningTimes < this->SELF_TEST_NUM)
    //     {
    //         if (this->sensor_data.gyro.offset_max.x - this->sensor_data.gyro.offset_min.x > 150 ||
    //             this->sensor_data.gyro.offset_max.y - this->sensor_data.gyro.offset_min.y > 150 ||
    //             this->sensor_data.gyro.offset_max.z - this->sensor_data.gyro.offset_min.z > 150)
    //         {
    //             this->sensor_data.runningTimes = 0;
    //         }
    //         if (this->sensor_data.runningTimes == 0)
    //         {
    //             this->sensor_data.gyro.origin.x = 0;
    //             this->sensor_data.gyro.origin.y = 0;
    //             this->sensor_data.gyro.origin.z = 0;

    //             this->sensor_data.gyro.dynamicSum.x = 0;
    //             this->sensor_data.gyro.dynamicSum.y = 0;
    //             this->sensor_data.gyro.dynamicSum.z = 0;

    //             this->sensor_data.gyro.offset_max.x = -32768;
    //             this->sensor_data.gyro.offset_max.y = -32768;
    //             this->sensor_data.gyro.offset_max.z = -32768;
    //             this->sensor_data.gyro.offset_min.x = 32768;
    //             this->sensor_data.gyro.offset_min.y = 32768;
    //             this->sensor_data.gyro.offset_min.z = 32768;
    //         }

    //         if (this->sensor_data.gyro.origin.x > this->sensor_data.gyro.offset_max.x)
    //             this->sensor_data.gyro.offset_max.x = this->sensor_data.gyro.origin.x;
    //         if (this->sensor_data.gyro.origin.y > this->sensor_data.gyro.offset_max.y)
    //             this->sensor_data.gyro.offset_max.y = this->sensor_data.gyro.origin.y;
    //         if (this->sensor_data.gyro.origin.z > this->sensor_data.gyro.offset_max.z)
    //             this->sensor_data.gyro.offset_max.z = this->sensor_data.gyro.origin.z;

    //         if (this->sensor_data.gyro.origin.x < this->sensor_data.gyro.offset_min.x)
    //             this->sensor_data.gyro.offset_min.x = this->sensor_data.gyro.origin.x;
    //         if (this->sensor_data.gyro.origin.y < this->sensor_data.gyro.offset_min.y)
    //             this->sensor_data.gyro.offset_min.y = this->sensor_data.gyro.origin.y;
    //         if (this->sensor_data.gyro.origin.z < this->sensor_data.gyro.offset_min.z)
    //             this->sensor_data.gyro.offset_min.z = this->sensor_data.gyro.origin.z;

    //         this->sensor_data.gyro.dynamicSum.x += this->sensor_data.gyro.origin.x;
    //         this->sensor_data.gyro.dynamicSum.y += this->sensor_data.gyro.origin.y;
    //         this->sensor_data.gyro.dynamicSum.z += this->sensor_data.gyro.origin.z;

    //         this->sensor_data.runningTimes++;
    //     }
    //     else
    //     {
    //         this->sensor_data.calibration = 1;
    //     }

    //     this->sensor_data.gyro.offset.x = (float)(this->sensor_data.gyro.dynamicSum.x) / this->sensor_data.runningTimes;
    //     this->sensor_data.gyro.offset.y = (float)(this->sensor_data.gyro.dynamicSum.y) / this->sensor_data.runningTimes;
    //     this->sensor_data.gyro.offset.z = (float)(this->sensor_data.gyro.dynamicSum.z) / this->sensor_data.runningTimes;

    //     this->sensor_data.gyro.dps.x = this->sensor_data.gyro.calibration.x / this->GyroResolution;
    //     this->sensor_data.gyro.dps.y = this->sensor_data.gyro.calibration.y / this->GyroResolution;
    //     this->sensor_data.gyro.dps.z = this->sensor_data.gyro.calibration.z / this->GyroResolution;

    //     if (abs(this->sensor_data.gyro.dps.x - this->last_gyro_x) > 7 && filter_count_x < 2)
    //         this->filter_count_x++;
    //     else
    //     {
    //         if (abs(this->sensor_data.gyro.dps.x) < abs(this->dead_zoom))
    //             this->sensor_data.gyro.x_nonoise = 0;
    //         else
    //             this->sensor_data.gyro.x_nonoise = this->sensor_data.gyro.dps.x;
    //         this->last_gyro_x = this->sensor_data.gyro.dps.x;
    //         this->filter_count_x = 0;
    //     }

    //     if (abs(this->sensor_data.gyro.dps.y - this->last_gyro_y) > 7 && filter_count_y < 2)
    //         this->filter_count_y++;
    //     else
    //     {
    //         if (abs(this->sensor_data.gyro.dps.y) < abs(this->dead_zoom))
    //             this->sensor_data.gyro.y_nonoise = 0;
    //         else
    //             this->sensor_data.gyro.y_nonoise = this->sensor_data.gyro.dps.y;
    //         this->last_gyro_y = this->sensor_data.gyro.dps.y;
    //         this->filter_count_y = 0;
    //     }

    //     if (abs(this->sensor_data.gyro.dps.z - this->last_gyro_z) > 7 && filter_count_z < 2)
    //         this->filter_count_z++;
    //     else
    //     {
    //         if (abs(this->sensor_data.gyro.dps.z) < abs(this->dead_zoom))
    //             this->sensor_data.gyro.z_nonoise = 0;
    //         else
    //             this->sensor_data.gyro.z_nonoise = this->sensor_data.gyro.dps.z;
    //         this->last_gyro_z = this->sensor_data.gyro.dps.z;
    //         this->filter_count_z = 0;
    //     }

    //     this->sensor_data.mang.x += this->sensor_data.gyro.x_nonoise * 0.001f;
    //     this->sensor_data.mang.y += this->sensor_data.gyro.y_nonoise * 0.001f;
    //     this->sensor_data.mang.z += this->sensor_data.gyro.z_nonoise * 0.001f;
    // }
}

void BMI088::BMI088_New_update(void)
{
    if (state != BMI088_OK)
    {
        return;
    }
    uint8_t BMI088_rev_buf[32];
    // int16_t temp = 0;

    BMI088_read_Acc(ACC_X_LSB, 6, BMI088_rev_buf);
    sensor_data.acc.x = ((int16_t)(BMI088_rev_buf[1] << 8 | BMI088_rev_buf[0]));
    sensor_data.acc.y = ((int16_t)(BMI088_rev_buf[3] << 8 | BMI088_rev_buf[2]));
    sensor_data.acc.z = ((int16_t)(BMI088_rev_buf[5] << 8 | BMI088_rev_buf[4]));
    low_pass_filter(sensor_data.acc.x, &sensor_data.acc.LPF_x, lpf_acc_param);
    low_pass_filter(sensor_data.acc.y, &sensor_data.acc.LPF_y, lpf_acc_param);
    low_pass_filter(sensor_data.acc.z, &sensor_data.acc.LPF_z, lpf_acc_param);
    // sensor_data.acc.LPF_x = sensor_data.acc.x;
    // sensor_data.acc.LPF_y = sensor_data.acc.y;
    // sensor_data.acc.LPF_z = sensor_data.acc.z;
    BMI088_read_Gyro(RATE_X_LSB, 6, BMI088_rev_buf);
    sensor_data.gyro.origin.x = ((int16_t)(BMI088_rev_buf[1] << 8 | BMI088_rev_buf[0]));
    sensor_data.gyro.origin.y = ((int16_t)(BMI088_rev_buf[3] << 8 | BMI088_rev_buf[2]));
    sensor_data.gyro.origin.z = ((int16_t)(BMI088_rev_buf[5] << 8 | BMI088_rev_buf[4]));
    // low_pass_filter(sensor_data.gyro.origin.x, &sensor_data.gyro.LPF.x, lpf_gyro_param);
    // low_pass_filter(sensor_data.gyro.origin.y, &sensor_data.gyro.LPF.y, lpf_gyro_param);
    // low_pass_filter(sensor_data.gyro.origin.z, &sensor_data.gyro.LPF.z, lpf_gyro_param);

    /*notch*/
    gyroNotchX->process(sensor_data.gyro.origin.x);
    gyroNotchY->process(sensor_data.gyro.origin.y);
    gyroNotchZ->process(sensor_data.gyro.origin.z);
    sensor_data.gyro.LPF.x = gyroNotchX->out;
    sensor_data.gyro.LPF.y = gyroNotchY->out;
    sensor_data.gyro.LPF.z = gyroNotchZ->out;

    // sensor_data.gyro.LPF.x = sensor_data.gyro.origin.x;
    // sensor_data.gyro.LPF.y = sensor_data.gyro.origin.y;
    // sensor_data.gyro.LPF.z = sensor_data.gyro.origin.z;

    // this->sensor_data.gyro.LPF.x = this->low_pass_filter(this->sensor_data.gyro.origin.x);
    // this->sensor_data.gyro.LPF.y = this->low_pass_filter(this->sensor_data.gyro.origin.y);
    // this->sensor_data.gyro.LPF.z = this->low_pass_filter(this->sensor_data.gyro.origin.z);
    if (enable_dwt)
    {
        delta_T = DWT_Get_time();
        if (delta_T > 1.0f / (freq / 2) || delta_T < 1.0f / (freq * 2))
        {
            delta_T = 1.0f / freq;
        }
    }
    else
    {
        delta_T = 1.0f / freq; // 1khz
    }
}

/*四元数↓*/

/*全局变量 初始数据*/

/*姿态解算常量*/

/*自检采样次数*/

// #define CALIBRATE_TIMES 3000 // 校准的次数

/*姿态解算宏定义及变量*/

// #define delta_T 0.001f // 5ms计算一次
// #define PI 3.1415926535 // 圆周率
// #define new_weight 0.7f // 新数据权重
// #define old_weight 0.3f // 旧数据权重
// #define ACC_CONVER 2048.f
// #define GYRO_CONVER 16.4f

/*算法部分*/

/*快速平方根（快速计算浮点数的倒数平方根）*/

/*对加速度计数据一阶低通滤波，对陀螺仪数据转成弧度每秒(2000dps)*/

void BMI088::getValues(void)
{
    // float unoffset_acc_x, unoffset_acc_y, unoffset_acc_z;
    // float unoffset_gyro_x, unoffset_gyro_y, unoffset_gyro_z;

    // 如果校准成功，则使原始值减去零漂值得到校准值
    if (calibrationState)
    {
        // acc.calibration.data[0] = sensor_data.acc.x - acc.offset.data[0];//校准的加速度数据没用到
        // acc.calibration.data[1] = sensor_data.acc.y - acc.offset.data[1];
        // acc.calibration.data[2] = sensor_data.acc.z - acc.offset.data[2];
        /*无滤波*/
        // gyro.calibration.data[0] = sensor_data.gyro.origin.x - gyro.offset.data[0];
        // gyro.calibration.data[1] = sensor_data.gyro.origin.y - gyro.offset.data[1];
        // gyro.calibration.data[2] = sensor_data.gyro.origin.z - gyro.offset.data[2];
        /*有滤波*/
        gyro.calibration.data[0] = sensor_data.gyro.LPF.x - gyro.offset.data[0];
        gyro.calibration.data[1] = sensor_data.gyro.LPF.y - gyro.offset.data[1];
        gyro.calibration.data[2] = sensor_data.gyro.LPF.z - gyro.offset.data[2]; //
    }
    else // 否则先暂时让校准值等于原始值
    {
        // acc.calibration.data[0] = sensor_data.acc.x;
        // acc.calibration.data[1] = sensor_data.acc.y;
        // acc.calibration.data[2] = sensor_data.acc.z;
        /*无滤波*/
        // gyro.calibration.data[0] = sensor_data.gyro.origin.x;
        // gyro.calibration.data[1] = sensor_data.gyro.origin.y;
        // gyro.calibration.data[2] = sensor_data.gyro.origin.z;
        /*有滤波*/
        gyro.calibration.data[0] = sensor_data.gyro.LPF.x;
        gyro.calibration.data[1] = sensor_data.gyro.LPF.y;
        gyro.calibration.data[2] = sensor_data.gyro.LPF.z;
    }

    // // 如果运行次数小于宏定义的校准次数则
    if (acc.runningTimes < CALIBRATE_TIMES)
    {
        // 如果运行次数等于0则初始化零漂值的最大和最小值
        // 并且初始化原始值和校准时的求和累加值
        if (acc.runningTimes == 0)
        {
            sensor_data.acc.LPF_x = 0;
            sensor_data.acc.LPF_y = 0;
            sensor_data.acc.LPF_z = 0;

            sensor_data.gyro.LPF.x = 0;
            sensor_data.gyro.LPF.y = 0;
            sensor_data.gyro.LPF.z = 0;

            acc.dynamicSum.data[0] = 0;
            acc.dynamicSum.data[1] = 0;
            acc.dynamicSum.data[2] = 0;

            gyro.dynamicSum.data[0] = 0;
            gyro.dynamicSum.data[1] = 0;
            gyro.dynamicSum.data[2] = 0;

            acc.offset_max.data[0] = -32768;
            acc.offset_max.data[1] = -32768;
            acc.offset_max.data[2] = -32768;

            acc.offset_min.data[0] = 32767;
            acc.offset_min.data[1] = 32767;
            acc.offset_min.data[2] = 32767;

            gyro.offset_max.data[0] = -32768;
            gyro.offset_max.data[1] = -32768;
            gyro.offset_max.data[2] = -32768;

            gyro.offset_min.data[0] = 32767;
            gyro.offset_min.data[1] = 32767;
            gyro.offset_min.data[2] = 32767;
        }

        // 如果有一个原始值是范围内的正常的值（-32767~32767）
        // 则直接替换这个最大/最小值
        if (sensor_data.acc.LPF_x > acc.offset_max.data[0])
            acc.offset_max.data[0] = sensor_data.acc.LPF_x;
        if (sensor_data.acc.LPF_y > acc.offset_max.data[1])
            acc.offset_max.data[1] = sensor_data.acc.LPF_y;
        if (sensor_data.acc.LPF_z > acc.offset_max.data[2])
            acc.offset_max.data[2] = sensor_data.acc.LPF_z;

        if (sensor_data.acc.LPF_x < acc.offset_min.data[0])
            acc.offset_min.data[0] = sensor_data.acc.LPF_x;
        if (sensor_data.acc.LPF_y < acc.offset_min.data[1])
            acc.offset_min.data[1] = sensor_data.acc.LPF_y;
        if (sensor_data.acc.LPF_z < acc.offset_min.data[2])
            acc.offset_min.data[2] = sensor_data.acc.LPF_z;

        if (sensor_data.gyro.LPF.x > gyro.offset_max.data[0])
            gyro.offset_max.data[0] = sensor_data.gyro.LPF.x;
        if (sensor_data.gyro.LPF.y > gyro.offset_max.data[1])
            gyro.offset_max.data[1] = sensor_data.gyro.LPF.y;
        if (sensor_data.gyro.LPF.z > gyro.offset_max.data[2])
            gyro.offset_max.data[2] = sensor_data.gyro.LPF.z;

        if (sensor_data.gyro.LPF.x < gyro.offset_min.data[0])
            gyro.offset_min.data[0] = sensor_data.gyro.LPF.x;
        if (sensor_data.gyro.LPF.y < gyro.offset_min.data[1])
            gyro.offset_min.data[1] = sensor_data.gyro.LPF.y;
        if (sensor_data.gyro.LPF.z < gyro.offset_min.data[2])
            gyro.offset_min.data[2] = sensor_data.gyro.LPF.z;

        acc.dynamicSum.data[0] += sensor_data.acc.LPF_x;
        acc.dynamicSum.data[1] += sensor_data.acc.LPF_y;
        acc.dynamicSum.data[2] += sensor_data.acc.LPF_z;

        gyro.dynamicSum.data[0] += sensor_data.gyro.LPF.x;
        gyro.dynamicSum.data[1] += sensor_data.gyro.LPF.y;
        gyro.dynamicSum.data[2] += sensor_data.gyro.LPF.z;

        acc.runningTimes++;

        // 如果误差值过大则重新开始计算运行次数
        if (gyro.offset_max.data[0] - gyro.offset_min.data[0] > 150 ||
            gyro.offset_max.data[1] - gyro.offset_min.data[1] > 150 ||
            gyro.offset_max.data[2] - gyro.offset_min.data[2] > 150)
        {
            acc.runningTimes = 0;
        }
    }
    else
    {

        calibrationState = 1;
        acc.offset.data[0] = (float)acc.dynamicSum.data[0] / acc.runningTimes;
        acc.offset.data[1] = (float)acc.dynamicSum.data[1] / acc.runningTimes;
        acc.offset.data[2] = (float)acc.dynamicSum.data[2] / acc.runningTimes;

        gyro.offset.data[0] = (float)(gyro.dynamicSum.data[0]) / acc.runningTimes;
        gyro.offset.data[1] = (float)(gyro.dynamicSum.data[1]) / acc.runningTimes;
        gyro.offset.data[2] = (float)(gyro.dynamicSum.data[2]) / acc.runningTimes;
    }

    // 获得校准之后给到values数组准备姿态融合和滤波
    Deal_acc.x = ((float)sensor_data.acc.LPF_x) * AccRangsetting; // 不offset先
    Deal_acc.y = ((float)sensor_data.acc.LPF_y) * AccRangsetting;
    Deal_acc.z = ((float)sensor_data.acc.LPF_z) * AccRangsetting;

    // Deal_gyro.x = ((float)gyro.calibration.data[0]) / GyroResolution; // dps
    // Deal_gyro.y = ((float)gyro.calibration.data[1]) / GyroResolution;
    // Deal_gyro.z = ((float)gyro.calibration.data[2]) / GyroResolution;

    Deg_gyro.x = ((float)gyro.calibration.data[0]) / GyroResolution; // dps
    Deg_gyro.y = ((float)gyro.calibration.data[1]) / GyroResolution;
    Deg_gyro.z = ((float)gyro.calibration.data[2]) / GyroResolution;

    Deal_gyro.x = Deg_gyro.x * m_pi / 180.0f;
    Deal_gyro.y = Deg_gyro.y * m_pi / 180.0f;
    Deal_gyro.z = Deg_gyro.z * m_pi / 180.0f;

    // // 当前值赋值给下一次运算的上一个值
    // int count;
    // static float lastaccel[3] = {0, 0, 0};
    // static float icm_values[10];
    // 获得校准之后给到values数组准备姿态融合和滤波
    // this->Deal_acc.x = (((float)this->sensor_data.acc.x) * new_weight + lastaccel[0] * old_weight) / ACC_CONVER;
    // this->Deal_acc.y = (((float)this->sensor_data.acc.y) * new_weight + lastaccel[1] * old_weight) / ACC_CONVER;
    // this->Deal_acc.z = (((float)this->sensor_data.acc.z) * new_weight + lastaccel[2] * old_weight) / ACC_CONVER;
    // for (count = 0; count < 3; count++)
    // lastaccel[count] = icm_values[count];
}

// IMU的积分在定时器里面的dt是不准的
// 你们找个人写DWT
// 用DWT定时器的dt去计算
// 今天之内安排一个人去做
// 一个星期之内搞定 非常简单的
/* 使能 DWT 计数器 */
void BMI088::DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 打开调试追踪模块
    DWT->CYCCNT = 0;                                // 清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 开始计数
}
float BMI088::DWT_Get_time(void)
{
    uint32_t time_temp = DWT->CYCCNT;
    float time_s = (float)time_temp / 168000000.0f; // 168Mhz
    DWT->CYCCNT = 0;                                // 清零
    return time_s;
}

/*姿态解算融合，互补滤波算法*/
// gx, gy, gz: 陀螺仪角速度，单位 rad/s
// ax, ay, az: 加速度计值，单位 g（归一化前）
void BMI088::BMI088_AHRS(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 通过时间常数tau来计算Kp和Ki，公式是KP=2beta，KI=beta^2，其中beta=2.146/tau
    // 代表系统需要多少秒来修正大部分（约63%）的姿态误差，tau=28.42s
    static float param_Kp = 0.17f; // 加速度计（磁力计）的收敛速率比例增益0.17
    static float param_Ki = 0;     // 陀螺仪收敛速率的积分增益 0.01|0.07225
    float halfT = 0.5f * delta_T;  // 半个周期
    float vx, vy, vz;              // 当前的机体坐标系上的重力单位向量
    float ex, ey, ez;              // 四元数计算值与加速度计测量值的误差

    // 四元数
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    // 四元数乘积
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 对加速度数据进行归一化 得到单位加速度
    float norm = inVSqrt(ax * ax + ay * ay + az * az); // 快速求解平方根倒数函数
    ax *= norm;                                        // 归一化语句
    ay *= norm;
    az *= norm;
    // 估算重力方向
    vx = 2 * (q1q3 - q0q2); // 提取姿态矩阵中的重力分量
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 求姿态误差，对两向量进行叉乘(定义ex、ey、ez为三个轴误差元素)
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /*
        用叉乘误差来做PI修正陀螺零偏
        通过调节 param_Kp，param_Ki 两个参数
        可以控制加速度计修正陀螺仪积分姿态的速度
    */

    // 积分误差缩放
    I_ex += delta_T * ex; // 对误差积分，ki为积分系数
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;
    /*积分项无限制，可能饱和*/
    // 限幅，例如限制在 ±0.5 弧度
    // const float I_limit = 0.5f;
    // I_ex = fmaxf(fminf(I_ex, I_limit), -I_limit);
    // I_ey = fmaxf(fminf(I_ey, I_limit), -I_limit);
    // I_ez = fmaxf(fminf(I_ez, I_limit), -I_limit);

    // 互补滤波，将误差输入PID控制器后与陀螺仪测得的角速度相加，修正角速度值，Kp为互补滤波系数
    gx = gx + param_Kp * ex + param_Ki * I_ex; // 用加速度计的误差对陀螺仪进行修正
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    // 数据修正完毕，进行四元数积分

    /*
        四元数微分方程，其中halfT为测量周期的1/2
        gx gy gz为陀螺仪角速度，以下都是已知量
        这里使用了一阶龙哥库塔求解四元数微分方程

        一阶龙格-库塔方法是一种显式的单步法，
        常用于求解一阶常微分方程的初值问题。
        它通过迭代计算逐步逼近连续解。
        一阶龙格-库塔方法的基本思想是
        将微分方程的导数变化率在一个步长内进行估计，
        并使用这个估计值来更新解的近似值。
    */

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 四元数归一化
    norm = inVSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

/*把四元数转换成欧拉角*/

void BMI088::QuatToEulerAngles(void)
{
    this->getValues();
    this->BMI088_AHRS(this->Deal_gyro.x, this->Deal_gyro.y, this->Deal_gyro.z, this->Deal_acc.x, this->Deal_acc.y, this->Deal_acc.z);
    // this->q0_t = Q_info.q0;
    // this->q1_t = Q_info.q1;
    // this->q2_t = Q_info.q2;
    // this->q3_t = Q_info.q3;
    // 反解欧拉角
    this->eulerAngle.pitch = asin(-2 * this->Q_info.q1 * this->Q_info.q3 + 2 * this->Q_info.q0 * this->Q_info.q2) * 180.0f / m_pi;                                                                                    // pitch
    this->eulerAngle.roll = atan2(2 * this->Q_info.q2 * this->Q_info.q3 + 2 * this->Q_info.q0 * this->Q_info.q1, -2 * this->Q_info.q1 * this->Q_info.q1 - 2 * this->Q_info.q2 * this->Q_info.q2 + 1) * 180.0f / m_pi; // roll
    this->eulerAngle.yaw = atan2(2 * this->Q_info.q1 * this->Q_info.q2 + 2 * this->Q_info.q0 * this->Q_info.q3, -2 * this->Q_info.q2 * this->Q_info.q2 - 2 * this->Q_info.q3 * this->Q_info.q3 + 1) * 180.0f / m_pi;  // yaw
    // Analyse_speed();
}

/*过圈检测*/

void BMI088::BMI_CrossRound_err(void)
{
    // Yaw轴过圈
    if (this->eulerAngle.yaw - this->lastAngle.yaw > 180)
        this->Round.roundYaw--;
    else if (this->eulerAngle.yaw - this->lastAngle.yaw < -180)
        this->Round.roundYaw++;
    this->realAngle.yaw = this->Round.roundYaw * 360 + this->eulerAngle.yaw;

    // Pitch轴过圈
    if (this->eulerAngle.pitch - this->lastAngle.pitch > 180)
        this->Round.roundPitch--;
    else if (this->eulerAngle.pitch - this->lastAngle.pitch < -180)
        this->Round.roundPitch++;
    this->realAngle.pitch = this->Round.roundPitch * 360 + this->eulerAngle.pitch;

    // Roll轴过圈
    if (this->eulerAngle.roll - this->lastAngle.roll > 180)
        this->Round.roundRoll--;
    else if (this->eulerAngle.roll - this->lastAngle.roll < -180)
        this->Round.roundRoll++;
    this->realAngle.roll = this->Round.roundRoll * 360 + this->eulerAngle.roll;
    // 更新上次的欧拉角
    this->lastAngle.pitch = this->eulerAngle.pitch;
    this->lastAngle.yaw = this->eulerAngle.yaw;
    this->lastAngle.roll = this->eulerAngle.roll;
}

/*角度直接微分得到角速度，有BUG，不建议用*/
// void BMI088::D_EulerAngles(void)
// {

//     this->Anglespeed.pitch = (this->eulerAngle.pitch - this->lastAngle.pitch) / delta_T; // dt
//     this->Anglespeed.yaw = (this->eulerAngle.yaw - this->lastAngle.yaw) / delta_T;
//     this->Anglespeed.roll = (this->eulerAngle.roll - this->lastAngle.roll) / delta_T;

//     if (abs(this->Anglespeed.pitch - this->last_gyro_x) > 7 && this->filter_count_x < 2)
//         this->filter_count_x++;
//     else
//     {
//         if (abs(this->Anglespeed.pitch) < abs(this->dead_zoom))
//             this->Anglespeed.Deal_pitch = 0;
//         else
//             this->Anglespeed.Deal_pitch = this->Anglespeed.pitch;
//         this->last_gyro_x = this->Anglespeed.pitch;
//         this->filter_count_x = 0;
//     }

//     if (abs(this->Anglespeed.yaw - this->last_gyro_y) > 7 && this->filter_count_y < 2)
//         this->filter_count_y++;
//     else
//     {
//         if (abs(this->Anglespeed.yaw) < abs(this->dead_zoom))
//             this->Anglespeed.Deal_yaw = 0;
//         else
//             this->Anglespeed.Deal_yaw = this->Anglespeed.yaw;
//         this->last_gyro_y = this->Anglespeed.yaw;
//         this->filter_count_y = 0;
//     }

//     if (abs(this->Anglespeed.roll - this->last_gyro_z) > 7 && this->filter_count_z < 2)
//         this->filter_count_z++;
//     else
//     {
//         if (abs(this->Anglespeed.roll) < abs(this->dead_zoom))
//             this->Anglespeed.Deal_roll = 0;
//         else
//             this->Anglespeed.Deal_roll = this->Anglespeed.roll;
//         this->last_gyro_z = this->Anglespeed.roll;
//         this->filter_count_z = 0;
//     }
//     last_realAngle.pitch = this->realAngle.pitch;
//     last_realAngle.yaw = this->realAngle.yaw;
//     last_realAngle.roll = this->realAngle.roll;
// }

/*角度直接微分得到角速度，有一定延时，不建议用*/
void BMI088::D_EulerAngles(void)
{

    /*过圈的角度微分*/
    this->E_DAngle.pitch = (this->realAngle.pitch - this->last_realAngle.pitch) / delta_T; // dt
    this->E_DAngle.yaw = (this->realAngle.yaw - this->last_realAngle.yaw) / delta_T;
    this->E_DAngle.roll = (this->realAngle.roll - this->last_realAngle.roll) / delta_T;

    if (abs(this->E_DAngle.pitch) < dead_zoom)
        this->E_DAngle.Deal_pitch = 0;
    else
        this->E_DAngle.Deal_pitch = this->E_DAngle.pitch;

    if (abs(this->E_DAngle.yaw) < dead_zoom)
        this->E_DAngle.Deal_yaw = 0;
    else
        this->E_DAngle.Deal_yaw = this->E_DAngle.yaw;

    if (abs(this->E_DAngle.roll) < dead_zoom)
        this->E_DAngle.Deal_roll = 0;
    else
        this->E_DAngle.Deal_roll = this->E_DAngle.roll;

    last_realAngle.pitch = this->realAngle.pitch;
    last_realAngle.yaw = this->realAngle.yaw;
    last_realAngle.roll = this->realAngle.roll;
}

/*直接使用陀螺仪原始角速度（载体坐标系），乘以旋转矩阵，得到世界坐标系角速度*/
void BMI088::Analyse_speed(void)
{
    // 从当前四元数计算旋转矩阵（机体 -> 世界）
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;

    // 旋转矩阵 R_nb 的各个元素
    float R11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    float R12 = 2 * (q1 * q2 - q0 * q3);
    float R13 = 2 * (q1 * q3 + q0 * q2);
    float R21 = 2 * (q1 * q2 + q0 * q3);
    float R22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    float R23 = 2 * (q2 * q3 - q0 * q1);
    float R31 = 2 * (q1 * q3 - q0 * q2);
    float R32 = 2 * (q2 * q3 + q0 * q1);
    float R33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 载体坐标系角速度（弧度/秒），直接使用 Deal_gyro 原始值（已经过校准和单位转换）
    float wx_body = this->Deal_gyro.x / m_pi * 180.0f;
    float wy_body = this->Deal_gyro.y / m_pi * 180.0f;
    float wz_body = this->Deal_gyro.z / m_pi * 180.0f;

    // 转换到世界坐标系角速度
    this->Anglespeed.pitch = R21 * wx_body + R22 * wy_body + R23 * wz_body; // 世界系 X 轴角速度（对应横滚速率？注意命名）
    this->Anglespeed.yaw = R31 * wx_body + R32 * wy_body + R33 * wz_body;   // 世界系 Y 轴角速度
    this->Anglespeed.roll = R11 * wx_body + R12 * wy_body + R13 * wz_body;  // 世界系 Z 轴角速度

    // 可选：添加死区处理
    // if (fabs(this->Anglespeed.pitch) < this->dead_zoom)
    //     this->Anglespeed.Deal_pitch = 0;
    // else
    //     this->Anglespeed.Deal_pitch = this->Anglespeed.pitch;
    // ... 其他轴类似
}

/*欧拉角解算*/
void BMI088::analyse(void)
{
    this->BMI088_New_update();
    this->QuatToEulerAngles();  // 四元数转换为欧拉角
    this->BMI_CrossRound_err(); // 过圈检测
    D_EulerAngles();
    Anglespeed.Deal_pitch = E_DAngle.Deal_pitch;
    Anglespeed.Deal_yaw = E_DAngle.Deal_yaw;
    Anglespeed.Deal_roll = E_DAngle.Deal_roll;
}
/*四元数↑*/

void BMI088::selftext_error_reset(void)
{
    switch (this->selftext_reset_step)
    {
    case 0:
        BMI088_write_Gyro(GYRO_SOFTRESET, 0xB6);
        this->timer_1ms = 0;
        this->selftext_reset_step = 1;
        break;

    case 1:
        this->timer_1ms++;
        if (timer_1ms > 30)
            this->selftext_reset_step = 2;
        break;

    case 2:
        BMI088_write_Gyro(GYRO_BANDWIDTH, 0x03);
        BMI088_write_Gyro(GYRO_RANGE, 0x01);
        this->selftext_reset_step = 3;
        break;

    case 3:
        BMI088_write_Gyro(GYRO_SELF_TEST, 0x01);
        this->timer_1ms = 0;
        this->selftext_reset_step = 4;
        break;

    case 4:
        this->timer_1ms++;
        if (timer_1ms > 10)
        {
            this->selftext_error_flag = 0;
            this->selftext_reset_step = 0;
        }
        break;

    default:
        this->selftext_reset_step = 0;
        break;
    }
}

/************************ 滤波器初始化 alpha *****************************/
void BMI088::low_pass_filter_init(void)
{

    float b = 2.0f * m_pi * LPF_factor.CUTOFF_FREQ * LPF_factor.SAMPLE_RATE;
    LPF_factor.alpha = b / (b + 1);
}

void BMI088::low_pass_filter(float value, float *out_last, float k)
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

#include <cmath>

NotchFilter::NotchFilter(float fs, float f0, float Q)
    : x1(0.0f), x2(0.0f), y1(0.0f), y2(0.0f), Q0(Q), FS(fs), F0(f0)
{
    NF_Param_update(fs, f0, Q);
}
void NotchFilter::NF_Param_update(float fs, float f0, float Q)
{
    float omega = 2.0f * m_pi * f0 / fs; // 数字角频率
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);

    float alpha = sin_omega / (2.0f * Q); // 带宽参数

    // 分子系数 (归一化分母 a0 = 1 + alpha)
    b0 = 1.0f / (1.0f + alpha);
    b1 = -2.0f * cos_omega * b0;
    b2 = 1.0f * b0;

    // 分母系数 (a0 已归一化)
    a1 = -2.0f * cos_omega / (1.0f + alpha);
    a2 = (1.0f - alpha) / (1.0f + alpha);
}
void NotchFilter::reset()
{
    x1 = x2 = 0.0f;
    y1 = y2 = 0.0f;
}

float NotchFilter::process(float input)
{
    // 直接 II 型结构
    out = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    // 更新延迟
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = out;

    return out;
}
#endif
#pragma endregion

#pragma region /*V_LPF*/
/**************************************Vision_LPF***********************************************/
void Vision_LPF::Vision_Low_Pass_Filter_Init(void)
{
    this->b = 2.0f * this->pi * this->CUTOFF_FREQ * this->SAMPLE_RATE;
    this->alpha = this->b / (this->b + 1);
}

float Vision_LPF::Vision_Low_Pass_Filter(float value)
{

    /***************** 如果第一次进入，则给 out_last 赋值 ******************/
    static char fisrt_flag = 1;
    if (fisrt_flag == 1)
    {
        fisrt_flag = 0;
        this->out_last = value;
    }

    /*************************** 一阶滤波 *********************************/
    this->out = this->out_last + this->alpha * (value - this->out_last);
    this->out_last = this->out;

    return this->out;
}
#pragma endregion

#pragma region /*MCL_snail*/
/****************************************PWM_MCL*********************************************/
#ifdef __TIM_H__

void MCL_snail::Init(void)
{
    while (strcmp("put the state_tick function in 50hz interrupt", this->string_check) != 0)
        ;
    HAL_TIM_PWM_Start(this->htim_z, this->Channel_z);
    HAL_TIM_PWM_Start(this->htim_y, this->Channel_y);
    set_speed(0, 0);
    first_state = 0;
}
void MCL_snail::Init_XC_Calibration(uint8_t speed_z_max, uint8_t speed_y_max) // 行程校准
{
    while (strcmp("put the state_tick function in 50hz interrupt", this->string_check) != 0)
        ;
    HAL_TIM_PWM_Start(this->htim_z, this->Channel_z);
    HAL_TIM_PWM_Start(this->htim_y, this->Channel_y);

    set_speed(speed_z_max, speed_y_max);
    HAL_Delay(1000);
    set_speed(0, 0);
    HAL_Delay(4000);
}
void MCL_snail::Init_Change_Steer(uint8_t dir, uint8_t speed_max) // 切换转向
{
    while (strcmp("put the state_tick function in 50hz interrupt", this->string_check) != 0)
        ;

    if (dir)
        HAL_TIM_PWM_Start(this->htim_z, this->Channel_z);
    else
        HAL_TIM_PWM_Start(this->htim_y, this->Channel_y);

    set_speed(speed_max, speed_max);
    HAL_Delay(6000);
    set_speed(0, 0);
    HAL_Delay(5000);
}
void MCL_snail::stop(void)
{
    if (first_state)
    {
        set_speed(0, 0);
        this->time_20ms = 0, this->shoot_state_byte = 0, this->run_stete = 0;
    }
}
void MCL_snail::run(uint8_t grade)
{
    if (first_state)
    {
        this->run_stete = 1;
        if (grade == 1)
            this->set_speed(this->grade_1, this->grade_1 - this->grade_1_error);
        else if (grade == 2)
            this->set_speed(this->grade_2, this->grade_2 - this->grade_2_error);
        else if (grade == 3)
            this->set_speed(this->grade_3, this->grade_3 - this->grade_3_error);
    }
}
HAL_StatusTypeDef MCL_snail::shoot_state(void)
{
    if (this->shoot_state_byte)
    {
        this->time_20ms = 200 / 20;
        this->shoot_state_byte = 0;
        return HAL_OK;
    }
    else
        return HAL_ERROR;
}
void MCL_snail::state_tick(TIM_HandleTypeDef *p)
{
    if (this->htim == p)
    {
        if (first_state)
        {
            if (this->run_stete)
            {
                this->time_20ms++;
                if (this->time_20ms >= 500 / 20)
                    this->shoot_state_byte = 1;
            }
        }
        else
        {
            this->time_20ms++;
            if (this->time_20ms >= 4000 / 20)
                first_state = 1, this->time_20ms = 0;
        }
    }
}
void MCL_snail::set_speed(uint8_t speed_z, uint8_t speed_y)
{
    __HAL_TIM_SET_COMPARE(this->htim_z, this->Channel_z, speed_z + 100);
    __HAL_TIM_SET_COMPARE(this->htim_y, this->Channel_y, speed_y + 100);
}
#endif /*__TIM_H__*/
#pragma endregion

#pragma region /*WS2812 RGB*/
/*************************************RGB**********************************************/
#ifdef __TIM_H__

// 启动DMA-PWM传输。
void RGB_UI::WS_Load(void)
{
    DMAstatus = HAL_TIM_PWM_Start_DMA(this->htim, this->Channel, (uint32_t *)send_Buf, NUM);
    //	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)send_Buf, NUM);
}

// 关闭所有灯：
void RGB_UI::WS_CloseAll(void)
{
    uint16_t i;
    for (i = 0; i < PIXEL_NUM * 24; i++)
        send_Buf[i] = 31; // 写入逻辑0的占空比
    for (i = PIXEL_NUM * 24; i < NUM; i++)
        send_Buf[i] = 0; // 占空比比为0，全为低电平
    WS_Load();
}

void RGB_UI::RGB_UI_Init(void)
{
    // while (strcmp("put the Update function in 800kHz interrupt", string_check_rgb_ui) != 0)
    //     ;
    WS_CloseAll();
    // HAL_Delay(10);
}

/**
 * @brief  将所有灯珠设置为同一颜色（RGB格式）。
 * @param  n_R: 红色分量 (0~255)
 * @param  n_G: 绿色分量 (0~255)
 * @param  n_B: 蓝色分量 (0~255)
 *
 * 实现步骤：
 * 1. 将RGB分量转换为24位数据序列（顺序为 G->R->B 高位在前）
 * 2. 根据每一位是1或0，用WS1或WS0填充对应PWM比较值
 * 3. 将生成的dat[24]复制到每个灯珠对应的缓冲区位置
 * 4. 填充复位部分为0
 * 5. 调用WS_Load()启动传输
 */
void RGB_UI::WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B)
{
    uint16_t i, j;
    uint8_t dat[24];
    // 将RGB数据进行转换
    for (i = 0; i < 8; i++)
    {
        dat[i] = ((n_G & 0x80) ? WS1 : WS0);
        n_G <<= 1;
    }
    for (i = 0; i < 8; i++)
    {
        dat[i + 8] = ((n_R & 0x80) ? WS1 : WS0);
        n_R <<= 1;
    }
    for (i = 0; i < 8; i++)
    {
        dat[i + 16] = ((n_B & 0x80) ? WS1 : WS0);
        n_B <<= 1;
    }

    for (i = 0; i < PIXEL_NUM; i++)
    {
        for (j = 0; j < 24; j++)
        {

            send_Buf[i * 24 + j] = dat[j];
        }
    }
    for (i = PIXEL_NUM * 24; i < NUM; i++)
    {
        send_Buf[i] = 0; // 占空比比为0，全为低电平
    }

    // WS_Load();
}

/**
 * @brief  将RGB分量转换为WS281x的GRB格式颜色值。
 * @param  red:   红色分量 (0~255)
 * @param  green: 绿色分量 (0~255)
 * @param  blue:  蓝色分量 (0~255)
 * @return 32位颜色值，高24位有效，格式为 [G][R][B]
 */
uint32_t RGB_UI::WS281x_Color(uint8_t red, uint8_t green, uint8_t blue)
{
    return green << 16 | red << 8 | blue;
}

/**
 * @brief  设置单个灯珠的颜色（GRB颜色值形式）。
 * @param  n:        灯珠索引 (0 ~ PIXEL_NUM-1)
 * @param  GRBColor: 32位颜色值，高24位有效，格式为 GRB
 *
 * 注意：该函数仅修改缓冲区，不会立即启动DMA传输。
 */
void RGB_UI::WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor)
{
    uint8_t i;
    if (n < PIXEL_NUM)
    {
        for (i = 0; i < 24; ++i)
            send_Buf[24 * n + i] = (((GRBColor << i) & 0X800000) ? WS1 : WS0);
    }
}

/**
 * @brief  设置单个灯珠的颜色（RGB分量形式），并立即启动DMA传输。
 * @param  n:      灯珠索引 (0 ~ PIXEL_NUM-1)
 * @param  red:    红色分量 (0~255)
 * @param  green:  绿色分量 (0~255)
 * @param  blue:   蓝色分量 (0~255)
 *
 * 该函数先修改指定灯珠的缓冲区，然后调用WS_Load()启动传输。
 */
void RGB_UI::WS281x_SetPixelRGB(uint16_t n, uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t i;
    if (n < PIXEL_NUM)
    {
        for (i = 0; i < 24; ++i)
            send_Buf[24 * n + i] = (((WS281x_Color(red, green, blue) << i) & 0X800000) ? WS1 : WS0);
    }
    // WS_Load();
}

#endif /*__TIM_H__*/

#pragma endregion

// netword={
//     ssid="NSQD 13mini"
//     psk="06625271101"
//     priority=8
// }
// netword={
//     ssid="AAA 13mini"
//     psk="66666678"
//     priority=7
// }
// netword={
//     ssid="BBB 13mini"
//     psk="66666678"
//     priority=6
// }
