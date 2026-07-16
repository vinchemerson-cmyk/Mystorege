/* USER CODE BEGIN Header */
/**
 ******************************************************************************

 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "c610.h"

HAL_StatusTypeDef C610_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filterConfig = {0};
    HAL_StatusTypeDef status;

    /* --- 配置 CAN 滤波器: 接收所有标准 ID 的报文 ---
     * 滤波器组 0, 掩码模式, 32位
     * Mask=0x00000000 → 所有位都不检查 → 任何 ID 都能通过
     * 报文存入 FIFO0
     */
    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;

    status = HAL_CAN_ConfigFilter(hcan, &filterConfig);
    if (status != HAL_OK)
        return status;

    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
        return status;

    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  发送一帧 CAN 电流控制命令 (底层函数, 内部使用)
 * @note   将 4 路 int16 电流值按大端序打包为 8 字节 CAN 报文
 *         使用 HAL_CAN_AddTxMessage() 将报文放入 3 个发送邮箱之一
 *         只要邮箱有空位就不会阻塞
 * @param  hcan:   CAN 外设句柄指针
 * @param  can_id: CAN 标准帧 ID (0x200 或 0x1FF)
 * @param  c1~c4:  4 路电机目标电流 (mA), 范围 -10000~+10000
 * @retval HAL_OK=成功放入邮箱, HAL_ERROR=3个邮箱全满(发送太频繁)
 */
static HAL_StatusTypeDef C610_SendFrame(CAN_HandleTypeDef *hcan, uint32_t can_id,
                                        int16_t c1, int16_t c2,
                                        int16_t c3, int16_t c4)
{
    CAN_TxHeaderTypeDef txHeader = {0};  /* CAN 发送帧头 */
    uint8_t data[8];                      /* 8 字节数据负载 */
    uint32_t txMailbox;                   /* HAL 分配的发送邮箱编号 (0/1/2) */

    /* --- 构造 CAN 帧头 --- */
    txHeader.StdId = can_id;              /* 标准帧 ID */
    txHeader.ExtId = 0;                   /* 不使用扩展 ID */
    txHeader.IDE   = CAN_ID_STD;          /* 标准帧格式 */
    txHeader.RTR   = CAN_RTR_DATA;        /* 数据帧 (非远程帧) */
    txHeader.DLC   = 8;                   /* 数据长度 = 8 字节 */
    txHeader.TransmitGlobalTime = DISABLE;/* 不发送时间戳 */

    /* --- 打包 4 路电流值, int16 大端序 (高字节在前) ---
     * 例: 2000mA = 0x07D0 → data[0]=0x07, data[1]=0xD0
     */
    data[0] = (uint8_t)((c1 >> 8) & 0xFF);  /* 电机 0 电流高字节 */
    data[1] = (uint8_t)(c1 & 0xFF);          /* 电机 0 电流低字节 */
    data[2] = (uint8_t)((c2 >> 8) & 0xFF);  /* 电机 1 电流高字节 */
    data[3] = (uint8_t)(c2 & 0xFF);          /* 电机 1 电流低字节 */
    data[4] = (uint8_t)((c3 >> 8) & 0xFF);  /* 电机 2 电流高字节 */
    data[5] = (uint8_t)(c3 & 0xFF);          /* 电机 2 电流低字节 */
    data[6] = (uint8_t)((c4 >> 8) & 0xFF);  /* 电机 3 电流高字节 */
    data[7] = (uint8_t)(c4 & 0xFF);          /* 电机 3 电流低字节 */

    /* --- 放入 CAN 发送邮箱, 硬件自动仲裁后发出 --- */
    return HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox);
}

/**
 * @brief  向单路电机发送目标电流 (其余通道置零)
 * @note   自动根据电机 ID 选择正确的 CAN ID:
 *           电机 0~3 → CAN ID 0x200, 电流写入对应字节位置
 *           电机 4~7 → CAN ID 0x1FF, 电流写入 motor_id-4 位置
 * @param  hcan:     CAN 外设句柄指针
 * @param  motor_id: 电机编号 (0~7)
 * @param  current:  目标电流 (mA), 正=正转, 负=反转, 范围 ±10000
 * @retval HAL_OK=成功, HAL_ERROR=电机编号越界
 */
HAL_StatusTypeDef C610_SendSingleMotor(CAN_HandleTypeDef *hcan,
                                       uint8_t motor_id, int16_t current)
{
    if (motor_id > C610_MOTOR_MAX)       /* 检查电机编号合法性 */
        return HAL_ERROR;

    int16_t currents[4] = {0, 0, 0, 0};  /* 4 路电流初始化为 0 */
    uint32_t can_id;

    if (motor_id < 4)
    {
        /* 电机 0~3 → 使用 CAN ID 0x200 */
        can_id = C610_CAN_CONTROL_ID1;
        currents[motor_id] = current;    /* 电流填入对应位置 [0:1]~[6:7] */
    }
    else
    {
        /* 电机 4~7 → 使用 CAN ID 0x1FF */
        can_id = C610_CAN_CONTROL_ID2;
        currents[motor_id - 4] = current;/* 电流填入对应位置, 偏移 -4 */
    }

    return C610_SendFrame(hcan, can_id,
                          currents[0], currents[1],
                          currents[2], currents[3]);
}

/**
 * @brief  解析 C610 电调反馈报文
 * @note   从 8 字节 CAN 数据中提取角度、转速、实际电流、温度
 *         帧格式 (大端序):
 *           [0:1] = 转子机械角度 (0~8191, 对应 0°~360°)
 *           [2:3] = 转子转速 (RPM, 有符号)
 *           [4:5] = 实际转矩电流 (mA, 有符号)
 *           [6]   = 电机温度 (°C)
 *           [7]   = 保留
 * @param  rxData:   收到的 8 字节 CAN 数据
 * @param  feedback: 解析结果存入此结构体
 */
void C610_ParseFeedback(uint8_t rxData[8], C610_Feedback_t *feedback)
{
    if (feedback == NULL || rxData == NULL)  /* 空指针保护 */
        return;

    feedback->angle   = ((uint16_t)rxData[0] << 8) | rxData[1];  /* 角度 (大端→小端) */
    feedback->speed   = ((int16_t)rxData[2] << 8)  | rxData[3];  /* 转速 */
    feedback->current = ((int16_t)rxData[4] << 8)  | rxData[5];  /* 实际电流 */
    feedback->temp    = rxData[6];                                 /* 温度 */
}
