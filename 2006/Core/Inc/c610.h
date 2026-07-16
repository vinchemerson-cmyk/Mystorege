/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    c610.h
 * @brief   C610 ESC driver for M2006 motor control via CAN bus
 ******************************************************************************
 * @attention
 *
 * C610 ESC Protocol (DJI RoboMaster):
 * - Control frame 1: CAN Std ID = 0x200, motors 0~3
 * - Control frame 2: CAN Std ID = 0x1FF, motors 4~7
 * - DLC = 8, 4 x int16 (big-endian, mA), range -10000 ~ +10000
 * - Feedback frame: CAN Std ID = 0x201 ~ 0x208 (one per motor)
 *
 ******************************************************************************
 */
/* USER CODE END Header */
#ifndef __C610_H__
#define __C610_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define C610_CURRENT_MAX       10000    /* max current: +10A */
#define C610_CURRENT_MIN      (-10000)  /* min current: -10A */
#define C610_CAN_CONTROL_ID1   0x200    /* control command CAN ID: motors 0~3 */
#define C610_CAN_CONTROL_ID2   0x1FF    /* control command CAN ID: motors 4~7 */
#define C610_CAN_FEEDBACK_BASE 0x201    /* feedback CAN ID base (0x201~0x208) */
#define C610_MOTOR_MAX         7        /* max motor index (0~7) */

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t angle;      /* rotor mechanical angle (0~8191, 0~360deg) */
    int16_t  speed;      /* rotor speed (RPM) */
    int16_t  current;    /* actual torque current (mA) */
    uint8_t  temp;       /* motor temperature (deg C) */
} C610_Feedback_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize CAN filter and start CAN for C610 communication
 * @param  hcan: pointer to CAN handle
 * @retval HAL status
 */
HAL_StatusTypeDef C610_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  Send current to a single motor (others set to 0)
 * @note   motor_id 0~3 -> CAN ID 0x200;  motor_id 4~7 -> CAN ID 0x1FF
 * @param  hcan: pointer to CAN handle
 * @param  motor_id: motor index (0~7)
 * @param  current: target current (mA), range -10000 ~ +10000
 * @retval HAL status
 */
HAL_StatusTypeDef C610_SendSingleMotor(CAN_HandleTypeDef *hcan,
                                       uint8_t motor_id, int16_t current);

/**
 * @brief  Parse C610 feedback frame received via CAN
 * @param  rxData: 8-byte CAN data payload from ID 0x201~0x208
 * @param  feedback: pointer to feedback struct to fill
 */
void C610_ParseFeedback(uint8_t rxData[8], C610_Feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif /* __C610_H__ */
