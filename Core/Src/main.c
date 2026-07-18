/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 1. 定义标准的 PID 控制器结构体，方便面向对象进行串级管理
typedef struct {
  float Kp;             // 比例增益
  float Ki;             // 积分增益
  float Kd;             // 微分增益

  float error;          // 当前偏差
  float last_error;     // 上一次偏差
  float integral;       // 积分累加值

  float max_integral;   // 积分限幅
  float max_output;     // 输出限幅

  float out;            // PID 最终输出值
} PID_Controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile float motor_angle = 0.0f;       // 减速后输出轴的累计绝对物理角度 (度)
volatile float motor_speed = 0.0f;       // 减速后输出轴的实际转速 (rpm)
volatile float motor_current = 0.0f;     // 实际反馈转矩电流
volatile float motor_error = 0.0f;       // 电机错误码反馈

// 多圈角度解算辅助变量
static int32_t rotor_round = 0;          // 转子累计转过的整圈数
static uint16_t last_angle = 0;          // 上一次接收到的转子单圈角度

volatile float expected_position = 0.0f; // 期望目标位置 (度)

// 2. 空载状态下的 PID 参数静态初始化
// 位置外环：纯 P 控制，输入偏差(deg)，输出目标速度(rpm)。Kp=2.2 时 90°偏差 → 198 rpm
static PID_Controller pos_pid = {
  .Kp = 2.7f,
  .Ki = 0.0f,
  .Kd = 0.0f,
  .error = 0.0f,
  .last_error = 0.0f,
  .integral = 0.0f,
  .max_integral = 50.0f,// 预留积分限幅，防止误开 Ki 时积分饱和
  .max_output = 1000.0f,// 输出轴最大速度限幅 1000 rpm（M2006 空载约 200 rpm）
  .out = 0.0f
};

// 速度内环：PI 控制，输入速度偏差(rpm)，输出控制电流 [-10000, 10000]
static PID_Controller spd_pid = {
  .Kp = 150.0f,
  .Ki = 0.5f,
  .Kd = 0.0f,
  .error = 0.0f,
  .last_error = 0.0f,
  .integral = 0.0f,
  .max_integral = 1000.0f,// 积分限幅防止积分饱和
  .max_output = 6000.0f,// 电流最高限幅 5000
  .out = 0.0f
};

volatile uint8_t new_data_flag = 0;

#define TWO_PI 6.283185307f
#define AMPLITUDE 360.0f                  // 位置目标幅值：让输出轴在 度 到 度 之间摆动
#define BASE_PERIOD_MS  2000
#define CH_COUNT  4                      // 发送给上位机的通道数
#define WAVE_NORM       (1.0f / 2.65f)

#pragma pack(push, 1)
typedef struct {
  float fdata[CH_COUNT];
  uint8_t tail[4];
} JustFloatFrame;
#pragma pack(pop)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void C610_SendCurrent(uint8_t id, int16_t current);
float PID_Calculate(PID_Controller *pid, float error);
float get_chaotic_periodic(uint32_t tick_ms, float max_offset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig;
  // 1. 配置过滤器参数
  sFilterConfig.FilterBank = 0;                        // 使用过滤器组 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;    // 掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;   // 32位位宽

  // ID 和 掩码 全部设为 0，代表"不过滤任何报文"，接收总线上的所有数据
  sFilterConfig.FilterIdHigh = 0x0000;                 // 验证码高位
  sFilterConfig.FilterIdLow = 0x0000;                  // 验证码低位
  sFilterConfig.FilterMaskIdHigh = 0x0000;             // 掩码高位
  sFilterConfig.FilterMaskIdLow = 0x0000;              // 掩码低位

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;   // 过滤后将数据放入 FIFO1
  sFilterConfig.FilterActivation = ENABLE;             // 激活该过滤器

  // 2. 将配置写入硬件寄存器
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler(); // 配置失败则进入死循环
  }

  // 3. 启动 CAN 外设
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. 开启 CAN 接收 FIFO1 挂起中断
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_tick = 0;
  JustFloatFrame txFrame = {
    .tail = {0x00, 0x00, 0x80, 0x7f}
  };
  while (1)
  {
    // 1ms 软件定时控制周期
    if (HAL_GetTick() - last_tick >= 1)
    {
      last_tick = HAL_GetTick();

      // 获取当前 1ms 拍子下正弦波计算出的期望物理位置
      expected_position = 720;    //get_chaotic_periodic(HAL_GetTick(), AMPLITUDE);

      // ================= 串级控制核心计算 =================

      // 步骤 1：位置外环计算
      // 输入：目标角度 - 实际绝对输出轴角度 (motor_angle)
      // 输出：计算出的目标速度 (target_speed)
      float pos_error = expected_position - motor_angle;
      float target_speed = PID_Calculate(&pos_pid, pos_error);

      // 步骤 2：速度内环计算
      // 输入：目标速度 - 实际输出轴转数 (motor_speed)
      // 输出：期望控制电流指令
      float spd_error = target_speed - motor_speed;
      float current_cmd_f = PID_Calculate(&spd_pid, spd_error);

      // 转换为 int16_t 指令
      int16_t current_cmd = (int16_t)current_cmd_f;

      // 发送电流控制信号给 ID 为 4 的电调[cite: 1]
      C610_SendCurrent(4, current_cmd);

      // 增加发送前对 UART 状态的校验，防止 1ms 极短时间内把 DMA 挤爆
      if (huart1.gState == HAL_UART_STATE_READY)
      {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&txFrame, sizeof(JustFloatFrame));
      }
    }

    // 处理接收到的反馈（仅在收到新数据时执行）
    if (new_data_flag)
    {
      new_data_flag = 0;                      // 清除标志
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED 闪烁指示接收

      // 填充发送给上位机的数据，以便上位机直观地观测实际位置和目标位置的贴合情况
      txFrame.fdata[0] = motor_angle;         // 通道 0：输出轴绝对位置 (度)
      txFrame.fdata[1] = motor_speed;         // 通道 1：输出轴实际转速 (rpm)
      txFrame.fdata[2] = motor_current;       // 通道 2：实际转矩电流 (A)
      txFrame.fdata[3] = expected_position;   // 通道 3：期望目标位置 (度)
    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 发送控制指令给某个 ID 的电调，控制电流值范围 -10000 ~ 10000 对应输出 -10A ~ 10A[cite: 1]
void C610_SendCurrent(uint8_t id, int16_t current)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox;

  tx_header.StdId = 0x200;   // 控制 1~4 号电调使用 0x200 标识符[cite: 1]
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  // 根据 ID 在数据域中的位置填充[cite: 1]
  int offset = ((id - 1) % 4) * 2;   // 针对 ID 4，offset 为 6，占用 DATA[6] 和 DATA[7][cite: 1]
  tx_data[offset] = (current >> 8) & 0xFF;
  tx_data[offset + 1] = current & 0xFF;

  HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

// 统一的通用 PID 计算函数
float PID_Calculate(PID_Controller *pid, float error) {
  pid->error = error;

  // 比例项
  float P_term = pid->Kp * error;

  // 积分项（1ms = 0.001s）
  pid->integral += error * 0.001f;

  // 积分项抗饱和限幅
  if (pid->integral > pid->max_integral)  pid->integral = pid->max_integral;
  if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

  float I_term = pid->Ki * pid->integral;

  // 微分项（采样周期 1ms）
  float derivative = (error - pid->last_error) / 0.001f;
  float D_term = pid->Kd * derivative;
  pid->last_error = error;

  // 综合计算
  pid->out = P_term + I_term + D_term;

  // 输出限幅
  if (pid->out > pid->max_output)  pid->out = pid->max_output;
  if (pid->out < -pid->max_output) pid->out = -pid->max_output;

  return pid->out;
}

// 获取期望的目标角度，以正弦波形进行平滑变化
// float get_chaotic_periodic(uint32_t tick_ms, float max_offset)
// {
//   // 1. 计算当前周期内的相位（0~1 之间）
//   float phase = (float)(tick_ms % BASE_PERIOD_MS) / BASE_PERIOD_MS;
//   float rad = phase * TWO_PI;
//
//   // 2. 叠加多个不同频率、不同幅值、不同相位偏移的正弦波
//   //    刻意使用非对称的相位偏移 (1.2, 2.7, 0.8, 1.5)，让波形看起来毫无规律
//   float wave = 1.0f * sinf(rad)
//              + 0.8f * sinf(2.0f * rad + 1.2f)
//              + 0.5f * sinf(3.0f * rad + 2.7f)
//              + 0.5f * sinf(5.0f * rad + 0.8f)
//              + 0.3f * sinf(7.0f * rad + 1.5f)
//              + 0.2f * sinf(11.0f * rad + 0.3f)   // 新增11次谐波
//              + 0.15f * sinf(13.0f * rad + 2.1f); // 新增13次谐波
//   // 总系数 = 1.0+0.8+0.5+0.5+0.3+0.2+0.15 = 3.45
// #define WAVE_NORM  (1.0f / 3.45f)
//   // 3. 归一化并乘以用户设定的最大偏移量
//   return max_offset * WAVE_NORM * wave;
// }
// 临时替换 get_chaotic_periodic 为阶跃函数
float get_chaotic_periodic(uint32_t tick_ms, float max_offset)
{
  // 每 2 秒切换一次目标值
  if ((tick_ms / 2000) % 2 == 0)
    return max_offset * 0.5f;   // 例如 180°
  else
    return -max_offset * 0.5f;  // -180°
}
// CAN1 接收中断回调函数
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    uint16_t std_id = rx_header.StdId;
    if (std_id >= 0x201 && std_id <= 0x208) // 接收电调反馈[cite: 1]
    {
      // 解析数据
      uint16_t angle = (rx_data[0] << 8) | rx_data[1];      // 单圈转子机械角度 0~8191[cite: 1]
      int16_t speed = (rx_data[2] << 8) | rx_data[3];       // 转子转速 rpm[cite: 1]
      int16_t actual_current = (rx_data[4] << 8) | rx_data[5]; // 实际转矩电流[cite: 1]
      uint8_t error_code = rx_data[7];                      // 电机错误码反馈[cite: 1]

      // 多圈过零检测解算：首次运行初始化，防止启动时跳变
      static uint8_t is_first_run = 1;
      if (is_first_run)
      {
        last_angle = angle;
        is_first_run = 0;
      }

      int16_t diff = angle - last_angle;
      if (diff < -4096)
      {
        rotor_round++; // 正向越过零点，累计圈数 +1
      }
      else if (diff > 4096)
      {
        rotor_round--; // 反向越过零点，累计圈数 -1
      }
      last_angle = angle;

      // 1. 转子单圈物理角度
      float rotor_single_angle = (float)angle / 8191.0f * 360.0f;
      // 2. 转子绝对多圈累计角度
      float total_rotor_angle = (float)rotor_round * 360.0f + rotor_single_angle;

      // 3. 计算 M2006 减速后输出轴的绝对角度（减速比 36:1）
      float output_shaft_angle = total_rotor_angle / 36.0f;

      // 物理量反馈写入全局变量
      motor_angle = output_shaft_angle;                     // 输出轴绝对物理角度 (度)
      motor_speed = (float)speed / 36.0f;                   // 输出轴实际速度 (rpm)
      motor_current = (float)actual_current / 1000.0f;      // 实际电流 (A)
      motor_error = (float)error_code;

      new_data_flag = 1; // 标记接收到最新反馈，通知主循环
    }
  }
}
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
#ifdef USE_FULL_ASSERT
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