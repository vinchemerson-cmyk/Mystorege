/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/**
 * ===========================================================================
 * @file    usart.c
 * @brief   USART3 (DBUS 遥控器) + USART6 (调试串口) 硬件初始化与 MSP 配置
 * ===========================================================================
 *
 * 【串口拓扑 (UART Topology)】
 *   USART3 — DBUS 遥控器接收机 (dr16):
 *     - 波特率 100 kbps, 9-bit Word + Even Parity = 8E1
 *     - 仅接收 (RX-only)，数据由 DMA1_Stream1 (Channel 4) 搬运
 *     - 引脚: PC11=RX, PC10=TX (复用 AF7) — 实际只使用 RX
 *     - DMA 模式: Normal, Peripheral→Memory, 字节对齐
 *     - 中断: USART3_IRQn 优先级 5, DMA1_Stream1_IRQn 优先级 0
 *
 *   USART6 — 调试串口输出:
 *     - 波特率 460800, 8-N-1
 *     - 收发 (TX/RX)，TX 使用 DMA2_Stream6 (Channel 5)
 *     - 引脚: PG9=RX, PG14=TX (复用 AF8)
 *     - 中断: USART6_IRQn 优先级 5
 *
 * 【波特率时钟来源】
 *   USART3 ← APB1 = 42 MHz
 *   USART6 ← APB2 = 84 MHz
 *   (详见 main.c SystemClock_Config 的时钟树注释)
 *
 * 【MSP 说明】
 *   HAL_UART_MspInit 为每个 UART 配置: 时钟 → GPIO 复用 → DMA → 中断。
 *   __HAL_LINKDMA 将 DMA 句柄链接到 UART 句柄，HAL 库据此调用 DMA API。
 * ===========================================================================
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ─── 全局 UART/DMA 句柄 (Global UART/DMA Handles) ─── */
UART_HandleTypeDef huart3;           /* USART3 句柄 — DBUS 遥控器 (RX only) */
UART_HandleTypeDef huart6;           /* USART6 句柄 — 调试串口 (TX/RX) */
DMA_HandleTypeDef hdma_usart3_rx;    /* USART3 RX DMA 句柄 — DMA1_Stream1_CH4 */
DMA_HandleTypeDef hdma_usart6_tx;    /* USART6 TX DMA 句柄 — DMA2_Stream6_CH5 */

/*
 * USART3 初始化 — DBUS 遥控器接收 (DBUS Remote Receiver).
 *
 * 配置为 DBUS 协议要求:
 *   100000 baud, 9-bit word (8 data + parity bit), Even parity, 1 stop
 *   即等效于 8E1 (8 data bits, Even parity, 1 stop bit)。
 * 仅接收模式 (UART_MODE_RX)，数据由 DMA1_Stream1 自动搬运。
 */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;            /* 100 kbps — DBUS 协议波特率 */
  huart3.Init.WordLength = UART_WORDLENGTH_9B; /* 9-bit = 8 data + parity bit */
  huart3.Init.StopBits = UART_STOPBITS_1;    /* 1 stop bit */
  huart3.Init.Parity = UART_PARITY_EVEN;     /* Even parity (DBUS 要求) */
  huart3.Init.Mode = UART_MODE_RX;           /* 仅接收 — RX only */
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE; /* 无硬件流控 */
  huart3.Init.OverSampling = UART_OVERSAMPLING_16; /* 16 倍过采样 */
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/*
 * USART6 初始化 — 调试串口 (Debug UART Output).
 *
 * 460800 baud, 8-N-1, 收发模式。
 * TX 使用 DMA2_Stream6 以减轻 CPU 负担；RX 使用中断。
 */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */
  /* USART6 调试输出配置为460800-8-N-1。 */
  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 460800;            /* 460800 baud — 高速调试波特率 */
  huart6.Init.WordLength = UART_WORDLENGTH_8B; /* 8 data bits */
  huart6.Init.StopBits = UART_STOPBITS_1;    /* 1 stop bit */
  huart6.Init.Parity = UART_PARITY_NONE;     /* 无校验 */
  huart6.Init.Mode = UART_MODE_TX_RX;        /* 收发模式 — TX + RX */
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE; /* 无硬件流控 */
  huart6.Init.OverSampling = UART_OVERSAMPLING_16; /* 16 倍过采样 */
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/*
 * UART MSP 初始化 (MCU Support Package — 由 HAL_UART_Init 自动调用)。
 * 为每个 UART 配置: 时钟 → GPIO 复用 → DMA → NVIC 中断。
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 时钟使能 */
    __HAL_RCC_USART3_CLK_ENABLE();

    /* GPIO 端口时钟 (GPIOC) */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX   (DBUS 数据接收)
    PC10     ------> USART3_TX   (未使用，保留配置)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      /* 复用推挽输出 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 — 100kbps 需要足够驱动能力 */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3; /* USART3 复用功能 */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ── USART3 RX DMA 配置 (DMA1_Stream1, Channel 4) ── */
    /*
     * DMA 将 USART3 RX 寄存器收到的字节自动搬运到内存缓冲区，
     * 无需 CPU 干预，配合 Receive-to-IDLE 实现 DBUS 帧的异步接收。
     * 配置: 外设→内存, 字节对齐, Normal 模式 (每次接收完需重启)。
     */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;   /* 外设→内存 */
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;       /* 外设地址不递增 (固定 USART3_DR) */
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;           /* 内存地址递增 (填充缓冲区) */
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; /* 8-bit 对齐 */
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 8-bit 对齐 */
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;                  /* Normal 模式 — 接收完需重启 */
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;    /* 不使用 FIFO */
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将 DMA 句柄链接到 UART 句柄 (HAL 通过 hdmarx 调用 DMA API) */
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* ── USART3 中断配置 ── */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);    /* 优先级 5 (子优先级 0) */
    HAL_NVIC_EnableIRQ(USART3_IRQn);            /* 使能 USART3 全局中断 */
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 时钟使能 */
    __HAL_RCC_USART6_CLK_ENABLE();

    /* GPIO 端口时钟 (GPIOG) */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX   (调试输出)
    PG9     ------> USART6_RX    (调试输入)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* ── USART6 TX DMA 配置 (DMA2_Stream6, Channel 5) ── */
    /*
     * 调试输出使用 DMA 减轻 CPU 负担:
     * 内存→外设方向, 字节对齐, Normal 模式。
     */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;   /* 内存→外设 */
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;       /* 外设地址不递增 */
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;           /* 内存地址递增 */
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;                  /* Normal 模式 */
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    /* 将 DMA 句柄链接到 UART 句柄 (HAL 通过 hdmatx 调用 DMA API) */
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* ── USART6 中断配置 ── */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);    /* 优先级 5 (子优先级 0) */
    HAL_NVIC_EnableIRQ(USART6_IRQn);            /* 使能 USART6 全局中断 */
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

