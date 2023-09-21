/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32wbaxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"
#include "app_debug.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* BPKA_TASK related resources */
extern osThreadId_t BPKA_ThreadHandle;
extern const osThreadAttr_t BPKA_Thread_attributes;
extern osSemaphoreId_t  BPKA_Thread_SemHandle;
extern const osSemaphoreAttr_t BPKA_Thread_Sem_attributes;

/* HW_RNG_TASK related resources  */
extern osThreadId_t HW_RNG_ThreadHandle;
extern const osThreadAttr_t HW_RNG_Thread_attributes;
extern osSemaphoreId_t  HW_RNG_Thread_SemHandle;
extern const osSemaphoreAttr_t HW_RNG_Thread_Sem_attributes;

/* BLE_STACK_TASK related resources */
extern osThreadId_t BLE_HOST_ThreadHandle;
extern const osThreadAttr_t BLE_HOST_Thread_attributes;
extern osSemaphoreId_t BLE_HOST_Thread_SemHandle;
extern const osSemaphoreAttr_t BLE_HOST_Thread_Sem_attributes;

/* HCI_ASYNCH_EVT_TASK related resources */
extern osThreadId_t HCI_ASYNCH_EVT_ThreadHandle;
extern const osThreadAttr_t HCI_ASYNCH_EVT_Thread_attributes;
extern osSemaphoreId_t HCI_ASYNCH_EVT_Thread_SemHandle;
extern const osSemaphoreAttr_t HCI_ASYNCH_EVT_Thread_Sem_attributes;

/* LINK_LAYER_TASK related resources */
extern osThreadId_t LINK_LAYER_ThreadHandle;
extern const osThreadAttr_t LINK_LAYER_Thread_attributes;
extern osSemaphoreId_t LINK_LAYER_Thread_SemHandle;
extern const osSemaphoreAttr_t LINK_LAYER_Thread_Sem_attributes;
extern osMutexId_t LINK_LAYER_Thread_MutexHandle;
extern const osMutexAttr_t LINK_LAYER_Thread_Mutex_attributes;

/* AMM_BCKGND_TASK related resources */
extern osThreadId_t AMM_BCKGND_ThreadHandle;
extern const osThreadAttr_t AMM_BCKGND_Thread_attributes;
extern osSemaphoreId_t AMM_BCKGND_Thread_SemHandle;
extern const osSemaphoreAttr_t AMM_BCKGND_Thread_Sem_attributes;

/* FLASH_MANAGER_BCKGND_TASK related resources */
extern osThreadId_t FLASH_MANAGER_BCKGND_ThreadHandle;
extern const osThreadAttr_t FLASH_MANAGER_BCKGND_Thread_attributes;
extern osSemaphoreId_t FLASH_MANAGER_BCKGND_Thread_SemHandle;
extern const osSemaphoreAttr_t FLASH_MANAGER_BCKGND_Thread_Sem_attributes;

/* LINK_LAYER_TEMP_MEAS_TASK related resources */
extern osThreadId_t LINK_LAYER_TEMP_MEAS_ThreadHandle;
extern const osThreadAttr_t LINK_LAYER_TEMP_MEAS_Thread_attributes;
extern osSemaphoreId_t LINK_LAYER_TEMP_MEAS_Thread_SemHandle;
extern const osSemaphoreAttr_t LINK_LAYER_TEMP_MEAS_Thread_Sem_attributes;

/* ADV_LP_REQ_TASK related resources */
extern osThreadId_t ADV_LP_REQ_ThreadHandle;
extern const osThreadAttr_t ADV_LP_REQ_Thread_attributes;
extern osSemaphoreId_t ADV_LP_REQ_Thread_SemHandle;
extern const osSemaphoreAttr_t ADV_LP_REQ_Thread_Sem_attributes;

/* MEAS_REQ_TASK related resources */
extern osThreadId_t MEAS_REQ_ThreadHandle;
extern const osThreadAttr_t MEAS_REQ_Thread_attributes;
extern osSemaphoreId_t MEAS_REQ_Thread_SemHandle;
extern const osSemaphoreAttr_t MEAS_REQ_Thread_Sem_attributes;

/* gap_cmd_resp sync mechanism */
extern osSemaphoreId_t IDLEEVT_PROC_GAP_COMPLETE_SemHandle;
extern const osSemaphoreAttr_t IDLEEVT_PROC_GAP_COMPLETE_Sem_attributes;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_GPDMA1_Init(void);
void MX_RAMCFG_Init(void);
void MX_RTC_Init(void);
void MX_USART1_UART_Init(void);
void MX_ADC4_Init(void);
void MX_RNG_Init(void);
void MX_CRC_Init(void);
void MX_ICACHE_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_11
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_8
#define LD3_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B3_Pin GPIO_PIN_7
#define B3_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_6
#define B2_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_4
#define LD1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Timeout for advertising */
#define INITIAL_ADV_TIMEOUT         (60*1000) /**< 60s */
#define MX_APPE_Process();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
