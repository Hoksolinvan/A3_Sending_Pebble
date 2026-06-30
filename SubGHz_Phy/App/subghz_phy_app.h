/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the SubGHz_Phy Middleware
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN FREERTOS_EC */
#define CFG_VCOM_PROCESS_NAME                      "VCOM_PROCESS"
#define CFG_VCOM_PROCESS_ATTR_BITS                 (0)
#define CFG_VCOM_PROCESS_CB_MEM                    (0)
#define CFG_VCOM_PROCESS_CB_SIZE                   (0)
#define CFG_VCOM_PROCESS_STACK_MEM                 (0)
#define CFG_VCOM_PROCESS_PRIORITY                  osPriorityNone
#define CFG_VCOM_PROCESS_STACK_SIZE                1024

/* USER CODE END FREERTOS_EC */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* Radio frequency, Hz */
#define RF_FREQUENCY                    915000000
#define CALL_SIGN                       "VA7ZGS"
/* TX output power, dBm. STM32WL HP PA range is roughly -9 to +22 */
#define TX_OUTPUT_POWER                 20


#define LORA_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#define LORA_SPREADING_FACTOR                       7         /* [SF7..SF12] */
#define LORA_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

/* Buffer and timing */
#define RX_TIMEOUT_VALUE                3000   /* ms */
#define TX_TIMEOUT_VALUE                3000   /* ms */
#define MAX_APP_BUFFER_SIZE             255
#define PAYLOAD_LEN                     64
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Subghz Application
  */
void SubghzApp_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__SUBGHZ_PHY_APP_H__*/
