/**
 ******************************************************************************
 * @file    HTS221_Driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the HTS221_Driver_HL.c firmware driver
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTS221_DRIVER_HL_H
#define __HTS221_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "humidity.h"
#include "temperature.h"

/* Include sensor component drivers. */
#include "HTS221_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup HTS221 HTS221
 * @{
 */

/** @addtogroup HTS221_Public_Constants Public constants
 * @{
 */

#define HTS221_SENSORS_MAX_NUM  1     /**< HTS221 max number of instances */

/** @addtogroup HTS221_I2C_Addresses HTS221 I2C Addresses
 * @{
 */

#define HTS221_ADDRESS_DEFAULT  0xBE  /**< HTS221 I2C Address */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup HTS221_Public_Types HTS221 Public Types
 * @{
 */

/**
 * @brief HTS221 combo specific data internal structure definition
 */

typedef struct
{
  uint8_t isHumInitialized;
  uint8_t isTempInitialized;
  uint8_t isHumEnabled;
  uint8_t isTempEnabled;
} HTS221_Combo_Data_t;

/**
 * @brief HTS221 humidity specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_H_Data_t;


/**
 * @brief HTS221 temperature specific data internal structure definition
 */

typedef struct
{
  HTS221_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} HTS221_T_Data_t;

/**
 * @}
 */

/** @addtogroup HTS221_Public_Variables Public variables
 * @{
 */

extern HUMIDITY_Drv_t HTS221_H_Drv;
extern TEMPERATURE_Drv_t HTS221_T_Drv;
extern HTS221_Combo_Data_t HTS221_Combo_Data[HTS221_SENSORS_MAX_NUM];

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HTS221_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
