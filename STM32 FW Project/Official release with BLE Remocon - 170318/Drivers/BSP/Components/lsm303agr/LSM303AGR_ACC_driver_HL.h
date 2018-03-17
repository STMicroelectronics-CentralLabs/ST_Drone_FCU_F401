/**
 ******************************************************************************
 * @file    LSM303AGR_ACC_driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the LSM303AGR_ACC_driver_HL.c firmware driver
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
#ifndef __LSM303AGR_ACC_DRIVER_HL_H
#define __LSM303AGR_ACC_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"

/* Include accelero sensor component drivers. */
#include "LSM303AGR_ACC_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM303AGR_ACC LSM303AGR_ACC
 * @{
 */

/** @addtogroup LSM303AGR_ACC_Public_Constants Public constants
 * @{
 */

/** @addtogroup LSM303AGR_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale
 * @{
 */

#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_NORMAL_MODE               3.900  /**< Sensitivity value for 2 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_HIGH_RESOLUTION_MODE      0.980  /**< Sensitivity value for 2 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_LOW_POWER_MODE           15.630  /**< Sensitivity value for 2 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_NORMAL_MODE               7.820  /**< Sensitivity value for 4 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_HIGH_RESOLUTION_MODE      1.950  /**< Sensitivity value for 4 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_LOW_POWER_MODE           31.260  /**< Sensitivity value for 4 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_NORMAL_MODE              15.630  /**< Sensitivity value for 8 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_HIGH_RESOLUTION_MODE      3.900  /**< Sensitivity value for 8 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_LOW_POWER_MODE           62.520  /**< Sensitivity value for 8 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_NORMAL_MODE             46.900  /**< Sensitivity value for 16 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_HIGH_RESOLUTION_MODE    11.720  /**< Sensitivity value for 16 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_LOW_POWER_MODE         187.580  /**< Sensitivity value for 16 g full scale and low power mode [mg/LSB] */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LSM303AGR_ACC_Public_Types LSM303AGR_ACC Public Types
 * @{
 */

/**
 * @brief LSM303AGR_ACC accelero specific data internal structure definition
 */

typedef struct
{
  float Previous_ODR;
} LSM303AGR_X_Data_t;

/**
 * @}
 */

/** @addtogroup LSM303AGR_ACC_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t LSM303AGR_X_Drv;

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

#endif /* __LSM303AGR_ACC_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
