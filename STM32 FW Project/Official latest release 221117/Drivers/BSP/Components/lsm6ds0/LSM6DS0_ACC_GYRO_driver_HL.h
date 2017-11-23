/**
 ******************************************************************************
 * @file    LSM6DS0_ACC_GYRO_driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the LSM6DS0_ACC_GYRO_driver_HL.c firmware driver
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
#ifndef __LSM6DS0_ACC_GYRO_DRIVER_HL_H
#define __LSM6DS0_ACC_GYRO_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"
#include "gyroscope.h"

/* Include accelero sensor component drivers. */
#include "LSM6DS0_ACC_GYRO_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM6DS0 LSM6DS0
 * @{
 */

/** @addtogroup LSM6DS0_Public_Constants Public constants
 * @{
 */

#define LSM6DS0_SENSORS_MAX_NUM  1     /**< LSM6DS0 max number of instances */

/** @addtogroup LSM6DS0_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale
 * @{
 */

#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G  0.732  /**< Sensitivity value for 16 g full scale [mg/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DS0_GYRO_SENSITIVITY Gyro sensitivity values based on selected full scale
 * @{
 */

#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS   08.75  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS   17.50  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.00  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LSM6DS0_Public_Types LSM6DS0 Public Types
 * @{
 */

/**
 * @brief LSM6DS0 combo specific data internal structure definition
 */

typedef struct
{
  uint8_t isGyroEnabled;
  float   lastGyroODR;
} LSM6DS0_Combo_Data_t;

/**
 * @brief LSM6DS0 accelero specific data internal structure definition
 */

typedef struct
{
  float Previous_ODR;
  LSM6DS0_Combo_Data_t *comboData;       /* Combo data to manage in software ODR of the combo sensors */
} LSM6DS0_X_Data_t;

/**
 * @brief LSM6DS0 gyro specific data internal structure definition
 */

typedef struct
{
  float Previous_ODR;
  LSM6DS0_Combo_Data_t *comboData;       /* Combo data to manage in software ODR of the combo sensors */
} LSM6DS0_G_Data_t;

/**
 * @}
 */

/** @addtogroup LSM6DS0_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t LSM6DS0_X_Drv;
extern GYRO_Drv_t LSM6DS0_G_Drv;
extern LSM6DS0_Combo_Data_t LSM6DS0_Combo_Data[LSM6DS0_SENSORS_MAX_NUM];

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

#endif /* __LSM6DS0_ACC_GYRO_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
