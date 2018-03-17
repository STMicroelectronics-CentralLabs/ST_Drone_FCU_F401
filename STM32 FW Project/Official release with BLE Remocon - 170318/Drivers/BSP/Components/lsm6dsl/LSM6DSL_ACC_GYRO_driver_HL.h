/**
 ******************************************************************************
 * @file    LSM6DSL_ACC_GYRO_driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the LSM6DSL_ACC_GYRO_driver_HL.c firmware driver
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
#ifndef __LSM6DSL_ACC_GYRO_DRIVER_HL_H
#define __LSM6DSL_ACC_GYRO_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/

#include "accelerometer.h"
#include "gyroscope.h"

/* Include accelero sensor component drivers. */
#include "LSM6DSL_ACC_GYRO_driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM6DSL LSM6DSL
 * @{
 */

/** @addtogroup LSM6DSL_Public_Constants Public constants
 * @{
 */

#define LSM6DSL_SENSORS_MAX_NUM  1     /**< LSM6DSL max number of instances */

/** @addtogroup LSM6DSL_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale
 * @{
 */

#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DSL_GYRO_SENSITIVITY Gyro sensitivity values based on selected full scale
 * @{
 */

#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DSL_PEDOMETER_THRESHOLD Pedometer threshold values
 * @{
 */

#define LSM6DSL_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DSL_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DSL_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_WAKE_UP_THRESHOLD Wake up threshold values
 * @{
 */

#define LSM6DSL_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DSL_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_THRESHOLD Tap threshold values
 * @{
 */

#define LSM6DSL_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DSL_TAP_THRESHOLD_MID       0x10
#define LSM6DSL_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DSL_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_SHOCK_TIME Tap shock time window values
 * @{
 */

#define LSM6DSL_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_QUIET_TIME Tap quiet time window values
 * @{
 */

#define LSM6DSL_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_DURATION_TIME Tap duration time window values
 * @{
 */

#define LSM6DSL_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DSL_TAP_DURATION_TIME_MID       0x08
#define LSM6DSL_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DSL_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LSM6DSL_Public_Types LSM6DSL Public Types
 * @{
 */

/**
 * @brief LSM6DSL accelero extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *Enable_Free_Fall_Detection      ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Free_Fall_Detection     ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Free_Fall_Detection_Status  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Free_Fall_Threshold         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Enable_Pedometer                ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Pedometer               ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Pedometer_Status            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_Step_Count                  ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *Enable_Step_Counter_Reset       ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Step_Counter_Reset      ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Set_Pedometer_Threshold         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Enable_Tilt_Detection           ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Tilt_Detection          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Tilt_Detection_Status       ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Enable_Wake_Up_Detection        ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Wake_Up_Detection       ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Wake_Up_Detection_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Wake_Up_Threshold           ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Enable_Single_Tap_Detection     ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Single_Tap_Detection    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Single_Tap_Detection_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Enable_Double_Tap_Detection     ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_Double_Tap_Detection    ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_Double_Tap_Detection_Status ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Set_Tap_Threshold               ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_Tap_Shock_Time              ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_Tap_Quiet_Time              ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_Tap_Duration_Time           ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Enable_6D_Orientation           ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Disable_6D_Orientation          ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_6D_Orientation_Status       ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_XL           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_XH           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_YL           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_YH           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_ZL           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *Get_6D_Orientation_ZH           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_ODR_Value              ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status           ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Overrun_Status         ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Pattern                ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data                   ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples         ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *FIFO_X_Set_Decimation           ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_X_Get_Axis                 ) ( DrvContextTypeDef*, int32_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode                   ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_INT1_FIFO_Full         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level        ) ( DrvContextTypeDef*, uint16_t );
  DrvStatusTypeDef ( *FIFO_Set_Stop_On_Fth            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_Interrupt_Latch             ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_SelfTest                    ) ( DrvContextTypeDef*, uint8_t );
} LSM6DSL_X_ExtDrv_t;

/**
 * @brief LSM6DSL gyro extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *FIFO_Set_ODR_Value       ) ( DrvContextTypeDef*, float );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Overrun_Status  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Pattern         ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples  ) ( DrvContextTypeDef*, uint16_t* );
  DrvStatusTypeDef ( *FIFO_G_Set_Decimation    ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_G_Get_Axis          ) ( DrvContextTypeDef*, int32_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_INT1_FIFO_Full  ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level ) ( DrvContextTypeDef*, uint16_t );
  DrvStatusTypeDef ( *FIFO_Set_Stop_On_Fth     ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_Interrupt_Latch      ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *Set_SelfTest             ) ( DrvContextTypeDef*, uint8_t );
} LSM6DSL_G_ExtDrv_t;

/**
 * @brief LSM6DSL combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isAccInitialized;
  uint8_t isGyroInitialized;
} LSM6DSL_Combo_Data_t;

/**
 * @brief LSM6DSL accelero specific data internal structure definition
 */
typedef struct
{
  LSM6DSL_Combo_Data_t
  *comboData;       /* Combo data to manage in software SPI 3-Wire initialization of the combo sensors */
  float Previous_ODR;
} LSM6DSL_X_Data_t;

/**
 * @brief LSM6DSL gyro specific data internal structure definition
 */
typedef struct
{
  LSM6DSL_Combo_Data_t
  *comboData;       /* Combo data to manage in software SPI 3-Wire initialization of the combo sensors */
  float Previous_ODR;
} LSM6DSL_G_Data_t;

/**
 * @}
 */

/** @addtogroup LSM6DSL_Public_Variables Public variables
 * @{
 */

extern ACCELERO_Drv_t LSM6DSL_X_Drv;
extern GYRO_Drv_t LSM6DSL_G_Drv;
extern LSM6DSL_X_ExtDrv_t LSM6DSL_X_ExtDrv;
extern LSM6DSL_G_ExtDrv_t LSM6DSL_G_ExtDrv;
extern LSM6DSL_Combo_Data_t LSM6DSL_Combo_Data[LSM6DSL_SENSORS_MAX_NUM];

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

#endif /* __LSM6DSL_ACC_GYRO_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
