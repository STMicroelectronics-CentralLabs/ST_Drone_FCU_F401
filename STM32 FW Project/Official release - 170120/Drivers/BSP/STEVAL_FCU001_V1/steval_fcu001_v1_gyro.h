/**
 ******************************************************************************
 * @file    steval_fcu001_v1_gyro.h
 * @author  MEMS Application Team, Competence Center Japan
 * @version V3.0.0 (MEMS library version)
 * @date    12-August-2016
 * @brief   This file contains definitions for the steval_fcu001_v1_gyro.c
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
#ifndef __STEVAL_FCU001_V1_GYRO_H
#define __STEVAL_FCU001_V1_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LSM6DSL_ACC_GYRO_driver_HL.h"
#include "steval_fcu001_v1.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1 STEVAL_FCU001_V1
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO Gyro
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Public_Types Public types
  * @{
  */

typedef enum
{
  GYRO_SENSORS_AUTO = -1,        /* Always first element and equal to -1 */
  LSM6DSL_G_0                    /* Default on board. */
} GYRO_ID_t;

/**
 * @}
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Public_Defines Public defines
  * @{
  */

#define GYRO_SENSORS_MAX_NUM 1

/**
 * @}
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_GYRO_Init( GYRO_ID_t id, void **handle );
DrvStatusTypeDef BSP_GYRO_DeInit( void **handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_GYRO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_GYRO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_GYRO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_GYRO_Get_Axes( void *handle, SensorAxes_t *angular_velocity );
DrvStatusTypeDef BSP_GYRO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_GYRO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_GYRO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_GYRO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_GYRO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled );
DrvStatusTypeDef BSP_GYRO_Set_Axes_Status( void *handle, uint8_t *enable_xyz );
DrvStatusTypeDef BSP_GYRO_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_GYRO_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_GYRO_Get_DRDY_Status( void *handle, uint8_t *status );

DrvStatusTypeDef BSP_GYRO_FIFO_Set_ODR_Value_Ext( void *handle, float odr );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Empty_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Overrun_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Pattern_Ext( void *handle, uint16_t *pattern );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Data_Ext( void *handle, uint8_t *aData );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Num_Of_Samples_Ext( void *handle, uint16_t *nSamples );
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Decimation_Ext( void *handle, uint8_t decimation );
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Axis_Ext( void *handle, int32_t *angular_velocity );
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Mode_Ext( void *handle, uint8_t mode );
DrvStatusTypeDef BSP_GYRO_FIFO_Set_INT1_FIFO_Full_Ext( void *handle, uint8_t status );
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Watermark_Level_Ext( void *handle, uint16_t watermark );
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Stop_On_Fth_Ext( void *handle, uint8_t status );

DrvStatusTypeDef BSP_GYRO_Set_Interrupt_Latch_Ext( void *handle, uint8_t status );
DrvStatusTypeDef BSP_GYRO_Set_SelfTest_Ext( void *handle, uint8_t status );

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

#endif /* __STEVAL_FCU001_V1_GYRO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
