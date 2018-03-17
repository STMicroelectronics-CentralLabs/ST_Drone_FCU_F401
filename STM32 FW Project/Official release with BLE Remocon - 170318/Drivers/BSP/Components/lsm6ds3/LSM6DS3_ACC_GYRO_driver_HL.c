/**
 ******************************************************************************
 * @file    LSM6DS3_ACC_GYRO_driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LSM6DS3 sensor
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

/* Includes ------------------------------------------------------------------*/

#include "LSM6DS3_ACC_GYRO_driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM6DS3 LSM6DS3
 * @{
 */

/** @addtogroup LSM6DS3_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM6DS3_X_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS3_X_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef LSM6DS3_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM6DS3_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM6DS3_X_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS3_X_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM6DS3_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef LSM6DS3_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM6DS3_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef LSM6DS3_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef LSM6DS3_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS3_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM6DS3_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LSM6DS3_G_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_G_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_G_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_G_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_G_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS3_G_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity );
static DrvStatusTypeDef LSM6DS3_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM6DS3_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM6DS3_G_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS3_G_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM6DS3_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale );
static DrvStatusTypeDef LSM6DS3_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM6DS3_G_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef LSM6DS3_G_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef LSM6DS3_G_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS3_G_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM6DS3_G_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM6DS3_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS3_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS3_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM6DS3_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status );

static DrvStatusTypeDef LSM6DS3_X_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

static DrvStatusTypeDef LSM6DS3_G_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

/**
 * @}
 */

/** @addtogroup LSM6DS3_Callable_Private_Function_Ext_Prototypes Callable private function for extended features prototypes
 * @{
 */

static DrvStatusTypeDef LSM6DS3_X_Enable_Free_Fall_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Free_Fall_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Free_Fall_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Set_Free_Fall_Threshold( DrvContextTypeDef *handle, uint8_t thr );

static DrvStatusTypeDef LSM6DS3_X_Enable_Pedometer( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Pedometer( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Pedometer_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Get_Step_Count( DrvContextTypeDef *handle, uint16_t *step_count );
static DrvStatusTypeDef LSM6DS3_X_Enable_Step_Counter_Reset( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Step_Counter_Reset( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Set_Pedometer_Threshold( DrvContextTypeDef *handle, uint8_t thr );

static DrvStatusTypeDef LSM6DS3_X_Enable_Tilt_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Tilt_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Tilt_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LSM6DS3_X_Enable_Wake_Up_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Wake_Up_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Wake_Up_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Set_Wake_Up_Threshold( DrvContextTypeDef *handle, uint8_t thr );

static DrvStatusTypeDef LSM6DS3_X_Enable_Single_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Single_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Single_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Enable_Double_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_Double_Tap_Detection( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_Double_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Threshold( DrvContextTypeDef *handle, uint8_t thr );
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Shock_Time( DrvContextTypeDef *handle, uint8_t time );
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Quiet_Time( DrvContextTypeDef *handle, uint8_t time );
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Duration_Time( DrvContextTypeDef *handle, uint8_t time );

static DrvStatusTypeDef LSM6DS3_X_Enable_6D_Orientation( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Disable_6D_Orientation( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_XL( DrvContextTypeDef *handle, uint8_t *xl );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_XH( DrvContextTypeDef *handle, uint8_t *xh );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_YL( DrvContextTypeDef *handle, uint8_t *yl );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_YH( DrvContextTypeDef *handle, uint8_t *yh );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_ZL( DrvContextTypeDef *handle, uint8_t *zl );
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_ZH( DrvContextTypeDef *handle, uint8_t *zh );

static DrvStatusTypeDef LSM6DS3_FIFO_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Overrun_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Pattern( DrvContextTypeDef *handle, uint16_t *pattern );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Data( DrvContextTypeDef *handle, uint8_t *aData );
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint16_t *nSamples );
static DrvStatusTypeDef LSM6DS3_FIFO_X_Set_Decimation( DrvContextTypeDef *handle, uint8_t decimation );
static DrvStatusTypeDef LSM6DS3_FIFO_G_Set_Decimation( DrvContextTypeDef *handle, uint8_t decimation );
static DrvStatusTypeDef LSM6DS3_FIFO_X_Get_Axis( DrvContextTypeDef *handle, int32_t *acceleration );
static DrvStatusTypeDef LSM6DS3_FIFO_G_Get_Axis( DrvContextTypeDef *handle, int32_t *acceleration );
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode );
static DrvStatusTypeDef LSM6DS3_FIFO_Set_INT1_FIFO_Full( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint16_t watermark );
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Stop_On_Fth( DrvContextTypeDef *handle, uint8_t status );

static DrvStatusTypeDef LSM6DS3_X_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LSM6DS3_X_Set_SelfTest( DrvContextTypeDef *handle, uint8_t status );

static DrvStatusTypeDef LSM6DS3_G_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LSM6DS3_G_Set_SelfTest( DrvContextTypeDef *handle, uint8_t status );

/**
 * @}
 */

/** @addtogroup LSM6DS3_Public_Variables Public variables
 * @{
 */

/**
 * @brief LSM6DS3 accelero extended features driver internal structure
 */
LSM6DS3_X_ExtDrv_t LSM6DS3_X_ExtDrv =
{
  LSM6DS3_X_Enable_Free_Fall_Detection,
  LSM6DS3_X_Disable_Free_Fall_Detection,
  LSM6DS3_X_Get_Free_Fall_Detection_Status,
  LSM6DS3_X_Set_Free_Fall_Threshold,
  LSM6DS3_X_Enable_Pedometer,
  LSM6DS3_X_Disable_Pedometer,
  LSM6DS3_X_Get_Pedometer_Status,
  LSM6DS3_X_Get_Step_Count,
  LSM6DS3_X_Enable_Step_Counter_Reset,
  LSM6DS3_X_Disable_Step_Counter_Reset,
  LSM6DS3_X_Set_Pedometer_Threshold,
  LSM6DS3_X_Enable_Tilt_Detection,
  LSM6DS3_X_Disable_Tilt_Detection,
  LSM6DS3_X_Get_Tilt_Detection_Status,
  LSM6DS3_X_Enable_Wake_Up_Detection,
  LSM6DS3_X_Disable_Wake_Up_Detection,
  LSM6DS3_X_Get_Wake_Up_Detection_Status,
  LSM6DS3_X_Set_Wake_Up_Threshold,
  LSM6DS3_X_Enable_Single_Tap_Detection,
  LSM6DS3_X_Disable_Single_Tap_Detection,
  LSM6DS3_X_Get_Single_Tap_Detection_Status,
  LSM6DS3_X_Enable_Double_Tap_Detection,
  LSM6DS3_X_Disable_Double_Tap_Detection,
  LSM6DS3_X_Get_Double_Tap_Detection_Status,
  LSM6DS3_X_Set_Tap_Threshold,
  LSM6DS3_X_Set_Tap_Shock_Time,
  LSM6DS3_X_Set_Tap_Quiet_Time,
  LSM6DS3_X_Set_Tap_Duration_Time,
  LSM6DS3_X_Enable_6D_Orientation,
  LSM6DS3_X_Disable_6D_Orientation,
  LSM6DS3_X_Get_6D_Orientation_Status,
  LSM6DS3_X_Get_6D_Orientation_XL,
  LSM6DS3_X_Get_6D_Orientation_XH,
  LSM6DS3_X_Get_6D_Orientation_YL,
  LSM6DS3_X_Get_6D_Orientation_YH,
  LSM6DS3_X_Get_6D_Orientation_ZL,
  LSM6DS3_X_Get_6D_Orientation_ZH,
  LSM6DS3_FIFO_Set_ODR_Value,
  LSM6DS3_FIFO_Get_Full_Status,
  LSM6DS3_FIFO_Get_Empty_Status,
  LSM6DS3_FIFO_Get_Overrun_Status,
  LSM6DS3_FIFO_Get_Pattern,
  LSM6DS3_FIFO_Get_Data,
  LSM6DS3_FIFO_Get_Num_Of_Samples,
  LSM6DS3_FIFO_X_Set_Decimation,
  LSM6DS3_FIFO_X_Get_Axis,
  LSM6DS3_FIFO_Set_Mode,
  LSM6DS3_FIFO_Set_INT1_FIFO_Full,
  LSM6DS3_FIFO_Set_Watermark_Level,
  LSM6DS3_FIFO_Set_Stop_On_Fth,
  LSM6DS3_X_Set_Interrupt_Latch,
  LSM6DS3_X_Set_SelfTest
};

/**
 * @brief LSM6DS3 gyro extended features driver internal structure
 */
LSM6DS3_G_ExtDrv_t LSM6DS3_G_ExtDrv =
{
  LSM6DS3_FIFO_Set_ODR_Value,
  LSM6DS3_FIFO_Get_Full_Status,
  LSM6DS3_FIFO_Get_Empty_Status,
  LSM6DS3_FIFO_Get_Overrun_Status,
  LSM6DS3_FIFO_Get_Pattern,
  LSM6DS3_FIFO_Get_Data,
  LSM6DS3_FIFO_Get_Num_Of_Samples,
  LSM6DS3_FIFO_G_Set_Decimation,
  LSM6DS3_FIFO_G_Get_Axis,
  LSM6DS3_FIFO_Set_Mode,
  LSM6DS3_FIFO_Set_INT1_FIFO_Full,
  LSM6DS3_FIFO_Set_Watermark_Level,
  LSM6DS3_FIFO_Set_Stop_On_Fth,
  LSM6DS3_G_Set_Interrupt_Latch,
  LSM6DS3_G_Set_SelfTest
};

/**
 * @brief LSM6DS3 accelero driver structure
 */
ACCELERO_Drv_t LSM6DS3_X_Drv =
{
  LSM6DS3_X_Init,
  LSM6DS3_X_DeInit,
  LSM6DS3_X_Sensor_Enable,
  LSM6DS3_X_Sensor_Disable,
  LSM6DS3_X_Get_WhoAmI,
  LSM6DS3_X_Check_WhoAmI,
  LSM6DS3_X_Get_Axes,
  LSM6DS3_X_Get_AxesRaw,
  LSM6DS3_X_Get_Sensitivity,
  LSM6DS3_X_Get_ODR,
  LSM6DS3_X_Set_ODR,
  LSM6DS3_X_Set_ODR_Value,
  LSM6DS3_X_Get_FS,
  LSM6DS3_X_Set_FS,
  LSM6DS3_X_Set_FS_Value,
  LSM6DS3_X_Get_Axes_Status,
  LSM6DS3_X_Set_Axes_Status,
  LSM6DS3_X_Read_Reg,
  LSM6DS3_X_Write_Reg,
  LSM6DS3_X_Get_DRDY_Status
};

/**
 * @brief LSM6DS3 gyro driver structure
 */
GYRO_Drv_t LSM6DS3_G_Drv =
{
  LSM6DS3_G_Init,
  LSM6DS3_G_DeInit,
  LSM6DS3_G_Sensor_Enable,
  LSM6DS3_G_Sensor_Disable,
  LSM6DS3_G_Get_WhoAmI,
  LSM6DS3_G_Check_WhoAmI,
  LSM6DS3_G_Get_Axes,
  LSM6DS3_G_Get_AxesRaw,
  LSM6DS3_G_Get_Sensitivity,
  LSM6DS3_G_Get_ODR,
  LSM6DS3_G_Set_ODR,
  LSM6DS3_G_Set_ODR_Value,
  LSM6DS3_G_Get_FS,
  LSM6DS3_G_Set_FS,
  LSM6DS3_G_Set_FS_Value,
  LSM6DS3_G_Get_Axes_Status,
  LSM6DS3_G_Set_Axes_Status,
  LSM6DS3_G_Read_Reg,
  LSM6DS3_G_Write_Reg,
  LSM6DS3_G_Get_DRDY_Status
};

/**
 * @}
 */

/** @addtogroup LSM6DS3_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };

  if ( LSM6DS3_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( LSM6DS3_ACC_GYRO_W_IF_Addr_Incr( (void *)handle, LSM6DS3_ACC_GYRO_IF_INC_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM6DS3_ACC_GYRO_W_BDU( (void *)handle, LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FIFO mode selection */
  if ( LSM6DS3_ACC_GYRO_W_FIFO_MODE( (void *)handle, LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if ( LSM6DS3_X_Set_ODR_When_Disabled( handle, ODR_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS3_ACC_GYRO_W_ODR_XL( (void *)handle, LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LSM6DS3_X_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes. */
  if ( LSM6DS3_X_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_DeInit( DrvContextTypeDef *handle )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS3_X_Data_t *pComponentData = ( LSM6DS3_X_Data_t * )pData->pComponentData;

  if ( LSM6DS3_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable free fall detection */
  if( LSM6DS3_X_Disable_Free_Fall_Detection( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable 6D orientation */
  if( LSM6DS3_X_Disable_6D_Orientation( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable pedometer */
  if( LSM6DS3_X_Disable_Pedometer( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable single tap detection */
  if( LSM6DS3_X_Disable_Single_Tap_Detection( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable double tap detection */
  if( LSM6DS3_X_Disable_Double_Tap_Detection( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable tilt detection */
  if( LSM6DS3_X_Disable_Tilt_Detection( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Try to disable wake up detection */
  if( LSM6DS3_X_Disable_Wake_Up_Detection( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if( LSM6DS3_X_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Sensor_Enable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS3_X_Data_t *pComponentData = ( LSM6DS3_X_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if ( LSM6DS3_X_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Sensor_Disable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS3_X_Data_t *pComponentData = ( LSM6DS3_X_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LSM6DS3_X_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS3_ACC_GYRO_W_ODR_XL( (void *)handle, LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LSM6DS3_Get_WhoAmI(handle, who_am_i);
}

/**
 * @brief Check the WHO_AM_I ID of the LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LSM6DS3_Check_WhoAmI(handle);
}


/**
 * @brief Get the LSM6DS3 accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{

  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM6DS3 output register. */
  if ( LSM6DS3_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LSM6DS3 actual sensitivity. */
  if ( LSM6DS3_X_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  acceleration->AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
  acceleration->AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
  acceleration->AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM6DS3 accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];

  /* Read raw data from LSM6DS3 output register. */
  if ( LSM6DS3_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM6DS3_ACC_GYRO_FS_XL_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM6DS3_ACC_GYRO_R_FS_XL( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM6DS3_ACC_GYRO_FS_XL_2g:
      *sensitivity = ( float )LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_4g:
      *sensitivity = ( float )LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_8g:
      *sensitivity = ( float )LSM6DS3_ACC_SENSITIVITY_FOR_FS_8G;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_16g:
      *sensitivity = ( float )LSM6DS3_ACC_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LSM6DS3_ACC_GYRO_ODR_XL_t odr_low_level;

  if ( LSM6DS3_ACC_GYRO_R_ODR_XL( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN:
      *odr =     0.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_13Hz:
      *odr =    13.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_26Hz:
      *odr =    26.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_52Hz:
      *odr =    52.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_104Hz:
      *odr =   104.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_208Hz:
      *odr =   208.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_416Hz:
      *odr =   416.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_833Hz:
      *odr =   833.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_1660Hz:
      *odr =  1660.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_3330Hz:
      *odr =  3330.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_XL_6660Hz:
      *odr =  6660.0f;
      break;
    default:
      *odr =    -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS3_X_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS3_X_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS3_X_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS3_X_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  LSM6DS3_ACC_GYRO_FS_XL_t fs_low_level;

  if ( LSM6DS3_ACC_GYRO_R_FS_XL( (void *)handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM6DS3_ACC_GYRO_FS_XL_2g:
      *fullScale =  2.0f;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_4g:
      *fullScale =  4.0f;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_8g:
      *fullScale =  8.0f;
      break;
    case LSM6DS3_ACC_GYRO_FS_XL_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  LSM6DS3_ACC_GYRO_FS_XL_t new_fs;

  switch( fullScale )
  {
    case FS_LOW:
      new_fs = LSM6DS3_ACC_GYRO_FS_XL_2g;
      break;
    case FS_MID:
      new_fs = LSM6DS3_ACC_GYRO_FS_XL_4g;
      break;
    case FS_HIGH:
      new_fs = LSM6DS3_ACC_GYRO_FS_XL_8g;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  LSM6DS3_ACC_GYRO_FS_XL_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ? LSM6DS3_ACC_GYRO_FS_XL_2g
           : ( fullScale <= 4.0f ) ? LSM6DS3_ACC_GYRO_FS_XL_4g
           : ( fullScale <= 8.0f ) ? LSM6DS3_ACC_GYRO_FS_XL_8g
           :                         LSM6DS3_ACC_GYRO_FS_XL_16g;

  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  LSM6DS3_ACC_GYRO_XEN_XL_t xStatus;
  LSM6DS3_ACC_GYRO_YEN_XL_t yStatus;
  LSM6DS3_ACC_GYRO_ZEN_XL_t zStatus;

  if ( LSM6DS3_ACC_GYRO_R_XEN_XL( (void *)handle, &xStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS3_ACC_GYRO_R_YEN_XL( (void *)handle, &yStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS3_ACC_GYRO_R_ZEN_XL( (void *)handle, &zStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  xyz_enabled[0] = ( xStatus == LSM6DS3_ACC_GYRO_XEN_XL_ENABLED ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == LSM6DS3_ACC_GYRO_YEN_XL_ENABLED ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED ) ? 1 : 0;

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the LSM6DS3 accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( LSM6DS3_ACC_GYRO_W_XEN_XL( (void *)handle,
                                  ( enable_xyz[0] == 1 ) ? LSM6DS3_ACC_GYRO_XEN_XL_ENABLED : LSM6DS3_ACC_GYRO_XEN_XL_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_YEN_XL( (void *)handle,
                                  ( enable_xyz[1] == 1 ) ? LSM6DS3_ACC_GYRO_YEN_XL_ENABLED : LSM6DS3_ACC_GYRO_YEN_XL_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_ZEN_XL( (void *)handle,
                                  ( enable_xyz[2] == 1 ) ? LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED : LSM6DS3_ACC_GYRO_ZEN_XL_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS3_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS3_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get accelerometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_XLDA_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_XLDA( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_XLDA_NO_DATA_AVAIL:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Initialize the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };

  if ( LSM6DS3_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( LSM6DS3_ACC_GYRO_W_IF_Addr_Incr( (void *)handle, LSM6DS3_ACC_GYRO_IF_INC_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM6DS3_ACC_GYRO_W_BDU( (void *)handle, LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FIFO mode selection */
  if ( LSM6DS3_ACC_GYRO_W_FIFO_MODE( (void *)handle, LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if ( LSM6DS3_G_Set_ODR_When_Disabled( handle, ODR_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down */
  if ( LSM6DS3_ACC_GYRO_W_ODR_G( (void *)handle, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LSM6DS3_G_Set_FS( handle, FS_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes */
  if ( LSM6DS3_G_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_DeInit( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS3_G_Data_t *pComponentData = ( LSM6DS3_G_Data_t * )pData->pComponentData;

  if ( LSM6DS3_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if ( LSM6DS3_G_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Sensor_Enable( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS3_G_Data_t *pComponentData = ( LSM6DS3_G_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if ( LSM6DS3_G_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Sensor_Disable( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS3_G_Data_t *pComponentData = ( LSM6DS3_G_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LSM6DS3_G_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down */
  if ( LSM6DS3_ACC_GYRO_W_ODR_G( (void *)handle, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LSM6DS3_Get_WhoAmI(handle, who_am_i);
}


/**
 * @brief Check the WHO_AM_I ID of the LSM6DS3 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LSM6DS3_Check_WhoAmI(handle);
}


/**
 * @brief Get the LSM6DS3 gyroscope sensor axes
 * @param handle the device handle
 * @param angular_velocity pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity )
{

  int16_t dataRaw[3];
  float   sensitivity = 0;

  /* Read raw data from LSM6DS3 output register. */
  if ( LSM6DS3_G_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LSM6DS3 actual sensitivity. */
  if ( LSM6DS3_G_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  angular_velocity->AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
  angular_velocity->AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
  angular_velocity->AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 gyroscope sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];

  /* Read raw data from LSM6DS3 output register. */
  if ( LSM6DS3_G_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set the raw data. */
  value->AXIS_X = dataRaw[0];
  value->AXIS_Y = dataRaw[1];
  value->AXIS_Z = dataRaw[2];

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 gyroscope sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM6DS3_ACC_GYRO_FS_125_t fullScale125;
  LSM6DS3_ACC_GYRO_FS_G_t   fullScale;

  /* Read full scale 125 selection from sensor. */
  if ( LSM6DS3_ACC_GYRO_R_FS_125( (void *)handle, &fullScale125 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( fullScale125 == LSM6DS3_ACC_GYRO_FS_125_ENABLED )
  {
    *sensitivity = ( float )LSM6DS3_GYRO_SENSITIVITY_FOR_FS_125DPS;
  }

  else
  {

    /* Read actual full scale selection from sensor. */
    if ( LSM6DS3_ACC_GYRO_R_FS_G( (void *)handle, &fullScale ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }

    /* Store the sensitivity based on actual full scale. */
    switch( fullScale )
    {
      case LSM6DS3_ACC_GYRO_FS_G_245dps:
        *sensitivity = ( float )LSM6DS3_GYRO_SENSITIVITY_FOR_FS_245DPS;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_500dps:
        *sensitivity = ( float )LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_1000dps:
        *sensitivity = ( float )LSM6DS3_GYRO_SENSITIVITY_FOR_FS_1000DPS;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_2000dps:
        *sensitivity = ( float )LSM6DS3_GYRO_SENSITIVITY_FOR_FS_2000DPS;
        break;
      default:
        *sensitivity = -1.0f;
        return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LSM6DS3_ACC_GYRO_ODR_G_t odr_low_level;

  if ( LSM6DS3_ACC_GYRO_R_ODR_G( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN:
      *odr =    0.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_13Hz:
      *odr =   13.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_26Hz:
      *odr =   26.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_52Hz:
      *odr =   52.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_104Hz:
      *odr =  104.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_208Hz:
      *odr =  208.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_416Hz:
      *odr =  416.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_833Hz:
      *odr =  833.0f;
      break;
    case LSM6DS3_ACC_GYRO_ODR_G_1660Hz:
      *odr = 1660.0f;
      break;
    default:
      *odr =   -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS3_G_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS3_G_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS3_G_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS3_G_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  LSM6DS3_ACC_GYRO_FS_G_t fs_low_level;
  LSM6DS3_ACC_GYRO_FS_125_t fs_125;

  if ( LSM6DS3_ACC_GYRO_R_FS_125( (void *)handle, &fs_125 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS3_ACC_GYRO_R_FS_G( (void *)handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( fs_125 == LSM6DS3_ACC_GYRO_FS_125_ENABLED )
  {
    *fullScale = 125.0f;
  }

  else
  {
    switch( fs_low_level )
    {
      case LSM6DS3_ACC_GYRO_FS_G_245dps:
        *fullScale =  245.0f;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_500dps:
        *fullScale =  500.0f;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_1000dps:
        *fullScale = 1000.0f;
        break;
      case LSM6DS3_ACC_GYRO_FS_G_2000dps:
        *fullScale = 2000.0f;
        break;
      default:
        *fullScale =   -1.0f;
        return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  LSM6DS3_ACC_GYRO_FS_G_t new_fs;

  switch( fullScale )
  {
    case FS_LOW:
      new_fs = LSM6DS3_ACC_GYRO_FS_G_245dps;
      break;
    case FS_MID:
      new_fs = LSM6DS3_ACC_GYRO_FS_G_500dps;
      break;
    case FS_HIGH:
      new_fs = LSM6DS3_ACC_GYRO_FS_G_2000dps;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_FS_G( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS3 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  LSM6DS3_ACC_GYRO_FS_G_t new_fs;

  if ( fullScale <= 125.0f )
  {
    if ( LSM6DS3_ACC_GYRO_W_FS_125( (void *)handle, LSM6DS3_ACC_GYRO_FS_125_ENABLED ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  else
  {
    new_fs = ( fullScale <=  245.0f ) ? LSM6DS3_ACC_GYRO_FS_G_245dps
             : ( fullScale <=  500.0f ) ? LSM6DS3_ACC_GYRO_FS_G_500dps
             : ( fullScale <= 1000.0f ) ? LSM6DS3_ACC_GYRO_FS_G_1000dps
             :                            LSM6DS3_ACC_GYRO_FS_G_2000dps;

    if ( LSM6DS3_ACC_GYRO_W_FS_125( (void *)handle, LSM6DS3_ACC_GYRO_FS_125_DISABLED ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }
    if ( LSM6DS3_ACC_GYRO_W_FS_G( (void *)handle, new_fs ) == MEMS_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS3 gyroscope sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  LSM6DS3_ACC_GYRO_XEN_G_t xStatus;
  LSM6DS3_ACC_GYRO_YEN_G_t yStatus;
  LSM6DS3_ACC_GYRO_ZEN_G_t zStatus;

  if ( LSM6DS3_ACC_GYRO_R_XEN_G( (void *)handle, &xStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS3_ACC_GYRO_R_YEN_G( (void *)handle, &yStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS3_ACC_GYRO_R_ZEN_G( (void *)handle, &zStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  xyz_enabled[0] = ( xStatus == LSM6DS3_ACC_GYRO_XEN_G_ENABLED ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == LSM6DS3_ACC_GYRO_YEN_G_ENABLED ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == LSM6DS3_ACC_GYRO_ZEN_G_ENABLED ) ? 1 : 0;

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the LSM6DS3 gyroscope sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( LSM6DS3_ACC_GYRO_W_XEN_G( (void *)handle,
                                 ( enable_xyz[0] == 1 ) ? LSM6DS3_ACC_GYRO_XEN_G_ENABLED : LSM6DS3_ACC_GYRO_XEN_G_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_YEN_G( (void *)handle,
                                 ( enable_xyz[1] == 1 ) ? LSM6DS3_ACC_GYRO_YEN_G_ENABLED : LSM6DS3_ACC_GYRO_YEN_G_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_ZEN_G( (void *)handle,
                                 ( enable_xyz[2] == 1 ) ? LSM6DS3_ACC_GYRO_ZEN_G_ENABLED : LSM6DS3_ACC_GYRO_ZEN_G_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS3_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS3_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get gyroscope data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_GDA_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_GDA( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_GDA_NO_DATA_AVAIL:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the WHO_AM_I ID of the LSM6DS3 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LSM6DS3_ACC_GYRO_R_WHO_AM_I( (void *)handle, ( uint8_t* )who_am_i ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LSM6DS3 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LSM6DS3_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( who_am_i != handle->who_am_i )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS3_ACC_GYRO_ReadReg( (void *)handle, reg, data, 1 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS3_ACC_GYRO_WriteReg( (void *)handle, reg, &data, 1 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_LIR_t )status )
  {
    case LSM6DS3_ACC_GYRO_LIR_DISABLED:
    case LSM6DS3_ACC_GYRO_LIR_ENABLED:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_LIR( handle, ( LSM6DS3_ACC_GYRO_LIR_t )status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM6DS3 accelerometer sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS3_ACC_GYRO_OUTX_L_XL to LSM6DS3_ACC_GYRO_OUTZ_H_XL. */
  if ( LSM6DS3_ACC_GYRO_GetRawAccData( (void *)handle, ( uint8_t* )regValue ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  LSM6DS3_ACC_GYRO_ODR_XL_t new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
      break;
    case ODR_MID:
      new_odr = LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_ODR_XL( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS3_X_Data_t *pComponentData = ( LSM6DS3_X_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 13.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 13.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 26.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 52.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 104.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

  LSM6DS3_ACC_GYRO_ODR_XL_t new_odr;

  new_odr = ( odr <=   13.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_13Hz
            : ( odr <=   26.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_26Hz
            : ( odr <=   52.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_52Hz
            : ( odr <=  104.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_104Hz
            : ( odr <=  208.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_208Hz
            : ( odr <=  416.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_416Hz
            : ( odr <=  833.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_833Hz
            : ( odr <= 1660.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_1660Hz
            : ( odr <= 3330.0f ) ? LSM6DS3_ACC_GYRO_ODR_XL_3330Hz
            :                      LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;

  if ( LSM6DS3_ACC_GYRO_W_ODR_XL( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS3_X_Data_t *pComponentData = ( LSM6DS3_X_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=   13.0f ) ? 13.0f
                                 : ( odr <=   26.0f ) ? 26.0f
                                 : ( odr <=   52.0f ) ? 52.0f
                                 : ( odr <=  104.0f ) ? 104.0f
                                 : ( odr <=  208.0f ) ? 208.0f
                                 : ( odr <=  416.0f ) ? 416.0f
                                 : ( odr <=  833.0f ) ? 833.0f
                                 : ( odr <= 1660.0f ) ? 1660.0f
                                 : ( odr <= 3330.0f ) ? 3330.0f
                                 :                      6660.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM6DS3 gyroscope sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS3_ACC_GYRO_OUTX_L_G to LSM6DS3_ACC_GYRO_OUTZ_H_G. */
  if ( LSM6DS3_ACC_GYRO_GetRawGyroData( (void *)handle, ( uint8_t* )regValue ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  LSM6DS3_ACC_GYRO_ODR_G_t new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM6DS3_ACC_GYRO_ODR_G_13Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM6DS3_ACC_GYRO_ODR_G_13Hz;
      break;
    case ODR_MID:
      new_odr = LSM6DS3_ACC_GYRO_ODR_G_26Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM6DS3_ACC_GYRO_ODR_G_52Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM6DS3_ACC_GYRO_ODR_G_104Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_ODR_G( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS3_G_Data_t *pComponentData = ( LSM6DS3_G_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 13.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 13.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 26.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 52.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 104.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

  LSM6DS3_ACC_GYRO_ODR_G_t new_odr;

  new_odr = ( odr <=  13.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_13Hz
            : ( odr <=  26.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_26Hz
            : ( odr <=  52.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_52Hz
            : ( odr <= 104.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_104Hz
            : ( odr <= 208.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_208Hz
            : ( odr <= 416.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_416Hz
            : ( odr <= 833.0f ) ? LSM6DS3_ACC_GYRO_ODR_G_833Hz
            :                     LSM6DS3_ACC_GYRO_ODR_G_1660Hz;

  if ( LSM6DS3_ACC_GYRO_W_ODR_G( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS3 gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS3_G_Data_t *pComponentData = ( LSM6DS3_G_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=  13.0f ) ? 13.0f
                                 : ( odr <=  26.0f ) ? 26.0f
                                 : ( odr <=  52.0f ) ? 52.0f
                                 : ( odr <= 104.0f ) ? 104.0f
                                 : ( odr <= 208.0f ) ? 208.0f
                                 : ( odr <= 416.0f ) ? 416.0f
                                 : ( odr <= 833.0f ) ? 833.0f
                                 :                     1660.0f;

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup LSM6DS3_Callable_Private_Functions_Ext Callable private functions for extended features
 * @{
 */

/**
 * @brief Enable the free fall detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Free_Fall_Detection( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FF_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_FF_Duration( (void *)handle, 0x06 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* WAKE_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* TIMER_HR setting */
  if ( LSM6DS3_ACC_GYRO_W_TIMER_HR( (void *)handle, LSM6DS3_ACC_GYRO_TIMER_HR_6_4ms ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* SLEEP_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_SLEEP_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FF_THS setting */
  if ( LSM6DS3_X_Set_Free_Fall_Threshold( handle, LSM6DS3_ACC_GYRO_FF_THS_10 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* INT1_FF setting */
  if ( LSM6DS3_ACC_GYRO_W_FFEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_FF_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the free fall detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Free_Fall_Detection( DrvContextTypeDef *handle )
{

  /* INT1_FF setting */
  if ( LSM6DS3_ACC_GYRO_W_FFEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_FF_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FF_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_FF_Duration( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FF_THS setting */
  if ( LSM6DS3_ACC_GYRO_W_FF_THS( (void *)handle, LSM6DS3_ACC_GYRO_FF_THS_5 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the status of the free fall detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of free fall detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Free_Fall_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_FF_EV_STATUS_t free_fall_status;

  if ( LSM6DS3_ACC_GYRO_R_FF_EV_STATUS( (void *)handle, &free_fall_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( free_fall_status )
  {
    case LSM6DS3_ACC_GYRO_FF_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_FF_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the free fall detection threshold for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Free_Fall_Threshold( DrvContextTypeDef *handle, uint8_t thr )
{

  if ( LSM6DS3_ACC_GYRO_W_FF_THS( (void *)handle, (LSM6DS3_ACC_GYRO_FF_THS_t)thr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the pedometer feature for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 26Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Pedometer( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 26.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set pedometer threshold. */
  if ( LSM6DS3_X_Set_Pedometer_Threshold( handle, LSM6DS3_PEDOMETER_THRESHOLD_MID_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable embedded functionalities. */
  if ( LSM6DS3_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable pedometer algorithm. */
  if ( LSM6DS3_ACC_GYRO_W_PEDO_EN( (void *)handle, LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable pedometer on INT1. */
  if ( LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the pedometer feature for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Pedometer( DrvContextTypeDef *handle )
{

  /* Disable pedometer on INT1. */
  if ( LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_PEDO_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable pedometer algorithm. */
  if ( LSM6DS3_ACC_GYRO_W_PEDO_EN( (void *)handle, LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable embedded functionalities. */
  if ( LSM6DS3_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset pedometer threshold. */
  if ( LSM6DS3_X_Set_Pedometer_Threshold( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the pedometer status for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the pedometer status: 0 means no step detected, 1 means step detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Pedometer_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t pedometer_status;

  if ( LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS( (void *)handle, &pedometer_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( pedometer_status )
  {
    case LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the step counter for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param step_count the pointer to the step counter
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Step_Count( DrvContextTypeDef *handle, uint16_t *step_count )
{

  if ( LSM6DS3_ACC_GYRO_Get_GetStepCounter( (void *)handle, ( uint8_t* )step_count ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the reset of the step counter for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Step_Counter_Reset( DrvContextTypeDef *handle )
{

  if ( LSM6DS3_ACC_GYRO_W_PedoStepReset( (void *)handle, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the reset of the step counter for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Step_Counter_Reset( DrvContextTypeDef *handle )
{

  if ( LSM6DS3_ACC_GYRO_W_PedoStepReset( (void *)handle, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the pedometer threshold for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Pedometer_Threshold( DrvContextTypeDef *handle, uint8_t thr )
{

  if ( LSM6DS3_ACC_GYRO_W_PedoThreshold( (void *)handle, thr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the tilt detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 26Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Tilt_Detection( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 26.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable embedded functionalities */
  if ( LSM6DS3_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable tilt calculation. */
  if ( LSM6DS3_ACC_GYRO_W_TILT_EN( (void *)handle, LSM6DS3_ACC_GYRO_TILT_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable tilt event on INT1. */
  if ( LSM6DS3_ACC_GYRO_W_TiltEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_TILT_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the tilt detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Tilt_Detection( DrvContextTypeDef *handle )
{

  /* Disable tilt event on INT1. */
  if ( LSM6DS3_ACC_GYRO_W_TiltEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_TILT_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable tilt calculation. */
  if ( LSM6DS3_ACC_GYRO_W_TILT_EN( (void *)handle, LSM6DS3_ACC_GYRO_TILT_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable embedded functionalities */
  if ( LSM6DS3_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the tilt detection status for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the tilt detection status: 0 means no tilt detected, 1 means tilt detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Tilt_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t tilt_status;

  if ( LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS( (void *)handle, &tilt_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( tilt_status )
  {
    case LSM6DS3_ACC_GYRO_TILT_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the wake up detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Wake_Up_Detection( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* WAKE_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set wake up threshold. */
  if ( LSM6DS3_ACC_GYRO_W_WK_THS( (void *)handle, 0x02 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable wake up event on INT1. */
  if ( LSM6DS3_ACC_GYRO_W_WUEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_WU_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the wake up detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Wake_Up_Detection( DrvContextTypeDef *handle )
{

  /* INT1_WU setting */
  if ( LSM6DS3_ACC_GYRO_W_WUEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_WU_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* WU_DUR setting */
  if ( LSM6DS3_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* WU_THS setting */
  if ( LSM6DS3_ACC_GYRO_W_WK_THS( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the status of the wake up detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of the wake up detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Wake_Up_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_WU_EV_STATUS_t wake_up_status;

  if ( LSM6DS3_ACC_GYRO_R_WU_EV_STATUS( (void *)handle, &wake_up_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( wake_up_status )
  {
    case LSM6DS3_ACC_GYRO_WU_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the wake up threshold for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Wake_Up_Threshold( DrvContextTypeDef *handle, uint8_t thr )
{

  if ( LSM6DS3_ACC_GYRO_W_WK_THS( (void *)handle, thr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the single tap detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Single_Tap_Detection( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap threshold. */
  if ( LSM6DS3_X_Set_Tap_Threshold( handle, LSM6DS3_TAP_THRESHOLD_MID_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap shock time window. */
  if ( LSM6DS3_X_Set_Tap_Shock_Time( handle, LSM6DS3_TAP_SHOCK_TIME_MID_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap quiet time window. */
  if ( LSM6DS3_X_Set_Tap_Quiet_Time( handle, LSM6DS3_TAP_QUIET_TIME_MID_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable single tap interrupt on INT1 pin. */
  if ( LSM6DS3_ACC_GYRO_W_SingleTapOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the single tap detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Single_Tap_Detection( DrvContextTypeDef *handle )
{

  /* Disable single tap interrupt on INT1 pin. */
  if ( LSM6DS3_ACC_GYRO_W_SingleTapOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap threshold. */
  if ( LSM6DS3_X_Set_Tap_Threshold( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap shock time window. */
  if ( LSM6DS3_X_Set_Tap_Shock_Time( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap quiet time window. */
  if ( LSM6DS3_X_Set_Tap_Quiet_Time( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Disable Z direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the single tap detection status for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the single tap detection status: 0 means no single tap detected, 1 means single tap detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Single_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t tap_status;

  if ( LSM6DS3_ACC_GYRO_R_SINGLE_TAP_EV_STATUS( (void *)handle, &tap_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( tap_status )
  {
    case LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_DETECTED:
      *status = 1;
      break;

    case LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;

    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the double tap detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_Double_Tap_Detection( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap threshold. */
  if ( LSM6DS3_X_Set_Tap_Threshold( handle, LSM6DS3_TAP_THRESHOLD_MID_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap shock time window. */
  if ( LSM6DS3_X_Set_Tap_Shock_Time( handle, LSM6DS3_TAP_SHOCK_TIME_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap quiet time window. */
  if ( LSM6DS3_X_Set_Tap_Quiet_Time( handle, LSM6DS3_TAP_QUIET_TIME_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set tap duration time window. */
  if ( LSM6DS3_X_Set_Tap_Duration_Time( handle, LSM6DS3_TAP_DURATION_TIME_MID ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Single and double tap enabled. */
  if ( LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)handle,
       LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable double tap interrupt on INT1 pin. */
  if ( LSM6DS3_ACC_GYRO_W_TapEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_TAP_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the double tap detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_Double_Tap_Detection( DrvContextTypeDef *handle )
{

  /* Disable double tap interrupt on INT1 pin. */
  if ( LSM6DS3_ACC_GYRO_W_TapEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_TAP_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap threshold. */
  if ( LSM6DS3_X_Set_Tap_Threshold( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap shock time window. */
  if ( LSM6DS3_X_Set_Tap_Shock_Time( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap quiet time window. */
  if ( LSM6DS3_X_Set_Tap_Quiet_Time( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset tap duration time window. */
  if ( LSM6DS3_X_Set_Tap_Duration_Time( handle, 0x0 ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Only single tap enabled. */
  if ( LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)handle,
       LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if ( LSM6DS3_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DS3_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the double tap detection status for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the double tap detection status: 0 means no double tap detected, 1 means double tap detected
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_Double_Tap_Detection_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t tap_status;

  if ( LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS( (void *)handle, &tap_status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( tap_status )
  {
    case LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED:
      *status = 1;
      break;

    case LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;

    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the tap threshold for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param thr the threshold to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Threshold( DrvContextTypeDef *handle, uint8_t thr )
{

  if ( LSM6DS3_ACC_GYRO_W_TAP_THS( (void *)handle, thr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the tap shock time window for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param time the shock time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Shock_Time( DrvContextTypeDef *handle, uint8_t time )
{

  if ( LSM6DS3_ACC_GYRO_W_SHOCK_Duration( (void *)handle, time ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the tap quiet time window for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param time the quiet time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Quiet_Time( DrvContextTypeDef *handle, uint8_t time )
{

  if ( LSM6DS3_ACC_GYRO_W_QUIET_Duration( (void *)handle, time ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the tap duration of the time window for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param time the duration of the time window to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Set_Tap_Duration_Time( DrvContextTypeDef *handle, uint8_t time )
{

  if ( LSM6DS3_ACC_GYRO_W_DUR( (void *)handle, time ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Enable the 6D orientation detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @note  This function sets the LSM6DS3 accelerometer ODR to 416Hz and the LSM6DS3 accelerometer full scale to 2g
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Enable_6D_Orientation( DrvContextTypeDef *handle )
{

  /* Output Data Rate selection */
  if(LSM6DS3_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LSM6DS3_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DS3_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set 6D threshold. */
  if ( LSM6DS3_ACC_GYRO_W_SIXD_THS( (void *)handle, LSM6DS3_ACC_GYRO_SIXD_THS_60_degree ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* INT1_6D setting. */
  if ( LSM6DS3_ACC_GYRO_W_6DEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_6D_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Disable the 6D orientation detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Disable_6D_Orientation( DrvContextTypeDef *handle )
{

  /* INT1_6D setting. */
  if ( LSM6DS3_ACC_GYRO_W_6DEvOnInt1( (void *)handle, LSM6DS3_ACC_GYRO_INT1_6D_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset 6D threshold. */
  if ( LSM6DS3_ACC_GYRO_W_SIXD_THS( (void *)handle, LSM6DS3_ACC_GYRO_SIXD_THS_80_degree ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the status of the 6D orientation detection for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param status the pointer to the status of the 6D orientation detection: 0 means no detection, 1 means detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_D6D_EV_STATUS( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_D6D_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DS3_ACC_GYRO_D6D_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation XL axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param xl the pointer to the 6D orientation XL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_XL( DrvContextTypeDef *handle, uint8_t *xl )
{

  LSM6DS3_ACC_GYRO_DSD_XL_t xl_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_XL( (void *)handle, &xl_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( xl_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_XL_DETECTED:
      *xl = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_XL_NOT_DETECTED:
      *xl = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation XH axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param xh the pointer to the 6D orientation XH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_XH( DrvContextTypeDef *handle, uint8_t *xh )
{

  LSM6DS3_ACC_GYRO_DSD_XH_t xh_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_XH( (void *)handle, &xh_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( xh_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_XH_DETECTED:
      *xh = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_XH_NOT_DETECTED:
      *xh = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation YL axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param yl the pointer to the 6D orientation YL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_YL( DrvContextTypeDef *handle, uint8_t *yl )
{

  LSM6DS3_ACC_GYRO_DSD_YL_t yl_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_YL( (void *)handle, &yl_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( yl_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_YL_DETECTED:
      *yl = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_YL_NOT_DETECTED:
      *yl = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation YH axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param yh the pointer to the 6D orientation YH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_YH( DrvContextTypeDef *handle, uint8_t *yh )
{

  LSM6DS3_ACC_GYRO_DSD_YH_t yh_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_YH( (void *)handle, &yh_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( yh_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_YH_DETECTED:
      *yh = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_YH_NOT_DETECTED:
      *yh = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation ZL axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_ZL( DrvContextTypeDef *handle, uint8_t *zl )
{

  LSM6DS3_ACC_GYRO_DSD_ZL_t zl_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_ZL( (void *)handle, &zl_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( zl_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_ZL_DETECTED:
      *zl = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_ZL_NOT_DETECTED:
      *zl = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the 6D orientation ZH axis for LSM6DS3 accelerometer sensor
 * @param handle the device handle
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_X_Get_6D_Orientation_ZH( DrvContextTypeDef *handle, uint8_t *zh )
{

  LSM6DS3_ACC_GYRO_DSD_ZH_t zh_raw;

  if ( LSM6DS3_ACC_GYRO_R_DSD_ZH( (void *)handle, &zh_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( zh_raw )
  {
    case LSM6DS3_ACC_GYRO_DSD_ZH_DETECTED:
      *zh = 1;
      break;
    case LSM6DS3_ACC_GYRO_DSD_ZH_NOT_DETECTED:
      *zh = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO output data rate
 * @param handle the device handle
 * @param odr Output data rate
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LSM6DS3_ACC_GYRO_ODR_FIFO_t new_odr;

  new_odr = ( odr <=   10.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz
            : ( odr <=   25.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz
            : ( odr <=   50.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz
            : ( odr <=  100.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz
            : ( odr <=  200.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz
            : ( odr <=  400.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz
            : ( odr <=  800.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz
            : ( odr <= 1600.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz
            : ( odr <= 3300.0f ) ? LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz
            :                      LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;

  if ( LSM6DS3_ACC_GYRO_W_ODR_FIFO( handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get status of FIFO full flag
 * @param handle the device handle
 * @param *status Pointer where the status is stored. Values:
 *        0 ... no detection
 *        1 ... detection happened
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_FIFO_FULL_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_FIFOFull( handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_FIFO_FULL_FIFO_NOT_FULL:
      *status = 0;
      break;
    case LSM6DS3_ACC_GYRO_FIFO_FULL_FIFO_FULL:
      *status = 1;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get status of FIFO empty flag
 * @param handle the device handle
 * @param *status The pointer where the status of FIFO is stored. Values:
 *        0 ... FIFO is not empty
 *        1 ... FIFO is empty
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_FIFO_EMPTY_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_FIFOEmpty( handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_FIFO_EMPTY_FIFO_NOT_EMPTY:
      *status = 0;
      break;
    case LSM6DS3_ACC_GYRO_FIFO_EMPTY_FIFO_EMPTY:
      *status = 1;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get status of FIFO_OVR flag
 * @param handle the device handle
 * @param *status The pointer where the status of FIFO is stored. Values:
 *        0 ... no sample overrun
 *        1 ... at least 1 sample overrun
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Overrun_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS3_ACC_GYRO_OVERRUN_t status_raw;

  if ( LSM6DS3_ACC_GYRO_R_OVERRUN( handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS3_ACC_GYRO_OVERRUN_NO_OVERRUN:
      *status = 0;
      break;
    case LSM6DS3_ACC_GYRO_OVERRUN_OVERRUN:
      *status = 1;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get FIFO pattern
 * @param handle the device handle
 * @param *pattern Pointer where the pattern is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Pattern( DrvContextTypeDef *handle, uint16_t *pattern)
{

  if ( LSM6DS3_ACC_GYRO_R_FIFOPattern( handle, pattern ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get FIFO data
 * @param handle the device handle
 * @param *aData Pointer to the array where the data are stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Data( DrvContextTypeDef *handle, uint8_t *aData )
{

  if ( LSM6DS3_ACC_GYRO_Get_GetFIFOData( handle, aData ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get number of unread FIFO samples
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint16_t *nSamples )
{

  if ( LSM6DS3_ACC_GYRO_R_FIFONumOfEntries( handle, nSamples ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO decimation for accelerometer
 * @param handle the device handle
 * @param decimation FIFO decimation for accelerometer
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_X_Set_Decimation( DrvContextTypeDef *handle, uint8_t decimation )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t )decimation )
  {
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL( handle, ( LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t )decimation ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO decimation for gyroscope
 * @param handle the device handle
 * @param decimation FIFO decimation for gyroscope
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_G_Set_Decimation( DrvContextTypeDef *handle, uint8_t decimation )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_DEC_FIFO_G_t )decimation )
  {
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16:
    case LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_DEC_FIFO_G( handle, ( LSM6DS3_ACC_GYRO_DEC_FIFO_G_t )decimation ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Read single FIFO sample (16-bit data) from LSM6DS3 Accelerometer and calculate acceleration in mg
 * @param handle the device handle
 * @param acceleration the pointer to the acceleration value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_FIFO_X_Get_Axis( DrvContextTypeDef *handle, int32_t *acceleration )
{

  uint8_t aData[2];
  int16_t rawData = 0;
  float sensitivity = 0;

  /* Read single axis raw data from LSM6DS3 FIFO output registers. */
  if ( LSM6DS3_FIFO_Get_Data( handle, aData ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  rawData = ( aData[1] << 8 ) | aData[0];

  /* Get LSM6DS3 actual sensitivity. */
  if ( LSM6DS3_X_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the acceleration. */
  *acceleration = ( int32_t )( rawData * sensitivity );

  return COMPONENT_OK;
}



/**
 * @brief Read single FIFO sample (16-bit data) from LSM6DS3 Gyroscope and calculate angular velocity in mDPS
 * @param handle the device handle
 * @param angular_velocity the pointer to the angular velocity value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS3_FIFO_G_Get_Axis( DrvContextTypeDef *handle, int32_t *angular_velocity )
{

  uint8_t aData[2];
  int16_t rawData = 0;
  float sensitivity = 0;

  /* Read single axis raw data from LSM6DS3 FIFO output registers. */
  if ( LSM6DS3_FIFO_Get_Data( handle, aData ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  rawData = ( aData[1] << 8 ) | aData[0];

  /* Get LSM6DS3 actual sensitivity. */
  if ( LSM6DS3_G_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the angular velocity. */
  *angular_velocity = ( int32_t )( rawData * sensitivity );

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO mode
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_FIFO_MODE_t )mode )
  {
    case LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS:       /* Bypass mode. */
    case LSM6DS3_ACC_GYRO_FIFO_MODE_FIFO:         /* FIFO mode. */
    case LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2: /* Continuous mode. */
    case LSM6DS3_ACC_GYRO_FIFO_MODE_BTS:          /* Bypass to Continuous mode. */
    case LSM6DS3_ACC_GYRO_FIFO_MODE_STF:          /* Continuous to FIFO mode. */
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_FIFO_MODE( handle, ( LSM6DS3_ACC_GYRO_FIFO_MODE_t )mode ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO_FULL interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Set_INT1_FIFO_Full( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_INT1_FSS5_t )status )
  {
    case LSM6DS3_ACC_GYRO_INT1_FSS5_DISABLED:
    case LSM6DS3_ACC_GYRO_INT1_FSS5_ENABLED:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_FSS5_on_INT1( handle, ( LSM6DS3_ACC_GYRO_INT1_FSS5_t )status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO watermark level
 * @param handle the device handle
 * @param watermark FIFO watermark level
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint16_t watermark )
{

  if ( LSM6DS3_ACC_GYRO_W_FIFO_Watermark( handle, watermark ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set FIFO to stop on FTH interrupt
 * @param handle the device handle
 * @param status FIFO stop on FTH interrupt enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_FIFO_Set_Stop_On_Fth( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_STOP_ON_FTH_t )status )
  {
    case LSM6DS3_ACC_GYRO_STOP_ON_FTH_DISABLED:
    case LSM6DS3_ACC_GYRO_STOP_ON_FTH_ENABLED:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_STOP_ON_FTH( handle, ( LSM6DS3_ACC_GYRO_STOP_ON_FTH_t )status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set accelero interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_X_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status )
{

  return LSM6DS3_Set_Interrupt_Latch( handle, status );
}



/**
 * @brief Set accelero self-test
 * @param handle the device handle
 * @param status self-test enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_X_Set_SelfTest( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_ST_XL_t )status )
  {
    case LSM6DS3_ACC_GYRO_ST_XL_NORMAL_MODE:
    case LSM6DS3_ACC_GYRO_ST_XL_POS_SIGN_TEST:
    case LSM6DS3_ACC_GYRO_ST_XL_NEG_SIGN_TEST:
    case LSM6DS3_ACC_GYRO_ST_XL_NA:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_SelfTest_XL( handle, ( LSM6DS3_ACC_GYRO_ST_XL_t )status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set gyro interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_G_Set_Interrupt_Latch( DrvContextTypeDef *handle, uint8_t status )
{

  return LSM6DS3_Set_Interrupt_Latch( handle, status );
}



/**
 * @brief Set gyro self-test
 * @param handle the device handle
 * @param status self-test enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LSM6DS3_G_Set_SelfTest( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values. */
  switch ( ( LSM6DS3_ACC_GYRO_ST_G_t )status )
  {
    case LSM6DS3_ACC_GYRO_ST_G_NORMAL_MODE:
    case LSM6DS3_ACC_GYRO_ST_G_POS_SIGN_TEST:
    case LSM6DS3_ACC_GYRO_ST_G_NA:
    case LSM6DS3_ACC_GYRO_ST_G_NEG_SIGN_TEST:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS3_ACC_GYRO_W_SelfTest_G( handle, ( LSM6DS3_ACC_GYRO_ST_G_t )status ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
