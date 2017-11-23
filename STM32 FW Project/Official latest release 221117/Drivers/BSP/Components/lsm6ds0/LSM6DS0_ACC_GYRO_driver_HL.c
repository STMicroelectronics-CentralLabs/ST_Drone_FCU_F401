/**
 ******************************************************************************
 * @file    LSM6DS0_ACC_GYRO_driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LSM6DS0 sensor
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

#include "LSM6DS0_ACC_GYRO_driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM6DS0 LSM6DS0
 * @{
 */

/** @addtogroup LSM6DS0_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM6DS0_X_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_X_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_X_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_X_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS0_X_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef LSM6DS0_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM6DS0_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM6DS0_X_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS0_X_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM6DS0_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef LSM6DS0_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM6DS0_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef LSM6DS0_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef LSM6DS0_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS0_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM6DS0_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LSM6DS0_G_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_G_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_G_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_G_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_G_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS0_G_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity );
static DrvStatusTypeDef LSM6DS0_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM6DS0_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM6DS0_G_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS0_G_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM6DS0_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale );
static DrvStatusTypeDef LSM6DS0_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM6DS0_G_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef LSM6DS0_G_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef LSM6DS0_G_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS0_G_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM6DS0_G_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LSM6DS0_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM6DS0_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM6DS0_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM6DS0_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM6DS0_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );

static DrvStatusTypeDef LSM6DS0_X_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

static DrvStatusTypeDef LSM6DS0_G_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );

/**
 * @}
 */

/** @addtogroup LSM6DS0_Public_Variables Public variables
 * @{
 */

/**
 * @brief LSM6DS0 accelero driver structure
 */
ACCELERO_Drv_t LSM6DS0_X_Drv =
{
  LSM6DS0_X_Init,
  LSM6DS0_X_DeInit,
  LSM6DS0_X_Sensor_Enable,
  LSM6DS0_X_Sensor_Disable,
  LSM6DS0_X_Get_WhoAmI,
  LSM6DS0_X_Check_WhoAmI,
  LSM6DS0_X_Get_Axes,
  LSM6DS0_X_Get_AxesRaw,
  LSM6DS0_X_Get_Sensitivity,
  LSM6DS0_X_Get_ODR,
  LSM6DS0_X_Set_ODR,
  LSM6DS0_X_Set_ODR_Value,
  LSM6DS0_X_Get_FS,
  LSM6DS0_X_Set_FS,
  LSM6DS0_X_Set_FS_Value,
  LSM6DS0_X_Get_Axes_Status,
  LSM6DS0_X_Set_Axes_Status,
  LSM6DS0_X_Read_Reg,
  LSM6DS0_X_Write_Reg,
  LSM6DS0_X_Get_DRDY_Status
};

/**
 * @brief LSM6DS0 gyro driver structure
 */
GYRO_Drv_t LSM6DS0_G_Drv =
{
  LSM6DS0_G_Init,
  LSM6DS0_G_DeInit,
  LSM6DS0_G_Sensor_Enable,
  LSM6DS0_G_Sensor_Disable,
  LSM6DS0_G_Get_WhoAmI,
  LSM6DS0_G_Check_WhoAmI,
  LSM6DS0_G_Get_Axes,
  LSM6DS0_G_Get_AxesRaw,
  LSM6DS0_G_Get_Sensitivity,
  LSM6DS0_G_Get_ODR,
  LSM6DS0_G_Set_ODR,
  LSM6DS0_G_Set_ODR_Value,
  LSM6DS0_G_Get_FS,
  LSM6DS0_G_Set_FS,
  LSM6DS0_G_Set_FS_Value,
  LSM6DS0_G_Get_Axes_Status,
  LSM6DS0_G_Set_Axes_Status,
  LSM6DS0_G_Read_Reg,
  LSM6DS0_G_Write_Reg,
  LSM6DS0_G_Get_DRDY_Status
};

/**
 * @brief LSM6DS0 combo data structure definition
 */
LSM6DS0_Combo_Data_t LSM6DS0_Combo_Data[LSM6DS0_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup LSM6DS0_Callable_Private_Functions Callable Private functions
 * @{
 */

/**
 * @brief Initialize the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };

  if ( LSM6DS0_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess( (void *)handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if ( LSM6DS0_X_Set_ODR_When_Disabled( handle, ODR_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM6DS0_ACC_GYRO_W_BlockDataUpdate( (void *)handle, LSM6DS0_ACC_GYRO_BDU_ENABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)handle, LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS0_X_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes */
  if ( LSM6DS0_X_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}



/**
 * @brief Deinitialize the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_DeInit( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  if ( LSM6DS0_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if ( LSM6DS0_X_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Sensor_Enable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection */
  if ( LSM6DS0_X_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Sensor_Disable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LSM6DS0_X_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)handle, LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LSM6DS0_Get_WhoAmI(handle, who_am_i);
}


/**
 * @brief Check the WHO_AM_I ID of the LSM6DS0 accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LSM6DS0_Check_WhoAmI(handle);
}


/**
 * @brief Get the LSM6DS0 accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{

  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM6DS0 output register. */
  if ( LSM6DS0_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LSM6DS0 actual sensitivity. */
  if ( LSM6DS0_X_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
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
 * @brief Get the LSM6DS0 accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];

  /* Read raw data from LSM6DS0 output register. */
  if ( LSM6DS0_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
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
 * @brief Get the LSM6DS0 accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM6DS0_ACC_GYRO_FS_XL_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM6DS0_ACC_GYRO_FS_XL_2g:
      *sensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_4g:
      *sensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_8g:
      *sensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_16g:
      *sensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LSM6DS0_ACC_GYRO_ODR_XL_t odr_low_level;
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  /* Accelerometer ODR forced to be same like gyroscope ODR. */
  if(pComponentData->comboData->isGyroEnabled == 1)
  {
    *odr = pComponentData->comboData->lastGyroODR;
    return COMPONENT_OK;
  }

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerDataRate( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN:
      *odr =   0.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_10Hz:
      *odr =  10.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_50Hz:
      *odr =  50.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_119Hz:
      *odr = 119.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_238Hz:
      *odr = 238.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_476Hz:
      *odr = 476.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_952Hz:
      *odr = 952.0f;
      break;
    default:
      *odr =  -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS0_X_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS0_X_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS0_X_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS0_X_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  LSM6DS0_ACC_GYRO_FS_XL_t fs_low_level;

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale( (void *)handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM6DS0_ACC_GYRO_FS_XL_2g:
      *fullScale =  2.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_4g:
      *fullScale =  4.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_8g:
      *fullScale =  8.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  LSM6DS0_ACC_GYRO_FS_XL_t new_fs;

  switch( fullScale )
  {
    case FS_LOW:
      new_fs = LSM6DS0_ACC_GYRO_FS_XL_2g;
      break;
    case FS_MID:
      new_fs = LSM6DS0_ACC_GYRO_FS_XL_4g;
      break;
    case FS_HIGH:
      new_fs = LSM6DS0_ACC_GYRO_FS_XL_8g;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerFullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  LSM6DS0_ACC_GYRO_FS_XL_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_2g
           : ( fullScale <= 4.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_4g
           : ( fullScale <= 8.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_8g
           :                         LSM6DS0_ACC_GYRO_FS_XL_16g;

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerFullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  LSM6DS0_ACC_GYRO_XEN_XL_t xStatus;
  LSM6DS0_ACC_GYRO_YEN_XL_t yStatus;
  LSM6DS0_ACC_GYRO_ZEN_XL_t zStatus;

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerAxisX( (void *)handle, &xStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS0_ACC_GYRO_R_AccelerometerAxisY( (void *)handle, &yStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS0_ACC_GYRO_R_AccelerometerAxisZ( (void *)handle, &zStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  xyz_enabled[0] = ( xStatus == LSM6DS0_ACC_GYRO_XEN_XL_ENABLE ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == LSM6DS0_ACC_GYRO_YEN_XL_ENABLE ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE ) ? 1 : 0;

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the LSM6DS0 accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisX( (void *)handle,
       ( enable_xyz[0] == 1 ) ? LSM6DS0_ACC_GYRO_XEN_XL_ENABLE : LSM6DS0_ACC_GYRO_XEN_XL_DISABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisY( (void *)handle,
       ( enable_xyz[1] == 1 ) ? LSM6DS0_ACC_GYRO_YEN_XL_ENABLE : LSM6DS0_ACC_GYRO_YEN_XL_DISABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ( (void *)handle,
       ( enable_xyz[2] == 1 ) ? LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE : LSM6DS0_ACC_GYRO_ZEN_XL_DISABLE ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM6DS0_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS0_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LSM6DS0_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS0_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LSM6DS0_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS0_ACC_GYRO_XLDA_t status_raw;

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerDataReadyFlag( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS0_ACC_GYRO_XLDA_UP:
      *status = 1;
      break;
    case LSM6DS0_ACC_GYRO_XLDA_DOWN:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Initialize the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };

  if ( LSM6DS0_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess( (void *)handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  if ( LSM6DS0_G_Set_ODR_When_Disabled( handle, ODR_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM6DS0_ACC_GYRO_W_BlockDataUpdate( (void *)handle, LSM6DS0_ACC_GYRO_BDU_ENABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)handle, LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection */
  if ( LSM6DS0_G_Set_FS( handle, FS_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes */
  if ( LSM6DS0_G_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_DeInit( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  if ( LSM6DS0_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if ( LSM6DS0_G_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Sensor_Enable( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection */
  if ( LSM6DS0_G_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  pComponentData->comboData->isGyroEnabled = 1;
  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Sensor_Disable( DrvContextTypeDef *handle )
{
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LSM6DS0_G_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)handle, LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  pComponentData->comboData->isGyroEnabled = 0;
  pComponentData->comboData->lastGyroODR = 0.0f;
  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LSM6DS0_Get_WhoAmI(handle, who_am_i);
}

/**
 * @brief Check the WHO_AM_I ID of the LSM6DS0 gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LSM6DS0_Check_WhoAmI(handle);
}


/**
 * @brief Get the LSM6DS0 gyroscope sensor axes
 * @param handle the device handle
 * @param angular_velocity pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *angular_velocity )
{

  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM6DS0 output register. */
  if ( LSM6DS0_G_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LSM6DS0 actual sensitivity. */
  if ( LSM6DS0_G_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
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
 * @brief Get the LSM6DS0 gyroscope sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];

  /* Read raw data from LSM6DS0 output register. */
  if ( LSM6DS0_G_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
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
 * @brief Get the LSM6DS0 gyroscope sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM6DS0_ACC_GYRO_FS_G_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM6DS0_ACC_GYRO_R_GyroFullScale( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM6DS0_ACC_GYRO_FS_G_245dps:
      *sensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_500dps:
      *sensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_2000dps:
      *sensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS;
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LSM6DS0_ACC_GYRO_ODR_G_t odr_low_level;

  if ( LSM6DS0_ACC_GYRO_R_GyroDataRate( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN:
      *odr =   0.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_15Hz:
      *odr =  15.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_60Hz:
      *odr =  60.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_119Hz:
      *odr = 119.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_238Hz:
      *odr = 238.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_476Hz:
      *odr = 476.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_952Hz:
      *odr = 952.0f;
      break;
    default:
      *odr =  -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS0_G_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS0_G_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM6DS0_G_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM6DS0_G_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  LSM6DS0_ACC_GYRO_FS_G_t fs_low_level;

  if ( LSM6DS0_ACC_GYRO_R_GyroFullScale( (void *)handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM6DS0_ACC_GYRO_FS_G_245dps:
      *fullScale =  245.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_500dps:
      *fullScale =  500.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_2000dps:
      *fullScale = 2000.0f;
      break;
    default:
      *fullScale =   -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  LSM6DS0_ACC_GYRO_FS_G_t new_fs;

  switch( fullScale )
  {
    case FS_LOW:
      new_fs = LSM6DS0_ACC_GYRO_FS_G_245dps;
      break;
    case FS_MID:
      new_fs = LSM6DS0_ACC_GYRO_FS_G_500dps;
      break;
    case FS_HIGH:
      new_fs = LSM6DS0_ACC_GYRO_FS_G_2000dps;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroFullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM6DS0 gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  LSM6DS0_ACC_GYRO_FS_G_t new_fs;

  new_fs = ( fullScale <= 245.0f ) ? LSM6DS0_ACC_GYRO_FS_G_245dps
           : ( fullScale <= 500.0f ) ? LSM6DS0_ACC_GYRO_FS_G_500dps
           :                           LSM6DS0_ACC_GYRO_FS_G_2000dps;

  if ( LSM6DS0_ACC_GYRO_W_GyroFullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM6DS0 gyroscope sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  LSM6DS0_ACC_GYRO_XEN_G_t xStatus;
  LSM6DS0_ACC_GYRO_YEN_G_t yStatus;
  LSM6DS0_ACC_GYRO_ZEN_G_t zStatus;

  if ( LSM6DS0_ACC_GYRO_R_GyroAxisX( (void *)handle, &xStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS0_ACC_GYRO_R_GyroAxisY( (void *)handle, &yStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM6DS0_ACC_GYRO_R_GyroAxisZ( (void *)handle, &zStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  xyz_enabled[0] = ( xStatus == LSM6DS0_ACC_GYRO_XEN_G_ENABLE ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == LSM6DS0_ACC_GYRO_YEN_G_ENABLE ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == LSM6DS0_ACC_GYRO_ZEN_G_ENABLE ) ? 1 : 0;

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the LSM6DS0 gyroscope sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( LSM6DS0_ACC_GYRO_W_GyroAxisX( (void *)handle,
                                     ( enable_xyz[0] == 1 ) ? LSM6DS0_ACC_GYRO_XEN_G_ENABLE : LSM6DS0_ACC_GYRO_XEN_G_DISABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroAxisY( (void *)handle,
                                     ( enable_xyz[1] == 1 ) ? LSM6DS0_ACC_GYRO_YEN_G_ENABLE : LSM6DS0_ACC_GYRO_YEN_G_DISABLE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroAxisZ( (void *)handle,
                                     ( enable_xyz[2] == 1 ) ? LSM6DS0_ACC_GYRO_ZEN_G_ENABLE : LSM6DS0_ACC_GYRO_ZEN_G_DISABLE ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM6DS0_G_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS0_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LSM6DS0_G_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS0_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LSM6DS0_G_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM6DS0_ACC_GYRO_GDA_t status_raw;

  if ( LSM6DS0_ACC_GYRO_R_GyroDataReadyFlag( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM6DS0_ACC_GYRO_GDA_UP:
      *status = 1;
      break;
    case LSM6DS0_ACC_GYRO_GDA_DOWN:
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

/** @addtogroup LSM6DS0_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the WHO_AM_I ID of the LSM6DS0 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LSM6DS0_ACC_GYRO_R_WHO_AM_I_( (void *)handle, ( uint8_t* )who_am_i ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the LSM6DS0 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LSM6DS0_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LSM6DS0_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM6DS0_ACC_GYRO_ReadReg( (void *)handle, reg, data, 1 ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM6DS0_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM6DS0_ACC_GYRO_WriteReg( (void *)handle, reg, &data, 1 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM6DS0 accelerometer sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS0_ACC_GYRO_OUT_X_L_XL to LSM6DS0_ACC_GYRO_OUT_Z_H_XL. */
  if ( LSM6DS0_ACC_GYRO_Get_Acceleration( (void *)handle, ( uint8_t* )regValue ) == MEMS_ERROR )
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
 * @brief Set the LSM6DS0 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LSM6DS0_ACC_GYRO_ODR_XL_t new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM6DS0_ACC_GYRO_ODR_XL_10Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM6DS0_ACC_GYRO_ODR_XL_10Hz;
      break;
    case ODR_MID:
      new_odr = LSM6DS0_ACC_GYRO_ODR_XL_50Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM6DS0_ACC_GYRO_ODR_XL_50Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM6DS0_ACC_GYRO_ODR_XL_119Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LSM6DS0 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 10.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 10.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 50.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 50.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 119.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LSM6DS0 accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

  LSM6DS0_ACC_GYRO_ODR_XL_t new_odr;

  new_odr = ( odr <=  10.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_10Hz
            : ( odr <=  50.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_50Hz
            : ( odr <= 119.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_119Hz
            : ( odr <= 238.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_238Hz
            : ( odr <= 476.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_476Hz
            :                     LSM6DS0_ACC_GYRO_ODR_XL_952Hz;

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS0 accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM6DS0_X_Data_t *pComponentData = ( LSM6DS0_X_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=  10.0f ) ? 10.0f
                                 : ( odr <=  50.0f ) ? 50.0f
                                 : ( odr <= 119.0f ) ? 119.0f
                                 : ( odr <= 238.0f ) ? 238.0f
                                 : ( odr <= 476.0f ) ? 476.0f
                                 :                     952.0f;

  return COMPONENT_OK;
}


/**
 * @brief Get the LSM6DS0 gyroscope sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData )
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS0_ACC_GYRO_OUT_X_L_G to LSM6DS0_ACC_GYRO_OUT_Z_H_G. */
  if ( LSM6DS0_ACC_GYRO_Get_AngularRate( (void *)handle, ( uint8_t* )regValue ) == MEMS_ERROR )
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
 * @brief Set the LSM6DS0 gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LSM6DS0_ACC_GYRO_ODR_G_t new_odr;
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM6DS0_ACC_GYRO_ODR_G_15Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM6DS0_ACC_GYRO_ODR_G_15Hz;
      break;
    case ODR_MID:
      new_odr = LSM6DS0_ACC_GYRO_ODR_G_60Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM6DS0_ACC_GYRO_ODR_G_60Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM6DS0_ACC_GYRO_ODR_G_119Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if(LSM6DS0_G_Get_ODR( handle, &pComponentData->comboData->lastGyroODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LSM6DS0 gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 15.0f;
      pComponentData->comboData->lastGyroODR = 15.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 15.0f;
      pComponentData->comboData->lastGyroODR = 15.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 60.0f;
      pComponentData->comboData->lastGyroODR = 60.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 60.0f;
      pComponentData->comboData->lastGyroODR = 60.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 119.0f;
      pComponentData->comboData->lastGyroODR = 119.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LSM6DS0 gyroscope sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

  LSM6DS0_ACC_GYRO_ODR_G_t new_odr;
  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  new_odr = ( odr <=  15.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_15Hz
            : ( odr <=  60.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_60Hz
            : ( odr <= 119.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_119Hz
            : ( odr <= 238.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_238Hz
            : ( odr <= 476.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_476Hz
            :                     LSM6DS0_ACC_GYRO_ODR_G_952Hz;

  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if(LSM6DS0_G_Get_ODR( handle, &pComponentData->comboData->lastGyroODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM6DS0 gyroscope sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM6DS0_G_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  GYRO_Data_t *pData = ( GYRO_Data_t * )handle->pData;
  LSM6DS0_G_Data_t *pComponentData = ( LSM6DS0_G_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=  15.0f ) ? 15.0f
                                 : ( odr <=  60.0f ) ? 60.0f
                                 : ( odr <= 119.0f ) ? 119.0f
                                 : ( odr <= 238.0f ) ? 238.0f
                                 : ( odr <= 476.0f ) ? 476.0f
                                 :                     952.0f;

  pComponentData->comboData->lastGyroODR = ( odr <=  15.0f ) ? 15.0f
      : ( odr <=  60.0f ) ? 60.0f
      : ( odr <= 119.0f ) ? 119.0f
      : ( odr <= 238.0f ) ? 238.0f
      : ( odr <= 476.0f ) ? 476.0f
      :                     952.0f;

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
