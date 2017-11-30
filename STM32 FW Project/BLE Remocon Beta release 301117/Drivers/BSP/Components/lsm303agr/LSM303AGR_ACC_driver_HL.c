/**
 ******************************************************************************
 * @file    LSM303AGR_ACC_driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LSM303AGR accelerator sensor
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

#include "LSM303AGR_ACC_driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM303AGR_ACC LSM303AGR_ACC
 * @{
 */

/** @addtogroup LSM303AGR_ACC_Callable_Private_Function_Prototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM303AGR_X_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_X_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_X_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_X_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM303AGR_X_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration );
static DrvStatusTypeDef LSM303AGR_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM303AGR_X_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM303AGR_X_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM303AGR_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fs );
static DrvStatusTypeDef LSM303AGR_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM303AGR_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled );
static DrvStatusTypeDef LSM303AGR_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz );
static DrvStatusTypeDef LSM303AGR_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM303AGR_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM303AGR_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LSM303AGR_ACC_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM303AGR_X_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t *pData );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_Normal_Mode( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_LP_Mode( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_HR_Mode( DrvContextTypeDef *handle, float *sensitivity );

/**
 * @}
 */

/** @addtogroup LSM303AGR_ACC_Public_Variables Public variables
 * @{
 */

/**
 * @brief LSM303AGR_ACC accelero driver structure
 */
ACCELERO_Drv_t LSM303AGR_X_Drv =
{
  LSM303AGR_X_Init,
  LSM303AGR_X_DeInit,
  LSM303AGR_X_Sensor_Enable,
  LSM303AGR_X_Sensor_Disable,
  LSM303AGR_X_Get_WhoAmI,
  LSM303AGR_X_Check_WhoAmI,
  LSM303AGR_X_Get_Axes,
  LSM303AGR_X_Get_AxesRaw,
  LSM303AGR_X_Get_Sensitivity,
  LSM303AGR_X_Get_ODR,
  LSM303AGR_X_Set_ODR,
  LSM303AGR_X_Set_ODR_Value,
  LSM303AGR_X_Get_FS,
  LSM303AGR_X_Set_FS,
  LSM303AGR_X_Set_FS_Value,
  LSM303AGR_X_Get_Axes_Status,
  LSM303AGR_X_Set_Axes_Status,
  LSM303AGR_X_Read_Reg,
  LSM303AGR_X_Write_Reg,
  LSM303AGR_X_Get_DRDY_Status
};

/**
 * @}
 */

/** @addtogroup LSM303AGR_ACC_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Init( DrvContextTypeDef *handle )
{

  uint8_t axes_status[] = { 1, 1, 1 };
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  if ( LSM303AGR_X_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM303AGR_ACC_W_BlockDataUpdate( (void *)handle, LSM303AGR_ACC_BDU_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* FIFO mode selection */
  if ( LSM303AGR_ACC_W_FifoMode( (void *)handle, LSM303AGR_ACC_FM_BYPASS ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Select default output data rate. */
  pComponentData->Previous_ODR = 100.0f;

  /* Output data rate selection - power down. */
  if ( LSM303AGR_ACC_W_ODR( (void *)handle, LSM303AGR_ACC_ODR_DO_PWR_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Full scale selection. */
  if ( LSM303AGR_X_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable axes. */
  if ( LSM303AGR_X_Set_Axes_Status( handle, axes_status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_DeInit( DrvContextTypeDef *handle )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  if ( LSM303AGR_X_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if( LSM303AGR_X_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Reset output data rate. */
  pComponentData->Previous_ODR = 0.0f;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Sensor_Enable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Output data rate selection. */
  if ( LSM303AGR_X_Set_ODR_Value_When_Enabled( handle, pComponentData->Previous_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Sensor_Disable( DrvContextTypeDef *handle )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Store actual output data rate. */
  if ( LSM303AGR_X_Get_ODR( handle, &( pComponentData->Previous_ODR ) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM303AGR_ACC_W_ODR( (void *)handle, LSM303AGR_ACC_ODR_DO_PWR_DOWN ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LSM303AGR_ACC_R_WHO_AM_I( (void *)handle, ( uint8_t* )who_am_i ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LSM303AGR accelerometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LSM303AGR_X_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
 * @brief Get the LSM303AGR accelerometer sensor axes
 * @param handle the device handle
 * @param acceleration pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *acceleration )
{

  int data[3];

  /* Read data from LSM303AGR. */
  if ( !LSM303AGR_ACC_Get_Acceleration((void *)handle, data) )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  acceleration->AXIS_X = data[0];
  acceleration->AXIS_Y = data[1];
  acceleration->AXIS_Z = data[2];

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM303AGR accelerometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t dataRaw[3];

  /* Read raw data from LSM303AGR output register. */
  if ( LSM303AGR_X_Get_Axes_Raw( handle, dataRaw ) == COMPONENT_ERROR )
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
 * @brief Get the LSM303AGR accelerometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM303AGR_ACC_LPEN_t lp_value;
  LSM303AGR_ACC_HR_t hr_value;

  /* Read low power flag */
  if( LSM303AGR_ACC_R_LOWPWR_EN( (void *)handle, &lp_value ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Read high performance flag */
  if( LSM303AGR_ACC_R_HiRes( (void *)handle, &hr_value ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if( lp_value == LSM303AGR_ACC_LPEN_DISABLED && hr_value == LSM303AGR_ACC_HR_DISABLED )
  {
    /* Normal Mode */
    return LSM303AGR_X_Get_Sensitivity_Normal_Mode( handle, sensitivity );
  }
  else if ( lp_value == LSM303AGR_ACC_LPEN_ENABLED && hr_value == LSM303AGR_ACC_HR_DISABLED )
  {
    /* Low Power Mode */
    return LSM303AGR_X_Get_Sensitivity_LP_Mode( handle, sensitivity );
  }
  else if ( lp_value == LSM303AGR_ACC_LPEN_DISABLED && hr_value == LSM303AGR_ACC_HR_ENABLED )
  {
    /* High Resolution Mode */
    return LSM303AGR_X_Get_Sensitivity_HR_Mode( handle, sensitivity );
  }
  else
  {
    /* Not allowed */
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get the LSM303AGR accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LSM303AGR_ACC_ODR_t odr_low_level;

  if ( LSM303AGR_ACC_R_ODR( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM303AGR_ACC_ODR_DO_PWR_DOWN:
      *odr =     0.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_1Hz:
      *odr =    1.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_10Hz:
      *odr =    10.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_25Hz:
      *odr =    25.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_50Hz:
      *odr =    50.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_100Hz:
      *odr =   100.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_200Hz:
      *odr =   200.0f;
      break;
    case LSM303AGR_ACC_ODR_DO_400Hz:
      *odr =   400.0f;
      break;
    default:
      *odr =    -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM303AGR_X_Set_ODR_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM303AGR_X_Set_ODR_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  if(handle->isEnabled == 1)
  {
    if(LSM303AGR_X_Set_ODR_Value_When_Enabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LSM303AGR_X_Set_ODR_Value_When_Disabled(handle, odr) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{

  LSM303AGR_ACC_FS_t fs_low_level;

  if ( LSM303AGR_ACC_R_FullScale( (void *)handle, &fs_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM303AGR_ACC_FS_2G:
      *fullScale =  2.0f;
      break;
    case LSM303AGR_ACC_FS_4G:
      *fullScale =  4.0f;
      break;
    case LSM303AGR_ACC_FS_8G:
      *fullScale =  8.0f;
      break;
    case LSM303AGR_ACC_FS_16G:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{

  LSM303AGR_ACC_FS_t new_fs;

  switch( fullScale )
  {
    case FS_LOW:
      new_fs = LSM303AGR_ACC_FS_2G;
      break;
    case FS_MID:
      new_fs = LSM303AGR_ACC_FS_4G;
      break;
    case FS_HIGH:
      new_fs = LSM303AGR_ACC_FS_8G;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM303AGR_ACC_W_FullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR accelerometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{

  LSM303AGR_ACC_FS_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ? LSM303AGR_ACC_FS_2G
           : ( fullScale <= 4.0f ) ? LSM303AGR_ACC_FS_4G
           : ( fullScale <= 8.0f ) ? LSM303AGR_ACC_FS_8G
           :                         LSM303AGR_ACC_FS_16G;

  if ( LSM303AGR_ACC_W_FullScale( (void *)handle, new_fs ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR accelerometer sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Axes_Status( DrvContextTypeDef *handle, uint8_t *xyz_enabled )
{

  LSM303AGR_ACC_XEN_t xStatus;
  LSM303AGR_ACC_YEN_t yStatus;
  LSM303AGR_ACC_ZEN_t zStatus;

  if ( LSM303AGR_ACC_R_XEN( (void *)handle, &xStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM303AGR_ACC_R_YEN( (void *)handle, &yStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  if ( LSM303AGR_ACC_R_ZEN( (void *)handle, &zStatus ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  xyz_enabled[0] = ( xStatus == LSM303AGR_ACC_XEN_ENABLED ) ? 1 : 0;
  xyz_enabled[1] = ( yStatus == LSM303AGR_ACC_YEN_ENABLED ) ? 1 : 0;
  xyz_enabled[2] = ( zStatus == LSM303AGR_ACC_ZEN_ENABLED ) ? 1 : 0;

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the LSM303AGR accelerometer sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_Axes_Status( DrvContextTypeDef *handle, uint8_t *enable_xyz )
{

  if ( LSM303AGR_ACC_W_XEN( (void *)handle,
                            ( enable_xyz[0] == 1 ) ? LSM303AGR_ACC_XEN_ENABLED : LSM303AGR_ACC_XEN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM303AGR_ACC_W_YEN ( (void *)handle,
                             ( enable_xyz[1] == 1 ) ? LSM303AGR_ACC_YEN_ENABLED : LSM303AGR_ACC_YEN_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM303AGR_ACC_W_ZEN ( (void *)handle,
                             ( enable_xyz[2] == 1 ) ? LSM303AGR_ACC_ZEN_ENABLED : LSM303AGR_ACC_ZEN_DISABLED ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM303AGR_X_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM303AGR_ACC_ReadReg( (void *)handle, reg, data, 1 ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM303AGR_X_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM303AGR_ACC_WriteReg( (void *)handle, reg, &data, 1 ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM303AGR_X_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM303AGR_ACC_XDA_t status_raw;

  if ( LSM303AGR_ACC_R_XDataAvail( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM303AGR_ACC_XDA_AVAILABLE:
      *status = 1;
      break;
    case LSM303AGR_ACC_XDA_NOT_AVAILABLE:
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


/** @addtogroup LSM303AGR_ACC_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the LSM303AGR accelerometer sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Axes_Raw(DrvContextTypeDef *handle, int16_t *pData)
{

  Type3Axis16bit_U raw_data_tmp;
  u8_t shift = 0;
  LSM303AGR_ACC_LPEN_t lp;
  LSM303AGR_ACC_HR_t hr;

  /* Determine which operational mode the acc is set */
  if(!LSM303AGR_ACC_R_HiRes( (void *)handle, &hr ))
  {
    return COMPONENT_ERROR;
  }

  if(!LSM303AGR_ACC_R_LOWPWR_EN( (void *)handle, &lp ))
  {
    return COMPONENT_ERROR;
  }

  if (lp == LSM303AGR_ACC_LPEN_ENABLED && hr == LSM303AGR_ACC_HR_DISABLED)
  {
    /* op mode is LP 8-bit */
    shift = 8;
  }
  else if (lp == LSM303AGR_ACC_LPEN_DISABLED && hr == LSM303AGR_ACC_HR_DISABLED)
  {
    /* op mode is Normal 10-bit */
    shift = 6;
  }
  else if (lp == LSM303AGR_ACC_LPEN_DISABLED && hr == LSM303AGR_ACC_HR_ENABLED)
  {
    /* op mode is HR 12-bit */
    shift = 4;
  }
  else
  {
    return COMPONENT_ERROR;
  }

  /* Read output registers from LSM303AGR_ACC_GYRO_OUTX_L_XL to LSM303AGR_ACC_GYRO_OUTZ_H_XL. */
  if (!LSM303AGR_ACC_Get_Raw_Acceleration( (void *)handle, raw_data_tmp.u8bit ))
  {
    return COMPONENT_ERROR;
  }

  /* Format the data. */
  pData[0] = ( raw_data_tmp.i16bit[0] >> shift );
  pData[1] = ( raw_data_tmp.i16bit[1] >> shift );
  pData[2] = ( raw_data_tmp.i16bit[2] >> shift );

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  LSM303AGR_ACC_ODR_t new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM303AGR_ACC_ODR_DO_1Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM303AGR_ACC_ODR_DO_10Hz;
      break;
    case ODR_MID:
      new_odr = LSM303AGR_ACC_ODR_DO_25Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM303AGR_ACC_ODR_DO_50Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM303AGR_ACC_ODR_DO_100Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM303AGR_ACC_W_ODR( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  switch( odr )
  {
    case ODR_LOW:
      pComponentData->Previous_ODR = 1.0f;
      break;
    case ODR_MID_LOW:
      pComponentData->Previous_ODR = 10.0f;
      break;
    case ODR_MID:
      pComponentData->Previous_ODR = 25.0f;
      break;
    case ODR_MID_HIGH:
      pComponentData->Previous_ODR = 50.0f;
      break;
    case ODR_HIGH:
      pComponentData->Previous_ODR = 100.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr )
{

  LSM303AGR_ACC_ODR_t new_odr;

  new_odr = ( odr <=    1.0f ) ? LSM303AGR_ACC_ODR_DO_1Hz
            : ( odr <=   10.0f ) ? LSM303AGR_ACC_ODR_DO_10Hz
            : ( odr <=   25.0f ) ? LSM303AGR_ACC_ODR_DO_25Hz
            : ( odr <=   50.0f ) ? LSM303AGR_ACC_ODR_DO_50Hz
            : ( odr <=  100.0f ) ? LSM303AGR_ACC_ODR_DO_100Hz
            : ( odr <=  200.0f ) ? LSM303AGR_ACC_ODR_DO_200Hz
            :                      LSM303AGR_ACC_ODR_DO_400Hz;

  if ( LSM303AGR_ACC_W_ODR( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LSM303AGR accelerometer sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr )
{

  ACCELERO_Data_t *pData = ( ACCELERO_Data_t * )handle->pData;
  LSM303AGR_X_Data_t *pComponentData = ( LSM303AGR_X_Data_t * )pData->pComponentData;

  pComponentData->Previous_ODR = ( odr <=    1.0f ) ?  1.0f
                                 : ( odr <=   10.0f ) ? 10.0f
                                 : ( odr <=   25.0f ) ? 25.0f
                                 : ( odr <=   50.0f ) ? 50.0f
                                 : ( odr <=  100.0f ) ? 100.0f
                                 : ( odr <=  200.0f ) ? 200.0f
                                 :                      400.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM303AGR accelerometer sensor sensitivity in normal mode
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_Normal_Mode( DrvContextTypeDef *handle, float *sensitivity )
{

  LSM303AGR_ACC_FS_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM303AGR_ACC_R_FullScale( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM303AGR_ACC_FS_2G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_NORMAL_MODE;
      break;
    case LSM303AGR_ACC_FS_4G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_NORMAL_MODE;
      break;
    case LSM303AGR_ACC_FS_8G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_NORMAL_MODE;
      break;
    case LSM303AGR_ACC_FS_16G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_NORMAL_MODE;
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM303AGR accelerometer sensor sensitivity in low power mode
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_LP_Mode( DrvContextTypeDef *handle, float *sensitivity )
{
  LSM303AGR_ACC_FS_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM303AGR_ACC_R_FullScale( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM303AGR_ACC_FS_2G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_LOW_POWER_MODE;
      break;
    case LSM303AGR_ACC_FS_4G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_LOW_POWER_MODE;
      break;
    case LSM303AGR_ACC_FS_8G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_LOW_POWER_MODE;
      break;
    case LSM303AGR_ACC_FS_16G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_LOW_POWER_MODE;
      break;
    default:
      *sensitivity = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the LSM303AGR accelerometer sensor sensitivity in high resolution mode
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_X_Get_Sensitivity_HR_Mode( DrvContextTypeDef *handle, float *sensitivity )
{
  LSM303AGR_ACC_FS_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM303AGR_ACC_R_FullScale( (void *)handle, &fullScale ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM303AGR_ACC_FS_2G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_2G_HIGH_RESOLUTION_MODE;
      break;
    case LSM303AGR_ACC_FS_4G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_4G_HIGH_RESOLUTION_MODE;
      break;
    case LSM303AGR_ACC_FS_8G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_8G_HIGH_RESOLUTION_MODE;
      break;
    case LSM303AGR_ACC_FS_16G:
      *sensitivity = ( float )LSM303AGR_ACC_SENSITIVITY_FOR_FS_16G_HIGH_RESOLUTION_MODE;
      break;
    default:
      *sensitivity = -1.0f;
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
