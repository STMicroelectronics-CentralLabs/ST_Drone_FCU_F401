/**
 *******************************************************************************
 * @file    LSM303AGR_MAG_driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LSM303AGR sensor
 *******************************************************************************
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
#include "LSM303AGR_MAG_driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LSM303AGR_MAG LSM303AGR_MAG
 * @{
 */

/** @addtogroup LSM303AGR_MAG_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM303AGR_M_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t* pData );

/**
 * @}
 */

/** @addtogroup LSM303AGR_MAG_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LSM303AGR_M_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_M_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_M_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_M_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_M_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LSM303AGR_M_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LSM303AGR_M_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field );
static DrvStatusTypeDef LSM303AGR_M_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value );
static DrvStatusTypeDef LSM303AGR_M_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity );
static DrvStatusTypeDef LSM303AGR_M_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LSM303AGR_M_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LSM303AGR_M_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LSM303AGR_M_Get_FS( DrvContextTypeDef *handle, float *fullScale );
static DrvStatusTypeDef LSM303AGR_M_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale );
static DrvStatusTypeDef LSM303AGR_M_Set_FS_Value( DrvContextTypeDef *handle, float fullScale );
static DrvStatusTypeDef LSM303AGR_M_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LSM303AGR_M_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LSM303AGR_M_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LSM303AGR_MAG_Private_Variables Private variables
 * @{
 */

/**
 * @brief LSM303AGR_MAG driver structure
 */
MAGNETO_Drv_t LSM303AGR_M_Drv =
{
  LSM303AGR_M_Init,
  LSM303AGR_M_DeInit,
  LSM303AGR_M_Sensor_Enable,
  LSM303AGR_M_Sensor_Disable,
  LSM303AGR_M_Get_WhoAmI,
  LSM303AGR_M_Check_WhoAmI,
  LSM303AGR_M_Get_Axes,
  LSM303AGR_M_Get_AxesRaw,
  LSM303AGR_M_Get_Sensitivity,
  LSM303AGR_M_Get_ODR,
  LSM303AGR_M_Set_ODR,
  LSM303AGR_M_Set_ODR_Value,
  LSM303AGR_M_Get_FS,
  LSM303AGR_M_Set_FS,
  LSM303AGR_M_Set_FS_Value,
  LSM303AGR_M_Read_Reg,
  LSM303AGR_M_Write_Reg,
  LSM303AGR_M_Get_DRDY_Status
};

/**
 * @}
 */

/** @addtogroup LSM303AGR_MAG_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LSM303AGR sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Init( DrvContextTypeDef *handle )
{

  if ( LSM303AGR_M_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Operating mode selection - power down */
  if ( LSM303AGR_MAG_W_MD( (void *)handle, LSM303AGR_MAG_MD_IDLE1_MODE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( LSM303AGR_MAG_W_BDU( (void *)handle, LSM303AGR_MAG_BDU_ENABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM303AGR_M_Set_ODR( handle, ODR_HIGH ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM303AGR_M_Set_FS( handle, FS_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LSM303AGR_MAG_W_ST( (void *)handle, LSM303AGR_MAG_ST_DISABLED ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LSM303AGR sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_DeInit( DrvContextTypeDef *handle )
{

  if ( LSM303AGR_M_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable the component */
  if ( LSM303AGR_M_Sensor_Disable( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the LSM303AGR sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Operating mode selection */
  if ( LSM303AGR_MAG_W_MD( (void *)handle, LSM303AGR_MAG_MD_CONTINUOS_MODE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LSM303AGR sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Operating mode selection - power down */
  if ( LSM303AGR_MAG_W_MD( (void *)handle, LSM303AGR_MAG_MD_IDLE1_MODE ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  handle->isEnabled = 0;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the LSM303AGR sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LSM303AGR_MAG_R_WHO_AM_I( (void *)handle, ( uint8_t* )who_am_i ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the LSM303AGR sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LSM303AGR_M_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
 * @brief Get the LSM303AGR sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_Axes( DrvContextTypeDef *handle, SensorAxes_t *magnetic_field )
{

  int16_t pDataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM303AGR output register. */
  if ( LSM303AGR_M_Get_Axes_Raw( handle, pDataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Get LSM303AGR actual sensitivity. */
  if ( LSM303AGR_M_Get_Sensitivity( handle, &sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Calculate the data. */
  magnetic_field->AXIS_X = ( int32_t )( pDataRaw[0] * sensitivity );
  magnetic_field->AXIS_Y = ( int32_t )( pDataRaw[1] * sensitivity );
  magnetic_field->AXIS_Z = ( int32_t )( pDataRaw[2] * sensitivity );

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_AxesRaw( DrvContextTypeDef *handle, SensorAxesRaw_t *value )
{

  int16_t pDataRaw[3];

  /* Read raw data from LSM303AGR output register. */
  if ( LSM303AGR_M_Get_Axes_Raw( handle, pDataRaw ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set the raw data. */
  value->AXIS_X = pDataRaw[0];
  value->AXIS_Y = pDataRaw[1];
  value->AXIS_Z = pDataRaw[2];

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_Sensitivity( DrvContextTypeDef *handle, float *sensitivity )
{
  *sensitivity = 1.5f;

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_ODR( DrvContextTypeDef *handle, float *odr )
{
  LSM303AGR_MAG_ODR_t odr_low_level;

  if ( LSM303AGR_MAG_R_ODR( (void *)handle, &odr_low_level ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM303AGR_MAG_ODR_10Hz:
      *odr = 10.000f;
      break;
    case LSM303AGR_MAG_ODR_20Hz:
      *odr = 20.000f;
      break;
    case LSM303AGR_MAG_ODR_50Hz:
      *odr = 50.000f;
      break;
    case LSM303AGR_MAG_ODR_100Hz:
      *odr = 100.000f;
      break;
    default:
      *odr = -1.000f;
      return COMPONENT_ERROR;
  }
  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{
  LSM303AGR_MAG_ODR_t new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LSM303AGR_MAG_ODR_10Hz;
      break;
    case ODR_MID_LOW:
      new_odr = LSM303AGR_MAG_ODR_20Hz;
      break;
    case ODR_MID:
      new_odr = LSM303AGR_MAG_ODR_50Hz;
      break;
    case ODR_MID_HIGH:
      new_odr = LSM303AGR_MAG_ODR_100Hz;
      break;
    case ODR_HIGH:
      new_odr = LSM303AGR_MAG_ODR_100Hz;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LSM303AGR_MAG_W_ODR( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{
  LSM303AGR_MAG_ODR_t new_odr;

  new_odr = ( ( odr <= 10.000f ) ? LSM303AGR_MAG_ODR_10Hz
              : ( odr <= 20.000f ) ? LSM303AGR_MAG_ODR_20Hz
              : ( odr <= 50.000f ) ? LSM303AGR_MAG_ODR_50Hz
              :                     LSM303AGR_MAG_ODR_100Hz );

  if ( LSM303AGR_MAG_W_ODR( (void *)handle, new_odr ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the LSM303AGR sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_FS( DrvContextTypeDef *handle, float *fullScale )
{
  *fullScale = 50.0f;

  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Set_FS( DrvContextTypeDef *handle, SensorFs_t fullScale )
{
  return COMPONENT_OK;
}



/**
 * @brief Set the LSM303AGR sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Set_FS_Value( DrvContextTypeDef *handle, float fullScale )
{
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
static DrvStatusTypeDef LSM303AGR_M_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LSM303AGR_MAG_ReadReg( (void *)handle, reg, data, 1 ) == MEMS_ERROR )
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
static DrvStatusTypeDef LSM303AGR_M_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LSM303AGR_MAG_WriteReg( (void *)handle, reg, &data, 1 ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get magnetometer data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LSM303AGR_MAG_ZYXDA_t status_raw;

  if ( LSM303AGR_MAG_R_ZYXDA( (void *)handle, &status_raw ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( status_raw )
  {
    case LSM303AGR_MAG_ZYXDA_EV_ON:
      *status = 1;
      break;
    case LSM303AGR_MAG_ZYXDA_EV_OFF:
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

/** @addtogroup LSM303AGR_MAG_Private_Functions Private functions
 * @{
 */

/**
 * @brief Get the LSM303AGR sensor raw axes
 * @param handle the device handle
 * @param pData pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LSM303AGR_M_Get_Axes_Raw( DrvContextTypeDef *handle, int16_t* pData )
{

  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
  int16_t *regValueInt16;

  /* Read output registers from LSM303AGR_MAG_OUTX_L to LSM303AGR_MAG_OUTZ_H. */
  if ( LSM303AGR_MAG_Get_Raw_Magnetic( (void *)handle, regValue ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }

  regValueInt16 = (int16_t *)regValue;

  /* Format the data. */
  pData[0] = regValueInt16[0];
  pData[1] = regValueInt16[1];
  pData[2] = regValueInt16[2];

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
