/**
 ******************************************************************************
 * @file    HTS221_Driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the HTS221 sensor
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
#include "HTS221_Driver_HL.h"
#include <math.h>



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup HTS221 HTS221
 * @{
 */

/** @addtogroup HTS221_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef HTS221_H_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_H_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_H_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_H_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_H_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef HTS221_H_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_H_Get_Hum( DrvContextTypeDef *handle, float *humidity );
static DrvStatusTypeDef HTS221_H_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef HTS221_H_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef HTS221_H_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef HTS221_H_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef HTS221_H_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef HTS221_H_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef HTS221_T_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_T_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_T_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_T_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef HTS221_T_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_T_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef HTS221_T_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef HTS221_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef HTS221_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef HTS221_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef HTS221_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef HTS221_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup HTS221_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef HTS221_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef HTS221_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef HTS221_Get_Hum( DrvContextTypeDef *handle, float *humidity );
static DrvStatusTypeDef HTS221_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef HTS221_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef HTS221_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef HTS221_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef HTS221_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef HTS221_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );

/**
 * @}
 */

/** @addtogroup HTS221_Private_Variables Private variables
 * @{
 */

/**
 * @brief HTS221 humidity driver structure
 */
HUMIDITY_Drv_t HTS221_H_Drv =
{
  HTS221_H_Init,
  HTS221_H_DeInit,
  HTS221_H_Sensor_Enable,
  HTS221_H_Sensor_Disable,
  HTS221_H_Get_WhoAmI,
  HTS221_H_Check_WhoAmI,
  HTS221_H_Get_Hum,
  HTS221_H_Get_ODR,
  HTS221_H_Set_ODR,
  HTS221_H_Set_ODR_Value,
  HTS221_H_Read_Reg,
  HTS221_H_Write_Reg,
  HTS221_H_Get_DRDY_Status
};

/**
 * @brief HTS221 temperature driver structure
 */
TEMPERATURE_Drv_t HTS221_T_Drv =
{
  HTS221_T_Init,
  HTS221_T_DeInit,
  HTS221_T_Sensor_Enable,
  HTS221_T_Sensor_Disable,
  HTS221_T_Get_WhoAmI,
  HTS221_T_Check_WhoAmI,
  HTS221_T_Get_Temp,
  HTS221_T_Get_ODR,
  HTS221_T_Set_ODR,
  HTS221_T_Set_ODR_Value,
  HTS221_T_Read_Reg,
  HTS221_T_Write_Reg,
  HTS221_T_Get_DRDY_Status
};

/**
 * @brief HTS221 combo data structure definition
 */
HTS221_Combo_Data_t HTS221_Combo_Data[HTS221_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup HTS221_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Init( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if ((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(HTS221_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if ((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(HTS221_H_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}



/**
 * @brief Enable the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 temperature sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(HTS221_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 temperature sensor is still enabled. */
  /* If yes, skip the disable function, if not call disable function */
  if((((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(HTS221_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_H_Data_t *)(((HUMIDITY_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the HTS221 humidity sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return HTS221_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the HTS221 humidity sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return HTS221_Check_WhoAmI( handle );
}


/**
 * @brief Get the humidity value of the HTS221 humidity sensor
 * @param handle the device handle
 * @param humidity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_Hum( DrvContextTypeDef *handle, float *humidity )
{

  return HTS221_Get_Hum( handle, humidity );
}


/**
 * @brief Get the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return HTS221_Get_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return HTS221_Set_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 humidity sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return HTS221_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_H_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get humidity data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_H_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  HTS221_BitStatus_et hum_status_raw;
  HTS221_BitStatus_et temp_status_raw;

  if ( HTS221_Get_DataStatus( (void *)handle, &hum_status_raw, &temp_status_raw ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( hum_status_raw )
  {
    case HTS221_SET:
      *status = 1;
      break;
    case HTS221_RESET:
      *status = 0;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Initialize the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Init( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 humidity sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized == 0))
  {
    if(HTS221_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the HTS221 humidity sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumInitialized == 0))
  {
    if(HTS221_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Enable the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 humidity sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled == 0))
  {
    if(HTS221_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the HTS221 humidity sensor is still enabled. */
  /* If yes, skip the disable function, if not call disable function */
  if((((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isHumEnabled == 0))
  {
    if(HTS221_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((HTS221_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the HTS221 temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return HTS221_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return HTS221_Check_WhoAmI( handle );
}


/**
 * @brief Get the temperature value of the HTS221 temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  return HTS221_Get_Temp( handle, temperature );
}


/**
 * @brief Get the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return HTS221_Get_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return HTS221_Set_ODR( handle, odr );
}


/**
 * @brief Set the HTS221 temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return HTS221_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef HTS221_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get temperature data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  HTS221_BitStatus_et hum_status_raw;
  HTS221_BitStatus_et temp_status_raw;

  if ( HTS221_Get_DataStatus( (void *)handle, &hum_status_raw, &temp_status_raw ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( temp_status_raw )
  {
    case HTS221_SET:
      *status = 1;
      break;
    case HTS221_RESET:
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

/** @addtogroup HTS221_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Init( DrvContextTypeDef *handle )
{

  if ( HTS221_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( HTS221_DeActivate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable BDU */
  if ( HTS221_Set_BduMode( (void *)handle, HTS221_ENABLE ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set default ODR */
  if ( HTS221_Set_ODR( handle, ODR_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Enable the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Power up the device */
  if ( HTS221_Activate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Power down the device */
  if ( HTS221_DeActivate( (void *)handle ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the HTS221 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( HTS221_Get_DeviceID( (void *)handle, who_am_i ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the HTS221 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( HTS221_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
 * @brief Get the humidity value of the HTS221 sensor
 * @param handle the device handle
 * @param humidity pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_Hum( DrvContextTypeDef *handle, float *humidity )
{

  uint16_t uint16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Humidity( (void *)handle, &uint16data ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *humidity = ( float )uint16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the HTS221 sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  int16_t int16data = 0;

  /* Read data from HTS221. */
  if ( HTS221_Get_Temperature( (void *)handle, &int16data ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *temperature = ( float )int16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  HTS221_Odr_et odr_low_level;

  if ( HTS221_Get_Odr( (void *)handle, &odr_low_level ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case HTS221_ODR_ONE_SHOT:
      *odr =  0.0f;
      break;
    case HTS221_ODR_1HZ     :
      *odr =  1.0f;
      break;
    case HTS221_ODR_7HZ     :
      *odr =  7.0f;
      break;
    case HTS221_ODR_12_5HZ  :
      *odr = 12.5f;
      break;
    default                 :
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  HTS221_Odr_et new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = HTS221_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_MID:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    case ODR_HIGH:
      new_odr = HTS221_ODR_12_5HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( HTS221_Set_Odr( (void *)handle, new_odr ) == HTS221_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the HTS221 sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef HTS221_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  HTS221_Odr_et new_odr;

  new_odr = ( odr <= 1.0f ) ? HTS221_ODR_1HZ
            : ( odr <= 7.0f ) ? HTS221_ODR_7HZ
            :                   HTS221_ODR_12_5HZ;

  if ( HTS221_Set_Odr( (void *)handle, new_odr ) == HTS221_ERROR )
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
static DrvStatusTypeDef HTS221_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( HTS221_ReadReg( (void *)handle, reg, 1, data ) == HTS221_ERROR )
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
static DrvStatusTypeDef HTS221_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( HTS221_WriteReg( (void *)handle, reg, 1, &data ) == HTS221_ERROR )
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
