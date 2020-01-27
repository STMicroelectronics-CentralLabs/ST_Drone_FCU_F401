/**
 *******************************************************************************
 * @file    LPS25HB_Driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LPS25HB sensor
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
#include "LPS25HB_Driver_HL.h"
#include <math.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS25HB LPS25HB
 * @{
 */

/** @addtogroup LPS25HB_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS25HB_P_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_P_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_P_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_P_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_P_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS25HB_P_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_P_Get_Press( DrvContextTypeDef *handle, float *pressure );
static DrvStatusTypeDef LPS25HB_P_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS25HB_P_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LPS25HB_P_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LPS25HB_P_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS25HB_P_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LPS25HB_P_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LPS25HB_T_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_T_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_T_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_T_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS25HB_T_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_T_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef LPS25HB_T_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS25HB_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LPS25HB_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LPS25HB_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS25HB_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LPS25HB_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LPS25HB_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS25HB_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS25HB_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS25HB_Get_Press( DrvContextTypeDef *handle, float *pressure );
static DrvStatusTypeDef LPS25HB_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef LPS25HB_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS25HB_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LPS25HB_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LPS25HB_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS25HB_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );

/**
 * @}
 */

/** @addtogroup LPS25HB_Private_Variables Private variables
 * @{
 */

/**
 * @brief LPS25HB pressure driver structure
 */
PRESSURE_Drv_t LPS25HB_P_Drv =
{
  LPS25HB_P_Init,
  LPS25HB_P_DeInit,
  LPS25HB_P_Sensor_Enable,
  LPS25HB_P_Sensor_Disable,
  LPS25HB_P_Get_WhoAmI,
  LPS25HB_P_Check_WhoAmI,
  LPS25HB_P_Get_Press,
  LPS25HB_P_Get_ODR,
  LPS25HB_P_Set_ODR,
  LPS25HB_P_Set_ODR_Value,
  LPS25HB_P_Read_Reg,
  LPS25HB_P_Write_Reg,
  LPS25HB_P_Get_DRDY_Status
};

/**
 * @brief LPS25HB temperature driver structure
 */
TEMPERATURE_Drv_t LPS25HB_T_Drv =
{
  LPS25HB_T_Init,
  LPS25HB_T_DeInit,
  LPS25HB_T_Sensor_Enable,
  LPS25HB_T_Sensor_Disable,
  LPS25HB_T_Get_WhoAmI,
  LPS25HB_T_Check_WhoAmI,
  LPS25HB_T_Get_Temp,
  LPS25HB_T_Get_ODR,
  LPS25HB_T_Set_ODR,
  LPS25HB_T_Set_ODR_Value,
  LPS25HB_T_Read_Reg,
  LPS25HB_T_Write_Reg,
  LPS25HB_T_Get_DRDY_Status
};

/**
 * @brief LPS25HB combo data structure definition
 */
LPS25HB_Combo_Data_t LPS25HB_Combo_Data[LPS25HB_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup LPS25HB_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LPS25HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Init( DrvContextTypeDef *handle )
{

  /* Check if the LPS25H/B temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if((((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(LPS25HB_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LPS25HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the LPS25H/B temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if((((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized == 0))
  {
    if(LPS25HB_P_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Enable the LPS25HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS25H/B temperature sensor is already enable. */
  /* If yes, skip the enable function, if not call enable function */
  if((((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(LPS25HB_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the LPS25HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS25H/B temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if((((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled == 0))
  {
    if(LPS25HB_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_P_Data_t *)(((PRESSURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LPS25HB pressure sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS25HB_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the LPS25HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS25HB_Check_WhoAmI( handle );
}


/**
 * @brief Get the pressure value of the LPS25HB pressure sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  return LPS25HB_Get_Press( handle, pressure );
}


/**
 * @brief Get the LPS25HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS25HB_Get_ODR( handle, odr );
}


/**
 * @brief Set the LPS25HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return LPS25HB_Set_ODR( handle, odr );
}


/**
 * @brief Set the LPS25HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return LPS25HB_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS25HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS25HB_P_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS25HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get pressure data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_P_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS25HB_DataStatus_st status_raw;

  if ( LPS25HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.PressDataAvailable;

  return COMPONENT_OK;
}


/**
 * @brief Initialize the LPS25HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Init( DrvContextTypeDef *handle )
{

  /* Check if the LPS25H/B pressure sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if((((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressInitialized ==
      0))
  {
    if(LPS25HB_Init(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Denitialize the LPS25HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_DeInit( DrvContextTypeDef *handle )
{

  /* Check if the LPS25H/B pressure sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if((((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressInitialized ==
      0))
  {
    if(LPS25HB_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Enable the LPS25HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS25H/B pressure sensor is already enable. */
  /* If yes, skip the enable function, if not call enable function */
  if((((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressEnabled == 0))
  {
    if(LPS25HB_Sensor_Enable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LPS25HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS25H/B pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if((((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isPressEnabled == 0))
  {
    if(LPS25HB_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  ((LPS25HB_T_Data_t *)(((TEMPERATURE_Data_t *)(handle->pData))->pComponentData))->comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LPS25HB temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS25HB_Get_WhoAmI( handle, who_am_i );
}



/**
 * @brief Check the WHO_AM_I ID of the LPS25HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS25HB_Check_WhoAmI( handle );
}


/**
 * @brief Get the temperature value of the LPS25HB temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  return LPS25HB_Get_Temp( handle, temperature );
}


/**
 * @brief Get the LPS25HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS25HB_Get_ODR( handle, odr );
}



/**
 * @brief Set the LPS25HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  return LPS25HB_Set_ODR( handle, odr );
}


/**
 * @brief Set the LPS25HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  return LPS25HB_Set_ODR_Value( handle, odr );
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS25HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS25HB_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS25HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS25HB_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS25HB_DataStatus_st status_raw;

  if ( LPS25HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.TempDataAvailable;

  return COMPONENT_OK;
}

/**
 * @}
 */

/** @addtogroup LPS25HB_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize the LPS25HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Init( DrvContextTypeDef *handle )
{

  if ( LPS25HB_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( LPS25HB_DeActivate( (void *)handle ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS25HB_Set_ODR( handle, ODR_LOW ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Enable interrupt circuit */
  if ( LPS25HB_Set_InterruptCircuitEnable( (void *)handle, LPS25HB_ENABLE ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set block data update mode */
  if ( LPS25HB_Set_Bdu( (void *)handle, LPS25HB_BDU_NO_UPDATE ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set SPI mode */
  if ( LPS25HB_Set_SpiInterface( (void *)handle, LPS25HB_SPI_3_WIRE ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set internal averaging sample counts for pressure and temperature */
  if ( LPS25HB_Set_Avg( (void *)handle, LPS25HB_AVGP_32, LPS25HB_AVGT_16 ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Enable the LPS25HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Sensor_Enable( DrvContextTypeDef *handle )
{

  /* Power up the device */
  if ( LPS25HB_Activate( (void *)handle ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Disable the LPS25HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Sensor_Disable( DrvContextTypeDef *handle )
{

  /* Power down the device */
  if ( LPS25HB_DeActivate( (void *)handle ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the LPS25HB sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LPS25HB_Get_DeviceID( (void *)handle, who_am_i ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LPS25HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LPS25HB_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
 * @brief Get the pressure value of the LPS25HB sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  int32_t int32data = 0;

  /* Read data from LPS25HB. */
  if ( LPS25HB_Get_Pressure( (void *)handle, &int32data ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *pressure = ( float )int32data / 100.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the LPS25HB sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  int16_t int16data = 0;

  /* Read data from LPS25HB. */
  if ( LPS25HB_Get_Temperature( (void *)handle, &int16data ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *temperature = ( float )int16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LPS25HB sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LPS25HB_Odr_et odr_low_level;

  if ( LPS25HB_Get_Odr( (void *)handle, &odr_low_level ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LPS25HB_ODR_ONE_SHOT:
      *odr =  0.0f;
      break;
    case LPS25HB_ODR_1HZ:
      *odr =  1.0f;
      break;
    case LPS25HB_ODR_7HZ:
      *odr =  7.0f;
      break;
    case LPS25HB_ODR_12_5HZ:
      *odr = 12.5f;
      break;
    case LPS25HB_ODR_25HZ:
      *odr = 25.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS25HB sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LPS25HB_Odr_et new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LPS25HB_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = LPS25HB_ODR_12_5HZ;
      break;
    case ODR_MID:
      new_odr = LPS25HB_ODR_25HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = LPS25HB_ODR_25HZ;
      break;
    case ODR_HIGH:
      new_odr = LPS25HB_ODR_25HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS25HB_Set_Odr( (void *)handle, new_odr ) == LPS25HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS25HB sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS25HB_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LPS25HB_Odr_et new_odr;

  new_odr = ( odr <=  1.0f ) ? LPS25HB_ODR_1HZ
            : ( odr <=  7.0f ) ? LPS25HB_ODR_7HZ
            : ( odr <= 12.5f ) ? LPS25HB_ODR_12_5HZ
            :                    LPS25HB_ODR_25HZ;

  if ( LPS25HB_Set_Odr( (void *)handle, new_odr ) == LPS25HB_ERROR )
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
static DrvStatusTypeDef LPS25HB_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS25HB_ReadReg( (void *)handle, reg, 1, data ) == LPS25HB_ERROR )
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
static DrvStatusTypeDef LPS25HB_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS25HB_WriteReg( (void *)handle, reg, 1, &data ) == LPS25HB_ERROR )
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
