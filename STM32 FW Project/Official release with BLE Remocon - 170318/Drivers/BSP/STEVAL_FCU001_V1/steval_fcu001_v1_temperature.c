/**
 ******************************************************************************
 * @file    steval_fcu001_v1_temperature.c
 * @author  Central Labs & Competence Center Japan
 * @version V1.2.0
 * @date    10-Nov-2016
 * @brief    This file provides the temperature sensor driver for the Sensor Tile
 *          board
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
#include "steval_fcu001_v1_temperature.h"



/** @addtogroup BSP 
 * @{
 */

/** @addtogroup STEVAL_FCU001V1 
 * @{
 */

/** @addtogroup STEVAL_FCU001V1_TEMPERATURE 
 * @brief This file provides a set of firmware functions to manage MEMS Temperature sensor
 * @{
 */

/** @addtogroup STEVAL_FCU001V1_TEMPERATURE_Private_Variables STEVAL_FCU001V1_TEMPERATURE Private variables
 * @{
 */

static DrvContextTypeDef TEMPERATURE_SensorHandle[ TEMPERATURE_SENSORS_MAX_NUM ];
static TEMPERATURE_Data_t TEMPERATURE_Data[ TEMPERATURE_SENSORS_MAX_NUM ]; // Temperature - all.
static LPS22HB_T_Data_t LPS22HB_T_0_Data; // Temperature - sensor 0 LPS22HB.

/**
 * @}
 */

/** @addtogroup STEVAL_FCU001V1_TEMPERATURE_Private_FunctionPrototypes STEVAL_FCU001V1_TEMPERATURE Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init( void **handle );


/**
 * @}
 */

/** @addtogroup STEVAL_FCU001V1_TEMPERATURE_Exported_Functions STEVAL_FCU001V1_TEMPERATURE Exported functions
 * @{
 */

/**
 * @brief Initialize a temperature sensor
 * @param id the temperature sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Init( TEMPERATURE_ID_t id, void **handle )
{

  *handle = NULL;
  
  if( BSP_LPS22HB_TEMPERATURE_Init(handle) == COMPONENT_ERROR )
        {
          return COMPONENT_ERROR;
  }
      
  return COMPONENT_OK;
}



static DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init( void **handle )
{
  TEMPERATURE_Drv_t *driver = NULL;
  uint8_t data = 0x01;
  
  if(TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }
  
  if ( Sensor_IO_SPI_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup sensor handle. */
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].who_am_i      = LPS22HB_WHO_AM_I_VAL;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].ifType        = 1; // SPI interface
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].address       = LPS22HB_ADDRESS_HIGH;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].spiDevice     = LPS22HB;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].instance      = LPS22HB_T_0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isEnabled     = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isCombo       = 1;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pData         = ( void * )&TEMPERATURE_Data[ LPS22HB_T_0 ];
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pVTable       = ( void * )&LPS22HB_T_Drv;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pExtVTable    = 0;
      
  LPS22HB_T_0_Data.comboData = &LPS22HB_Combo_Data[0];
  TEMPERATURE_Data[ LPS22HB_T_0 ].pComponentData = ( void * )&LPS22HB_T_0_Data;
  TEMPERATURE_Data[ LPS22HB_T_0 ].pExtData       = 0;
      
  *handle = (void *)&TEMPERATURE_SensorHandle[ LPS22HB_T_0 ];
  
  Sensor_IO_SPI_CS_Init(*handle);
  
  if(LPS22HB_Combo_Data[0].isPressInitialized == 0)
  { 
    // SPI Serial Interface Mode selection --> 3Wires
    if( Sensor_IO_Write(*handle, LPS22HB_CTRL_REG1, &data, 1) )
    {
      return COMPONENT_ERROR;
    }
    
    if(LPS22HB_SwResetAndMemoryBoot(*handle))
    {
      return COMPONENT_ERROR;
    }
    
    HAL_Delay(100);
    
    data = 0x01;
    
    if( Sensor_IO_Write(*handle, LPS22HB_CTRL_REG1, &data, 1) )
    {
      return COMPONENT_ERROR;
    }
  }
  
  driver = ( TEMPERATURE_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;
  
  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
  
}





/**
 * @brief Deinitialize a temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  memset(ctx, 0, sizeof(DrvContextTypeDef));
  
  *handle = NULL;
  
  return COMPONENT_OK;
}


/**
 * @brief Enable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Disable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isInitialized;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isEnabled;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isCombo;
  
  return COMPONENT_OK;
}


/**
 * @brief Get the temperature sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *instance = ctx->instance;
  
  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the temperature value
 * @param handle the device handle
 * @param temperature pointer where the value is written [C]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Temp( void *handle, float *temperature )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( temperature == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Temp == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Temp( ctx, temperature ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( odr == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_TEMPERATURE_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_TEMPERATURE_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_TEMPERATURE_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Get_DRDY_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_DRDY_Status( ctx, status ) == COMPONENT_ERROR )
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
