/**
 ******************************************************************************
 * @file    steval_fcu001_v1.c
 * @author  MEMS Application Team, Competence Center Japan
 * @version V3.0.0, Competence Center Japan
 * @date    12-August-2016
 * @brief   This file provides a set of functions needed to manage the gyroscope sensor
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
#include "steval_fcu001_v1_gyro.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1 STEVAL_FCU001_V1
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO Gyro
 * @{
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef GYRO_SensorHandle[ GYRO_SENSORS_MAX_NUM ];
static GYRO_Data_t GYRO_Data[ GYRO_SENSORS_MAX_NUM ]; // Gyroscope - all.
static LSM6DSL_G_Data_t LSM6DSL_G_0_Data; // Gyroscope - sensor 0.
/**
 * @}
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Private_FunctionPrototypes Private function prototypes
 * @{
 */
static DrvStatusTypeDef BSP_LSM6DSL_GYRO_Init( void **handle );
/**
 * @}
 */

/** @addtogroup STEVAL_FCU001_V1_GYRO_Public_Functions Public functions
 * @{
 */

/**
 * @brief Initialize a gyroscope sensor
 * @param id the gyroscope sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Init( GYRO_ID_t id, void **handle )
{

  *handle = NULL;

  switch(id)
  {
    case GYRO_SENSORS_AUTO:
    default:
    {
      /* Try to init the LSM6DSL first */
      if( BSP_LSM6DSL_GYRO_Init(handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LSM6DSL_G_0:
    {
      if( BSP_LSM6DSL_GYRO_Init(handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}


static DrvStatusTypeDef BSP_LSM6DSL_GYRO_Init( void **handle )
{
  GYRO_Drv_t *driver = NULL;
  uint8_t data = 0x0C;
  
  if(GYRO_SensorHandle[ LSM6DSL_G_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }
  
  if ( Sensor_IO_SPI_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup sensor handle. */
  /* Gyroscope - sensor 0 */
  GYRO_SensorHandle[ LSM6DSL_G_0 ].who_am_i      = LSM6DSL_ACC_GYRO_WHO_AM_I;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].ifType        = 1; // SPI interface
  GYRO_SensorHandle[ LSM6DSL_G_0 ].address       = LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].spiDevice     = LSM6DSL;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].instance      = LSM6DSL_G_0;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].isInitialized = 0;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].isEnabled     = 0;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].isCombo       = 1;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].pData         = ( void * )&GYRO_Data[ LSM6DSL_G_0 ];
  GYRO_SensorHandle[ LSM6DSL_G_0 ].pVTable       = ( void * )&LSM6DSL_G_Drv;
  GYRO_SensorHandle[ LSM6DSL_G_0 ].pExtVTable    = 0;

  LSM6DSL_G_0_Data.comboData = &LSM6DSL_Combo_Data[0];
  GYRO_Data[ LSM6DSL_G_0 ].pComponentData = ( void * )&LSM6DSL_G_0_Data;
  GYRO_Data[ LSM6DSL_G_0 ].pExtData       = 0;
  
  *handle = (void *)&GYRO_SensorHandle[ LSM6DSL_G_0 ];
  
  Sensor_IO_SPI_CS_Init(*handle);
  
  if(LSM6DSL_Combo_Data[0].isAccInitialized == 0)
  { 
    // SPI Serial Interface Mode selection --> 3Wires
    if( Sensor_IO_Write(*handle, LSM6DSL_ACC_GYRO_CTRL3_C, &data, 1) )
    {
      return COMPONENT_ERROR;
    }
  }
  
  driver = ( GYRO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;
  
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
  
  /* Disable I2C interface */
  if ( LSM6DSL_ACC_GYRO_W_I2C_DISABLE( *handle, LSM6DSL_ACC_GYRO_I2C_DISABLE_SPI_ONLY ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Deinitialize a gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_DeInit( void **handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Enable gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Disable gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Check if the gyroscope sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_IsInitialized( void *handle, uint8_t *status )
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
 * @brief Check if the gyroscope sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_IsEnabled( void *handle, uint8_t *status )
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
 * @brief Check if the gyroscope sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_IsCombo( void *handle, uint8_t *status )
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
 * @brief Get the gyroscope sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_Instance( void *handle, uint8_t *instance )
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
 * @brief Get the WHO_AM_I ID of the gyroscope sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Check the WHO_AM_I ID of the gyroscope sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Get the gyroscope sensor axes
 * @param handle the device handle
 * @param angular_velocity pointer where the values of the axes are written [mdps]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_Axes( void *handle, SensorAxes_t *angular_velocity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( angular_velocity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes( ctx, angular_velocity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the gyroscope sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( value == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_AxesRaw( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the gyroscope sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [mdps/LSB]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_Sensitivity( void *handle, float *sensitivity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Set the gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Set the gyroscope sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Get the gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_FS( void *handle, float *fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( fullScale == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_FS( void *handle, SensorFs_t fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the gyroscope sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_FS_Value( void *handle, float fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( driver->Set_FS_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Get the gyroscope sensor axes status
 * @param handle the device handle
 * @param xyz_enabled the pointer to the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( xyz_enabled == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes_Status( ctx, xyz_enabled ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief Set the enabled/disabled status of the gyroscope sensor axes
 * @param handle the device handle
 * @param enable_xyz vector of the axes enabled/disabled status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_Axes_Status( void *handle, uint8_t *enable_xyz )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

  if ( enable_xyz == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_Axes_Status == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_Axes_Status( ctx, enable_xyz ) == COMPONENT_ERROR )
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
DrvStatusTypeDef BSP_GYRO_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
DrvStatusTypeDef BSP_GYRO_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Get gyroscope data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GYRO_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( GYRO_Drv_t * )ctx->pVTable;

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
 * @brief Set FIFO output data rate (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param odr the output data rate
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_ODR_Value_Ext( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_ODR_Value == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_ODR_Value( handle, odr );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO full status (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *status FIFO full status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Full_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO empty status (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *status FIFO empty status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Empty_Status_Ext( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Empty_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Empty_Status( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO_OVR bit status (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *status FIFO_OVR bit status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Overrun_Status_Ext( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Overrun_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Overrun_Status( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO pattern (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *pattern FIFO pattern
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Pattern_Ext( void *handle, uint16_t *pattern )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( pattern == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Pattern == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Pattern( handle, pattern );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO data (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *aData FIFO data array
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Data_Ext( void *handle, uint8_t *aData )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( aData == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Data == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data( handle, aData );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get number of unread FIFO samples (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Num_Of_Samples_Ext( void *handle, uint16_t *nSamples )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( nSamples == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Num_Of_Samples == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Num_Of_Samples( handle, nSamples );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO decimation for gyroscope (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param decimation FIFO decimation for gyroscope
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Decimation_Ext( void *handle, uint8_t decimation )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_G_Set_Decimation == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_G_Set_Decimation( handle, decimation );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get single gyro axis from the FIFO (available only for LSM6DSL sensor)
 * @param handle the device handle
 * @param angular_velocity the pointer to the angular velocity value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Get_Axis_Ext( void *handle, int32_t *angular_velocity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( angular_velocity == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_G_Get_Axis == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_G_Get_Axis( handle, angular_velocity );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO mode
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Mode_Ext( void *handle, uint8_t mode )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Mode == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Mode( handle, mode );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO_FULL interrupt on INT1 pin
 * @param handle the device handle
 * @param status FIFO_FULL interrupt on INT1 pin enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_INT1_FIFO_Full_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_INT1_FIFO_Full == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_INT1_FIFO_Full( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO watermark level
 * @param handle the device handle
 * @param watermark FIFO watermark level
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Watermark_Level_Ext( void *handle, uint16_t watermark )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Watermark_Level == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level( handle, watermark );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO to stop on FTH interrupt
 * @param handle the device handle
 * @param status FIFO stop on FTH interrupt enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_FIFO_Set_Stop_On_Fth_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Stop_On_Fth == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Stop_On_Fth( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set gyro interrupt latch
 * @param handle the device handle
 * @param status interrupt latch enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_Interrupt_Latch_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->Set_Interrupt_Latch == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_Interrupt_Latch( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set gyro self-test
 * @param handle the device handle
 * @param status self-test enable/disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GYRO_Set_SelfTest_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LSM6DSL */
  if ( ctx->who_am_i == LSM6DSL_ACC_GYRO_WHO_AM_I )
  {
    LSM6DSL_G_ExtDrv_t *extDriver = ( LSM6DSL_G_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->Set_SelfTest == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->Set_SelfTest( handle, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
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
