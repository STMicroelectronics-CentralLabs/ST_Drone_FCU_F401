/**
 *******************************************************************************
 * @file    LPS22HB_Driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file provides a set of high-level functions needed to manage
            the LPS22HB sensor
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
#include "LPS22HB_Driver_HL.h"
#include <math.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS22HB LPS22HB
 * @{
 */

/** @addtogroup LPS22HB_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS22HB_P_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_P_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_P_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_P_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_P_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS22HB_P_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_P_Get_Press( DrvContextTypeDef *handle, float *pressure );
static DrvStatusTypeDef LPS22HB_P_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS22HB_P_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LPS22HB_P_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LPS22HB_P_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS22HB_P_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LPS22HB_P_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

static DrvStatusTypeDef LPS22HB_T_Init( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_T_DeInit( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_T_Sensor_Enable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_T_Sensor_Disable( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS22HB_T_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_T_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef LPS22HB_T_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS22HB_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr );
static DrvStatusTypeDef LPS22HB_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr );
static DrvStatusTypeDef LPS22HB_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS22HB_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );
static DrvStatusTypeDef LPS22HB_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status );

/**
 * @}
 */

/** @addtogroup LPS22HB_Private_Function_Prototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef LPS22HB_Initialize( DrvContextTypeDef *handle, LPS22HB_Combo_Data_t *combo );
static DrvStatusTypeDef LPS22HB_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i );
static DrvStatusTypeDef LPS22HB_Check_WhoAmI( DrvContextTypeDef *handle );
static DrvStatusTypeDef LPS22HB_Get_Press( DrvContextTypeDef *handle, float *pressure );
static DrvStatusTypeDef LPS22HB_Get_Temp( DrvContextTypeDef *handle, float *temperature );
static DrvStatusTypeDef LPS22HB_Get_ODR( DrvContextTypeDef *handle, float *odr );
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo );
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo );
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo );
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo );
static DrvStatusTypeDef LPS22HB_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data );
static DrvStatusTypeDef LPS22HB_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data );

/**
 * @}
 */

/** @addtogroup LPS22HB_Callable_Private_Function_Ext_Prototypes Callable private function for extended features prototypes
 * @{
 */

static DrvStatusTypeDef LPS22HB_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Ovr_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Fth_Status( DrvContextTypeDef *handle, uint8_t *status );
static DrvStatusTypeDef LPS22HB_FIFO_Stop_On_Fth( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LPS22HB_FIFO_Usage( DrvContextTypeDef *handle, uint8_t status );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint8_t *nSamples );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Data( DrvContextTypeDef *handle, float *pressure, float *temperature );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Mode( DrvContextTypeDef *handle, uint8_t *mode );
static DrvStatusTypeDef LPS22HB_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode );
static DrvStatusTypeDef LPS22HB_FIFO_Get_Watermark_Level( DrvContextTypeDef *handle, uint8_t *watermark );
static DrvStatusTypeDef LPS22HB_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint8_t watermark );
static DrvStatusTypeDef LPS22HB_FIFO_Watermark_Usage( DrvContextTypeDef *handle, uint8_t usage );
static DrvStatusTypeDef LPS22HB_FIFO_Set_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt );
static DrvStatusTypeDef LPS22HB_FIFO_Reset_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt );

/**
 * @}
 */

/** @addtogroup LPS22HB_Private_Variables Private variables
 * @{
 */

/**
 * @brief LPS22HB pressure driver structure
 */
PRESSURE_Drv_t LPS22HB_P_Drv =
{
  LPS22HB_P_Init,
  LPS22HB_P_DeInit,
  LPS22HB_P_Sensor_Enable,
  LPS22HB_P_Sensor_Disable,
  LPS22HB_P_Get_WhoAmI,
  LPS22HB_P_Check_WhoAmI,
  LPS22HB_P_Get_Press,
  LPS22HB_P_Get_ODR,
  LPS22HB_P_Set_ODR,
  LPS22HB_P_Set_ODR_Value,
  LPS22HB_P_Read_Reg,
  LPS22HB_P_Write_Reg,
  LPS22HB_P_Get_DRDY_Status
};

/**
 * @brief LPS22HB temperature driver structure
 */
TEMPERATURE_Drv_t LPS22HB_T_Drv =
{
  LPS22HB_T_Init,
  LPS22HB_T_DeInit,
  LPS22HB_T_Sensor_Enable,
  LPS22HB_T_Sensor_Disable,
  LPS22HB_T_Get_WhoAmI,
  LPS22HB_T_Check_WhoAmI,
  LPS22HB_T_Get_Temp,
  LPS22HB_T_Get_ODR,
  LPS22HB_T_Set_ODR,
  LPS22HB_T_Set_ODR_Value,
  LPS22HB_T_Read_Reg,
  LPS22HB_T_Write_Reg,
  LPS22HB_T_Get_DRDY_Status
};

/**
 * @brief LPS22HB pressure extended features driver internal structure
 */
LPS22HB_P_ExtDrv_t LPS22HB_P_ExtDrv =
{
  LPS22HB_FIFO_Get_Empty_Status,
  LPS22HB_FIFO_Get_Full_Status,
  LPS22HB_FIFO_Get_Ovr_Status,
  LPS22HB_FIFO_Get_Fth_Status,
  LPS22HB_FIFO_Stop_On_Fth,
  LPS22HB_FIFO_Usage,
  LPS22HB_FIFO_Get_Num_Of_Samples,
  LPS22HB_FIFO_Get_Data,
  LPS22HB_FIFO_Get_Mode,
  LPS22HB_FIFO_Set_Mode,
  LPS22HB_FIFO_Get_Watermark_Level,
  LPS22HB_FIFO_Set_Watermark_Level,
  LPS22HB_FIFO_Watermark_Usage,
  LPS22HB_FIFO_Set_Interrupt,
  LPS22HB_FIFO_Reset_Interrupt
};

/**
 * @brief LPS22HB temperature extended features driver internal structure
 */
LPS22HB_T_ExtDrv_t LPS22HB_T_ExtDrv =
{
  LPS22HB_FIFO_Get_Empty_Status,
  LPS22HB_FIFO_Get_Full_Status,
  LPS22HB_FIFO_Get_Ovr_Status,
  LPS22HB_FIFO_Get_Fth_Status,
  LPS22HB_FIFO_Stop_On_Fth,
  LPS22HB_FIFO_Usage,
  LPS22HB_FIFO_Get_Num_Of_Samples,
  LPS22HB_FIFO_Get_Data,
  LPS22HB_FIFO_Get_Mode,
  LPS22HB_FIFO_Set_Mode,
  LPS22HB_FIFO_Get_Watermark_Level,
  LPS22HB_FIFO_Set_Watermark_Level,
  LPS22HB_FIFO_Watermark_Usage,
  LPS22HB_FIFO_Set_Interrupt,
  LPS22HB_FIFO_Reset_Interrupt
};

/**
 * @brief LPS22HB combo data structure definition
 */
LPS22HB_Combo_Data_t LPS22HB_Combo_Data[LPS22HB_SENSORS_MAX_NUM];

/**
 * @}
 */

/** @addtogroup LPS22HB_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief Initialize the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Init( DrvContextTypeDef *handle )
{
  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB temperature sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if(comboData->isTempInitialized == 0)
  {
    if(LPS22HB_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_DeInit( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB temperature sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if(comboData->isTempInitialized == 0)
  {
    if(LPS22HB_P_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Enable the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Sensor_Enable( DrvContextTypeDef *handle )
{
  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  if(LPS22HB_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isPressEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}


/**
 * @brief Disable the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Sensor_Disable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HB temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if(comboData->isTempEnabled == 0)
  {
    /* Power down the device */
    if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isPressEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LPS22HB pressure sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS22HB_Get_WhoAmI( handle, who_am_i );
}


/**
 * @brief Check the WHO_AM_I ID of the LPS22HB pressure sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS22HB_Check_WhoAmI( handle );
}


/**
 * @brief Get the pressure value of the LPS22HB pressure sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  return LPS22HB_Get_Press( handle, pressure );
}


/**
 * @brief Get the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS22HB_Get_ODR( handle, odr );
}


/**
 * @brief Set the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB pressure sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_P_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_P_Data_t *)(((PRESSURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HB_P_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_P_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_P_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_DataStatus_st status_raw;

  if ( LPS22HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.PressDataAvailable;

  return COMPONENT_OK;
}


/**
 * @brief Initialize the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Init( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB pressure sensor is already initialized. */
  /* If yes, skip the initialize function, if not call initialize function */
  if(comboData->isPressInitialized == 0)
  {
    if(LPS22HB_Initialize(handle, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 1;

  handle->isInitialized = 1;

  return COMPONENT_OK;
}


/**
 * @brief Denitialize the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_DeInit( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the LPS22HB pressure sensor is already initialized. */
  /* If yes, skip the deinitialize function, if not call deinitialize function */
  if(comboData->isPressInitialized == 0)
  {
    if(LPS22HB_T_Sensor_Disable(handle) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempInitialized = 0;

  handle->isInitialized = 0;

  return COMPONENT_OK;
}


/**
 * @brief Enable the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Sensor_Enable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already enabled */
  if ( handle->isEnabled == 1 )
  {
    return COMPONENT_OK;
  }

  if(LPS22HB_Set_ODR_Value_When_Enabled(handle, comboData->Last_ODR, comboData) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  comboData->isTempEnabled = 1;

  handle->isEnabled = 1;

  return COMPONENT_OK;
}



/**
 * @brief Disable the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Sensor_Disable( DrvContextTypeDef *handle )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  /* Check if the component is already disabled */
  if ( handle->isEnabled == 0 )
  {
    return COMPONENT_OK;
  }

  /* Check if the LPS22HB pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if(comboData->isPressEnabled == 0)
  {
    /* Power down the device */
    if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
    {
      return COMPONENT_ERROR;
    }
  }

  comboData->isTempEnabled = 0;

  handle->isEnabled = 0;

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LPS22HB temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  return LPS22HB_Get_WhoAmI( handle, who_am_i );
}



/**
 * @brief Check the WHO_AM_I ID of the LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Check_WhoAmI( DrvContextTypeDef *handle )
{

  return LPS22HB_Check_WhoAmI( handle );
}


/**
 * @brief Get the temperature value of the LPS22HB temperature sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  return LPS22HB_Get_Temp( handle, temperature );
}


/**
 * @brief Get the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  return LPS22HB_Get_ODR( handle, odr );
}



/**
 * @brief Set the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Set_ODR( DrvContextTypeDef *handle, SensorOdr_t odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_T_Set_ODR_Value( DrvContextTypeDef *handle, float odr )
{

  LPS22HB_Combo_Data_t *comboData = ((LPS22HB_T_Data_t *)(((TEMPERATURE_Data_t *)(
                                       handle->pData))->pComponentData))->comboData;

  if(handle->isEnabled == 1)
  {
    if(LPS22HB_Set_ODR_Value_When_Enabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
  }
  else
  {
    if(LPS22HB_Set_ODR_Value_When_Disabled(handle, odr, comboData) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }
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
static DrvStatusTypeDef LPS22HB_T_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_Read_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_T_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_Write_Reg( handle, reg, data ) == COMPONENT_ERROR )
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
static DrvStatusTypeDef LPS22HB_T_Get_DRDY_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_DataStatus_st status_raw;

  if ( LPS22HB_Get_DataStatus( (void *)handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.TempDataAvailable;

  return COMPONENT_OK;
}


/**
 * @}
 */

/** @addtogroup LPS22HB_Private_Functions Private functions
 * @{
 */

/**
 * @brief Initialize the LPS22HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Initialize( DrvContextTypeDef *handle, LPS22HB_Combo_Data_t *combo )
{
  if( LPS22HB_Set_SpiInterface((void *)handle, LPS22HB_SPI_3_WIRE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Check_WhoAmI( handle ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  combo->Last_ODR = 25.0f;

  /* Set Power mode */
  if ( LPS22HB_Set_PowerMode( (void *)handle, LPS22HB_LowPower) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Power down the device */
  if ( LPS22HB_Set_Odr( (void *)handle, LPS22HB_ODR_ONE_SHOT ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if( LPS22HB_Set_LowPassFilter( (void *)handle, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set low-pass filter cutoff configuration*/
  if( LPS22HB_Set_LowPassFilterCutoff( (void *)handle, LPS22HB_ODR_9) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Set block data update mode */
  if ( LPS22HB_Set_Bdu( (void *)handle, LPS22HB_BDU_NO_UPDATE ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Disable automatic increment for multi-byte read/write */
  if( LPS22HB_Set_AutomaticIncrementRegAddress( (void *)handle, LPS22HB_DISABLE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  if( LPS22HB_Set_SpiInterface((void *)handle, LPS22HB_SPI_3_WIRE) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the LPS22HB sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_WhoAmI( DrvContextTypeDef *handle, uint8_t *who_am_i )
{

  /* Read WHO AM I register */
  if ( LPS22HB_Get_DeviceID( (void *)handle, who_am_i ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the LPS22HB sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Check_WhoAmI( DrvContextTypeDef *handle )
{

  uint8_t who_am_i = 0x00;

  if ( LPS22HB_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
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
 * @brief Get the pressure value of the LPS22HB sensor
 * @param handle the device handle
 * @param pressure pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_Press( DrvContextTypeDef *handle, float *pressure )
{

  int32_t int32data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Pressure( (void *)handle, &int32data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *pressure = ( float )int32data / 100.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the temperature value of the LPS22HB sensor
 * @param handle the device handle
 * @param temperature pointer where the value is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_Temp( DrvContextTypeDef *handle, float *temperature )
{

  int16_t int16data = 0;

  /* Read data from LPS22HB. */
  if ( LPS22HB_Get_Temperature( (void *)handle, &int16data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *temperature = ( float )int16data / 10.0f;

  return COMPONENT_OK;
}

/**
 * @brief Get the LPS22HB sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Get_ODR( DrvContextTypeDef *handle, float *odr )
{

  LPS22HB_Odr_et odr_low_level;

  if ( LPS22HB_Get_Odr( (void *)handle, &odr_low_level ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  switch( odr_low_level )
  {
    case LPS22HB_ODR_ONE_SHOT:
      *odr = 0.0f;
      break;
    case LPS22HB_ODR_1HZ:
      *odr = 1.0f;
      break;
    case LPS22HB_ODR_10HZ:
      *odr = 10.0f;
      break;
    case LPS22HB_ODR_25HZ:
      *odr = 25.0f;
      break;
    case LPS22HB_ODR_50HZ:
      *odr = 50.0f;
      break;
    case LPS22HB_ODR_75HZ:
      *odr = 75.0f;
      break;
    default:
      *odr = -1.0f;
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Enabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo )
{

  LPS22HB_Odr_et new_odr;

  switch( odr )
  {
    case ODR_LOW:
      new_odr = LPS22HB_ODR_1HZ;
      break;
    case ODR_MID_LOW:
      new_odr = LPS22HB_ODR_10HZ;
      break;
    case ODR_MID:
      new_odr = LPS22HB_ODR_25HZ;
      break;
    case ODR_MID_HIGH:
      new_odr = LPS22HB_ODR_50HZ;
      break;
    case ODR_HIGH:
      new_odr = LPS22HB_ODR_75HZ;
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_Odr( (void *)handle, new_odr ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_ODR( handle, &combo->Last_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_When_Disabled( DrvContextTypeDef *handle, SensorOdr_t odr,
    LPS22HB_Combo_Data_t *combo )
{

  switch( odr )
  {
    case ODR_LOW:
      combo->Last_ODR = 1.0f;
      break;
    case ODR_MID_LOW:
      combo->Last_ODR = 10.0f;
      break;
    case ODR_MID:
      combo->Last_ODR = 25.0f;
      break;
    case ODR_MID_HIGH:
      combo->Last_ODR = 50.0f;
      break;
    case ODR_HIGH:
      combo->Last_ODR = 75.0f;
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the LPS22HB sensor output data rate when enabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Enabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo )
{

  LPS22HB_Odr_et new_odr;

  new_odr = ( odr <=  1.0f ) ? LPS22HB_ODR_1HZ
            : ( odr <= 10.0f ) ? LPS22HB_ODR_10HZ
            : ( odr <= 25.0f ) ? LPS22HB_ODR_25HZ
            : ( odr <= 50.0f ) ? LPS22HB_ODR_50HZ
            :                    LPS22HB_ODR_75HZ;

  if ( LPS22HB_Set_Odr( (void *)handle, new_odr ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_ODR( handle, &combo->Last_ODR ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the LPS22HB sensor output data rate when disabled
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @param combo the pointer to the combo shared structure
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef LPS22HB_Set_ODR_Value_When_Disabled( DrvContextTypeDef *handle, float odr,
    LPS22HB_Combo_Data_t *combo )
{

  combo->Last_ODR = ( odr <=  1.0f ) ? 1.0f
                    : ( odr <= 10.0f ) ? 10.0f
                    : ( odr <= 25.0f ) ? 25.0f
                    : ( odr <= 50.0f ) ? 50.0f
                    :                    75.0f;

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
static DrvStatusTypeDef LPS22HB_Read_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t *data )
{

  if ( LPS22HB_ReadReg( (void *)handle, reg, 1, data ) == LPS22HB_ERROR )
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
static DrvStatusTypeDef LPS22HB_Write_Reg( DrvContextTypeDef *handle, uint8_t reg, uint8_t data )
{

  if ( LPS22HB_WriteReg( (void *)handle, reg, 1, &data ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @}
 */

/** @addtogroup LPS22HB_Callable_Private_Functions_Ext Callable private functions for extended features
 * @{
 */

/**
 * @brief Get the FIFO_EMPTY status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_EMPTY
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Empty_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_EMPTY;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_FULL status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_FULL
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Full_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_FULL;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_OVR status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Ovr_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_OVR;

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO_FTH status
 * @param handle the device handle
 * @param status the pointer where the status is stored. 1 means FIFO_FTH
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Fth_Status( DrvContextTypeDef *handle, uint8_t *status )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *status = status_raw.FIFO_FTH;

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO to stop on FTH interrupt
 * @param handle the device handle
 * @param status enable or disable stopping on FTH interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Stop_On_Fth( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )status )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoWatermarkLevelUse( handle, ( LPS22HB_State_et )status ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief FIFO usage
 * @param handle the device handle
 * @param status enable or disable FIFO
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Usage( DrvContextTypeDef *handle, uint8_t status )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )status )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoModeUse( handle, ( LPS22HB_State_et )status ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the number of FIFO unread samples
 * @param handle the device handle
 * @param nSamples the pointer where the number of FIFO unread samples is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Num_Of_Samples( DrvContextTypeDef *handle, uint8_t *nSamples )
{

  LPS22HB_FifoStatus_st status_raw;

  if ( LPS22HB_Get_FifoStatus( handle, &status_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *nSamples = status_raw.FIFO_LEVEL;

  return COMPONENT_OK;
}

/**
 * @brief Get the oldest pressure and temperature sample from the FIFO
 * @param handle the device handle
 * @param pressure the pointer where the pressure part of FIFO sample is stored
 * @param temperature the pointer where the temperature part of FIFO sample is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Data( DrvContextTypeDef *handle, float *pressure, float *temperature )
{

  if ( LPS22HB_Get_Press( handle, pressure ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  if ( LPS22HB_Get_Temp( handle, temperature ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO mode
 * @param handle the device handle
 * @param mode the pointer where the FIFO mode is stored
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Mode( DrvContextTypeDef *handle, uint8_t *mode )
{

  LPS22HB_FifoMode_et mode_raw;

  if ( LPS22HB_Get_FifoMode( handle, &mode_raw ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  *mode = ( uint8_t )mode_raw;

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO mode
 * @param handle the device handle
 * @param mode The FIFO mode to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Mode( DrvContextTypeDef *handle, uint8_t mode )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_FifoMode_et )mode )
  {
    case LPS22HB_FIFO_BYPASS_MODE:
    case LPS22HB_FIFO_MODE:
    case LPS22HB_FIFO_STREAM_MODE:
    case LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE:
    case LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE:
    case LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoMode( handle, ( LPS22HB_FifoMode_et )mode ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Get the FIFO watermark level
 * @param handle the device handle
 * @param watermark the pointer where the FIFO watermark level is stored; values: from 0 to 31
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Get_Watermark_Level( DrvContextTypeDef *handle, uint8_t *watermark )
{

  if ( LPS22HB_Get_FifoWatermarkLevel( handle, watermark ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO watermark level
 * @param handle the device handle
 * @param watermark The FIFO watermark level to be set; values: from 0 to 31
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Watermark_Level( DrvContextTypeDef *handle, uint8_t watermark )
{

  if ( LPS22HB_Set_FifoWatermarkLevel( handle, watermark ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief The FIFO watermark enable or disable
 * @param handle the device handle
 * @param usage The FIFO watermark enable or disable
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Watermark_Usage( DrvContextTypeDef *handle, uint8_t usage )
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ( ( LPS22HB_State_et )usage )
  {
    case LPS22HB_DISABLE:
    case LPS22HB_ENABLE:
      break;
    default:
      return COMPONENT_ERROR;
  }

  if ( LPS22HB_Set_FifoWatermarkLevelUse( handle, ( LPS22HB_State_et )usage ) == LPS22HB_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Set the FIFO interrupt
 * @param handle the device handle
 * @param interrupt The FIFO interrupt to be set; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Set_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt )
{

  switch( interrupt )
  {
    case 0:
      if ( LPS22HB_Set_FIFO_FTH_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 1:
      if ( LPS22HB_Set_FIFO_FULL_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 2:
      if ( LPS22HB_Set_FIFO_OVR_Interrupt( handle, LPS22HB_ENABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    default:
      return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}

/**
 * @brief Reset the FIFO interrupt
 * @param handle the device handle
 * @param interrupt The FIFO interrupt to be reset; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
*/
static DrvStatusTypeDef LPS22HB_FIFO_Reset_Interrupt( DrvContextTypeDef *handle, uint8_t interrupt )
{

  switch( interrupt )
  {
    case 0:
      if ( LPS22HB_Set_FIFO_FTH_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 1:
      if ( LPS22HB_Set_FIFO_FULL_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    case 2:
      if ( LPS22HB_Set_FIFO_OVR_Interrupt( handle, LPS22HB_DISABLE ) == LPS22HB_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    default:
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
