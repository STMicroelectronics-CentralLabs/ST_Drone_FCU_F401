/**
 ******************************************************************************
 * @file    steval_fcu001_v1.c
 * @author  Competence Center Japan
 * @version V3.0.0 (MEMS library version)
 * @date    12-August-2016
 * @brief   This file provides STEVAL_FCU001_V1 board specific functions
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

#include "steval_fcu001_v1.h"


/** @addtogroup BSP
* @{
*/ 

/** @addtogroup STEVAL_FCU001_V1
* @{
*/

  /** @addtogroup STEVAL_FCU001_V1_LOW_LEVEL
  * @brief This file provides a set of low level firmware functions 
  * @{
  */

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Private_TypesDefinitions STEVAL_FCU001_V1_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL__Private_Defines STEVAL_FCU001_V1_LOW_LEVEL Private Defines
* @{
*/

//#define SYNCHRO_WAIT(NB)       for(int i=0; i<NB; i++){__asm("dsb\n");}
//#define SYNCHRO_SPI_DELAY     (1)

/**
* @brief STEVAL_FCU001_V1 BSP Driver version number V1.0.0
*/
#define __STEVAL_FCU001_V1_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STEVAL_FCU001_V1_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STEVAL_FCU001_V1_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STEVAL_FCU001_V1_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STEVAL_FCU001_V1_BSP_VERSION         ((__STEVAL_FCU001_V1_BSP_VERSION_MAIN << 24)\
|(__STEVAL_FCU001_V1_BSP_VERSION_SUB1 << 16)\
  |(__STEVAL_FCU001_V1_BSP_VERSION_SUB2 << 8 )\
    |(__STEVAL_FCU001_V1_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Private_Variables STEVAL_FCU001_V1_LOW_LEVEL Private Variables 
* @{
*/
//static uint32_t I2C_STEVAL_FCU001_V1_Timeout = STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
//static I2C_HandleTypeDef I2C_STEVAL_FCU001_V1_Handle;
static SPI_HandleTypeDef SPI_Sensor_Handle;
//static SPI_HandleTypeDef SPI_SD_Handle;

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};

const uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};


/**
* @}
*/


/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Private_Functions STEVAL_FCU001_V1_LOW_LEVEL Private Functions
* @{
*/ 
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead );
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);

/**
* @}
*/



/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Exported_Functions STEVAL_FCU001_V1_LOW_LEVEL Exported Functions
  * @{
  */

/**
* @brief  This method returns the STM32446E EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __STEVAL_FCU001_V1_BSP_VERSION;
}


/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}


/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
  
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
* @brief  Turns selected LED Off. 
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  if(HAL_SPI_GetState( &SPI_Sensor_Handle) == HAL_SPI_STATE_RESET )
  {
    STEVAL_FCU001_V1_SENSORS_SPI_CLK_ENABLE();
    STEVAL_FCU001_V1_SENSORS_SPI_GPIO_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = STEVAL_FCU001_V1_SENSORS_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(STEVAL_FCU001_V1_SENSORS_SPI_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = STEVAL_FCU001_V1_SENSORS_SPI_SCK_Pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STEVAL_FCU001_V1_SENSORS_SPI_Port, &GPIO_InitStruct);
    
    SPI_Sensor_Handle.Instance = STEVAL_FCU001_V1_SENSORS_SPI;
    SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_1LINE;
    SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    SPI_Sensor_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 2.5 MHz
    SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SPI_Sensor_Handle.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&SPI_Sensor_Handle);
    
    SPI_1LINE_TX(&SPI_Sensor_Handle);
    __HAL_SPI_ENABLE(&SPI_Sensor_Handle);
  }  
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Set all the pins before init to avoid glitch */
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin, GPIO_PIN_SET);
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  STEVAL_FCU001_V1_LSM6DSL_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin;
  HAL_GPIO_Init(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin, GPIO_PIN_SET);
    
  STEVAL_FCU001_V1_LIS2MDL_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin;
  HAL_GPIO_Init(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin, GPIO_PIN_SET);

  
  STEVAL_FCU001_V1_LPS22HB_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin;
  HAL_GPIO_Init(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);

  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Init(void *handle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSL:
    STEVAL_FCU001_V1_LSM6DSL_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin;
    /* Set the pin before init to avoid glitch */
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, &GPIO_InitStruct);
    break;
  case LIS2MDL:
    STEVAL_FCU001_V1_LIS2MDL_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin;
    /* Set the pin before init to avoid glitch */
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, &GPIO_InitStruct);
    break;  
  case LPS22HB:
    STEVAL_FCU001_V1_LPS22HB_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin;
    /* Set the pin before init to avoid glitch */
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, &GPIO_InitStruct);
    break;
  default:
    return COMPONENT_NOT_IMPLEMENTED;
  }
  return COMPONENT_OK;
}

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{  
   return Sensor_IO_SPI_Write( handle, WriteAddr, pBuffer, nBytesToWrite ); 
  //return COMPONENT_ERROR;
}


/**
 * @brief  Reads from the sensor to a buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 0)
  {
     
  if ( nBytesToRead > 1 ) 
    if (ctx->who_am_i == HTS221_WHO_AM_I_VAL)
        ReadAddr |= 0x80;  /* Enable I2C multi-bytes Write */
  
  }
  
  if(ctx->ifType == 1 )
  {
    if ( nBytesToRead > 1 ) {
      switch(ctx->who_am_i)
        {
          case LSM303AGR_ACC_WHO_AM_I: ReadAddr |= 0x40; break; /* Enable I2C multi-bytes Write */
          case LSM303AGR_MAG_WHO_AM_I: break;
          default:;
        }
    }
   return Sensor_IO_SPI_Read( handle, ReadAddr, pBuffer, nBytesToRead );
  }
  
  return COMPONENT_ERROR;
}



/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  uint8_t i;
  
// Select the correct device
  Sensor_IO_SPI_CS_Enable(handle);
  
  SPI_Write(&SPI_Sensor_Handle, WriteAddr);

  for(i=0;i<nBytesToWrite;i++)
  {
    SPI_Write(&SPI_Sensor_Handle, pBuffer[i]);
  }
// Deselect the device
  Sensor_IO_SPI_CS_Disable(handle);
  
  return COMPONENT_OK;
}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  /* Select the correct device */
  Sensor_IO_SPI_CS_Enable(handle);
  
  /* Write Reg Address */
  SPI_Write(&SPI_Sensor_Handle, ReadAddr | 0x80);

  /* Disable the SPI and change the data line to input */
  __HAL_SPI_DISABLE(&SPI_Sensor_Handle);
  SPI_1LINE_RX(&SPI_Sensor_Handle);

  /* Check if we need to read one byte or more */
  if(nBytesToRead > 1U)
  {
    SPI_Read_nBytes(&SPI_Sensor_Handle, pBuffer, nBytesToRead);
  }
  else
  {
    SPI_Read(&SPI_Sensor_Handle, pBuffer);
  }
  
  /* Deselect the device */
  Sensor_IO_SPI_CS_Disable(handle);  
  
  /* Change the data line to output and enable the SPI */
  SPI_1LINE_TX(&SPI_Sensor_Handle);
  __HAL_SPI_ENABLE(&SPI_Sensor_Handle);
  
  return COMPONENT_OK;
}


uint8_t Sensor_IO_SPI_CS_Enable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSL:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  case LIS2MDL:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  case LPS22HB:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  }
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Disable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSL:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port, STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LIS2MDL:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port, STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LPS22HB:
    HAL_GPIO_WritePin(STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port, STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  }
  return COMPONENT_OK;
}

/**
 * @brief  Configures sensor interrupts interface for LSM6DSL sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DSL_Sensor_IO_ITConfig( void )
{
  return COMPONENT_OK;
}

#if defined(__ICCARM__)
#pragma optimize=none
#endif

/**
 * @brief  This function reads a single byte on SPI 3-wire.
 * @param  xSpiHandle : SPI Handler.
 * @param  val : value.
 * @retval None
 */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */
  
  __disable_irq();
  //GPIOA->BSRR = (uint32_t)GPIO_PIN_8 << 16U;
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");__asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function reads multiple bytes on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @param  nBytesToRead: number of bytes to read.
 * @retval None
 */
void SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  
  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */
  
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  This function writes a single byte on SPI 3-wire.
 * @param  xSpiHandle: SPI Handler.
 * @param  val: value.
 * @retval None
 */
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
  
  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;
  
  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_SR_TXE) != SPI_SR_TXE);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
 * @brief  Configures sensor I2C interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_Init( void )
{
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
