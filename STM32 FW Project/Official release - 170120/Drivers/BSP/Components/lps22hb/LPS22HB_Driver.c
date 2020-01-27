/**
 *******************************************************************************
 * @file    LPS22HB_Driver.c
 * @author  HESA Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   LPS22HB driver file
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
#include "LPS22HB_Driver.h"
#ifdef  USE_FULL_ASSERT_LPS22HB
#include <stdio.h>
#endif

/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup LPS22HB_DRIVER
* @brief LPS22HB DRIVER
* @{
*/

/** @defgroup LPS22HB_Imported_Function_Prototypes
* @{
*/

extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

/**
* @}
*/

/** @defgroup LPS22HB_Private_Function_Prototypes
* @{
*/

/**
* @}
*/

/** @defgroup LPS22HB_Private_Functions
* @{
*/

/**
* @}
*/

/** @defgroup LPS22HB_Public_Functions
* @{
*/

/*******************************************************************************
* Function Name   : LPS22HB_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data Read
* Return      : None
*******************************************************************************/
LPS22HB_Error_et LPS22HB_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
  int i = 0;

  for (i = 0; i < NumByteToRead; i++ )
  {
    if( Sensor_IO_Read(handle, RegAddr + i, &Data[i], 1 ))
      return LPS22HB_ERROR;
  }
  
  return LPS22HB_OK;
  
  /*if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return LPS22HB_ERROR;
  else
    return LPS22HB_OK;
  */
}

/*******************************************************************************
* Function Name   : LPS22HB_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
LPS22HB_Error_et LPS22HB_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{
  int i = 0;

  for (i = 0; i < NumByteToWrite; i++ )
  {
    if( Sensor_IO_Write(handle, RegAddr + i, &Data[i], 1 ))
      return LPS22HB_ERROR;
  }
  
  return LPS22HB_OK;
}

/**
* @brief  Read identification code by WHO_AM_I register
* @param  *handle Device handle.
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(LPS22HB_ReadReg(handle, LPS22HB_WHO_AM_I_REG, 1, deviceid))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get the LPS22HB driver version.
* @param  None
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DriverVersion(LPS22HB_DriverVersion_st *Version)
{
  Version->Major = LPS22HB_DriverVersion_Major;
  Version->Minor = LPS22HB_DriverVersion_Minor;
  Version->Point = LPS22HB_DriverVersion_Point;

  return LPS22HB_OK;
}


/**
* @brief  Set LPS22HB Low Power or Low Noise Mode Configuration
* @param  *handle Device handle.
* @param  LPS22HB_LowNoise or LPS22HB_LowPower mode
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PowerMode(void *handle, LPS22HB_PowerMode_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_PowerMode(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LCEN_MASK;
  tmp |= (uint8_t)mode;

  if(LPS22HB_WriteReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get LPS22HB Power Mode
* @param  *handle Device handle.
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PowerMode(void *handle, LPS22HB_PowerMode_et* mode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_RES_CONF_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *mode = (LPS22HB_PowerMode_et)(tmp & LPS22HB_LCEN_MASK);

  return LPS22HB_OK;
}


/**
* @brief  Set LPS22HB Output Data Rate
* @param  *handle Device handle.
* @param  Output Data Rate
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_Odr(void *handle, LPS22HB_Odr_et odr)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_ODR(odr));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get LPS22HB Output Data Rate
* @param  *handle Device handle.
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Odr(void *handle, LPS22HB_Odr_et* odr)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *odr = (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Enable/Disale low-pass filter on LPS22HB pressure data
* @param  *handle Device handle.
* @param  state: enable or disable
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_LowPassFilter(void *handle, LPS22HB_State_et state)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(state));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_MASK;
  tmp |= ((uint8_t)state) << LPS22HB_LPFP_BIT;


  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Set low-pass filter cutoff configuration on LPS22HB pressure data
* @param  *handle Device handle.
* @param  Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_LowPassFilterCutoff(void *handle, LPS22HB_LPF_Cutoff_et cutoff)
{

  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_LPF_Cutoff(cutoff));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LPFP_CUTOFF_MASK;
  tmp |= (uint8_t)cutoff;



  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;

}

/**
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to ‘1’.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param  *handle Device handle.
* @param  LPS22HB_BDU_CONTINUOUS_UPDATE, LPS22HB_BDU_NO_UPDATE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

LPS22HB_Error_et LPS22HB_Set_Bdu(void *handle, LPS22HB_Bdu_et bdu)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_BDUMode(bdu));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_BDU_MASK;
  tmp |= ((uint8_t)bdu);


  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_OK;

  return LPS22HB_OK;
}

/**
* @brief  Get Block Data Mode
* @param  *handle Device handle.
* @param Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Bdu(void *handle, LPS22HB_Bdu_et* bdu)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *bdu = (LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);

  return LPS22HB_OK;
}

/**
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param  *handle Device handle.
* @param LPS22HB_SPI_3_WIRE, LPS22HB_SPI_4_WIRE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_SpiInterface(void *handle, LPS22HB_SPIMode_et spimode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_SPIMode(spimode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_SIM_MASK;
  tmp |= (uint8_t)spimode;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Clock Tree Configuration
* @param  *handle Device handle.
* @param  LPS22HB_CTE_NotBalanced, LPS22HB_CTE_ABalanced
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_ClockTreeConfifuration(void *handle, LPS22HB_CTE_et mode)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_CTE(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_CTE_MASK;
  tmp |= (uint8_t)mode;


  if(LPS22HB_WriteReg(handle, LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param  *handle Device handle.
* @param Buffet to empty with spi mode read from Sensor
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_SpiInterface(void *handle, LPS22HB_SPIMode_et* spimode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  *spimode = (LPS22HB_SPIMode_et)(tmp & LPS22HB_SIM_MASK);

  return LPS22HB_OK;
}

/**
* @brief   Software Reset. Self-clearing upon completion
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwReset(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01 << LPS22HB_SW_RESET_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Reboot Memory Content
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/

LPS22HB_Error_et LPS22HB_MemoryBoot(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= (0x01 << LPS22HB_BOOT_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Software Reset ann Reboot Memory Content.
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to ‘1’
 + and BOOT is set to ‘1’; Self-clearing upon completion.
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_SwResetAndMemoryBoot(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((0x01 << LPS22HB_SW_RESET_BIT) | (0x01 << LPS22HB_BOOT_BIT));

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable FIFO Mode
* @param  *handle Device handle.
* @param LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoModeUse(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_EN_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}
/**
* @brief   Enable/Disable FIFO Watermark Level Use
* @param  *handle Device handle.
* @param   LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevelUse(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_EN_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_WTM_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE. Default is LPS22HB_ENABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutomaticIncrementRegAddress(void *handle, LPS22HB_State_et status)
{

  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_ADD_INC_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_ADD_INC_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;

}

/**
* @brief  Enable/Disable I2C Interface
* @param  *handle Device handle.
* @param State: LPS22HB_ENABLE (reset bit)/ LPS22HB_DISABLE (set bit)
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_I2C(void *handle, LPS22HB_State_et statei2c)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(statei2c));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /*Reset Bit->I2C Enabled*/
  tmp &= ~LPS22HB_I2C_MASK;
  tmp |= ((uint8_t)~statei2c) << LPS22HB_I2C_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set the one-shot bit in order to start acquisition when the ONE SHOT mode
*          has been selected by the ODR configuration.
* @detail  Once the measurement is done, ONE_SHOT bit will self-clear.
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_StartOneShotMeasurement(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= LPS22HB_ONE_SHOT_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;

}

/**
* @brief  Set Interrupt Active on High or Low Level
* @param  *handle Device handle.
* @param  LPS22HB_ActiveHigh/LPS22HB_ActiveLow
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptActiveLevel(void *handle, LPS22HB_InterruptActiveLevel_et mode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_InterruptActiveLevel(mode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_INT_H_L_MASK;
  tmp |= ((uint8_t)mode);

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param  *handle Device handle.
* @param   LPS22HB_PushPull/LPS22HB_OpenDrain
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptOutputType(void *handle, LPS22HB_OutputType_et output)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputType(output));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PP_OD_MASK;
  tmp |= (uint8_t)output;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Data signal on INT pad control bits.
* @param  *handle Device handle.
* @param  LPS22HB_DATA,LPS22HB_P_HIGH_LPS22HB_P_LOW,LPS22HB_P_LOW_HIGH
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptControlConfig(void *handle, LPS22HB_OutputSignalConfig_et config)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_OutputSignal(config));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~(LPS22HB_INT_S12_MASK);
  tmp |= (uint8_t)config;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_DRDYInterrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DRDY_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_DRDY_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_OVR_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;


  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_OVR_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_OVR_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_FTH_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FTH_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_FTH_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FIFO_FULL_Interrupt(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_FULL_MASK;
  tmp |= ((uint8_t)status) << LPS22HB_FIFO_FULL_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



/**
* @brief   Enable AutoRifP function
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*               the AutoRifP is slf creared.
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutoRifP(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= ((uint8_t)LPS22HB_AUTORIFP_MASK);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Disable AutoRifP function
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_ResetAutoRifP(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  tmp |= ((uint8_t)LPS22HB_RESET_ARP_MASK);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param  *handle Device handle.
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_AutoZeroFunction(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp |= LPS22HB_AUTOZERO_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/*
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  *handle Device handle.
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_ResetAutoZeroFunction(void *handle)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= LPS22HB_RESET_AZ_MASK;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE,LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et diff_en)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(diff_en));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en) << LPS22HB_DIFF_EN_BIT;

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief  Get the DIFF_EN bit value
* @param  *handle Device handle.
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialGeneration(void *handle, LPS22HB_State_et* diff_en)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  *diff_en = (LPS22HB_State_et)((tmp & LPS22HB_DIFF_EN_MASK) >> LPS22HB_DIFF_EN_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_LatchInterruptRequest(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_LIR_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_LIR_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}



/**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PLE(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PLE_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_PLE_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  *handle Device handle.
* @param  LPS22HB_ENABLE/LPS22HB_DISABLE
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PHE(void *handle, LPS22HB_State_et status)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_State(status));

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_PHE_MASK;
  tmp |= (((uint8_t)status) << LPS22HB_PHE_BIT);

  if(LPS22HB_WriteReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param  *handle Device handle.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptDifferentialEventStatus(void *handle,
    LPS22HB_InterruptDiffStatus_st* interruptsource)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_SOURCE_REG, 1, &tmp))
    return LPS22HB_ERROR;

  interruptsource->PH = (uint8_t)(tmp & LPS22HB_PH_MASK);
  interruptsource->PL = (uint8_t)((tmp & LPS22HB_PL_MASK) >> LPS22HB_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & LPS22HB_IA_MASK) >> LPS22HB_IA_BIT);
  interruptsource->BOOT = (uint8_t)((tmp & LPS22HB_BOOT_STATUS_MASK) >> LPS22HB_BOOT_STATUS_BIT);

  return LPS22HB_OK;
}

/**
* @brief  Get the status of Pressure and Temperature data
* @param  *handle Device handle.
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_DataStatus(void *handle, LPS22HB_DataStatus_st* datastatus)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus->PressDataAvailable = (uint8_t)(tmp & LPS22HB_PDA_MASK);
  datastatus->TempDataAvailable = (uint8_t)((tmp & LPS22HB_TDA_MASK) >> LPS22HB_PDA_BIT);
  datastatus->TempDataOverrun = (uint8_t)((tmp & LPS22HB_TOR_MASK) >> LPS22HB_TOR_BIT);
  datastatus->PressDataOverrun = (uint8_t)((tmp & LPS22HB_POR_MASK) >> LPS22HB_POR_BIT);

  return LPS22HB_OK;
}



/**
* @brief  Get the LPS22HB raw presure value
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  *handle Device handle.
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_RawPressure(void *handle, int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;
  uint8_t i;

  if(LPS22HB_ReadReg(handle, LPS22HB_PRESS_OUT_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(i = 0; i < 3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8 * i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
    tmp |= 0xFF000000;

  *raw_press = ((int32_t)tmp);

  return LPS22HB_OK;
}

/**
* @brief    Get the LPS22HB Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  *handle Device handle.
* @param      The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Pressure(void *handle, int32_t* Pout)
{
  int32_t raw_press;

  if(LPS22HB_Get_RawPressure(handle, &raw_press))
    return LPS22HB_ERROR;

  *Pout = (raw_press * 100) / 4096;

  return LPS22HB_OK;
}

/**
* @brief    Get the Raw Temperature value.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*            Tout(degC)=TEMP_OUT/100
* @param  *handle Device handle.
* @param     Buffer to empty with the temperature raw tmp.
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_RawTemperature(void *handle, int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_TEMP_OUT_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

  *raw_data = ((int16_t)tmp);

  return LPS22HB_OK;
}


/**
* @brief    Get the Temperature value in °C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*           Tout(degC)=TEMP_OUT/100
* @param  *handle Device handle.
* @param Buffer to empty with the temperature value that must be divided by 10 to get the value in °C
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Temperature(void *handle, int16_t* Tout)
{
  int16_t raw_data;

  if(LPS22HB_Get_RawTemperature(handle, &raw_data))
    return LPS22HB_ERROR;

  *Tout = (raw_data * 10) / 100;

  return LPS22HB_OK;
}

/**
* @brief    Get the threshold value used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param  *handle Device handle.
* @param    Buffer to empty with the pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureThreshold(void *handle, int16_t* P_ths)
{
  uint8_t tempReg[2];

  if(LPS22HB_ReadReg(handle, LPS22HB_THS_P_LOW_REG, 2, tempReg))
    return LPS22HB_ERROR;

  *P_ths = (((((uint16_t)tempReg[1]) << 8) + tempReg[0]) / 16);

  return LPS22HB_OK;
}

/**
* @brief    Set the threshold value  used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param  *handle Device handle.
* @param    Pressure threshold in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_PressureThreshold(void *handle, int16_t P_ths)
{
  uint8_t buffer[2];

  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths)) >> 8);

  if(LPS22HB_WriteReg(handle, LPS22HB_THS_P_LOW_REG, 2, buffer))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo Mode.
* @param  *handle Device handle.
* @param  Fifo Mode struct
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoMode(void *handle, LPS22HB_FifoMode_et fifomode)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_FifoMode(fifomode));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_FIFO_MODE_MASK;
  tmp |= (uint8_t)fifomode;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief    Get Fifo Mode
* @param  *handle Device handle.
* @param   buffer to empty with fifo mode tmp
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoMode(void *handle, LPS22HB_FifoMode_et* fifomode)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= LPS22HB_FIFO_MODE_MASK;
  *fifomode = (LPS22HB_FifoMode_et)tmp;

  return LPS22HB_OK;
}

/**
* @brief    Set Fifo Watermark Level.
* @param  *handle Device handle.
* @param    Watermark level value [0 31]
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel)
{
  uint8_t tmp;

  LPS22HB_assert_param(IS_LPS22HB_WtmLevel(wtmlevel));

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  tmp &= ~LPS22HB_WTM_POINT_MASK;
  tmp |= wtmlevel;

  if(LPS22HB_WriteReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief   Get FIFO Watermark Level
* @param  *handle Device handle.
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel)
{
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, wtmlevel))
    return LPS22HB_ERROR;

  *wtmlevel &= LPS22HB_WTM_POINT_MASK;

  return LPS22HB_OK;
}

/**
* @brief    Get the Fifo Status
* @param  *handle Device handle.
* @param    Status Flag: FIFO_FTH,FIFO_EMPTY,FIFO_FULL,FIFO_OVR and level of the FIFO->FIFO_LEVEL
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoStatus(void *handle, LPS22HB_FifoStatus_st* status)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  status->FIFO_FTH = (uint8_t)((tmp & LPS22HB_FTH_FIFO_MASK) >> LPS22HB_FTH_FIFO_BIT);
  status->FIFO_OVR = (uint8_t)((tmp & LPS22HB_OVR_FIFO_MASK) >> LPS22HB_OVR_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & LPS22HB_LEVEL_FIFO_MASK);

  if(status->FIFO_LEVEL == LPS22HB_FIFO_EMPTY)
    status->FIFO_EMPTY = 0x01;
  else
    status->FIFO_EMPTY = 0x00;

  if (status->FIFO_LEVEL == LPS22HB_FIFO_FULL)
    status->FIFO_FULL = 0x01;
  else
    status->FIFO_FULL = 0x00;


  return LPS22HB_OK;
}

/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param  *handle Device handle.
* @param buffer to empty with the he pressure value (hPA)
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_PressureOffsetValue(void *handle, int16_t *pressoffset)
{
  uint8_t buffer[2];
  int16_t raw_press;

  if(LPS22HB_ReadReg(handle, LPS22HB_RPDS_L_REG, 2, buffer))
    return LPS22HB_ERROR;

  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);

  *pressoffset = (raw_press * 100) / 4096;

  return LPS22HB_OK;
}


/**
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  *handle Device handle.
* @param  Buffer to empty with reference pressure value
* @retval  Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_ReferencePressure(void *handle, int32_t* RefP)
{
  uint8_t buffer[3];
  uint32_t tempVal = 0;
  int32_t raw_press;
  uint8_t i;

  if(LPS22HB_ReadReg(handle, LPS22HB_REF_P_XL_REG, 3, buffer))
    return LPS22HB_ERROR;

  /* Build the raw data */
  for(i = 0; i < 3; i++)
    tempVal |= (((uint32_t)buffer[i]) << (8 * i));

  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;

  raw_press = ((int32_t)tempVal);
  *RefP = (raw_press * 100) / 4096;


  return LPS22HB_OK;
}


/**
* @brief  Check if the single measurement has completed.
* @param  *handle Device handle.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_IsMeasurementCompleted(void *handle, uint8_t* Is_Measurement_Completed)
{
  uint8_t tmp;
  LPS22HB_DataStatus_st datastatus;

  if(LPS22HB_ReadReg(handle, LPS22HB_STATUS_REG, 1, &tmp))
    return LPS22HB_ERROR;

  datastatus.TempDataAvailable = (uint8_t)((tmp & LPS22HB_TDA_MASK) >> LPS22HB_TDA_BIT);
  datastatus.PressDataAvailable = (uint8_t)(tmp & LPS22HB_PDA_MASK);

  *Is_Measurement_Completed = (uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));

  return LPS22HB_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  *handle Device handle.
* @param  Pressure and temperature tmp
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Measurement(void *handle, LPS22HB_MeasureTypeDef_st *Measurement_Value)
{
  int16_t Tout;
  int32_t Pout;

  if(LPS22HB_Get_Temperature(handle, &Tout))
    return LPS22HB_ERROR;

  Measurement_Value->Tout = Tout;

  if(LPS22HB_Get_Pressure(handle, &Pout))
    return LPS22HB_ERROR;

  Measurement_Value->Pout = Pout;

  return LPS22HB_OK;

}

/**
* @brief  Initialization function for LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*        NO FIFO; NO Interrupt Enabled.
* @param  *handle Device handle.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Init(void *handle)
{
  LPS22HB_ConfigTypeDef_st pLPS22HBInit;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot(handle))
    return LPS22HB_ERROR;

  pLPS22HBInit.PowerMode = LPS22HB_LowPower;
  pLPS22HBInit.OutputDataRate = LPS22HB_ODR_25HZ;
  pLPS22HBInit.LowPassFilter = LPS22HB_DISABLE;
  pLPS22HBInit.LPF_Cutoff = LPS22HB_ODR_9;
  pLPS22HBInit.BDU = LPS22HB_BDU_NO_UPDATE;
  pLPS22HBInit.IfAddInc = LPS22HB_ENABLE; //default
  pLPS22HBInit.Sim = LPS22HB_SPI_4_WIRE;

  /* Set Generic Configuration*/
  if(LPS22HB_Set_GenericConfig(handle, &pLPS22HBInit))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  De initialization function for LPS22HB.
*         This function make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_DeInit(void *handle)
{
  LPS22HB_MeasureTypeDef_st Measurement_Value;

  /* Make LPS22HB Reset and Reboot */
  if(LPS22HB_SwResetAndMemoryBoot(handle))
    return LPS22HB_ERROR;

  /* Dump of data output */
  if(LPS22HB_Get_Measurement(handle, &Measurement_Value))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}


/**
* @brief   Set Generic Configuration
* @param  *handle Device handle.
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS22HB_Set_PowerMode(handle, pxLPS22HBInit->PowerMode))
    return LPS22HB_ERROR;

  /* Init the Output Data Rate*/
  if(LPS22HB_Set_Odr(handle, pxLPS22HBInit->OutputDataRate))
    return LPS22HB_ERROR;

  /* BDU bit is used to inhibit the output registers update between the reading of upper and
  lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
  updated continuously. If it is not sure to read faster than output data rate, it is recommended
  to set BDU bit to ‘1’. In this way, after the reading of the lower (upper) register part, the
  content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/

  if(LPS22HB_Set_Bdu(handle, pxLPS22HBInit->BDU))
    return LPS22HB_ERROR;

  /*Enable/Disale low-pass filter on LPS22HB pressure data*/
  if(LPS22HB_Set_LowPassFilter(handle, pxLPS22HBInit->LowPassFilter))
    return LPS22HB_ERROR;

  /* Set low-pass filter cutoff configuration*/
  if(LPS22HB_Set_LowPassFilterCutoff(handle, pxLPS22HBInit->LPF_Cutoff))
    return LPS22HB_ERROR;

  /* SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/

  if(LPS22HB_Set_SpiInterface(handle, pxLPS22HBInit->Sim))
    return LPS22HB_ERROR;

  /*Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)*/
  if(LPS22HB_Set_AutomaticIncrementRegAddress(handle, pxLPS22HBInit->IfAddInc))
    return LPS22HB_ERROR;


  return LPS22HB_OK;
}

/**
* @brief  Get Generic configuration
* @param  *handle Device handle.
* @param  Struct to empty with configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_GenericConfig(void *handle, LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{

  uint8_t tmp;

  /*Read LPS22HB_RES_CONF_REG*/
  if(LPS22HB_Get_PowerMode(handle, &pxLPS22HBInit->PowerMode))
    return LPS22HB_ERROR;

  /*Read LPS22HB_CTRL_REG1*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG1, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->OutputDataRate = (LPS22HB_Odr_et)(tmp & LPS22HB_ODR_MASK);
  pxLPS22HBInit->BDU = (LPS22HB_Bdu_et)(tmp & LPS22HB_BDU_MASK);
  pxLPS22HBInit->Sim = (LPS22HB_SPIMode_et)(tmp & LPS22HB_SIM_MASK);
  pxLPS22HBInit->LowPassFilter = (LPS22HB_State_et)((tmp & LPS22HB_LPFP_MASK) >> LPS22HB_LPFP_BIT);
  pxLPS22HBInit->LPF_Cutoff = (LPS22HB_LPF_Cutoff_et)(tmp & LPS22HB_LPFP_CUTOFF_MASK);

  /*Read LPS22HB_CTRL_REG2*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  pxLPS22HBInit->IfAddInc = (LPS22HB_State_et)((tmp & LPS22HB_ADD_INC_MASK) >> LPS22HB_ADD_INC_BIT);

  return LPS22HB_OK;
}


/**
* @brief  Set Interrupt configuration
* @param  *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS22HB_Set_InterruptActiveLevel(handle, pLPS22HBInt->INT_H_L))
    return LPS22HB_ERROR;

  /* Push-pull/open drain selection on interrupt pads.*/
  if(LPS22HB_Set_InterruptOutputType(handle, pLPS22HBInt->PP_OD))
    return LPS22HB_ERROR;

  /* Set Data signal on INT pad control bits.*/
  if(LPS22HB_Set_InterruptControlConfig(handle, pLPS22HBInt->OutputSignal_INT))
    return LPS22HB_ERROR;

  /* Enable/Disable Data-ready signal on INT_DRDY pin. */
  if(LPS22HB_Set_DRDYInterrupt(handle, pLPS22HBInt->DRDY))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO overrun interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_OVR_Interrupt(handle, pLPS22HBInt->FIFO_OVR))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO Treshold interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, pLPS22HBInt->FIFO_FTH))
    return LPS22HB_ERROR;

  /* Enable/Disable FIFO FULL interrupt on INT_DRDY pin. */
  if(LPS22HB_Set_FIFO_FULL_Interrupt(handle, pLPS22HBInt->FIFO_FULL))
    return LPS22HB_ERROR;

  /* Latch Interrupt request to the INT_SOURCE register. */
  if(LPS22HB_LatchInterruptRequest(handle, pLPS22HBInt->LatchIRQ))
    return LPS22HB_ERROR;

  /* Set the threshold value  used for pressure interrupt generation (hPA). */
  if(LPS22HB_Set_PressureThreshold(handle, pLPS22HBInt->THS_threshold))
    return LPS22HB_ERROR;

  /*Enable/Disable  AutoRifP function */
  if(pLPS22HBInt->AutoRifP == LPS22HB_ENABLE)
  {
    if(LPS22HB_Set_AutoRifP(handle))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_ResetAutoRifP(handle))
      return LPS22HB_ERROR;
  }

  /*Enable/Disable AutoZero function*/
  if(pLPS22HBInt->AutoZero == LPS22HB_ENABLE)
  {
    if(LPS22HB_Set_AutoZeroFunction(handle))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_ResetAutoZeroFunction(handle))
      return LPS22HB_ERROR;
  }


  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_HIGH)
  {
    /* Enable\Disable Interrupt Generation on differential pressure high event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_LOW)
  {
    /* Enable Interrupt Generation on differential pressure Loe event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else  if(pLPS22HBInt->OutputSignal_INT == LPS22HB_P_LOW_HIGH)
  {
    /* Enable Interrupt Generation on differential pressure high event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    /* Enable\Disable Interrupt Generation on differential pressure Loe event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;
  }
  else
  {
    if(LPS22HB_Set_InterruptDifferentialGeneration(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Disable Interrupt Generation on differential pressure High event*/
    if(LPS22HB_Set_PHE(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Disable Interrupt Generation on differential pressure Low event*/
    if(LPS22HB_Set_PLE(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
* @brief  LPS22HBGet_InterruptConfig
* @param  *handle Device handle.
* @param  Struct to empty with configuration values
* @retval S Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_InterruptConfig(void *handle, LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  uint8_t tmp;

  /*Read LPS22HB_CTRL_REG3*/
  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG3, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->INT_H_L = (LPS22HB_InterruptActiveLevel_et)(tmp & LPS22HB_INT_H_L_MASK);
  pLPS22HBInt->PP_OD = (LPS22HB_OutputType_et)(tmp & LPS22HB_PP_OD_MASK);
  pLPS22HBInt->OutputSignal_INT = (LPS22HB_OutputSignalConfig_et)(tmp & LPS22HB_INT_S12_MASK);
  pLPS22HBInt->DRDY = (LPS22HB_State_et)((tmp & LPS22HB_DRDY_MASK) >> LPS22HB_DRDY_BIT);
  pLPS22HBInt->FIFO_OVR = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_OVR_MASK) >> LPS22HB_FIFO_OVR_BIT);
  pLPS22HBInt->FIFO_FTH = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_FTH_MASK) >> LPS22HB_FIFO_FTH_BIT);
  pLPS22HBInt->FIFO_FULL = (LPS22HB_State_et)((tmp & LPS22HB_FIFO_FULL_MASK) >> LPS22HB_FIFO_FULL_BIT);

  /*Read LPS22HB_INTERRUPT_CFG_REG*/
  if(LPS22HB_ReadReg(handle, LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS22HB_ERROR;

  pLPS22HBInt->LatchIRQ = (LPS22HB_State_et)((tmp & LPS22HB_LIR_MASK) >> LPS22HB_LIR_BIT);

  if(LPS22HB_Get_PressureThreshold(handle, &pLPS22HBInt->THS_threshold))
    return LPS22HB_ERROR;

  //AutoRifP and Autozero are self clear //
  pLPS22HBInt->AutoRifP = LPS22HB_DISABLE;
  pLPS22HBInt->AutoZero = LPS22HB_DISABLE;

  return LPS22HB_OK;
}

/**
* @brief  Set Fifo configuration
* @param  *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Set_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{

  if(pLPS22HBFIFO->FIFO_MODE == LPS22HB_FIFO_BYPASS_MODE)
  {
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
    /* Force->Disable FIFO Watermark Level Use*/
    if(LPS22HB_Set_FifoWatermarkLevelUse(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;

    /* Force->Disable FIFO Treshold interrupt on INT_DRDY pin. */
    if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, LPS22HB_DISABLE))
      return LPS22HB_ERROR;
  }
  else
  {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(LPS22HB_Set_FifoModeUse(handle, LPS22HB_ENABLE))
      return LPS22HB_ERROR;

    if (pLPS22HBFIFO->WTM_INT)
    {
      /* Enable FIFO Watermark Level Use*/
      if(LPS22HB_Set_FifoWatermarkLevelUse(handle, LPS22HB_ENABLE))
        return LPS22HB_ERROR;
      /*Set Fifo Watermark Level*/
      if(LPS22HB_Set_FifoWatermarkLevel(handle, pLPS22HBFIFO->WTM_LEVEL))
        return LPS22HB_ERROR;
      /* Force->Enable FIFO Treshold interrupt on INT_DRDY pin. */
      if(LPS22HB_Set_FIFO_FTH_Interrupt(handle, LPS22HB_ENABLE))
        return LPS22HB_ERROR;
    }
  }

  if(LPS22HB_Set_FifoMode(handle, pLPS22HBFIFO->FIFO_MODE))
    return LPS22HB_ERROR;

  return LPS22HB_OK;
}

/**
* @brief  Get Fifo configuration
* @param  *handle Device handle.
* @param  Struct to empty with the configuration values
* @retval Error code[LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_FifoConfig(void *handle, LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{
  uint8_t tmp;

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return LPS22HB_ERROR;

  /*!< Fifo Mode Selection */
  pLPS22HBFIFO->FIFO_MODE = (LPS22HB_FifoMode_et)(tmp & LPS22HB_FIFO_MODE_MASK);

  /*!< FIFO threshold/Watermark level selection*/
  pLPS22HBFIFO->WTM_LEVEL = (uint8_t)(tmp & LPS22HB_WTM_POINT_MASK);

  if(LPS22HB_ReadReg(handle, LPS22HB_CTRL_REG2, 1, &tmp))
    return LPS22HB_ERROR;

  /*!< Enable/Disable the watermark interrupt*/
  pLPS22HBFIFO->WTM_INT = (LPS22HB_State_et)((tmp & LPS22HB_WTM_EN_MASK) >> LPS22HB_WTM_EN_BIT);


  return LPS22HB_OK;
}

#ifdef  USE_FULL_ASSERT_LPS22HB
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void LPS22HB_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters tmp: file %s on line %d\r\n", file, (int)line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
