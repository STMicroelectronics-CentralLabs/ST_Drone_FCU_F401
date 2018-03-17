/**
 *******************************************************************************
 * @file    LIS2MDL_MAG_driver.c
 * @author  MEMS Application Team
 * @version v1.0
 * @date    13-October-2016
 * @brief   LIS2MDL Magnetometer driver file
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "LIS2MDL_MAG_driver.h"

/* Imported function prototypes ----------------------------------------------*/
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name    : LIS2MDL_MAG_WriteReg
* Description    : Generic Writing function. It must be fullfilled with either
*          : I2C or SPI writing function
* Input        : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
status_t LIS2MDL_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  if (Sensor_IO_Write(handle, Reg, Bufp, len))
  {
    return MEMS_ERROR;
  }
  else
  {
    return MEMS_SUCCESS;
  }
}

/*******************************************************************************
* Function Name    : LIS2MDL_MAG_ReadReg
* Description    : Generic Reading function. It must be fullfilled with either
*          : I2C or SPI reading functions          
* Input        : Register Address
* Output      : Data REad
* Return      : None
*******************************************************************************/
status_t LIS2MDL_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{

  if (Sensor_IO_Read(handle, Reg, Bufp, len))
  {
    return MEMS_ERROR;
  }
  else
  {
    return MEMS_SUCCESS;
  }
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_WhoAmI_Bits
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_WhoAmI_Bits(void *handle, u8_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_WHO_AM_I, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_WHO_AM_I_BIT_MASK; //coerce  
  *value = *value >> LIS2MDL_MAG_WHO_AM_I_BIT_POSITION; //mask  

  return MEMS_SUCCESS;
}

/************** Configuration Functions  *******************/

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_Operating_Mode
* Description    : Write MD
* Input          : LIS2MDL_MAG_MD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_Operating_Mode(void *handle, LIS2MDL_MAG_MD_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_MD_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_Operating_Mode
* Description    : Read MD
* Input          : Pointer to LIS2MDL_MAG_MD_t
* Output         : Status of MD see LIS2MDL_MAG_MD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_Operating_Mode(void *handle, LIS2MDL_MAG_MD_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_MD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_DataRate
* Description    : Write ODR
* Input          : LIS2MDL_MAG_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_DataRate(void *handle, LIS2MDL_MAG_ODR_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_ODR_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_DataRate
* Description    : Read ODR
* Input          : Pointer to LIS2MDL_MAG_ODR_t
* Output         : Status of ODR see LIS2MDL_MAG_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_DataRate(void *handle, LIS2MDL_MAG_ODR_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_ODR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_PowerMode
* Description    : Write LP
* Input          : LIS2MDL_MAG_LP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_PowerMode(void *handle, LIS2MDL_MAG_LP_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_LP_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_PowerMode
* Description    : Read LP
* Input          : Pointer to LIS2MDL_MAG_LP_t
* Output         : Status of LP see LIS2MDL_MAG_LP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_PowerMode(void *handle, LIS2MDL_MAG_LP_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_LP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_SoftwareReset
* Description    : Write SOFT_RST
* Input          : LIS2MDL_MAG_SOFT_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_SoftwareReset(void *handle, LIS2MDL_MAG_SOFT_RST_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_SOFT_RST_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_SoftwareReset
* Description    : Read SOFT_RST
* Input          : Pointer to LIS2MDL_MAG_SOFT_RST_t
* Output         : Status of SOFT_RST see LIS2MDL_MAG_SOFT_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_SoftwareReset(void *handle, LIS2MDL_MAG_SOFT_RST_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_SOFT_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_Reboot
* Description    : Write REBOOT
* Input          : LIS2MDL_MAG_REBOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_Reboot(void *handle, LIS2MDL_MAG_REBOOT_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_REBOOT_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_Reboot
* Description    : Read REBOOT
* Input          : Pointer to LIS2MDL_MAG_REBOOT_t
* Output         : Status of REBOOT see LIS2MDL_MAG_REBOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_Reboot(void *handle, LIS2MDL_MAG_REBOOT_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_REBOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_TemperatureCompensation
* Description    : Write COMP_TEMP
* Input          : LIS2MDL_MAG_COMP_TEMP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_TemperatureCompensation(void *handle, LIS2MDL_MAG_COMP_TEMP_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_COMP_TEMP_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_TemperatureCompensation
* Description    : Read COMP_TEMP
* Input          : Pointer to LIS2MDL_MAG_COMP_TEMP_t
* Output         : Status of COMP_TEMP see LIS2MDL_MAG_COMP_TEMP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_TemperatureCompensation(void *handle, LIS2MDL_MAG_COMP_TEMP_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_COMP_TEMP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_LowPassFilter
* Description    : Write LPF
* Input          : LIS2MDL_MAG_LPF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_LowPassFilter(void *handle, LIS2MDL_MAG_LPF_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_LPF_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_LowPassFilter
* Description    : Read LPF
* Input          : Pointer to LIS2MDL_MAG_LPF_t
* Output         : Status of LPF see LIS2MDL_MAG_LPF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_LowPassFilter(void *handle, LIS2MDL_MAG_LPF_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_LPF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_OffsetCancellation
* Description    : Write OFF_CANC
* Input          : LIS2MDL_MAG_OFF_CANC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_OffsetCancellation(void *handle, LIS2MDL_MAG_OFF_CANC_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_OFF_CANC_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_OffsetCancellation
* Description    : Read OFF_CANC
* Input          : Pointer to LIS2MDL_MAG_OFF_CANC_t
* Output         : Status of OFF_CANC see LIS2MDL_MAG_OFF_CANC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_OffsetCancellation(void *handle, LIS2MDL_MAG_OFF_CANC_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_OFF_CANC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_PulseFrequancy
* Description    : Write SET_FREQ
* Input          : LIS2MDL_MAG_SET_FREQ_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_PulseFrequancy(void *handle, LIS2MDL_MAG_SET_FREQ_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_SET_FREQ_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_PulseFrequancy
* Description    : Read SET_FREQ
* Input          : Pointer to LIS2MDL_MAG_SET_FREQ_t
* Output         : Status of SET_FREQ see LIS2MDL_MAG_SET_FREQ_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_PulseFrequancy(void *handle, LIS2MDL_MAG_SET_FREQ_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_SET_FREQ_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptThresholdMode
* Description    : Write INT_ON_DATAOFF
* Input          : LIS2MDL_MAG_INT_ON_DATAOFF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptThresholdMode(void *handle, LIS2MDL_MAG_INT_ON_DATAOFF_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_INT_ON_DATAOFF_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptThresholdMode
* Description    : Read INT_ON_DATAOFF
* Input          : Pointer to LIS2MDL_MAG_INT_ON_DATAOFF_t
* Output         : Status of INT_ON_DATAOFF see LIS2MDL_MAG_INT_ON_DATAOFF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptThresholdMode(void *handle, LIS2MDL_MAG_INT_ON_DATAOFF_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_INT_ON_DATAOFF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_OffsetCancellationInSingleMode
* Description    : Write OFF_CANC_ONE_SHOT
* Input          : LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_OffsetCancellationInSingleMode(void *handle, LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_OFF_CANC_ONE_SHOT_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_OffsetCancellationInSingleMode
* Description    : Read OFF_CANC_ONE_SHOT
* Input          : Pointer to LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t
* Output         : Status of OFF_CANC_ONE_SHOT see LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_OffsetCancellationInSingleMode(void *handle, LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_OFF_CANC_ONE_SHOT_MASK; //mask

  return MEMS_SUCCESS;
}

/************** Interrupt Configuration Functions  *******************/

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_PinMode
* Description    : Write INT_MAG
* Input          : LIS2MDL_MAG_INT_MAG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_PinMode(void *handle, LIS2MDL_MAG_INT_MAG_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_INT_MAG_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_PinMode
* Description    : Read INT_MAG
* Input          : Pointer to LIS2MDL_MAG_INT_MAG_t
* Output         : Status of INT_MAG see LIS2MDL_MAG_INT_MAG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_PinMode(void *handle, LIS2MDL_MAG_INT_MAG_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_INT_MAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_SelfTest
* Description    : Write SELF_TEST
* Input          : LIS2MDL_MAG_SELF_TEST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_SelfTest(void *handle, LIS2MDL_MAG_SELF_TEST_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_SELF_TEST_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_SelfTest
* Description    : Read SELF_TEST
* Input          : Pointer to LIS2MDL_MAG_SELF_TEST_t
* Output         : Status of SELF_TEST see LIS2MDL_MAG_SELF_TEST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_SelfTest(void *handle, LIS2MDL_MAG_SELF_TEST_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_SELF_TEST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_OutputsOrder
* Description    : Write BLE
* Input          : LIS2MDL_MAG_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_OutputsOrder(void *handle, LIS2MDL_MAG_BLE_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_BLE_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_OutputsOrder
* Description    : Read BLE
* Input          : Pointer to LIS2MDL_MAG_BLE_t
* Output         : Status of BLE see LIS2MDL_MAG_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_OutputsOrder(void *handle, LIS2MDL_MAG_BLE_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LIS2MDL_MAG_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_BlockDataUpdate(void *handle, LIS2MDL_MAG_BDU_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_BDU_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LIS2MDL_MAG_BDU_t
* Output         : Status of BDU see LIS2MDL_MAG_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_BlockDataUpdate(void *handle, LIS2MDL_MAG_BDU_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_I2C_InterfaceSatus
* Description    : Write I2C_DIS
* Input          : LIS2MDL_MAG_I2C_DIS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_I2C_InterfaceSatus(void *handle, LIS2MDL_MAG_I2C_DIS_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_I2C_DIS_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_I2C_InterfaceSatus
* Description    : Read I2C_DIS
* Input          : Pointer to LIS2MDL_MAG_I2C_DIS_t
* Output         : Status of I2C_DIS see LIS2MDL_MAG_I2C_DIS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_I2C_InterfaceSatus(void *handle, LIS2MDL_MAG_I2C_DIS_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_I2C_DIS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptGenerator
* Description    : Write INT_MAG_PIN
* Input          : LIS2MDL_MAG_INT_MAG_PIN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptGenerator(void *handle, LIS2MDL_MAG_INT_MAG_PIN_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_INT_MAG_PIN_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptGenerator
* Description    : Read INT_MAG_PIN
* Input          : Pointer to LIS2MDL_MAG_INT_MAG_PIN_t
* Output         : Status of INT_MAG_PIN see LIS2MDL_MAG_INT_MAG_PIN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptGenerator(void *handle, LIS2MDL_MAG_INT_MAG_PIN_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_INT_MAG_PIN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptOn_Xaxis
* Description    : Write IEN
* Input          : LIS2MDL_MAG_IEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptGeneration(void *handle, LIS2MDL_MAG_IEN_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_IEN_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptOn_Xaxis
* Description    : Read IEN
* Input          : Pointer to LIS2MDL_MAG_IEN_t
* Output         : Status of IEN see LIS2MDL_MAG_IEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptGeneration(void *handle, LIS2MDL_MAG_IEN_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_IEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptMode
* Description    : Write IEL
* Input          : LIS2MDL_MAG_IEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptMode(void *handle, LIS2MDL_MAG_IEL_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_IEL_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptMode
* Description    : Read IEL
* Input          : Pointer to LIS2MDL_MAG_IEL_t
* Output         : Status of IEL see LIS2MDL_MAG_IEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptMode(void *handle, LIS2MDL_MAG_IEL_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_IEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptPolarity
* Description    : Write IEA
* Input          : LIS2MDL_MAG_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptPolarity(void *handle, LIS2MDL_MAG_IEA_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_IEA_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptPolarity
* Description    : Read IEA
* Input          : Pointer to LIS2MDL_MAG_IEA_t
* Output         : Status of IEA see LIS2MDL_MAG_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptPolarity(void *handle, LIS2MDL_MAG_IEA_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptOn_Zaxis
* Description    : Write ZIEN
* Input          : LIS2MDL_MAG_ZIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptOn_Zaxis(void *handle, LIS2MDL_MAG_ZIEN_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_ZIEN_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptOn_Zaxis
* Description    : Read ZIEN
* Input          : Pointer to LIS2MDL_MAG_ZIEN_t
* Output         : Status of ZIEN see LIS2MDL_MAG_ZIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptOn_Zaxis(void *handle, LIS2MDL_MAG_ZIEN_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_ZIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptOn_Yaxis
* Description    : Write YIEN
* Input          : LIS2MDL_MAG_YIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptOn_Yaxis(void *handle, LIS2MDL_MAG_YIEN_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_YIEN_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptOn_Yaxis
* Description    : Read YIEN
* Input          : Pointer to LIS2MDL_MAG_YIEN_t
* Output         : Status of YIEN see LIS2MDL_MAG_YIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptOn_Yaxis(void *handle, LIS2MDL_MAG_YIEN_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_YIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_W_InterruptOn_Xaxis
* Description    : Write XIEN
* Input          : LIS2MDL_MAG_XIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2MDL_MAG_W_InterruptOn_Xaxis(void *handle, LIS2MDL_MAG_XIEN_t newValue)
{
  u8_t value;

  if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2MDL_MAG_XIEN_MASK; 
  value |= newValue;
  
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_INT_CRTL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptOn_Xaxis
* Description    : Read XIEN
* Input          : Pointer to LIS2MDL_MAG_XIEN_t
* Output         : Status of XIEN see LIS2MDL_MAG_XIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptOn_Xaxis(void *handle, LIS2MDL_MAG_XIEN_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_CRTL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_XIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_InterruptSources
* Description    : Read INT
* Input          : Pointer to LIS2MDL_MAG_INT_t
* Output         : Status of INT see LIS2MDL_MAG_INT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_InterruptSources(void *handle, LIS2MDL_MAG_INT_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_INT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LIS2MDL_MAG_Set_InterruptThreshold(u8_t *buff) 
* Description    : Set InterruptThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Set_InterruptThreshold(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_OUTX_L_REG+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LIS2MDL_MAG_Get_InterruptThreshold(u8_t *buff)
* Description    : Read InterruptThreshold output register
* Input          : pointer to [u8_t]
* Output         : InterruptThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Get_InterruptThreshold(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
  for (j=0; j<numberOfByteForDimension;j++ )
  {  
    if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_OUTX_L_REG+k, &buff[k], 1))
      return MEMS_ERROR;
    k++;  
  }
  }

  return MEMS_SUCCESS; 
}

/************** Output Functions  *******************/

/*******************************************************************************
* Function Name  : LIS2MDL_MAG_R_STATUS_bits
* Description    : Read STATUS_BIT
* Input          : Pointer to LIS2MDL_MAG_STATUS_t
* Output         : Status of STATUS_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_R_STATUS_bits(void *handle, LIS2MDL_MAG_STATUS_t *value)
{
 if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS2MDL_MAG_STATUS_BIT_MASK; //coerce  

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LIS2MDL_MAG_Get_MagneticOutputs(u8_t *buff)
* Description    : Read MagneticOutputs output register
* Input          : pointer to [u8_t]
* Output         : MagneticOutputs buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Get_MagneticOutputs(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
  for (j=0; j<numberOfByteForDimension;j++ )
  {  
    if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_OUTX_L_REG+k, &buff[k], 1))
      return MEMS_ERROR;
    k++;  
  }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LIS2MDL_MAG_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Get_Temperature(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
  for (j=0; j<numberOfByteForDimension;j++ )
  {  
    if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_TEMP_OUT_L_REG+k, &buff[k], 1))
      return MEMS_ERROR;
    k++;  
  }
  }

  return MEMS_SUCCESS; 
}

/************** Hard-iron Values  *******************/

/*******************************************************************************
  * Function Name  : status_t LIS2MDL_MAG_Set_HI_Offset(u8_t *buff) 
* Description    : Set HI_Offset data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Set_HI_Offsets(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<6;i++ ) 
  {
  if( !LIS2MDL_MAG_WriteReg(handle, LIS2MDL_MAG_OFFSET_X_REG_L+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LIS2MDL_MAG_Get_HI_Offset(u8_t *buff)
* Description    : Read HI_Offset output register
* Input          : pointer to [u8_t]
* Output         : HI_Offset buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2MDL_MAG_Get_HI_Offsets(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
  for (j=0; j<numberOfByteForDimension;j++ )
  {  
    if( !LIS2MDL_MAG_ReadReg(handle, LIS2MDL_MAG_OFFSET_X_REG_L+k, &buff[k], 1))
      return MEMS_ERROR;
    k++;  
  }
  }

  return MEMS_SUCCESS; 
}

/************** Utility  *******************/

/*******************************************************************************
* Function Name    : SwapHighLowByte
* Description    : Swap High/low byte in multiple byte values 
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input        : bufferToSwap -> buffer to swap 
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension 
*
* Output      : bufferToSwap -> buffer swapped 
* Return      : None
*******************************************************************************/
void LIS2MDL_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
{

  u8_t numberOfByteForDimension, i, j;
  u8_t tempValue[10];
  
  numberOfByteForDimension=numberOfByte/dimension;
  
  for (i=0; i<dimension;i++ )
  {
  for (j=0; j<numberOfByteForDimension;j++ )
    tempValue[j]=bufferToSwap[j+i*numberOfByteForDimension];
  for (j=0; j<numberOfByteForDimension;j++ )
    *(bufferToSwap+i*(numberOfByteForDimension)+j)=*(tempValue+(numberOfByteForDimension-1)-j);
  } 
}
