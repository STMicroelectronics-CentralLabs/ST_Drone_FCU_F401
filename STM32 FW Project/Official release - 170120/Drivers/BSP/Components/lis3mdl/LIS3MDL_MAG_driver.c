/**
 *******************************************************************************
 * @file    LIS3MDL_MAG_driver.c
 * @author  MEMS Application Team
 * @version V1.2
 * @date    9-August-2016
 * @brief   LIS3MDL driver file
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
#include "LIS3MDL_MAG_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Imported function prototypes ----------------------------------------------*/
extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name   : LIS3MDL_MAG_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
status_t LIS3MDL_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  if ( len > 1 ) Reg |= 0x80;

  if ( Sensor_IO_Write( handle, Reg, Bufp, len ) )
    return MEMS_ERROR;
  else
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name   : LIS3MDL_MAG_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data REad
* Return      : None
*******************************************************************************/
status_t LIS3MDL_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  if ( len > 1 ) Reg |= 0x80;

  if ( Sensor_IO_Read( handle, Reg, Bufp, len ) )
    return MEMS_ERROR;
  else
    return MEMS_SUCCESS;
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_WHO_AM_I_(void *handle, u8_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_WHO_AM_I_BIT_MASK; //coerce
  *value = *value >> LIS3MDL_MAG_WHO_AM_I_BIT_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SystemOperatingMode
* Description    : Write MD
* Input          : LIS3MDL_MAG_MD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_SystemOperatingMode(void *handle, LIS3MDL_MAG_MD_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_MD_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SystemOperatingMode
* Description    : Read MD
* Input          : Pointer to LIS3MDL_MAG_MD_t
* Output         : Status of MD see LIS3MDL_MAG_MD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_SystemOperatingMode(void *handle, LIS3MDL_MAG_MD_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_MD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LIS3MDL_MAG_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_BlockDataUpdate(void *handle, LIS3MDL_MAG_BDU_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_BDU_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LIS3MDL_MAG_BDU_t
* Output         : Status of BDU see LIS3MDL_MAG_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_BlockDataUpdate(void *handle, LIS3MDL_MAG_BDU_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_FullScale
* Description    : Write FS
* Input          : LIS3MDL_MAG_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_FullScale(void *handle, LIS3MDL_MAG_FS_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_FS_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_FullScale
* Description    : Read FS
* Input          : Pointer to LIS3MDL_MAG_FS_t
* Output         : Status of FS see LIS3MDL_MAG_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_FullScale(void *handle, LIS3MDL_MAG_FS_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_FS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OutputDataRate
* Description    : Write DO
* Input          : LIS3MDL_MAG_DO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_OutputDataRate(void *handle, LIS3MDL_MAG_DO_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_DO_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OutputDataRate
* Description    : Read DO
* Input          : Pointer to LIS3MDL_MAG_DO_t
* Output         : Status of DO see LIS3MDL_MAG_DO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_OutputDataRate(void *handle, LIS3MDL_MAG_DO_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_DO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LIS3MDL_MAG_Get_Magnetic(u8_t *buff)
* Description    : Read Magnetic output register
* Input          : pointer to [u8_t]
* Output         : Magnetic buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_Get_Magnetic(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_OUTX_L + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SelfTest
* Description    : Write ST
* Input          : LIS3MDL_MAG_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_SelfTest(void *handle, LIS3MDL_MAG_ST_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_ST_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SelfTest
* Description    : Read ST
* Input          : Pointer to LIS3MDL_MAG_ST_t
* Output         : Status of ST see LIS3MDL_MAG_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_SelfTest(void *handle, LIS3MDL_MAG_ST_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OperatingModeXY
* Description    : Write OM
* Input          : LIS3MDL_MAG_OM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_OperatingModeXY(void *handle, LIS3MDL_MAG_OM_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_OM_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OperatingModeXY
* Description    : Read OM
* Input          : Pointer to LIS3MDL_MAG_OM_t
* Output         : Status of OM see LIS3MDL_MAG_OM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_OperatingModeXY(void *handle, LIS3MDL_MAG_OM_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_OM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_TemperatureSensor
* Description    : Write TEMP_EN
* Input          : LIS3MDL_MAG_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_TemperatureSensor(void *handle, LIS3MDL_MAG_TEMP_EN_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_TEMP_EN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_TemperatureSensor
* Description    : Read TEMP_EN
* Input          : Pointer to LIS3MDL_MAG_TEMP_EN_t
* Output         : Status of TEMP_EN see LIS3MDL_MAG_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_TemperatureSensor(void *handle, LIS3MDL_MAG_TEMP_EN_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_TEMP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SoftRST
* Description    : Write SOFT_RST
* Input          : LIS3MDL_MAG_SOFT_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_SoftRST(void *handle, LIS3MDL_MAG_SOFT_RST_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_SOFT_RST_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SoftRST
* Description    : Read SOFT_RST
* Input          : Pointer to LIS3MDL_MAG_SOFT_RST_t
* Output         : Status of SOFT_RST see LIS3MDL_MAG_SOFT_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_SoftRST(void *handle, LIS3MDL_MAG_SOFT_RST_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_SOFT_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_Reboot
* Description    : Write REBOOT
* Input          : LIS3MDL_MAG_REBOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_Reboot(void *handle, LIS3MDL_MAG_REBOOT_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_REBOOT_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_Reboot
* Description    : Read REBOOT
* Input          : Pointer to LIS3MDL_MAG_REBOOT_t
* Output         : Status of REBOOT see LIS3MDL_MAG_REBOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_Reboot(void *handle, LIS3MDL_MAG_REBOOT_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_REBOOT_MASK; //mask

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SerialInterfaceMode
* Description    : Write SIM
* Input          : LIS3MDL_MAG_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_SerialInterfaceMode(void *handle, LIS3MDL_MAG_SIM_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_SIM_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SerialInterfaceMode
* Description    : Read SIM
* Input          : Pointer to LIS3MDL_MAG_SIM_t
* Output         : Status of SIM see LIS3MDL_MAG_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_SerialInterfaceMode(void *handle, LIS3MDL_MAG_SIM_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_FastLowPowerXYZ
* Description    : Write LP
* Input          : LIS3MDL_MAG_LP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_FastLowPowerXYZ(void *handle, LIS3MDL_MAG_LP_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_LP_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_FastLowPowerXYZ
* Description    : Read LP
* Input          : Pointer to LIS3MDL_MAG_LP_t
* Output         : Status of LP see LIS3MDL_MAG_LP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_FastLowPowerXYZ(void *handle, LIS3MDL_MAG_LP_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_LP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_LittleBigEndianInversion
* Description    : Write BLE
* Input          : LIS3MDL_MAG_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_LittleBigEndianInversion(void *handle, LIS3MDL_MAG_BLE_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_BLE_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_LittleBigEndianInversion
* Description    : Read BLE
* Input          : Pointer to LIS3MDL_MAG_BLE_t
* Output         : Status of BLE see LIS3MDL_MAG_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_LittleBigEndianInversion(void *handle, LIS3MDL_MAG_BLE_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OperatingModeZ
* Description    : Write OMZ
* Input          : LIS3MDL_MAG_OMZ_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_OperatingModeZ(void *handle, LIS3MDL_MAG_OMZ_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_OMZ_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OperatingModeZ
* Description    : Read OMZ
* Input          : Pointer to LIS3MDL_MAG_OMZ_t
* Output         : Status of OMZ see LIS3MDL_MAG_OMZ_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_OperatingModeZ(void *handle, LIS3MDL_MAG_OMZ_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_OMZ_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewXData
* Description    : Read XDA
* Input          : Pointer to LIS3MDL_MAG_XDA_t
* Output         : Status of XDA see LIS3MDL_MAG_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NewXData(void *handle, LIS3MDL_MAG_XDA_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewYData
* Description    : Read YDA
* Input          : Pointer to LIS3MDL_MAG_YDA_t
* Output         : Status of YDA see LIS3MDL_MAG_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NewYData(void *handle, LIS3MDL_MAG_YDA_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewZData
* Description    : Read ZDA
* Input          : Pointer to LIS3MDL_MAG_ZDA_t
* Output         : Status of ZDA see LIS3MDL_MAG_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NewZData(void *handle, LIS3MDL_MAG_ZDA_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewXYZData
* Description    : Read ZYXDA
* Input          : Pointer to LIS3MDL_MAG_ZYXDA_t
* Output         : Status of ZYXDA see LIS3MDL_MAG_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NewXYZData(void *handle, LIS3MDL_MAG_ZYXDA_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataXOverrun
* Description    : Read XOR
* Input          : Pointer to LIS3MDL_MAG_XOR_t
* Output         : Status of XOR see LIS3MDL_MAG_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_DataXOverrun(void *handle, LIS3MDL_MAG_XOR_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataYOverrun
* Description    : Read YOR
* Input          : Pointer to LIS3MDL_MAG_YOR_t
* Output         : Status of YOR see LIS3MDL_MAG_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_DataYOverrun(void *handle, LIS3MDL_MAG_YOR_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataZOverrun
* Description    : Read ZOR
* Input          : Pointer to LIS3MDL_MAG_ZOR_t
* Output         : Status of ZOR see LIS3MDL_MAG_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_DataZOverrun(void *handle, LIS3MDL_MAG_ZOR_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataXYZOverrun
* Description    : Read ZYXOR
* Input          : Pointer to LIS3MDL_MAG_ZYXOR_t
* Output         : Status of ZYXOR see LIS3MDL_MAG_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_DataXYZOverrun(void *handle, LIS3MDL_MAG_ZYXOR_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptEnable
* Description    : Write IEN
* Input          : LIS3MDL_MAG_IEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptEnable(void *handle, LIS3MDL_MAG_IEN_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_IEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptEnable
* Description    : Read IEN
* Input          : Pointer to LIS3MDL_MAG_IEN_t
* Output         : Status of IEN see LIS3MDL_MAG_IEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptEnable(void *handle, LIS3MDL_MAG_IEN_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_IEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_LatchInterruptRq
* Description    : Write LIR
* Input          : LIS3MDL_MAG_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_LatchInterruptRq(void *handle, LIS3MDL_MAG_LIR_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_LIR_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_LatchInterruptRq
* Description    : Read LIR
* Input          : Pointer to LIS3MDL_MAG_LIR_t
* Output         : Status of LIR see LIS3MDL_MAG_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_LatchInterruptRq(void *handle, LIS3MDL_MAG_LIR_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptActive
* Description    : Write IEA
* Input          : LIS3MDL_MAG_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptActive(void *handle, LIS3MDL_MAG_IEA_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_IEA_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptActive
* Description    : Read IEA
* Input          : Pointer to LIS3MDL_MAG_IEA_t
* Output         : Status of IEA see LIS3MDL_MAG_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptActive(void *handle, LIS3MDL_MAG_IEA_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnZ
* Description    : Write ZIEN
* Input          : LIS3MDL_MAG_ZIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptOnZ(void *handle, LIS3MDL_MAG_ZIEN_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_ZIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnZ
* Description    : Read ZIEN
* Input          : Pointer to LIS3MDL_MAG_ZIEN_t
* Output         : Status of ZIEN see LIS3MDL_MAG_ZIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptOnZ(void *handle, LIS3MDL_MAG_ZIEN_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnY
* Description    : Write YIEN
* Input          : LIS3MDL_MAG_YIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptOnY(void *handle, LIS3MDL_MAG_YIEN_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_YIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnY
* Description    : Read YIEN
* Input          : Pointer to LIS3MDL_MAG_YIEN_t
* Output         : Status of YIEN see LIS3MDL_MAG_YIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptOnY(void *handle, LIS3MDL_MAG_YIEN_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnX
* Description    : Write XIEN
* Input          : LIS3MDL_MAG_XIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptOnX(void *handle, LIS3MDL_MAG_XIEN_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_XIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnX
* Description    : Read XIEN
* Input          : Pointer to LIS3MDL_MAG_XIEN_t
* Output         : Status of XIEN see LIS3MDL_MAG_XIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptOnX(void *handle, LIS3MDL_MAG_XIEN_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptFlag
* Description    : Write INT
* Input          : LIS3MDL_MAG_INT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_InterruptFlag(void *handle, LIS3MDL_MAG_INT_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_INT_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptFlag
* Description    : Read INT
* Input          : Pointer to LIS3MDL_MAG_INT_t
* Output         : Status of INT see LIS3MDL_MAG_INT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_InterruptFlag(void *handle, LIS3MDL_MAG_INT_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_INT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_MagneticFieldOverflow
* Description    : Write MROI
* Input          : LIS3MDL_MAG_MROI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_MagneticFieldOverflow(void *handle, LIS3MDL_MAG_MROI_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_MROI_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_MagneticFieldOverflow
* Description    : Read MROI
* Input          : Pointer to LIS3MDL_MAG_MROI_t
* Output         : Status of MROI see LIS3MDL_MAG_MROI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_MagneticFieldOverflow(void *handle, LIS3MDL_MAG_MROI_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_MROI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagZ
* Description    : Write NTH_Z
* Input          : LIS3MDL_MAG_NTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_NegativeThresholdFlagZ(void *handle, LIS3MDL_MAG_NTH_Z_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_Z_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagZ
* Description    : Read NTH_Z
* Input          : Pointer to LIS3MDL_MAG_NTH_Z_t
* Output         : Status of NTH_Z see LIS3MDL_MAG_NTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NegativeThresholdFlagZ(void *handle, LIS3MDL_MAG_NTH_Z_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagY
* Description    : Write NTH_Y
* Input          : LIS3MDL_MAG_NTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_NegativeThresholdFlagY(void *handle, LIS3MDL_MAG_NTH_Y_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_Y_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagY
* Description    : Read NTH_Y
* Input          : Pointer to LIS3MDL_MAG_NTH_Y_t
* Output         : Status of NTH_Y see LIS3MDL_MAG_NTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NegativeThresholdFlagY(void *handle, LIS3MDL_MAG_NTH_Y_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagX
* Description    : Write NTH_X
* Input          : LIS3MDL_MAG_NTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_NegativeThresholdFlagX(void *handle, LIS3MDL_MAG_NTH_X_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_X_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagX
* Description    : Read NTH_X
* Input          : Pointer to LIS3MDL_MAG_NTH_X_t
* Output         : Status of NTH_X see LIS3MDL_MAG_NTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_NegativeThresholdFlagX(void *handle, LIS3MDL_MAG_NTH_X_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagZ
* Description    : Write PTH_Z
* Input          : LIS3MDL_MAG_PTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_PositiveThresholdFlagZ(void *handle, LIS3MDL_MAG_PTH_Z_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_Z_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagZ
* Description    : Read PTH_Z
* Input          : Pointer to LIS3MDL_MAG_PTH_Z_t
* Output         : Status of PTH_Z see LIS3MDL_MAG_PTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_PositiveThresholdFlagZ(void *handle, LIS3MDL_MAG_PTH_Z_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagY
* Description    : Write PTH_Y
* Input          : LIS3MDL_MAG_PTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_PositiveThresholdFlagY(void *handle, LIS3MDL_MAG_PTH_Y_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_Y_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagY
* Description    : Read PTH_Y
* Input          : Pointer to LIS3MDL_MAG_PTH_Y_t
* Output         : Status of PTH_Y see LIS3MDL_MAG_PTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_PositiveThresholdFlagY(void *handle, LIS3MDL_MAG_PTH_Y_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagX
* Description    : Write PTH_X
* Input          : LIS3MDL_MAG_PTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS3MDL_MAG_W_PositiveThresholdFlagX(void *handle, LIS3MDL_MAG_PTH_X_t newValue)
{
  u8_t value;

  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_X_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagX
* Description    : Read PTH_X
* Input          : Pointer to LIS3MDL_MAG_PTH_X_t
* Output         : Status of PTH_X see LIS3MDL_MAG_PTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_R_PositiveThresholdFlagX(void *handle, LIS3MDL_MAG_PTH_X_t *value)
{
  if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LIS3MDL_MAG_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_Get_Temperature(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_TEMP_OUT_L + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : status_t LIS3MDL_MAG_Set_MagneticThreshold(u8_t *buff)
* Description    : Set MagneticThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_Set_MagneticThreshold(void *handle, u8_t *buff)
{
  u8_t  i;

  for (i = 0; i < 2; i++ )
  {
    if( !LIS3MDL_MAG_WriteReg(handle, LIS3MDL_MAG_INT_THS_L + i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LIS3MDL_MAG_Get_MagneticThreshold(u8_t *buff)
* Description    : Read MagneticThreshold output register
* Input          : pointer to [u8_t]
* Output         : MagneticThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3MDL_MAG_Get_MagneticThreshold(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LIS3MDL_MAG_ReadReg(handle, LIS3MDL_MAG_INT_THS_L + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name   : SwapHighLowByte
* Description   : Swap High/low byte in multiple byte values
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input       : bufferToSwap -> buffer to swap
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension
*
* Output      : bufferToSwap -> buffer swapped
* Return      : None
*******************************************************************************/
void LIS3MDL_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
{
  u8_t numberOfByteForDimension, i, j;
  u8_t tempValue[10];

  numberOfByteForDimension = numberOfByte / dimension;

  for (i = 0; i < dimension; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
      tempValue[j] = bufferToSwap[j + i * numberOfByteForDimension];
    for (j = 0; j < numberOfByteForDimension; j++ )
      *(bufferToSwap + i * (numberOfByteForDimension) + j) = *(tempValue + (numberOfByteForDimension - 1) - j);
  }
}
