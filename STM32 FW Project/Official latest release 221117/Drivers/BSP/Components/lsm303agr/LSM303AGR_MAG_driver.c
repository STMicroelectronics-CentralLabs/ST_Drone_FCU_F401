/**
 *******************************************************************************
 * @file    LSM303AGR_MAG_driver.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    25-February-2016
 * @brief   LSM303AGR Magnetometer driver file
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
#include "LSM303AGR_MAG_driver.h"

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
* Function Name   : LSM303AGR_MAG_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output      : None
* Return      : None
*******************************************************************************/
status_t LSM303AGR_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{

  if ( len > 1 ) Reg |= 0x80;

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
* Function Name   : LSM303AGR_MAG_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading function
* Input       : Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output      : None
* Return      : None
*******************************************************************************/
status_t LSM303AGR_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{

  if ( len > 1 ) Reg |= 0x80;

  if (Sensor_IO_Read(handle, Reg, Bufp, len))
  {
    return MEMS_ERROR;
  }
  else
  {
    return MEMS_SUCCESS;
  }
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_WHO_AM_I
* Description    : Read WHO_AM_I
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_WHO_AM_I(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_WHO_AM_I_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_WHO_AM_I_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_BDU
* Description    : Write BDU
* Input          : LSM303AGR_MAG_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_BDU(void *handle, LSM303AGR_MAG_BDU_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_BDU_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_BDU
* Description    : Read BDU
* Input          : Pointer to LSM303AGR_MAG_BDU_t
* Output         : Status of BDU see LSM303AGR_MAG_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_BDU(void *handle, LSM303AGR_MAG_BDU_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_MD
* Description    : Write MD
* Input          : LSM303AGR_MAG_MD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_MD(void *handle, LSM303AGR_MAG_MD_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_MD_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_MD
* Description    : Read MD
* Input          : Pointer to LSM303AGR_MAG_MD_t
* Output         : Status of MD see LSM303AGR_MAG_MD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_MD(void *handle, LSM303AGR_MAG_MD_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_MD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303AGR_MAG_Get_Raw_Magnetic(u8_t *buff)
* Description    : Read Magnetic output register
* Input          : pointer to [u8_t]
* Output         : Magnetic buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_Get_Raw_Magnetic(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OUTX_L_REG + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303AGR_MAG_Get_Magnetic(void *handle, int *buff)
* Description    : Read GetMagData output register
* Input          : pointer to [u8_t]
* Output         : values are expressed in mGa
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
#define LSM303AGR_MAG_SENSITIVITY 15/10
status_t LSM303AGR_MAG_Get_Magnetic(void *handle, int *buff)
{
  Type3Axis16bit_U raw_data_tmp;

  /* Read out raw magnetometer samples */
  LSM303AGR_MAG_Get_Raw_Magnetic(handle, raw_data_tmp.u8bit);

  /* Applysensitivity */
  buff[0] = raw_data_tmp.i16bit[0] * LSM303AGR_MAG_SENSITIVITY;
  buff[1] = raw_data_tmp.i16bit[1] * LSM303AGR_MAG_SENSITIVITY;
  buff[2] = raw_data_tmp.i16bit[2] * LSM303AGR_MAG_SENSITIVITY;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_ODR
* Description    : Write ODR
* Input          : LSM303AGR_MAG_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_ODR(void *handle, LSM303AGR_MAG_ODR_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_ODR_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ODR
* Description    : Read ODR
* Input          : Pointer to LSM303AGR_MAG_ODR_t
* Output         : Status of ODR see LSM303AGR_MAG_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ODR(void *handle, LSM303AGR_MAG_ODR_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ODR_MASK; //mask

  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_X_L
* Description    : Write OFF_X_L
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_X_L(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_X_L_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_X_L_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_X_REG_L, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_X_L_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_X_REG_L, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_X_L
* Description    : Read OFF_X_L
* Input          : Pointer to u8_t
* Output         : Status of OFF_X_L
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_X_L(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_X_REG_L, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_X_L_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_X_L_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_X_H
* Description    : Write OFF_X_H
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_X_H(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_X_H_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_X_H_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_X_REG_H, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_X_H_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_X_REG_H, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_X_H
* Description    : Read OFF_X_H
* Input          : Pointer to u8_t
* Output         : Status of OFF_X_H
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_X_H(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_X_REG_H, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_X_H_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_X_H_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_Y_L
* Description    : Write OFF_Y_L
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_Y_L(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_Y_L_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_Y_L_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Y_REG_L, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_Y_L_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_Y_REG_L, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_Y_L
* Description    : Read OFF_Y_L
* Input          : Pointer to u8_t
* Output         : Status of OFF_Y_L
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_Y_L(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Y_REG_L, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_Y_L_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_Y_L_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_Y_H
* Description    : Write OFF_Y_H
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_Y_H(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_Y_H_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_Y_H_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Y_REG_H, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_Y_H_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_Y_REG_H, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_Y_H
* Description    : Read OFF_Y_H
* Input          : Pointer to u8_t
* Output         : Status of OFF_Y_H
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_Y_H(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Y_REG_H, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_Y_H_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_Y_H_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_Z_L
* Description    : Write OFF_Z_L
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_Z_L(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_Z_L_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_Z_L_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Z_REG_L, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_Z_L_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_Z_REG_L, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_Z_L
* Description    : Read OFF_Z_L
* Input          : Pointer to u8_t
* Output         : Status of OFF_Z_L
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_Z_L(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Z_REG_L, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_Z_L_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_Z_L_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_Z_H
* Description    : Write OFF_Z_H
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_Z_H(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM303AGR_MAG_OFF_Z_H_POSITION; //mask
  newValue &= LSM303AGR_MAG_OFF_Z_H_MASK; //coerce

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Z_REG_H, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM303AGR_MAG_OFF_Z_H_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_OFFSET_Z_REG_H, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_x_x
* Description    : Get the Magnetic offsets
* Input          : Pointer to u16_t
* Output         : Status of OFF_x_x
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_Get_MagOff(void *handle, u16_t *magx_off, u16_t *magy_off, u16_t *magz_off)
{
  u8_t reg_l, reg_h;

  /* read mag_x_off */
  LSM303AGR_MAG_R_OFF_X_L(handle, &reg_l);
  LSM303AGR_MAG_R_OFF_X_H(handle, &reg_h);
  *magx_off = ((reg_h << 8) & 0xff00) | reg_l;

  /* read mag_y_off */
  LSM303AGR_MAG_R_OFF_Y_L(handle, &reg_l);
  LSM303AGR_MAG_R_OFF_Y_H(handle, &reg_h);
  *magy_off = ((reg_h << 8) & 0xff00) | reg_l;

  /* read mag_z_off */
  LSM303AGR_MAG_R_OFF_Z_L(handle, &reg_l);
  LSM303AGR_MAG_R_OFF_Z_H(handle, &reg_h);
  *magz_off = ((reg_h << 8) & 0xff00) | reg_l;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_x_x
* Description    : Set the Magnetic offsets
* Input          : Pointer to u16_t
* Output         : Status of OFF_x_x
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_Set_MagOff(void *handle, u16_t magx_off, u16_t magy_off, u16_t magz_off)
{
  /* write mag_x_off */
  LSM303AGR_MAG_W_OFF_X_L(handle, magx_off & 0xff);
  LSM303AGR_MAG_W_OFF_X_H(handle, (magx_off >> 8) & 0xff);

  /* write mag_y_off */
  LSM303AGR_MAG_W_OFF_Y_L(handle, magy_off & 0xff);
  LSM303AGR_MAG_W_OFF_Y_H(handle, (magy_off >> 8) & 0xff);

  /* write mag_z_off */
  LSM303AGR_MAG_W_OFF_Z_L(handle, magz_off & 0xff);
  LSM303AGR_MAG_W_OFF_Z_H(handle, (magz_off >> 8) & 0xff);

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_Z_H
* Description    : Read OFF_Z_H
* Input          : Pointer to u8_t
* Output         : Status of OFF_Z_H
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_Z_H(void *handle, u8_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_OFFSET_Z_REG_H, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_Z_H_MASK; //coerce
  *value = *value >> LSM303AGR_MAG_OFF_Z_H_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_LP
* Description    : Write LP
* Input          : LSM303AGR_MAG_LP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_LP(void *handle, LSM303AGR_MAG_LP_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_LP_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_LP
* Description    : Read LP
* Input          : Pointer to LSM303AGR_MAG_LP_t
* Output         : Status of LP see LSM303AGR_MAG_LP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_LP(void *handle, LSM303AGR_MAG_LP_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_LP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_SOFT_RST
* Description    : Write SOFT_RST
* Input          : LSM303AGR_MAG_SOFT_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_SOFT_RST(void *handle, LSM303AGR_MAG_SOFT_RST_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_SOFT_RST_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_A, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_SOFT_RST
* Description    : Read SOFT_RST
* Input          : Pointer to LSM303AGR_MAG_SOFT_RST_t
* Output         : Status of SOFT_RST see LSM303AGR_MAG_SOFT_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_SOFT_RST(void *handle, LSM303AGR_MAG_SOFT_RST_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_A, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_SOFT_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_LPF
* Description    : Write LPF
* Input          : LSM303AGR_MAG_LPF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_LPF(void *handle, LSM303AGR_MAG_LPF_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_LPF_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_LPF
* Description    : Read LPF
* Input          : Pointer to LSM303AGR_MAG_LPF_t
* Output         : Status of LPF see LSM303AGR_MAG_LPF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_LPF(void *handle, LSM303AGR_MAG_LPF_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_LPF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_OFF_CANC
* Description    : Write OFF_CANC
* Input          : LSM303AGR_MAG_OFF_CANC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_OFF_CANC(void *handle, LSM303AGR_MAG_OFF_CANC_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_OFF_CANC_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_OFF_CANC
* Description    : Read OFF_CANC
* Input          : Pointer to LSM303AGR_MAG_OFF_CANC_t
* Output         : Status of OFF_CANC see LSM303AGR_MAG_OFF_CANC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_OFF_CANC(void *handle, LSM303AGR_MAG_OFF_CANC_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_OFF_CANC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_SET_FREQ
* Description    : Write SET_FREQ
* Input          : LSM303AGR_MAG_SET_FREQ_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_SET_FREQ(void *handle, LSM303AGR_MAG_SET_FREQ_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_SET_FREQ_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_SET_FREQ
* Description    : Read SET_FREQ
* Input          : Pointer to LSM303AGR_MAG_SET_FREQ_t
* Output         : Status of SET_FREQ see LSM303AGR_MAG_SET_FREQ_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_SET_FREQ(void *handle, LSM303AGR_MAG_SET_FREQ_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_SET_FREQ_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_INT_ON_DATAOFF
* Description    : Write INT_ON_DATAOFF
* Input          : LSM303AGR_MAG_INT_ON_DATAOFF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_INT_ON_DATAOFF(void *handle, LSM303AGR_MAG_INT_ON_DATAOFF_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_INT_ON_DATAOFF_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_B, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_INT_ON_DATAOFF
* Description    : Read INT_ON_DATAOFF
* Input          : Pointer to LSM303AGR_MAG_INT_ON_DATAOFF_t
* Output         : Status of INT_ON_DATAOFF see LSM303AGR_MAG_INT_ON_DATAOFF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_INT_ON_DATAOFF(void *handle, LSM303AGR_MAG_INT_ON_DATAOFF_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_B, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_INT_ON_DATAOFF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_INT_MAG
* Description    : Write INT_MAG
* Input          : LSM303AGR_MAG_INT_MAG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_INT_MAG(void *handle, LSM303AGR_MAG_INT_MAG_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_INT_MAG_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_INT_MAG
* Description    : Read INT_MAG
* Input          : Pointer to LSM303AGR_MAG_INT_MAG_t
* Output         : Status of INT_MAG see LSM303AGR_MAG_INT_MAG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_INT_MAG(void *handle, LSM303AGR_MAG_INT_MAG_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_INT_MAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_ST
* Description    : Write ST
* Input          : LSM303AGR_MAG_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_ST(void *handle, LSM303AGR_MAG_ST_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_ST_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ST
* Description    : Read ST
* Input          : Pointer to LSM303AGR_MAG_ST_t
* Output         : Status of ST see LSM303AGR_MAG_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ST(void *handle, LSM303AGR_MAG_ST_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_BLE
* Description    : Write BLE
* Input          : LSM303AGR_MAG_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_BLE(void *handle, LSM303AGR_MAG_BLE_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_BLE_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_BLE
* Description    : Read BLE
* Input          : Pointer to LSM303AGR_MAG_BLE_t
* Output         : Status of BLE see LSM303AGR_MAG_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_BLE(void *handle, LSM303AGR_MAG_BLE_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_I2C_DIS
* Description    : Write I2C_DIS
* Input          : LSM303AGR_MAG_I2C_DIS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_I2C_DIS(void *handle, LSM303AGR_MAG_I2C_DIS_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_I2C_DIS_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_I2C_DIS
* Description    : Read I2C_DIS
* Input          : Pointer to LSM303AGR_MAG_I2C_DIS_t
* Output         : Status of I2C_DIS see LSM303AGR_MAG_I2C_DIS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_I2C_DIS(void *handle, LSM303AGR_MAG_I2C_DIS_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_I2C_DIS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_INT_MAG_PIN
* Description    : Write INT_MAG_PIN
* Input          : LSM303AGR_MAG_INT_MAG_PIN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_INT_MAG_PIN(void *handle, LSM303AGR_MAG_INT_MAG_PIN_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_INT_MAG_PIN_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_CFG_REG_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_INT_MAG_PIN
* Description    : Read INT_MAG_PIN
* Input          : Pointer to LSM303AGR_MAG_INT_MAG_PIN_t
* Output         : Status of INT_MAG_PIN see LSM303AGR_MAG_INT_MAG_PIN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_INT_MAG_PIN(void *handle, LSM303AGR_MAG_INT_MAG_PIN_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_CFG_REG_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_INT_MAG_PIN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_IEN
* Description    : Write IEN
* Input          : LSM303AGR_MAG_IEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_IEN(void *handle, LSM303AGR_MAG_IEN_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_IEN_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_IEN
* Description    : Read IEN
* Input          : Pointer to LSM303AGR_MAG_IEN_t
* Output         : Status of IEN see LSM303AGR_MAG_IEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_IEN(void *handle, LSM303AGR_MAG_IEN_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_IEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_IEL
* Description    : Write IEL
* Input          : LSM303AGR_MAG_IEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_IEL(void *handle, LSM303AGR_MAG_IEL_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_IEL_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_IEL
* Description    : Read IEL
* Input          : Pointer to LSM303AGR_MAG_IEL_t
* Output         : Status of IEL see LSM303AGR_MAG_IEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_IEL(void *handle, LSM303AGR_MAG_IEL_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_IEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_IEA
* Description    : Write IEA
* Input          : LSM303AGR_MAG_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_IEA(void *handle, LSM303AGR_MAG_IEA_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_IEA_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_IEA
* Description    : Read IEA
* Input          : Pointer to LSM303AGR_MAG_IEA_t
* Output         : Status of IEA see LSM303AGR_MAG_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_IEA(void *handle, LSM303AGR_MAG_IEA_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_ZIEN
* Description    : Write ZIEN
* Input          : LSM303AGR_MAG_ZIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_ZIEN(void *handle, LSM303AGR_MAG_ZIEN_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_ZIEN_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ZIEN
* Description    : Read ZIEN
* Input          : Pointer to LSM303AGR_MAG_ZIEN_t
* Output         : Status of ZIEN see LSM303AGR_MAG_ZIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ZIEN(void *handle, LSM303AGR_MAG_ZIEN_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ZIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_YIEN
* Description    : Write YIEN
* Input          : LSM303AGR_MAG_YIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_YIEN(void *handle, LSM303AGR_MAG_YIEN_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_YIEN_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_YIEN
* Description    : Read YIEN
* Input          : Pointer to LSM303AGR_MAG_YIEN_t
* Output         : Status of YIEN see LSM303AGR_MAG_YIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_YIEN(void *handle, LSM303AGR_MAG_YIEN_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_YIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_W_XIEN
* Description    : Write XIEN
* Input          : LSM303AGR_MAG_XIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_MAG_W_XIEN(void *handle, LSM303AGR_MAG_XIEN_t newValue)
{
  u8_t value;

  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303AGR_MAG_XIEN_MASK;
  value |= newValue;

  if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_XIEN
* Description    : Read XIEN
* Input          : Pointer to LSM303AGR_MAG_XIEN_t
* Output         : Status of XIEN see LSM303AGR_MAG_XIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_XIEN(void *handle, LSM303AGR_MAG_XIEN_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_XIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_INT
* Description    : Read INT
* Input          : Pointer to LSM303AGR_MAG_INT_t
* Output         : Status of INT see LSM303AGR_MAG_INT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_INT(void *handle, LSM303AGR_MAG_INT_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_INT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_MROI
* Description    : Read MROI
* Input          : Pointer to LSM303AGR_MAG_MROI_t
* Output         : Status of MROI see LSM303AGR_MAG_MROI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_MROI(void *handle, LSM303AGR_MAG_MROI_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_MROI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_N_TH_S_Z
* Description    : Read N_TH_S_Z
* Input          : Pointer to LSM303AGR_MAG_N_TH_S_Z_t
* Output         : Status of N_TH_S_Z see LSM303AGR_MAG_N_TH_S_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_N_TH_S_Z(void *handle, LSM303AGR_MAG_N_TH_S_Z_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_N_TH_S_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_N_TH_S_Y
* Description    : Read N_TH_S_Y
* Input          : Pointer to LSM303AGR_MAG_N_TH_S_Y_t
* Output         : Status of N_TH_S_Y see LSM303AGR_MAG_N_TH_S_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_N_TH_S_Y(void *handle, LSM303AGR_MAG_N_TH_S_Y_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_N_TH_S_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_N_TH_S_X
* Description    : Read N_TH_S_X
* Input          : Pointer to LSM303AGR_MAG_N_TH_S_X_t
* Output         : Status of N_TH_S_X see LSM303AGR_MAG_N_TH_S_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_N_TH_S_X(void *handle, LSM303AGR_MAG_N_TH_S_X_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_N_TH_S_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_P_TH_S_Z
* Description    : Read P_TH_S_Z
* Input          : Pointer to LSM303AGR_MAG_P_TH_S_Z_t
* Output         : Status of P_TH_S_Z see LSM303AGR_MAG_P_TH_S_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_P_TH_S_Z(void *handle, LSM303AGR_MAG_P_TH_S_Z_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_P_TH_S_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_P_TH_S_Y
* Description    : Read P_TH_S_Y
* Input          : Pointer to LSM303AGR_MAG_P_TH_S_Y_t
* Output         : Status of P_TH_S_Y see LSM303AGR_MAG_P_TH_S_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_P_TH_S_Y(void *handle, LSM303AGR_MAG_P_TH_S_Y_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_P_TH_S_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_P_TH_S_X
* Description    : Read P_TH_S_X
* Input          : Pointer to LSM303AGR_MAG_P_TH_S_X_t
* Output         : Status of P_TH_S_X see LSM303AGR_MAG_P_TH_S_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_P_TH_S_X(void *handle, LSM303AGR_MAG_P_TH_S_X_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_SOURCE_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_P_TH_S_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_XDA
* Description    : Read XDA
* Input          : Pointer to LSM303AGR_MAG_XDA_t
* Output         : Status of XDA see LSM303AGR_MAG_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_XDA(void *handle, LSM303AGR_MAG_XDA_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_YDA
* Description    : Read YDA
* Input          : Pointer to LSM303AGR_MAG_YDA_t
* Output         : Status of YDA see LSM303AGR_MAG_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_YDA(void *handle, LSM303AGR_MAG_YDA_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ZDA
* Description    : Read ZDA
* Input          : Pointer to LSM303AGR_MAG_ZDA_t
* Output         : Status of ZDA see LSM303AGR_MAG_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ZDA(void *handle, LSM303AGR_MAG_ZDA_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ZYXDA
* Description    : Read ZYXDA
* Input          : Pointer to LSM303AGR_MAG_ZYXDA_t
* Output         : Status of ZYXDA see LSM303AGR_MAG_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ZYXDA(void *handle, LSM303AGR_MAG_ZYXDA_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_XOR
* Description    : Read XOR
* Input          : Pointer to LSM303AGR_MAG_XOR_t
* Output         : Status of XOR see LSM303AGR_MAG_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_XOR(void *handle, LSM303AGR_MAG_XOR_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_YOR
* Description    : Read YOR
* Input          : Pointer to LSM303AGR_MAG_YOR_t
* Output         : Status of YOR see LSM303AGR_MAG_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_YOR(void *handle, LSM303AGR_MAG_YOR_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ZOR
* Description    : Read ZOR
* Input          : Pointer to LSM303AGR_MAG_ZOR_t
* Output         : Status of ZOR see LSM303AGR_MAG_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ZOR(void *handle, LSM303AGR_MAG_ZOR_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_MAG_R_ZYXOR
* Description    : Read ZYXOR
* Input          : Pointer to LSM303AGR_MAG_ZYXOR_t
* Output         : Status of ZYXOR see LSM303AGR_MAG_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_R_ZYXOR(void *handle, LSM303AGR_MAG_ZYXOR_t *value)
{
  if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303AGR_MAG_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : status_t LSM303AGR_MAG_Get_IntThreshld(u8_t *buff)
* Description    : Read IntThreshld output register
* Input          : pointer to [u8_t]
* Output         : IntThreshld buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_Get_IntThreshld(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM303AGR_MAG_ReadReg(handle, LSM303AGR_MAG_INT_THS_L_REG + k, &buff[k], 1 ))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303AGR_MAG_Set_IntThreshld(u8_t *buff)
* Description    : Write IntThreshld output register
* Input          : pointer to [u8_t]
* Output         : IntThreshld buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_MAG_Set_IntThreshld(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM303AGR_MAG_WriteReg( handle, LSM303AGR_MAG_INT_THS_L_REG + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}
