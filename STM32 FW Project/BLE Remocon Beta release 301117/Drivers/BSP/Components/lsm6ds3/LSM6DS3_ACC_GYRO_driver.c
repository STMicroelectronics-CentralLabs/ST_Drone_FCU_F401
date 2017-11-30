/**
 ******************************************************************************
 * @file    LSM6DS3_ACC_GYRO_driver.c
 * @author  MEMS Application Team
 * @version V2.0
 * @date    27-June-2016
 * @brief   LSM6DS3 driver file
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
#include "LSM6DS3_ACC_GYRO_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Imported function prototypes ----------------------------------------------*/
extern uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name   : LSM6DS3_ACC_GYRO_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output      : None
* Return      : None
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
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
* Function Name   : LSM6DS3_ACC_GYRO_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output      : None
* Return      : None
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
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

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WHO_AM_I
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WHO_AM_I(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BDU
* Description    : Write BDU
* Input          : LSM6DS3_ACC_GYRO_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BDU(void *handle, LSM6DS3_ACC_GYRO_BDU_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_BDU_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BDU
* Description    : Read BDU
* Input          : Pointer to LSM6DS3_ACC_GYRO_BDU_t
* Output         : Status of BDU see LSM6DS3_ACC_GYRO_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BDU(void *handle, LSM6DS3_ACC_GYRO_BDU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FS_XL
* Description    : Write FS_XL
* Input          : LSM6DS3_ACC_GYRO_FS_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FS_XL(void *handle, LSM6DS3_ACC_GYRO_FS_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FS_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FS_XL
* Description    : Read FS_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_FS_XL_t
* Output         : Status of FS_XL see LSM6DS3_ACC_GYRO_FS_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FS_XL(void *handle, LSM6DS3_ACC_GYRO_FS_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FS_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_GYRO_GetRawAccData(u8_t *buff)
* Description    : Read GetAccData output register
* Input          : pointer to [u8_t]
* Output         : GetAccData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_GetRawAccData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_OUTX_L_XL + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_Get_Acceleration(void *handle, int *buff, u8_t from_fifo)
* Description    : Read GetAccData output register
* Input          : pointer to [u8_t]
* Output         : values are expressed in mg
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
/*
 * Following is the table of sensitivity values for each case.
 * Values are espressed in ug/digit.
 */
static const long long LSM6DS3_ACC_Sensitivity_List[4] =
{
  61, /* FS @2g */
  122,  /* FS @4g */
  244,  /* FS @8g */
  488,  /* FS @16g */
};
status_t LSM6DS3_ACC_Get_Acceleration(void *handle, int *buff, u8_t from_fifo)
{
  LSM6DS3_ACC_GYRO_FS_XL_t fs;
  long long sensitivity = 0;
  Type3Axis16bit_U raw_data_tmp;

  /* Read out current odr, fs, hf setting */
  LSM6DS3_ACC_GYRO_R_FS_XL(handle, &fs);

  /* Determine the sensitivity according to fs */
  switch(fs)
  {
    case LSM6DS3_ACC_GYRO_FS_XL_2g:
      sensitivity = LSM6DS3_ACC_Sensitivity_List[0];
      break;

    case LSM6DS3_ACC_GYRO_FS_XL_4g:
      sensitivity = LSM6DS3_ACC_Sensitivity_List[1];
      break;

    case LSM6DS3_ACC_GYRO_FS_XL_8g:
      sensitivity = LSM6DS3_ACC_Sensitivity_List[2];
      break;

    case LSM6DS3_ACC_GYRO_FS_XL_16g:
      sensitivity = LSM6DS3_ACC_Sensitivity_List[3];
      break;
  }

  /* Read out raw accelerometer samples */
  if (from_fifo)
  {
    u8_t i;

    /* read all 3 axis from FIFO */
    for(i = 0; i < 3; i++)
      LSM6DS3_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp.u8bit + 2 * i);
  }
  else
    LSM6DS3_ACC_GYRO_GetRawAccData(handle, raw_data_tmp.u8bit);

  /* Apply proper shift and sensitivity */
  buff[0] = (raw_data_tmp.i16bit[0] * sensitivity + 500) / 1000;
  buff[1] = (raw_data_tmp.i16bit[1] * sensitivity + 500) / 1000;
  buff[2] = (raw_data_tmp.i16bit[2] * sensitivity + 500) / 1000;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ODR_XL
* Description    : Write ODR_XL
* Input          : LSM6DS3_ACC_GYRO_ODR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ODR_XL(void *handle, LSM6DS3_ACC_GYRO_ODR_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ODR_XL
* Description    : Read ODR_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_ODR_XL_t
* Output         : Status of ODR_XL see LSM6DS3_ACC_GYRO_ODR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ODR_XL(void *handle, LSM6DS3_ACC_GYRO_ODR_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ODR_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_translate_ODR_XL
* Description    : Read ODR_XL
* Input          : LSM6DSL_ACC_GYRO_ODR_XL_t
* Output         : The ODR value in Hz
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_translate_ODR_XL(LSM6DS3_ACC_GYRO_ODR_XL_t value, u16_t *odr_hz_val)
{
  switch(value)
  {
    case LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN:
      *odr_hz_val = 0;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_13Hz:
      *odr_hz_val = 13;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_26Hz:
      *odr_hz_val = 26;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_52Hz:
      *odr_hz_val = 52;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_104Hz:
      *odr_hz_val = 104;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_208Hz:
      *odr_hz_val = 208;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_416Hz:
      *odr_hz_val = 416;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_833Hz:
      *odr_hz_val = 833;
      break;

    case LSM6DS3_ACC_GYRO_ODR_XL_1660Hz:
      *odr_hz_val = 1660;
      break;

    default:
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FS_G
* Description    : Write FS_G
* Input          : LSM6DS3_ACC_GYRO_FS_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FS_G(void *handle, LSM6DS3_ACC_GYRO_FS_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FS_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FS_G
* Description    : Read FS_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_FS_G_t
* Output         : Status of FS_G see LSM6DS3_ACC_GYRO_FS_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FS_G(void *handle, LSM6DS3_ACC_GYRO_FS_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FS_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ODR_G
* Description    : Write ODR_G
* Input          : LSM6DS3_ACC_GYRO_ODR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ODR_G(void *handle, LSM6DS3_ACC_GYRO_ODR_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ODR_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ODR_G
* Description    : Read ODR_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_ODR_G_t
* Output         : Status of ODR_G see LSM6DS3_ACC_GYRO_ODR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ODR_G(void *handle, LSM6DS3_ACC_GYRO_ODR_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ODR_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_translate_ODR_G
* Description    : Read ODR_G
* Input          : LSM6DSL_ACC_GYRO_ODR_G_t
* Output         : The ODR value in Hz
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_translate_ODR_G(LSM6DS3_ACC_GYRO_ODR_G_t value, u16_t *odr_hz_val)
{
  switch(value)
  {
    case LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN:
      *odr_hz_val = 0;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_13Hz:
      *odr_hz_val = 13;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_26Hz:
      *odr_hz_val = 26;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_52Hz:
      *odr_hz_val = 52;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_104Hz:
      *odr_hz_val = 104;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_208Hz:
      *odr_hz_val = 208;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_416Hz:
      *odr_hz_val = 416;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_833Hz:
      *odr_hz_val = 833;
      break;

    case LSM6DS3_ACC_GYRO_ODR_G_1660Hz:
      *odr_hz_val = 1660;
      break;

    default:
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_GYRO_GetRawGyroData(u8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [u8_t]
* Output         : GetGyroData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_GetRawGyroData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_OUTX_L_G + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_Get_AngularRate(u8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [u8_t]
* Output         : Returned values are espressed in mdps
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
/*
 * Following is the table of sensitivity values for each case.
 * Values are espressed in udps/digit.
 */
static const long long LSM6DS3_GYRO_Sensitivity_List[5] =
{
  4375, /* FS @125 */
  8750, /* FS @245 */
  17500,  /* FS @500 */
  35000,  /* FS @1000 */
  70000,  /* FS @2000 */
};
status_t LSM6DS3_ACC_Get_AngularRate(void *handle, int *buff, u8_t from_fifo)
{
  LSM6DS3_ACC_GYRO_FS_125_t fs_125;
  LSM6DS3_ACC_GYRO_FS_G_t fs;
  long long sensitivity = 0;
  Type3Axis16bit_U raw_data_tmp;

  /* Read out current odr, fs, hf setting */
  LSM6DS3_ACC_GYRO_R_FS_125(handle, &fs_125);
  if (fs_125 == LSM6DS3_ACC_GYRO_FS_125_ENABLED)
  {
    sensitivity = LSM6DS3_GYRO_Sensitivity_List[0];
  }
  else
  {
    LSM6DS3_ACC_GYRO_R_FS_G(handle, &fs);

    /* Determine the sensitivity according to fs */
    switch(fs)
    {
      case LSM6DS3_ACC_GYRO_FS_G_245dps:
        sensitivity = LSM6DS3_GYRO_Sensitivity_List[1];
        break;

      case LSM6DS3_ACC_GYRO_FS_G_500dps:
        sensitivity = LSM6DS3_GYRO_Sensitivity_List[2];
        break;

      case LSM6DS3_ACC_GYRO_FS_G_1000dps:
        sensitivity = LSM6DS3_GYRO_Sensitivity_List[3];
        break;

      case LSM6DS3_ACC_GYRO_FS_G_2000dps:
        sensitivity = LSM6DS3_GYRO_Sensitivity_List[4];
        break;
    }
  }

  /* Read out raw accelerometer samples */
  if (from_fifo)
  {
    u8_t i;

    /* read all 3 axis from FIFO */
    for(i = 0; i < 3; i++)
      LSM6DS3_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp.u8bit + 2 * i);
  }
  else
    LSM6DS3_ACC_GYRO_GetRawGyroData(handle, raw_data_tmp.u8bit);

  /* Apply proper shift and sensitivity */
  buff[0] = (raw_data_tmp.i16bit[0] * sensitivity + 500) / 1000;
  buff[1] = (raw_data_tmp.i16bit[1] * sensitivity + 500) / 1000;
  buff[2] = (raw_data_tmp.i16bit[2] * sensitivity + 500) / 1000;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BW_XL
* Description    : Write BW_XL
* Input          : LSM6DS3_ACC_GYRO_BW_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BW_XL(void *handle, LSM6DS3_ACC_GYRO_BW_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_BW_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BW_XL
* Description    : Read BW_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_BW_XL_t
* Output         : Status of BW_XL see LSM6DS3_ACC_GYRO_BW_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BW_XL(void *handle, LSM6DS3_ACC_GYRO_BW_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_BW_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FS_125
* Description    : Write FS_125
* Input          : LSM6DS3_ACC_GYRO_FS_125_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FS_125(void *handle, LSM6DS3_ACC_GYRO_FS_125_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FS_125_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FS_125
* Description    : Read FS_125
* Input          : Pointer to LSM6DS3_ACC_GYRO_FS_125_t
* Output         : Status of FS_125 see LSM6DS3_ACC_GYRO_FS_125_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FS_125(void *handle, LSM6DS3_ACC_GYRO_FS_125_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL2_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FS_125_MASK; //mask

  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_EmbeddedAccess
* Description    : Write FUNC_CFG_EN
* Input          : LSM6DS3_ACC_GYRO_EMB_ACC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(void *handle, LSM6DS3_ACC_GYRO_EMB_ACC_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_CFG_ACCESS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DSM_ACC_GYRO_EMB_ACC_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FUNC_CFG_ACCESS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_EmbeddedAccess
* Description    : Read FUNC_CFG_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_EMB_ACC_t
* Output         : Status of FUNC_CFG_EN see LSM6DS3_ACC_GYRO_EMB_ACC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_EmbeddedAccess(void *handle, LSM6DS3_ACC_GYRO_EMB_ACC_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_CFG_ACCESS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DSM_ACC_GYRO_EMB_ACC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_Stamping_Time_Frame
* Description    : Write TPH
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_Stamping_Time_Frame(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_TPH_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_TPH_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS3_ACC_GYRO_TPH_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Stamping_Time_Frame
* Description    : Read TPH
* Input          : Pointer to u8_t
* Output         : Status of TPH
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Stamping_Time_Frame(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TPH_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_TPH_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FIFO_Watermark
* Description    : Write WTM_FIFO
* Input          : u16_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FIFO_Watermark(void *handle, u16_t newValue)
{
  u8_t valueH, valueL;
  u8_t value;

  valueL = newValue & 0xFF;
  valueH = (newValue >> 8) & 0xFF;

  /* Low part goes in FIFO_CTRL1 */
  valueL = valueL << LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_POSITION; //mask
  valueL &= LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_MASK;
  value |= valueL;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL1, &value, 1) )
    return MEMS_ERROR;

  /* High part goes in FIFO_CTRL2 */
  valueH = valueH << LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_POSITION; //mask
  valueH &= LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_MASK;
  value |= valueH;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFO_Watermark
* Description    : Read WTM_FIFO
* Input          : Pointer to u16_t
* Output         : Status of WTM_FIFO
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFO_Watermark(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_CTRL1 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL1, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_MASK; //coerce
  valueL = valueL >> LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL1_POSITION; //mask

  /* High part from FIFO_CTRL2 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_MASK; //coerce
  valueH = valueH >> LSM6DS3_ACC_GYRO_WTM_FIFO_CTRL2_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_Write_En
* Description    : Write TIM_PEDO_FIFO_DRDY
* Input          : LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_Write_En(void *handle, LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_Write_En
* Description    : Read TIM_PEDO_FIFO_DRDY
* Input          : Pointer to LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t
* Output         : Status of TIM_PEDO_FIFO_DRDY see LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_Write_En(void *handle, LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_En
* Description    : Write TIM_PEDO_FIFO_EN
* Input          : LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TIM_PEDO_FIFO_En(void *handle, LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_En
* Description    : Read TIM_PEDO_FIFO_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t
* Output         : Status of TIM_PEDO_FIFO_EN see LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TIM_PEDO_FIFO_En(void *handle, LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL
* Description    : Write DEC_FIFO_XL
* Input          : LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEC_FIFO_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL_val
* Description    : Write DEC_FIFO_XL
* Input          : u16_t
* Output         : Program XL decimation value from unsigned short
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL_val(void *handle, u16_t newValue)
{
  switch(newValue)
  {
    case 0:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO);
      break;

    case 1:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION);
      break;

    case 2:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2);
      break;

    case 3:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3);
      break;

    case 4:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4);
      break;

    case 8:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8);
      break;

    case 16:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16);
      break;

    case 32:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32);
      break;

    default:
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEC_FIFO_XL
* Description    : Read DEC_FIFO_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t
* Output         : Status of DEC_FIFO_XL see LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_XL(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEC_FIFO_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_G
* Description    : Write DEC_FIFO_G
* Input          : LSM6DS3_ACC_GYRO_DEC_FIFO_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEC_FIFO_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_G_val
* Description    : Write DEC_FIFO_G
* Input          : u16_t
* Output         : Program G decimation value from unsigned short
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_G_val(void *handle, u16_t newValue)
{
  switch(newValue)
  {
    case 0:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO);
      break;

    case 1:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION);
      break;

    case 2:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2);
      break;

    case 3:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3);
      break;

    case 4:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4);
      break;

    case 8:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8);
      break;

    case 16:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16);
      break;

    case 32:
      LSM6DS3_ACC_GYRO_W_DEC_FIFO_G(handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32);
      break;

    default:
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEC_FIFO_G
* Description    : Read DEC_FIFO_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEC_FIFO_G_t
* Output         : Status of DEC_FIFO_G see LSM6DS3_ACC_GYRO_DEC_FIFO_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_G(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEC_FIFO_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV0
* Description    : Write DEC_FIFO_SLV0
* Input          : LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV0(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV0
* Description    : Read DEC_FIFO_SLV0
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t
* Output         : Status of DEC_FIFO_SLV0 see LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV0(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEC_FIFO_SLV0_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV1
* Description    : Write DEC_FIFO_SLV1
* Input          : LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEC_FIFO_SLV1(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV1
* Description    : Read DEC_FIFO_SLV1
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t
* Output         : Status of DEC_FIFO_SLV1 see LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEC_FIFO_SLV1(void *handle, LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEC_FIFO_SLV1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HI_DATA_ONLY
* Description    : Write HI_DATA_ONLY
* Input          : LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HI_DATA_ONLY(void *handle, LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HI_DATA_ONLY_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HI_DATA_ONLY
* Description    : Read HI_DATA_ONLY
* Input          : Pointer to LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t
* Output         : Status of HI_DATA_ONLY see LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HI_DATA_ONLY(void *handle, LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HI_DATA_ONLY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FIFO_MODE
* Description    : Write FIFO_MODE
* Input          : LSM6DS3_ACC_GYRO_FIFO_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FIFO_MODE(void *handle, LSM6DS3_ACC_GYRO_FIFO_MODE_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFO_MODE
* Description    : Read FIFO_MODE
* Input          : Pointer to LSM6DS3_ACC_GYRO_FIFO_MODE_t
* Output         : Status of FIFO_MODE see LSM6DS3_ACC_GYRO_FIFO_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFO_MODE(void *handle, LSM6DS3_ACC_GYRO_FIFO_MODE_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FIFO_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ODR_FIFO
* Description    : Write ODR_FIFO
* Input          : LSM6DS3_ACC_GYRO_ODR_FIFO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ODR_FIFO(void *handle, LSM6DS3_ACC_GYRO_ODR_FIFO_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ODR_FIFO_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ODR_FIFO
* Description    : Read ODR_FIFO
* Input          : Pointer to LSM6DS3_ACC_GYRO_ODR_FIFO_t
* Output         : Status of ODR_FIFO see LSM6DS3_ACC_GYRO_ODR_FIFO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ODR_FIFO(void *handle, LSM6DS3_ACC_GYRO_ODR_FIFO_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ODR_FIFO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_Orientation
* Description    : Write ORIENT
* Input          : LSM6DS3_ACC_GYRO_ORIENT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_Orientation(void *handle, LSM6DS3_ACC_GYRO_ORIENT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ORIENT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Orientation
* Description    : Read ORIENT
* Input          : Pointer to LSM6DS3_ACC_GYRO_ORIENT_t
* Output         : Status of ORIENT see LSM6DS3_ACC_GYRO_ORIENT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Orientation(void *handle, LSM6DS3_ACC_GYRO_ORIENT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ORIENT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SignZ_G
* Description    : Write SIGN_Z_G
* Input          : LSM6DS3_ACC_GYRO_SIGN_Z_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SignZ_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_Z_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIGN_Z_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SignZ_G
* Description    : Read SIGN_Z_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIGN_Z_G_t
* Output         : Status of SIGN_Z_G see LSM6DS3_ACC_GYRO_SIGN_Z_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SignZ_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_Z_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIGN_Z_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SignY_G
* Description    : Write SIGN_Y_G
* Input          : LSM6DS3_ACC_GYRO_SIGN_Y_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SignY_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_Y_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIGN_Y_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SignY_G
* Description    : Read SIGN_Y_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIGN_Y_G_t
* Output         : Status of SIGN_Y_G see LSM6DS3_ACC_GYRO_SIGN_Y_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SignY_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_Y_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIGN_Y_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SignX_G
* Description    : Write SIGN_X_G
* Input          : LSM6DS3_ACC_GYRO_SIGN_X_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SignX_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_X_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIGN_X_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SignX_G
* Description    : Read SIGN_X_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIGN_X_G_t
* Output         : Status of SIGN_X_G see LSM6DS3_ACC_GYRO_SIGN_X_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SignX_G(void *handle, LSM6DS3_ACC_GYRO_SIGN_X_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIGN_X_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT1
* Description    : Write INT1_DRDY_XL
* Input          : LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_DRDY_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT1
* Description    : Read INT1_DRDY_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t
* Output         : Status of INT1_DRDY_XL see LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_DRDY_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT1
* Description    : Write INT1_DRDY_G
* Input          : LSM6DS3_ACC_GYRO_INT1_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_DRDY_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_DRDY_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT1
* Description    : Read INT1_DRDY_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_DRDY_G_t
* Output         : Status of INT1_DRDY_G see LSM6DS3_ACC_GYRO_INT1_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_DRDY_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_DRDY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BOOT_on_INT1
* Description    : Write INT1_BOOT
* Input          : LSM6DS3_ACC_GYRO_INT1_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BOOT_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_BOOT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_BOOT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BOOT_on_INT1
* Description    : Read INT1_BOOT
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_BOOT_t
* Output         : Status of INT1_BOOT see LSM6DS3_ACC_GYRO_INT1_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BOOT_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_BOOT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT1
* Description    : Write INT1_FTH
* Input          : LSM6DS3_ACC_GYRO_INT1_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_FTH_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_FTH_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT1
* Description    : Read INT1_FTH
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_FTH_t
* Output         : Status of INT1_FTH see LSM6DS3_ACC_GYRO_INT1_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_FTH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT1
* Description    : Write INT1_OVR
* Input          : LSM6DS3_ACC_GYRO_INT1_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_OVR_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_OVR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT1
* Description    : Read INT1_OVR
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_OVR_t
* Output         : Status of INT1_OVR see LSM6DS3_ACC_GYRO_INT1_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_OVR_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FSS5_on_INT1
* Description    : Write INT1_FSS5
* Input          : LSM6DS3_ACC_GYRO_INT1_FSS5_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FSS5_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_FSS5_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_FSS5_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FSS5_on_INT1
* Description    : Read INT1_FSS5
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_FSS5_t
* Output         : Status of INT1_FSS5 see LSM6DS3_ACC_GYRO_INT1_FSS5_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FSS5_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_FSS5_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_FSS5_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT1
* Description    : Write INT1_SIGN_MOT
* Input          : LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT1
* Description    : Read INT1_SIGN_MOT
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t
* Output         : Status of INT1_SIGN_MOT see LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1
* Description    : Write INT1_PEDO
* Input          : LSM6DS3_ACC_GYRO_INT1_PEDO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_PEDO_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_PEDO_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT1
* Description    : Read INT1_PEDO
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_PEDO_t
* Output         : Status of INT1_PEDO see LSM6DS3_ACC_GYRO_INT1_PEDO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT1(void *handle, LSM6DS3_ACC_GYRO_INT1_PEDO_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_PEDO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT2
* Description    : Write INT2_DRDY_XL
* Input          : LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_DRDY_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT2
* Description    : Read INT2_DRDY_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t
* Output         : Status of INT2_DRDY_XL see LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_XL_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_DRDY_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2
* Description    : Write INT2_DRDY_G
* Input          : LSM6DS3_ACC_GYRO_INT2_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_DRDY_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT2
* Description    : Read INT2_DRDY_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_DRDY_G_t
* Output         : Status of INT2_DRDY_G see LSM6DS3_ACC_GYRO_INT2_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_G_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_DRDY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_TEMP_on_INT2
* Description    : Write INT2_DRDY_TEMP
* Input          : LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_TEMP_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_TEMP_on_INT2
* Description    : Read INT2_DRDY_TEMP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_t
* Output         : Status of INT2_DRDY_TEMP see LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_TEMP_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT2
* Description    : Write INT2_FTH
* Input          : LSM6DS3_ACC_GYRO_INT2_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_FTH_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_FTH_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT2
* Description    : Read INT2_FTH
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_FTH_t
* Output         : Status of INT2_FTH see LSM6DS3_ACC_GYRO_INT2_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFO_TSHLD_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_FTH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT2
* Description    : Write INT2_OVR
* Input          : LSM6DS3_ACC_GYRO_INT2_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_OVR_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_OVR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT2
* Description    : Read INT2_OVR
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_OVR_t
* Output         : Status of INT2_OVR see LSM6DS3_ACC_GYRO_INT2_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_OVERRUN_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_OVR_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FSS5_on_INT2
* Description    : Write INT2_FSS5
* Input          : LSM6DS3_ACC_GYRO_INT2_FSS5_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FSS5_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_FSS5_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_FSS5_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FSS5_on_INT2
* Description    : Read INT2_FSS5
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_FSS5_t
* Output         : Status of INT2_FSS5 see LSM6DS3_ACC_GYRO_INT2_FSS5_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FSS5_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_FSS5_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_FSS5_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT2
* Description    : Write INT2_SIGN_MOT
* Input          : LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT2
* Description    : Read INT2_SIGN_MOT
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t
* Output         : Status of INT2_SIGN_MOT see LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT2
* Description    : Write INT2_PEDO
* Input          : LSM6DS3_ACC_GYRO_INT2_PEDO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_PEDO_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_PEDO_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT2
* Description    : Read INT2_PEDO
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_PEDO_t
* Output         : Status of INT2_PEDO see LSM6DS3_ACC_GYRO_INT2_PEDO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PEDO_STEP_on_INT2(void *handle, LSM6DS3_ACC_GYRO_INT2_PEDO_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_PEDO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SW_RESET
* Description    : Write SW_RESET
* Input          : LSM6DS3_ACC_GYRO_SW_RESET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SW_RESET(void *handle, LSM6DS3_ACC_GYRO_SW_RESET_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SW_RESET_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SW_RESET
* Description    : Read SW_RESET
* Input          : Pointer to LSM6DS3_ACC_GYRO_SW_RESET_t
* Output         : Status of SW_RESET see LSM6DS3_ACC_GYRO_SW_RESET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SW_RESET(void *handle, LSM6DS3_ACC_GYRO_SW_RESET_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SW_RESET_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BLE
* Description    : Write BLE
* Input          : LSM6DS3_ACC_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BLE(void *handle, LSM6DS3_ACC_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_BLE_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BLE
* Description    : Read BLE
* Input          : Pointer to LSM6DS3_ACC_GYRO_BLE_t
* Output         : Status of BLE see LSM6DS3_ACC_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BLE(void *handle, LSM6DS3_ACC_GYRO_BLE_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_IF_Addr_Incr
* Description    : Write IF_INC
* Input          : LSM6DS3_ACC_GYRO_IF_INC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(void *handle, LSM6DS3_ACC_GYRO_IF_INC_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_IF_INC_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_IF_Addr_Incr
* Description    : Read IF_INC
* Input          : Pointer to LSM6DS3_ACC_GYRO_IF_INC_t
* Output         : Status of IF_INC see LSM6DS3_ACC_GYRO_IF_INC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_IF_Addr_Incr(void *handle, LSM6DS3_ACC_GYRO_IF_INC_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_IF_INC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SPI_Mode
* Description    : Write SIM
* Input          : LSM6DS3_ACC_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SPI_Mode(void *handle, LSM6DS3_ACC_GYRO_SIM_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIM_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SPI_Mode
* Description    : Read SIM
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIM_t
* Output         : Status of SIM see LSM6DS3_ACC_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SPI_Mode(void *handle, LSM6DS3_ACC_GYRO_SIM_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PadSel
* Description    : Write PP_OD
* Input          : LSM6DS3_ACC_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PadSel(void *handle, LSM6DS3_ACC_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_PP_OD_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PadSel
* Description    : Read PP_OD
* Input          : Pointer to LSM6DS3_ACC_GYRO_PP_OD_t
* Output         : Status of PP_OD see LSM6DS3_ACC_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PadSel(void *handle, LSM6DS3_ACC_GYRO_PP_OD_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_INT_ACT_LEVEL
* Description    : Write INT_ACT_LEVEL
* Input          : LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_INT_ACT_LEVEL(void *handle, LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_INT_ACT_LEVEL
* Description    : Read INT_ACT_LEVEL
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t
* Output         : Status of INT_ACT_LEVEL see LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_INT_ACT_LEVEL(void *handle, LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT_ACT_LEVEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BOOT
* Description    : Write BOOT
* Input          : LSM6DS3_ACC_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BOOT(void *handle, LSM6DS3_ACC_GYRO_BOOT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_BOOT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BOOT
* Description    : Read BOOT
* Input          : Pointer to LSM6DS3_ACC_GYRO_BOOT_t
* Output         : Status of BOOT see LSM6DS3_ACC_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BOOT(void *handle, LSM6DS3_ACC_GYRO_BOOT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_STOP_ON_FTH
* Description    : Write STOP_ON_FTH
* Input          : LSM6DS3_ACC_GYRO_STOP_ON_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_STOP_ON_FTH(void *handle, LSM6DS3_ACC_GYRO_STOP_ON_FTH_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_STOP_ON_FTH_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_STOP_ON_FTH
* Description    : Read STOP_ON_FTH
* Input          : Pointer to LSM6DS3_ACC_GYRO_STOP_ON_FTH_t
* Output         : Status of STOP_ON_FTH see LSM6DS3_ACC_GYRO_STOP_ON_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_STOP_ON_FTH(void *handle, LSM6DS3_ACC_GYRO_STOP_ON_FTH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_STOP_ON_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_MODE3_Enable
* Description    : Write MODE3_EN
* Input          : LSM6DS3_ACC_GYRO_MODE3_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_MODE3_Enable(void *handle, LSM6DS3_ACC_GYRO_MODE3_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_MODE3_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_MODE3_Enable
* Description    : Read MODE3_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_MODE3_EN_t
* Output         : Status of MODE3_EN see LSM6DS3_ACC_GYRO_MODE3_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_MODE3_Enable(void *handle, LSM6DS3_ACC_GYRO_MODE3_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_MODE3_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_I2C_DISABLE
* Description    : Write I2C_DISABLE
* Input          : LSM6DS3_ACC_GYRO_I2C_DISABLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_I2C_DISABLE(void *handle, LSM6DS3_ACC_GYRO_I2C_DISABLE_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_I2C_DISABLE_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_I2C_DISABLE
* Description    : Read I2C_DISABLE
* Input          : Pointer to LSM6DS3_ACC_GYRO_I2C_DISABLE_t
* Output         : Status of I2C_DISABLE see LSM6DS3_ACC_GYRO_I2C_DISABLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_I2C_DISABLE(void *handle, LSM6DS3_ACC_GYRO_I2C_DISABLE_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_I2C_DISABLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_MSK
* Description    : Write DRDY_MSK
* Input          : LSM6DS3_ACC_GYRO_DRDY_MSK_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_MSK(void *handle, LSM6DS3_ACC_GYRO_DRDY_MSK_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DRDY_MSK_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_MSK
* Description    : Read DRDY_MSK
* Input          : Pointer to LSM6DS3_ACC_GYRO_DRDY_MSK_t
* Output         : Status of DRDY_MSK see LSM6DS3_ACC_GYRO_DRDY_MSK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_MSK(void *handle, LSM6DS3_ACC_GYRO_DRDY_MSK_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DRDY_MSK_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FIFO_TEMP_EN
* Description    : Write FIFO_TEMP_EN
* Input          : LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FIFO_TEMP_EN(void *handle, LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFO_TEMP_EN
* Description    : Read FIFO_TEMP_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : Status of FIFO_TEMP_EN see LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFO_TEMP_EN(void *handle, LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FIFO_TEMP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_INT2_ON_INT1
* Description    : Write INT2_ON_INT1
* Input          : LSM6DS3_ACC_GYRO_INT2_ON_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_INT2_ON_INT1(void *handle, LSM6DS3_ACC_GYRO_INT2_ON_INT1_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_ON_INT1_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_INT2_ON_INT1
* Description    : Read INT2_ON_INT1
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_ON_INT1_t
* Output         : Status of INT2_ON_INT1 see LSM6DS3_ACC_GYRO_INT2_ON_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_INT2_ON_INT1(void *handle, LSM6DS3_ACC_GYRO_INT2_ON_INT1_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_ON_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SleepMode_G
* Description    : Write SLEEP_G
* Input          : LSM6DS3_ACC_GYRO_SLEEP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SleepMode_G(void *handle, LSM6DS3_ACC_GYRO_SLEEP_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SLEEP_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SleepMode_G
* Description    : Read SLEEP_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_SLEEP_G_t
* Output         : Status of SLEEP_G see LSM6DS3_ACC_GYRO_SLEEP_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SleepMode_G(void *handle, LSM6DS3_ACC_GYRO_SLEEP_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SLEEP_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_BW_Fixed_By_ODR
* Description    : Write BW_SCAL_ODR
* Input          : LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_BW_Fixed_By_ODR(void *handle, LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_BW_SCAL_ODR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_BW_Fixed_By_ODR
* Description    : Read BW_SCAL_ODR
* Input          : Pointer to LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t
* Output         : Status of BW_SCAL_ODR see LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_BW_Fixed_By_ODR(void *handle, LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SelfTest_XL
* Description    : Write ST_XL
* Input          : LSM6DS3_ACC_GYRO_ST_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SelfTest_XL(void *handle, LSM6DS3_ACC_GYRO_ST_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ST_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SelfTest_XL
* Description    : Read ST_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_ST_XL_t
* Output         : Status of ST_XL see LSM6DS3_ACC_GYRO_ST_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SelfTest_XL(void *handle, LSM6DS3_ACC_GYRO_ST_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ST_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SelfTest_G
* Description    : Write ST_G
* Input          : LSM6DS3_ACC_GYRO_ST_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SelfTest_G(void *handle, LSM6DS3_ACC_GYRO_ST_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ST_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SelfTest_G
* Description    : Read ST_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_ST_G_t
* Output         : Status of ST_G see LSM6DS3_ACC_GYRO_ST_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SelfTest_G(void *handle, LSM6DS3_ACC_GYRO_ST_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ST_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_CircularBurstMode
* Description    : Write ST_ROUNDING
* Input          : LSM6DS3_ACC_GYRO_ROUNDING_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_CircularBurstMode(void *handle, LSM6DS3_ACC_GYRO_ROUNDING_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LSM6DS3_ACC_GYRO_ROUNDING_t_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_CircularBurstMode
* Description    : Read ST_ROUNDING
* Input          : Pointer to LSM6DS3_ACC_GYRO_ROUNDING_t
* Output         : Status of ST_ROUNDING see LSM6DS3_ACC_GYRO_ROUNDING_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM6DS3_ACC_GYRO_R_CircularBurstMode(void *handle, LSM6DS3_ACC_GYRO_ROUNDING_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL5_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LSM6DS3_ACC_GYRO_ROUNDING_t_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_LowPower_XL
* Description    : Write LP_XL
* Input          : LSM6DS3_ACC_GYRO_LP_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_LowPower_XL(void *handle, LSM6DS3_ACC_GYRO_LP_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LP_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_LowPower_XL
* Description    : Read LP_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_LP_XL_t
* Output         : Status of LP_XL see LSM6DS3_ACC_GYRO_LP_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_LowPower_XL(void *handle, LSM6DS3_ACC_GYRO_LP_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LP_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEN_LVL2_EN
* Description    : Write DEN_LVL2_EN
* Input          : LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEN_LVL2_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEN_LVL2_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEN_LVL2_EN
* Description    : Read DEN_LVL2_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t
* Output         : Status of DEN_LVL2_EN see LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEN_LVL2_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_LVL2_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEN_LVL2_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEN_LVL_EN
* Description    : Write DEN_LVL_EN
* Input          : LSM6DS3_ACC_GYRO_DEN_LVL_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEN_LVL_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_LVL_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEN_LVL_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEN_LVL_EN
* Description    : Read DEN_LVL_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEN_LVL_EN_t
* Output         : Status of DEN_LVL_EN see LSM6DS3_ACC_GYRO_DEN_LVL_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEN_LVL_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_LVL_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEN_LVL_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DEN_EDGE_EN
* Description    : Write DEN_EDGE_EN
* Input          : LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DEN_EDGE_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DEN_EDGE_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DEN_EDGE_EN
* Description    : Read DEN_EDGE_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t
* Output         : Status of DEN_EDGE_EN see LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DEN_EDGE_EN(void *handle, LSM6DS3_ACC_GYRO_DEN_EDGE_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL6_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DEN_EDGE_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HPCF_G
* Description    : Write HPCF_G
* Input          : LSM6DS3_ACC_GYRO_HPCF_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HPCF_G(void *handle, LSM6DS3_ACC_GYRO_HPCF_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HPCF_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HPCF_G
* Description    : Read HPCF_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_HPCF_G_t
* Output         : Status of HPCF_G see LSM6DS3_ACC_GYRO_HPCF_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HPCF_G(void *handle, LSM6DS3_ACC_GYRO_HPCF_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HPCF_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HPFilter_En
* Description    : Write HP_EN
* Input          : LSM6DS3_ACC_GYRO_HP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HPFilter_En(void *handle, LSM6DS3_ACC_GYRO_HP_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HP_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HPFilter_En
* Description    : Read HP_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_HP_EN_t
* Output         : Status of HP_EN see LSM6DS3_ACC_GYRO_HP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HPFilter_En(void *handle, LSM6DS3_ACC_GYRO_HP_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_LP_Mode
* Description    : Write LP_EN
* Input          : LSM6DS3_ACC_GYRO_LP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_LP_Mode(void *handle, LSM6DS3_ACC_GYRO_LP_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LP_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_LP_Mode
* Description    : Read LP_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_LP_EN_t
* Output         : Status of LP_EN see LSM6DS3_ACC_GYRO_LP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_LP_Mode(void *handle, LSM6DS3_ACC_GYRO_LP_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ROUNDING_STATUS
* Description    : Write ROUNDING_STATUS
* Input          : LSM6DS3_ACC_GYRO_ROUNDING_STATUS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ROUNDING_STATUS(void *handle, LSM6DS3_ACC_GYRO_ROUNDING_STATUS_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ROUNDING_STATUS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ROUNDING_STATUS
* Description    : Read ROUNDING_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_ROUNDING_STATUS_t
* Output         : Status of ROUNDING_STATUS see LSM6DS3_ACC_GYRO_ROUNDING_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ROUNDING_STATUS(void *handle, LSM6DS3_ACC_GYRO_ROUNDING_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ROUNDING_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HP_G_RST
* Description    : Write HP_G_RST
* Input          : LSM6DS3_ACC_GYRO_HP_G_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HP_G_RST(void *handle, LSM6DS3_ACC_GYRO_HP_G_RST_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HP_G_RST_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HP_G_RST
* Description    : Read HP_G_RST
* Input          : Pointer to LSM6DS3_ACC_GYRO_HP_G_RST_t
* Output         : Status of HP_G_RST see LSM6DS3_ACC_GYRO_HP_G_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HP_G_RST(void *handle, LSM6DS3_ACC_GYRO_HP_G_RST_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HP_G_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_LPF2_XL
* Description    : Write LPF2_XL_EN
* Input          : LSM6DS3_ACC_GYRO_LPF2_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_LPF2_XL(void *handle, LSM6DS3_ACC_GYRO_LPF2_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LPF2_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_LPF2_XL
* Description    : Read LPF2_XL_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_LPF2_XL_t
* Output         : Status of LPF2_XL_EN see LSM6DS3_ACC_GYRO_LPF2_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_LPF2_XL(void *handle, LSM6DS3_ACC_GYRO_LPF2_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LPF2_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HPCF_XL
* Description    : Write HPCF_XL
* Input          : LSM6DS3_ACC_GYRO_HPCF_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HPCF_XL(void *handle, LSM6DS3_ACC_GYRO_HPCF_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HPCF_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HPCF_XL
* Description    : Read HPCF_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_HPCF_XL_t
* Output         : Status of HPCF_XL see LSM6DS3_ACC_GYRO_HPCF_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HPCF_XL(void *handle, LSM6DS3_ACC_GYRO_HPCF_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HPCF_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_LOW_PASS_ON_6D
* Description    : Write LOW_PASS_ON_6D
* Input          : LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_LOW_PASS_ON_6D(void *handle, LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_LOW_PASS_ON_6D
* Description    : Read LOW_PASS_ON_6D
* Input          : Pointer to LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_t
* Output         : Status of LOW_PASS_ON_6D see LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_LOW_PASS_ON_6D(void *handle, LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LOW_PASS_ON_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_HP_SLOPE_XL
* Description    : Write HP_SLOPE_XL_EN
* Input          : LSM6DS3_ACC_GYRO_HP_SLOPE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_HP_SLOPE_XL(void *handle, LSM6DS3_ACC_GYRO_HP_SLOPE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_HP_SLOPE_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_HP_SLOPE_XL
* Description    : Read HP_SLOPE_XL_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_HP_SLOPE_XL_t
* Output         : Status of HP_SLOPE_XL_EN see LSM6DS3_ACC_GYRO_HP_SLOPE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_HP_SLOPE_XL(void *handle, LSM6DS3_ACC_GYRO_HP_SLOPE_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_HP_SLOPE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_XEN_XL
* Description    : Write XEN_XL
* Input          : LSM6DS3_ACC_GYRO_XEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_XEN_XL(void *handle, LSM6DS3_ACC_GYRO_XEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_XEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_XEN_XL
* Description    : Read XEN_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_XEN_XL_t
* Output         : Status of XEN_XL see LSM6DS3_ACC_GYRO_XEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_XEN_XL(void *handle, LSM6DS3_ACC_GYRO_XEN_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_XEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_YEN_XL
* Description    : Write YEN_XL
* Input          : LSM6DS3_ACC_GYRO_YEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_YEN_XL(void *handle, LSM6DS3_ACC_GYRO_YEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_YEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_YEN_XL
* Description    : Read YEN_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_YEN_XL_t
* Output         : Status of YEN_XL see LSM6DS3_ACC_GYRO_YEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_YEN_XL(void *handle, LSM6DS3_ACC_GYRO_YEN_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_YEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ZEN_XL
* Description    : Write ZEN_XL
* Input          : LSM6DS3_ACC_GYRO_ZEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ZEN_XL(void *handle, LSM6DS3_ACC_GYRO_ZEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ZEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ZEN_XL
* Description    : Read ZEN_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_ZEN_XL_t
* Output         : Status of ZEN_XL see LSM6DS3_ACC_GYRO_ZEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ZEN_XL(void *handle, LSM6DS3_ACC_GYRO_ZEN_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ZEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SOFT
* Description    : Write SOFT_EN
* Input          : LSM6DS3_ACC_GYRO_SOFT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SOFT(void *handle, LSM6DS3_ACC_GYRO_SOFT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SOFT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SOFT
* Description    : Read SOFT_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_SOFT_t
* Output         : Status of SOFT_EN see LSM6DS3_ACC_GYRO_SOFT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SOFT(void *handle, LSM6DS3_ACC_GYRO_SOFT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL9_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SOFT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SignifcantMotion
* Description    : Write SIGN_MOTION_EN
* Input          : LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SignifcantMotion(void *handle, LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SignifcantMotion
* Description    : Read SIGN_MOTION_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t
* Output         : Status of SIGN_MOTION_EN see LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SignifcantMotion(void *handle, LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIGN_MOTION_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PedoStepReset
* Description    : Write PEDO_RST_STEP
* Input          : LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PedoStepReset(void *handle, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_PEDO_RST_STEP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PedoStepReset
* Description    : Read PEDO_RST_STEP
* Input          : Pointer to LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t
* Output         : Status of PEDO_RST_STEP see LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PedoStepReset(void *handle, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PEDO_RST_STEP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_XEN_G
* Description    : Write XEN_G
* Input          : LSM6DS3_ACC_GYRO_XEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_XEN_G(void *handle, LSM6DS3_ACC_GYRO_XEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_XEN_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_XEN_G
* Description    : Read XEN_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_XEN_G_t
* Output         : Status of XEN_G see LSM6DS3_ACC_GYRO_XEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_XEN_G(void *handle, LSM6DS3_ACC_GYRO_XEN_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_XEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_YEN_G
* Description    : Write YEN_G
* Input          : LSM6DS3_ACC_GYRO_YEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_YEN_G(void *handle, LSM6DS3_ACC_GYRO_YEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_YEN_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_YEN_G
* Description    : Read YEN_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_YEN_G_t
* Output         : Status of YEN_G see LSM6DS3_ACC_GYRO_YEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_YEN_G(void *handle, LSM6DS3_ACC_GYRO_YEN_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_YEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_ZEN_G
* Description    : Write ZEN_G
* Input          : LSM6DS3_ACC_GYRO_ZEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_ZEN_G(void *handle, LSM6DS3_ACC_GYRO_ZEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_ZEN_G_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_ZEN_G
* Description    : Read ZEN_G
* Input          : Pointer to LSM6DS3_ACC_GYRO_ZEN_G_t
* Output         : Status of ZEN_G see LSM6DS3_ACC_GYRO_ZEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_ZEN_G(void *handle, LSM6DS3_ACC_GYRO_ZEN_G_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_ZEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FUNC_EN
* Description    : Write FUNC_EN
* Input          : LSM6DS3_ACC_GYRO_FUNC_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FUNC_EN(void *handle, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FUNC_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FUNC_EN
* Description    : Read FUNC_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_FUNC_EN_t
* Output         : Status of FUNC_EN see LSM6DS3_ACC_GYRO_FUNC_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FUNC_EN(void *handle, LSM6DS3_ACC_GYRO_FUNC_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FUNC_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable
* Description    : Write MASTER_ON
* Input          : LSM6DS3_ACC_GYRO_MASTER_ON_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(void *handle, LSM6DS3_ACC_GYRO_MASTER_ON_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_MASTER_ON_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_I2C_MASTER_Enable
* Description    : Read MASTER_ON
* Input          : Pointer to LSM6DS3_ACC_GYRO_MASTER_ON_t
* Output         : Status of MASTER_ON see LSM6DS3_ACC_GYRO_MASTER_ON_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_I2C_MASTER_Enable(void *handle, LSM6DS3_ACC_GYRO_MASTER_ON_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_MASTER_ON_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_IronCorrection_EN
* Description    : Write IRON_EN
* Input          : LSM6DS3_ACC_GYRO_IRON_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_IronCorrection_EN(void *handle, LSM6DS3_ACC_GYRO_IRON_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_IRON_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_IronCorrection_EN
* Description    : Read IRON_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_IRON_EN_t
* Output         : Status of IRON_EN see LSM6DS3_ACC_GYRO_IRON_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_IronCorrection_EN(void *handle, LSM6DS3_ACC_GYRO_IRON_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_IRON_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PASS_THRU_MODE
* Description    : Write PASS_THRU_MODE
* Input          : LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PASS_THRU_MODE(void *handle, LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_PASS_THRU_MODE_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PASS_THRU_MODE
* Description    : Read PASS_THRU_MODE
* Input          : Pointer to LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t
* Output         : Status of PASS_THRU_MODE see LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PASS_THRU_MODE(void *handle, LSM6DS3_ACC_GYRO_PASS_THRU_MODE_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PASS_THRU_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PULL_UP_EN
* Description    : Write PULL_UP_EN
* Input          : LSM6DS3_ACC_GYRO_PULL_UP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PULL_UP_EN(void *handle, LSM6DS3_ACC_GYRO_PULL_UP_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_PULL_UP_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PULL_UP_EN
* Description    : Read PULL_UP_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_PULL_UP_EN_t
* Output         : Status of PULL_UP_EN see LSM6DS3_ACC_GYRO_PULL_UP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PULL_UP_EN(void *handle, LSM6DS3_ACC_GYRO_PULL_UP_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PULL_UP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SensorHUB_Trigger_Sel
* Description    : Write START_CONFIG
* Input          : LSM6DS3_ACC_GYRO_START_CONFIG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SensorHUB_Trigger_Sel(void *handle, LSM6DS3_ACC_GYRO_START_CONFIG_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_START_CONFIG_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SensorHUB_Trigger_Sel
* Description    : Read START_CONFIG
* Input          : Pointer to LSM6DS3_ACC_GYRO_START_CONFIG_t
* Output         : Status of START_CONFIG see LSM6DS3_ACC_GYRO_START_CONFIG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SensorHUB_Trigger_Sel(void *handle, LSM6DS3_ACC_GYRO_START_CONFIG_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_START_CONFIG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DATA_VAL_SEL_FIFO
* Description    : Write DATA_VAL_SEL_FIFO
* Input          : LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DATA_VAL_SEL_FIFO(void *handle, LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DATA_VAL_SEL_FIFO
* Description    : Read DATA_VAL_SEL_FIFO
* Input          : Pointer to LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Output         : Status of DATA_VAL_SEL_FIFO see LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DATA_VAL_SEL_FIFO(void *handle, LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DRDY_ON_INT1
* Description    : Write DRDY_ON_INT1
* Input          : LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DRDY_ON_INT1(void *handle, LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DRDY_ON_INT1_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DRDY_ON_INT1
* Description    : Read DRDY_ON_INT1
* Input          : Pointer to LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t
* Output         : Status of DRDY_ON_INT1 see LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DRDY_ON_INT1(void *handle, LSM6DS3_ACC_GYRO_DRDY_ON_INT1_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DRDY_ON_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Z_WU
* Description    : Read Z_WU
* Input          : Pointer to LSM6DS3_ACC_GYRO_Z_WU_t
* Output         : Status of Z_WU see LSM6DS3_ACC_GYRO_Z_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Z_WU(void *handle, LSM6DS3_ACC_GYRO_Z_WU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_Z_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Y_WU
* Description    : Read Y_WU
* Input          : Pointer to LSM6DS3_ACC_GYRO_Y_WU_t
* Output         : Status of Y_WU see LSM6DS3_ACC_GYRO_Y_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Y_WU(void *handle, LSM6DS3_ACC_GYRO_Y_WU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_Y_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_X_WU
* Description    : Read X_WU
* Input          : Pointer to LSM6DS3_ACC_GYRO_X_WU_t
* Output         : Status of X_WU see LSM6DS3_ACC_GYRO_X_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_X_WU(void *handle, LSM6DS3_ACC_GYRO_X_WU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_X_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WU_EV_STATUS
* Description    : Read WU_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_WU_EV_STATUS_t
* Output         : Status of WU_EV_STATUS see LSM6DS3_ACC_GYRO_WU_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WU_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_WU_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_WU_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SLEEP_EV_STATUS
* Description    : Read SLEEP_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_t
* Output         : Status of SLEEP_EV_STATUS see LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SLEEP_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SLEEP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FF_EV_STATUS
* Description    : Read FF_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_FF_EV_STATUS_t
* Output         : Status of FF_EV_STATUS see LSM6DS3_ACC_GYRO_FF_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FF_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_FF_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FF_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Z_TAP
* Description    : Read Z_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_Z_TAP_t
* Output         : Status of Z_TAP see LSM6DS3_ACC_GYRO_Z_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Z_TAP(void *handle, LSM6DS3_ACC_GYRO_Z_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_Z_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_Y_TAP
* Description    : Read Y_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_Y_TAP_t
* Output         : Status of Y_TAP see LSM6DS3_ACC_GYRO_Y_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_Y_TAP(void *handle, LSM6DS3_ACC_GYRO_Y_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_Y_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_X_TAP
* Description    : Read X_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_X_TAP_t
* Output         : Status of X_TAP see LSM6DS3_ACC_GYRO_X_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_X_TAP(void *handle, LSM6DS3_ACC_GYRO_X_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_X_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_SIGN
* Description    : Read TAP_SIGN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TAP_SIGN_t
* Output         : Status of TAP_SIGN see LSM6DS3_ACC_GYRO_TAP_SIGN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_SIGN(void *handle, LSM6DS3_ACC_GYRO_TAP_SIGN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_SIGN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS
* Description    : Read DOUBLE_TAP_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t
* Output         : Status of DOUBLE_TAP_EV_STATUS see LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SINGLE_TAP_EV_STATUS
* Description    : Read SINGLE_TAP_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t
* Output         : Status of SINGLE_TAP_EV_STATUS see LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SINGLE_TAP_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_EV_STATUS
* Description    : Read TAP_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_TAP_EV_STATUS_t
* Output         : Status of TAP_EV_STATUS see LSM6DS3_ACC_GYRO_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_TAP_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_XL
* Description    : Read DSD_XL
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_XL_t
* Output         : Status of DSD_XL see LSM6DS3_ACC_GYRO_DSD_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_XL(void *handle, LSM6DS3_ACC_GYRO_DSD_XL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_XH
* Description    : Read DSD_XH
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_XH_t
* Output         : Status of DSD_XH see LSM6DS3_ACC_GYRO_DSD_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_XH(void *handle, LSM6DS3_ACC_GYRO_DSD_XH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_YL
* Description    : Read DSD_YL
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_YL_t
* Output         : Status of DSD_YL see LSM6DS3_ACC_GYRO_DSD_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_YL(void *handle, LSM6DS3_ACC_GYRO_DSD_YL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_YH
* Description    : Read DSD_YH
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_YH_t
* Output         : Status of DSD_YH see LSM6DS3_ACC_GYRO_DSD_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_YH(void *handle, LSM6DS3_ACC_GYRO_DSD_YH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_ZL
* Description    : Read DSD_ZL
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_ZL_t
* Output         : Status of DSD_ZL see LSM6DS3_ACC_GYRO_DSD_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_ZL(void *handle, LSM6DS3_ACC_GYRO_DSD_ZL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DSD_ZH
* Description    : Read DSD_ZH
* Input          : Pointer to LSM6DS3_ACC_GYRO_DSD_ZH_t
* Output         : Status of DSD_ZH see LSM6DS3_ACC_GYRO_DSD_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DSD_ZH(void *handle, LSM6DS3_ACC_GYRO_DSD_ZH_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DSD_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_D6D_EV_STATUS
* Description    : Read D6D_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t
* Output         : Status of D6D_EV_STATUS see LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_D6D_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_D6D_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_D6D_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_XLDA
* Description    : Read XLDA
* Input          : Pointer to LSM6DS3_ACC_GYRO_XLDA_t
* Output         : Status of XLDA see LSM6DS3_ACC_GYRO_XLDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_XLDA(void *handle, LSM6DS3_ACC_GYRO_XLDA_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_XLDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_GDA
* Description    : Read GDA
* Input          : Pointer to LSM6DS3_ACC_GYRO_GDA_t
* Output         : Status of GDA see LSM6DS3_ACC_GYRO_GDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_GDA(void *handle, LSM6DS3_ACC_GYRO_GDA_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_GDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TDA
* Description    : Read GDA
* Input          : Pointer to LSM6DS3_ACC_GYRO_TDA_t
* Output         : Status of GDA see LSM6DS3_ACC_GYRO_TDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TDA(void *handle, LSM6DS3_ACC_GYRO_TDA_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_EV_BOOT
* Description    : Read EV_BOOT
* Input          : Pointer to LSM6DS3_ACC_GYRO_EV_BOOT_t
* Output         : Status of EV_BOOT see LSM6DS3_ACC_GYRO_EV_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_EV_BOOT(void *handle, LSM6DS3_ACC_GYRO_EV_BOOT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_EV_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFONumOfEntries
* Description    : Read DIFF_FIFO
* Input          : Pointer to u16_t
* Output         : Status of DIFF_FIFO
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFONumOfEntries(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_STATUS1 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS1, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_MASK; //coerce
  valueL = valueL >> LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION; //mask

  /* High part from FIFO_STATUS2 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS2, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_MASK; //coerce
  valueH = valueH >> LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFOEmpty
* Description    : Read FIFO_EMPTY
* Input          : Pointer to LSM6DS3_ACC_GYRO_FIFO_EMPTY_t
* Output         : Status of FIFO_EMPTY see LSM6DS3_ACC_GYRO_FIFO_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFOEmpty(void *handle, LSM6DS3_ACC_GYRO_FIFO_EMPTY_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FIFO_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFOFull
* Description    : Read FIFO_FULL
* Input          : Pointer to LSM6DS3_ACC_GYRO_FIFO_FULL_t
* Output         : Status of FIFO_FULL see LSM6DS3_ACC_GYRO_FIFO_FULL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFOFull(void *handle, LSM6DS3_ACC_GYRO_FIFO_FULL_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FIFO_FULL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_OVERRUN
* Description    : Read OVERRUN
* Input          : Pointer to LSM6DS3_ACC_GYRO_OVERRUN_t
* Output         : Status of OVERRUN see LSM6DS3_ACC_GYRO_OVERRUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_OVERRUN(void *handle, LSM6DS3_ACC_GYRO_OVERRUN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_OVERRUN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WaterMark
* Description    : Read WTM
* Input          : Pointer to LSM6DS3_ACC_GYRO_WTM_t
* Output         : Status of WTM see LSM6DS3_ACC_GYRO_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WaterMark(void *handle, LSM6DS3_ACC_GYRO_WTM_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FIFOPattern
* Description    : Read FIFO_PATTERN
* Input          : Pointer to u16_t
* Output         : Status of FIFO_PATTERN
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FIFOPattern(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_STATUS3 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS3, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK; //coerce
  valueL = valueL >> LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION; //mask

  /* High part from FIFO_STATUS4 */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_STATUS4, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK; //coerce
  valueH = valueH >> LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SENS_HUB_END
* Description    : Read SENS_HUB_END
* Input          : Pointer to LSM6DS3_ACC_GYRO_SENS_HUB_END_t
* Output         : Status of SENS_HUB_END see LSM6DS3_ACC_GYRO_SENS_HUB_END_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SENS_HUB_END(void *handle, LSM6DS3_ACC_GYRO_SENS_HUB_END_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SENS_HUB_END_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SOFT_IRON_END
* Description    : Read SOFT_IRON_END
* Input          : Pointer to LSM6DS3_ACC_GYRO_SOFT_IRON_END_t
* Output         : Status of SOFT_IRON_END see LSM6DS3_ACC_GYRO_SOFT_IRON_END_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SOFT_IRON_END(void *handle, LSM6DS3_ACC_GYRO_SOFT_IRON_END_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SOFT_IRON_END_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_STEP_OVERFLOW
* Description    : Read STEP_OVERFLOW
* Input          : Pointer to LSM6DS3_ACC_GYRO_STEP_OVERFLOW_t
* Output         : Status of STEP_OVERFLOW see LSM6DS3_ACC_GYRO_STEP_OVERFLOW_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_STEP_OVERFLOW(void *handle, LSM6DS3_ACC_GYRO_STEP_OVERFLOW_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_STEP_OVERFLOW_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_STEP_COUNT_DELTA
* Description    : Read STEP_COUNT_DELTA_IA
* Input          : Pointer to LSM6DS3_ACC_GYRO_STEP_COUNT_DELTA_t
* Output         : Status of STEP_COUNT_DELTA_IA see LSM6DS3_ACC_GYRO_STEP_COUNT_DELTA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_STEP_COUNT_DELTA(void *handle, LSM6DS3_ACC_GYRO_STEP_COUNT_DELTA_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_STEP_COUNT_DELTA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS
* Description    : Read PEDO_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t
* Output         : Status of PEDO_EV_STATUS see LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS
* Description    : Read TILT_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t
* Output         : Status of TILT_EV_STATUS see LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TILT_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SIGN_MOT_EV_STATUS
* Description    : Read SIGN_MOT_EV_STATUS
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t
* Output         : Status of SIGN_MOT_EV_STATUS see LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SIGN_MOT_EV_STATUS(void *handle, LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_LIR
* Description    : Write LIR
* Input          : LSM6DS3_ACC_GYRO_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_LIR(void *handle, LSM6DS3_ACC_GYRO_LIR_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_LIR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_LIR
* Description    : Read LIR
* Input          : Pointer to LSM6DS3_ACC_GYRO_LIR_t
* Output         : Status of LIR see LSM6DS3_ACC_GYRO_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_LIR(void *handle, LSM6DS3_ACC_GYRO_LIR_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TAP_Z_EN
* Description    : Write TAP_Z_EN
* Input          : LSM6DS3_ACC_GYRO_TAP_Z_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TAP_Z_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TAP_Z_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_Z_EN
* Description    : Read TAP_Z_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TAP_Z_EN_t
* Output         : Status of TAP_Z_EN see LSM6DS3_ACC_GYRO_TAP_Z_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_Z_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_Z_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_Z_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TAP_Y_EN
* Description    : Write TAP_Y_EN
* Input          : LSM6DS3_ACC_GYRO_TAP_Y_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TAP_Y_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TAP_Y_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_Y_EN
* Description    : Read TAP_Y_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TAP_Y_EN_t
* Output         : Status of TAP_Y_EN see LSM6DS3_ACC_GYRO_TAP_Y_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_Y_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_Y_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_Y_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TAP_X_EN
* Description    : Write TAP_X_EN
* Input          : LSM6DS3_ACC_GYRO_TAP_X_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TAP_X_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_X_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TAP_X_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_X_EN
* Description    : Read TAP_X_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TAP_X_EN_t
* Output         : Status of TAP_X_EN see LSM6DS3_ACC_GYRO_TAP_X_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_X_EN(void *handle, LSM6DS3_ACC_GYRO_TAP_X_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_X_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TILT_EN
* Description    : Write TILT_EN
* Input          : LSM6DS3_ACC_GYRO_TILT_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TILT_EN(void *handle, LSM6DS3_ACC_GYRO_TILT_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TILT_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TILT_EN
* Description    : Read TILT_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TILT_EN_t
* Output         : Status of TILT_EN see LSM6DS3_ACC_GYRO_TILT_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TILT_EN(void *handle, LSM6DS3_ACC_GYRO_TILT_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TILT_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SLOPE_FDS
* Description    : Write SLOPE_FDS
* Input          : LSM6DS3_ACC_GYRO_SLOPE_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SLOPE_FDS(void *handle, LSM6DS3_ACC_GYRO_SLOPE_FDS_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SLOPE_FDS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SLOPE_FDS
* Description    : Read SLOPE_FDS
* Input          : Pointer to LSM6DS3_ACC_GYRO_SLOPE_FDS_t
* Output         : Status of SLOPE_FDS see LSM6DS3_ACC_GYRO_SLOPE_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SLOPE_FDS(void *handle, LSM6DS3_ACC_GYRO_SLOPE_FDS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SLOPE_FDS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PEDO_EN
* Description    : Write PEDO_EN
* Input          : LSM6DS3_ACC_GYRO_PEDO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PEDO_EN(void *handle, LSM6DS3_ACC_GYRO_PEDO_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_PEDO_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_PEDO_EN
* Description    : Read PEDO_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_PEDO_EN_t
* Output         : Status of PEDO_EN see LSM6DS3_ACC_GYRO_PEDO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_PEDO_EN(void *handle, LSM6DS3_ACC_GYRO_PEDO_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_PEDO_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TIMER_EN
* Description    : Write TIMER_EN
* Input          : LSM6DS3_ACC_GYRO_TIMER_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TIMER_EN(void *handle, LSM6DS3_ACC_GYRO_TIMER_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TIMER_EN_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TIMER_EN
* Description    : Read TIMER_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_TIMER_EN_t
* Output         : Status of TIMER_EN see LSM6DS3_ACC_GYRO_TIMER_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TIMER_EN(void *handle, LSM6DS3_ACC_GYRO_TIMER_EN_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TIMER_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TAP_THS
* Description    : Write TAP_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TAP_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_TAP_THS_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_TAP_THS_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TAP_THS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TAP_THS
* Description    : Read TAP_THS
* Input          : Pointer to u8_t
* Output         : Status of TAP_THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TAP_THS(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TAP_THS_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_TAP_THS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SIXD_THS
* Description    : Write SIXD_THS
* Input          : LSM6DS3_ACC_GYRO_SIXD_THS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SIXD_THS(void *handle, LSM6DS3_ACC_GYRO_SIXD_THS_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SIXD_THS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SIXD_THS
* Description    : Read SIXD_THS
* Input          : Pointer to LSM6DS3_ACC_GYRO_SIXD_THS_t
* Output         : Status of SIXD_THS see LSM6DS3_ACC_GYRO_SIXD_THS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SIXD_THS(void *handle, LSM6DS3_ACC_GYRO_SIXD_THS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SIXD_THS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_D4D
* Description    : Write D4D_EN
* Input          : LSM6DS3_ACC_GYRO_D4D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_D4D(void *handle, LSM6DS3_ACC_GYRO_D4D_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_D4D_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_D4D
* Description    : Read D4D_EN
* Input          : Pointer to LSM6DS3_ACC_GYRO_D4D_t
* Output         : Status of D4D_EN see LSM6DS3_ACC_GYRO_D4D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_D4D(void *handle, LSM6DS3_ACC_GYRO_D4D_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_D4D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SHOCK_Duration
* Description    : Write SHOCK
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SHOCK_Duration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_SHOCK_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_SHOCK_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SHOCK_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SHOCK_Duration
* Description    : Read SHOCK
* Input          : Pointer to u8_t
* Output         : Status of SHOCK
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SHOCK_Duration(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SHOCK_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_SHOCK_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_QUIET_Duration
* Description    : Write QUIET
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_QUIET_Duration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_QUIET_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_QUIET_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_QUIET_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_QUIET_Duration
* Description    : Read QUIET
* Input          : Pointer to u8_t
* Output         : Status of QUIET
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_QUIET_Duration(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_QUIET_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_QUIET_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_DUR
* Description    : Write DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_DUR_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_DUR_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_DUR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_DUR
* Description    : Read DUR
* Input          : Pointer to u8_t
* Output         : Status of DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_DUR(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_DUR_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_WK_THS
* Description    : Write WK_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_WK_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_WK_THS_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_WK_THS_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_WK_THS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WK_THS
* Description    : Read WK_THS
* Input          : Pointer to u8_t
* Output         : Status of WK_THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WK_THS(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_WK_THS_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_WK_THS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_INACTIVITY_ON
* Description    : Write INACTIVITY_ON
* Input          : LSM6DS3_ACC_GYRO_INACTIVITY_ON_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_INACTIVITY_ON(void *handle, LSM6DS3_ACC_GYRO_INACTIVITY_ON_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INACTIVITY_ON_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_INACTIVITY_ON
* Description    : Read INACTIVITY_ON
* Input          : Pointer to LSM6DS3_ACC_GYRO_INACTIVITY_ON_t
* Output         : Status of INACTIVITY_ON see LSM6DS3_ACC_GYRO_INACTIVITY_ON_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_INACTIVITY_ON(void *handle, LSM6DS3_ACC_GYRO_INACTIVITY_ON_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INACTIVITY_ON_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV
* Description    : Write SINGLE_DOUBLE_TAP
* Input          : LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(void *handle, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV
* Description    : Read SINGLE_DOUBLE_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Output         : Status of SINGLE_DOUBLE_TAP see LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV(void *handle, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SLEEP_DUR
* Description    : Write SLEEP_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SLEEP_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_SLEEP_DUR_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_SLEEP_DUR_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_SLEEP_DUR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SLEEP_DUR
* Description    : Read SLEEP_DUR
* Input          : Pointer to u8_t
* Output         : Status of SLEEP_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SLEEP_DUR(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_SLEEP_DUR_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_SLEEP_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TIMER_HR
* Description    : Write TIMER_HR
* Input          : LSM6DS3_ACC_GYRO_TIMER_HR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TIMER_HR(void *handle, LSM6DS3_ACC_GYRO_TIMER_HR_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_TIMER_HR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TIMER_HR
* Description    : Read TIMER_HR
* Input          : Pointer to LSM6DS3_ACC_GYRO_TIMER_HR_t
* Output         : Status of TIMER_HR see LSM6DS3_ACC_GYRO_TIMER_HR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TIMER_HR(void *handle, LSM6DS3_ACC_GYRO_TIMER_HR_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_TIMER_HR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_WAKE_DUR
* Description    : Write WAKE_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_WAKE_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS3_ACC_GYRO_WAKE_DUR_POSITION; //mask
  newValue &= LSM6DS3_ACC_GYRO_WAKE_DUR_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_WAKE_DUR_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WAKE_DUR
* Description    : Read WAKE_DUR
* Input          : Pointer to u8_t
* Output         : Status of WAKE_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WAKE_DUR(void *handle, u8_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_WAKE_DUR_MASK; //coerce
  *value = *value >> LSM6DS3_ACC_GYRO_WAKE_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FF_THS
* Description    : Write FF_THS
* Input          : LSM6DS3_ACC_GYRO_FF_THS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FF_THS(void *handle, LSM6DS3_ACC_GYRO_FF_THS_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FF_THS_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FF_THS
* Description    : Read FF_THS
* Input          : Pointer to LSM6DS3_ACC_GYRO_FF_THS_t
* Output         : Status of FF_THS see LSM6DS3_ACC_GYRO_FF_THS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FF_THS(void *handle, LSM6DS3_ACC_GYRO_FF_THS_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_FF_THS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FF_Duration
* Description    : Write FF_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FF_Duration(void *handle, u8_t newValue)
{
  u8_t valueH, valueL;
  u8_t value;

  valueL = newValue & 0x1F;
  valueH = (newValue >> 5) & 0x1;

  /* Low part in FREE_FALL reg */
  valueL = valueL << LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_POSITION; //mask
  valueL &= LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_MASK;
  value |= valueL;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  /* High part in WAKE_UP_DUR reg */
  valueH = valueH << LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_POSITION; //mask
  valueH &= LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_MASK; //coerce

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_MASK;
  value |= valueH;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FF_Duration
* Description    : Read FF_DUR
* Input          : Pointer to u8_t
* Output         : Status of FF_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FF_Duration(void *handle, u8_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FREE_FALL reg */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FREE_FALL, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_MASK; //coerce
  valueL = valueL >> LSM6DS3_ACC_GYRO_FF_FREE_FALL_DUR_POSITION; //mask

  /* High part from WAKE_UP_DUR reg */
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_WAKE_UP_DUR, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_MASK; //coerce
  valueH = valueH >> LSM6DS3_ACC_GYRO_FF_WAKE_UP_DUR_POSITION; //mask

  *value = ((valueH << 5) & 0x20) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TimerEvRouteInt1
* Description    : Write INT1_TIMER
* Input          : LSM6DS3_ACC_GYRO_INT1_TIMER_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TimerEvRouteInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TIMER_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_TIMER_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TimerEvRouteInt1
* Description    : Read INT1_TIMER
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_TIMER_t
* Output         : Status of INT1_TIMER see LSM6DS3_ACC_GYRO_INT1_TIMER_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TimerEvRouteInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TIMER_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_TIMER_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TiltEvOnInt1
* Description    : Write INT1_TILT
* Input          : LSM6DS3_ACC_GYRO_INT1_TILT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TiltEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TILT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_TILT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TiltEvOnInt1
* Description    : Read INT1_TILT
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_TILT_t
* Output         : Status of INT1_TILT see LSM6DS3_ACC_GYRO_INT1_TILT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TiltEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TILT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_TILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_6DEvOnInt1
* Description    : Write INT1_6D
* Input          : LSM6DS3_ACC_GYRO_INT1_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_6DEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_6D_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_6D_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_6DEvOnInt1
* Description    : Read INT1_6D
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_6D_t
* Output         : Status of INT1_6D see LSM6DS3_ACC_GYRO_INT1_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_6DEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_6D_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TapEvOnInt1
* Description    : Write INT1_TAP
* Input          : LSM6DS3_ACC_GYRO_INT1_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TapEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TAP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_TAP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TapEvOnInt1
* Description    : Read INT1_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_TAP_t
* Output         : Status of INT1_TAP see LSM6DS3_ACC_GYRO_INT1_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TapEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FFEvOnInt1
* Description    : Write INT1_FF
* Input          : LSM6DS3_ACC_GYRO_INT1_FF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FFEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_FF_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_FF_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FFEvOnInt1
* Description    : Read INT1_FF
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_FF_t
* Output         : Status of INT1_FF see LSM6DS3_ACC_GYRO_INT1_FF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FFEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_FF_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_FF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_WUEvOnInt1
* Description    : Write INT1_WU
* Input          : LSM6DS3_ACC_GYRO_INT1_WU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_WUEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_WU_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_WU_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WUEvOnInt1
* Description    : Read INT1_WU
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_WU_t
* Output         : Status of INT1_WU see LSM6DS3_ACC_GYRO_INT1_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WUEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_WU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SingleTapOnInt1
* Description    : Write INT1_SINGLE_TAP
* Input          : LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SingleTapOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SingleTapOnInt1
* Description    : Read INT1_SINGLE_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t
* Output         : Status of INT1_SINGLE_TAP see LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SingleTapOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SleepEvOnInt1
* Description    : Write INT1_SLEEP
* Input          : LSM6DS3_ACC_GYRO_INT1_SLEEP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SleepEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_SLEEP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT1_SLEEP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SleepEvOnInt1
* Description    : Read INT1_SLEEP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT1_SLEEP_t
* Output         : Status of INT1_SLEEP see LSM6DS3_ACC_GYRO_INT1_SLEEP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SleepEvOnInt1(void *handle, LSM6DS3_ACC_GYRO_INT1_SLEEP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT1_SLEEP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TimerEvRouteInt2
* Description    : Write INT2_TIMER
* Input          : LSM6DS3_ACC_GYRO_INT2_TIMER_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TimerEvRouteInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TIMER_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_TIMER_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TimerEvRouteInt2
* Description    : Read INT2_TIMER
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_TIMER_t
* Output         : Status of INT2_TIMER see LSM6DS3_ACC_GYRO_INT2_TIMER_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TimerEvRouteInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TIMER_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_TIMER_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TiltEvOnInt2
* Description    : Write INT2_TILT
* Input          : LSM6DS3_ACC_GYRO_INT2_TILT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TiltEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TILT_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_TILT_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TiltEvOnInt2
* Description    : Read INT2_TILT
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_TILT_t
* Output         : Status of INT2_TILT see LSM6DS3_ACC_GYRO_INT2_TILT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TiltEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TILT_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_TILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_6DEvOnInt2
* Description    : Write INT2_6D
* Input          : LSM6DS3_ACC_GYRO_INT2_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_6DEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_6D_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_6D_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_6DEvOnInt2
* Description    : Read INT2_6D
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_6D_t
* Output         : Status of INT2_6D see LSM6DS3_ACC_GYRO_INT2_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_6DEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_6D_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_6D_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_TapEvOnInt2
* Description    : Write INT2_TAP
* Input          : LSM6DS3_ACC_GYRO_INT2_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_TapEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TAP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_TAP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_TapEvOnInt2
* Description    : Read INT2_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_TAP_t
* Output         : Status of INT2_TAP see LSM6DS3_ACC_GYRO_INT2_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_TapEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_FFEvOnInt2
* Description    : Write INT2_FF
* Input          : LSM6DS3_ACC_GYRO_INT2_FF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_FFEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_FF_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_FF_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_FFEvOnInt2
* Description    : Read INT2_FF
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_FF_t
* Output         : Status of INT2_FF see LSM6DS3_ACC_GYRO_INT2_FF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_FFEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_FF_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_FF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_WUEvOnInt2
* Description    : Write INT2_WU
* Input          : LSM6DS3_ACC_GYRO_INT2_WU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_WUEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_WU_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_WU_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_WUEvOnInt2
* Description    : Read INT2_WU
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_WU_t
* Output         : Status of INT2_WU see LSM6DS3_ACC_GYRO_INT2_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_WUEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_WU_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SingleTapOnInt2
* Description    : Write INT2_SINGLE_TAP
* Input          : LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SingleTapOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SingleTapOnInt2
* Description    : Read INT2_SINGLE_TAP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t
* Output         : Status of INT2_SINGLE_TAP see LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SingleTapOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_SleepEvOnInt2
* Description    : Write INT2_SLEEP
* Input          : LSM6DS3_ACC_GYRO_INT2_SLEEP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_SleepEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_SLEEP_t newValue)
{
  u8_t value;

  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS3_ACC_GYRO_INT2_SLEEP_MASK;
  value |= newValue;

  if( !LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_R_SleepEvOnInt2
* Description    : Read INT2_SLEEP
* Input          : Pointer to LSM6DS3_ACC_GYRO_INT2_SLEEP_t
* Output         : Status of INT2_SLEEP see LSM6DS3_ACC_GYRO_INT2_SLEEP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_R_SleepEvOnInt2(void *handle, LSM6DS3_ACC_GYRO_INT2_SLEEP_t *value)
{
  if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS3_ACC_GYRO_INT2_SLEEP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_GYRO_Get_GetFIFOData(u8_t *buff)
* Description    : Read GetFIFOData output register
* Input          : pointer to [u8_t]
* Output         : GetFIFOData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetFIFOData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_GYRO_Get_GetTimestamp(u8_t *buff)
* Description    : Read GetTimestamp output register
* Input          : pointer to [u8_t]
* Output         : GetTimestamp buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetTimestamp(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 3 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_TIMESTAMP0_REG + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS3_ACC_GYRO_Get_GetStepCounter(u8_t *buff)
* Description    : Read GetStepCounter output register
* Input          : pointer to [u8_t]
* Output         : GetStepCounter buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS3_ACC_GYRO_Get_GetStepCounter(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_STEP_COUNTER_L + k, &buff[k], 1))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS3_ACC_GYRO_W_PedoThreshold(void *handle, u8_t newValue)
* Description    : Set accelerometer threshold for pedometer
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS3_ACC_GYRO_W_PedoThreshold(void *handle, u8_t newValue)
{
  u8_t value;

  /* Open Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_ENABLED);

  /* read current value */
  LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN, &value, 1);

  value &= ~0x1F;
  value |= (newValue & 0x1F);

  /* write new value */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN, &value, 1);

  /* Close Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_DISABLED);

  return MEMS_SUCCESS;
}


/************** Use Sensor Hub  *******************/

/*
 * Program the nine Soft Iron Matrix coefficients.
 * The SI_Matrix buffer must provide coefficients
 * in xx, xy, xz, yx, yy, yz, zx, zy, zz order.
 */
status_t LSM6DS3_ACC_GYRO_SH_init_SI_Matrix(void *handle, u8_t *SI_matrix)
{
  /* Open Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_ENABLED);

  /* Write the Soft Iron Matrix coefficients */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_MAG_SI_XX, SI_matrix, 9);

  /* Close Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_DISABLED);

  return MEMS_SUCCESS;
}

/* Read a remote device through I2C Sensor Hub Slave 0 */
status_t LSM6DS3_ACC_GYRO_SH0_Program(void *handle, u8_t SlvAddr, u8_t Reg, u8_t len)
{
  /* Open Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_ENABLED);

  /* Write remote device I2C slave address */
  SlvAddr |= 0x1; /* Raise the read op bit */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SLV0_ADD, &SlvAddr, 1);

  /* Write remote device I2C subaddress */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SLV0_SUBADD, &Reg, 1);

  /* Write number of bytes to read */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SLAVE0_CONFIG, &len, 1);

  /* Close Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_DISABLED);

  /* Enable FUNC */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(handle, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);

  /* Enable PULL_UP_EN and MASTER_EN */
  LSM6DS3_ACC_GYRO_W_PULL_UP_EN(handle, LSM6DS3_ACC_GYRO_PULL_UP_EN_ENABLED);
  LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(handle, LSM6DS3_ACC_GYRO_MASTER_ON_ENABLED);

  return MEMS_SUCCESS;
}

/* Read a remote device through I2C Sensor Hub Slave 0 */
status_t LSM6DS3_ACC_GYRO_SH0_ReadMem(void *handle, u8_t SlvAddr, u8_t Reg, u8_t *Bufp, u8_t len, u8_t stop)
{
  LSM6DS3_ACC_GYRO_SENS_HUB_END_t op_cmpl = LSM6DS3_ACC_GYRO_SENS_HUB_END_STILL_ONGOING;

  LSM6DS3_ACC_GYRO_SH0_Program(handle, SlvAddr, Reg, len);

  /* Wait until operation is not completed */
  do
  {
    LSM6DS3_ACC_GYRO_R_SENS_HUB_END(handle, &op_cmpl);
  }
  while(op_cmpl != LSM6DS3_ACC_GYRO_SENS_HUB_END_OP_COMPLETED);

  /* Read the result */
  LSM6DS3_ACC_GYRO_ReadReg(handle, LSM6DS3_ACC_GYRO_SENSORHUB1_REG, Bufp, len);

  if (stop)
  {
    /* Stop everything */
    LSM6DS3_ACC_GYRO_W_FUNC_EN(handle, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED);
    LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(handle, LSM6DS3_ACC_GYRO_MASTER_ON_DISABLED);
  }

  return MEMS_SUCCESS;
}

/* Write a remote device through I2C Sensor Hub Slave 0 */
status_t LSM6DS3_ACC_GYRO_SH0_WriteByte(void *handle, u8_t SlvAddr, u8_t Reg, u8_t Bufp)
{
  /* Open Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_ENABLED);

  /* Write remote device I2C slave address */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SLV0_ADD, &SlvAddr, 1);

  /* Write remote device I2C subaddress */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_SLV0_SUBADD, &Reg, 1);

  /* Write the data */
  LSM6DS3_ACC_GYRO_WriteReg(handle, LSM6DS3_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0, &Bufp, 1);

  /* Close Embedded Function Register page*/
  LSM6DS3_ACC_GYRO_W_EmbeddedAccess(handle, LSM6DS3H_ACC_GYRO_FUNC_CFG_DISABLED);

  /* Enable FUNC */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(handle, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);

  /* Enable PULL_UP_EN and MASTER_EN */
  LSM6DS3_ACC_GYRO_W_PULL_UP_EN(handle, LSM6DS3_ACC_GYRO_PULL_UP_EN_ENABLED);
  LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(handle, LSM6DS3_ACC_GYRO_MASTER_ON_ENABLED);

  /* Add 20ms of delay, because for now there's no write complete indication */
  //Delay(20);

  /* Stop everything */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(handle, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED);
  LSM6DS3_ACC_GYRO_W_I2C_MASTER_Enable(handle, LSM6DS3_ACC_GYRO_MASTER_ON_DISABLED);

  return MEMS_SUCCESS;
}
