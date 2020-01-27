/**
 ******************************************************************************
 * @file    LSM6DS0_ACC_GYRO_driver.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   LSM6DS0 driver file
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
#include "LSM6DS0_ACC_GYRO_driver.h"
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
* Function Name   : LSM6DS0_ACC_GYRO_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  if ( Sensor_IO_Write( handle, Reg, Bufp, len ) )
    return MEMS_ERROR;
  else
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name   : LSM6DS0_ACC_GYRO_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data REad
* Return      : None
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  if ( Sensor_IO_Read( handle, Reg, Bufp, len ) )
    return MEMS_ERROR;
  else
    return MEMS_SUCCESS;
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_WHO_AM_I_(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroFullScale
* Description    : Write FS_G
* Input          : LSM6DS0_ACC_GYRO_FS_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FS_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroFullScale
* Description    : Read FS_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_FS_G_t
* Output         : Status of FS_G see LSM6DS0_ACC_GYRO_FS_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FS_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroDataRate
* Description    : Write ODR_G
* Input          : LSM6DS0_ACC_GYRO_ODR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ODR_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroDataRate
* Description    : Read ODR_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ODR_G_t
* Output         : Status of ODR_G see LSM6DS0_ACC_GYRO_ODR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ODR_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerFullScale
* Description    : Write FS_XL
* Input          : LSM6DS0_ACC_GYRO_FS_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FS_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerFullScale
* Description    : Read FS_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_FS_XL_t
* Output         : Status of FS_XL see LSM6DS0_ACC_GYRO_FS_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FS_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerDataRate
* Description    : Write ODR_XL
* Input          : LSM6DS0_ACC_GYRO_ODR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ODR_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerDataRate
* Description    : Read ODR_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ODR_XL_t
* Output         : Status of ODR_XL see LSM6DS0_ACC_GYRO_ODR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ODR_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM6DS0_ACC_GYRO_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_BlockDataUpdate(void *handle, LSM6DS0_ACC_GYRO_BDU_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BDU_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM6DS0_ACC_GYRO_BDU_t
* Output         : Status of BDU see LSM6DS0_ACC_GYRO_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_BlockDataUpdate(void *handle, LSM6DS0_ACC_GYRO_BDU_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS0_ACC_GYRO_Get_AngularRate(u8_t *buff)
* Description    : Read AngularRate output register
* Input          : pointer to [u8_t]
* Output         : AngularRate buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_AngularRate(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_OUT_X_L_G + k, &buff[k], 1 ))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS0_ACC_GYRO_Get_Acceleration(u8_t *buff)
* Description    : Read Acceleration output register
* Input          : pointer to [u8_t]
* Output         : Acceleration buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_Acceleration(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_OUT_X_L_XL + k, &buff[k], 1 ))
        return MEMS_ERROR;
      k++;
    }
  }
  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInactivityThreshold
* Description    : Write ACT_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityThreshold(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_ACT_THS_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_ACT_THS_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ACT_THS_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInactivityThreshold
* Description    : Read ACT_THS
* Input          : Pointer to u8_t
* Output         : Status of ACT_THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityThreshold(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ACT_THS_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_ACT_THS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInactivityMode
* Description    : Write SLEEP_ON_INACT_EN
* Input          : LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInactivityMode
* Description    : Read SLEEP_ON_INACT_EN
* Input          : Pointer to LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Output         : Status of SLEEP_ON_INACT_EN see LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInactivityDuration
* Description    : Write ACT_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityDuration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_ACT_DUR_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_ACT_DUR_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS0_ACC_GYRO_ACT_DUR_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ACT_DUR, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInactivityDuration
* Description    : Read ACT_DUR
* Input          : Pointer to u8_t
* Output         : Status of ACT_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityDuration(void *handle, u8_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ACT_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ACT_DUR_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_ACT_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisX
* Description    : Write XLIE_XL
* Input          : LSM6DS0_ACC_GYRO_XLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XLIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisX
* Description    : Read XLIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_XLIE_XL_t
* Output         : Status of XLIE_XL see LSM6DS0_ACC_GYRO_XLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XLIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisX
* Description    : Write XHIE_XL
* Input          : LSM6DS0_ACC_GYRO_XHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XHIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisX
* Description    : Read XHIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_XHIE_XL_t
* Output         : Status of XHIE_XL see LSM6DS0_ACC_GYRO_XHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XHIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisY
* Description    : Write YLIE_XL
* Input          : LSM6DS0_ACC_GYRO_YLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YLIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisY
* Description    : Read YLIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_YLIE_XL_t
* Output         : Status of YLIE_XL see LSM6DS0_ACC_GYRO_YLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YLIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisY
* Description    : Write YHIE_XL
* Input          : LSM6DS0_ACC_GYRO_YHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_XL_t newValue)
{
  u8_t value;


  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YHIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisY
* Description    : Read YHIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_YHIE_XL_t
* Output         : Status of YHIE_XL see LSM6DS0_ACC_GYRO_YHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YHIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisZ
* Description    : Write ZLIE_XL
* Input          : LSM6DS0_ACC_GYRO_ZLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZLIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisZ
* Description    : Read ZLIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZLIE_XL_t
* Output         : Status of ZLIE_XL see LSM6DS0_ACC_GYRO_ZLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZLIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisZ
* Description    : Write ZHIE_XL
* Input          : LSM6DS0_ACC_GYRO_ZHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZHIE_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisZ
* Description    : Read ZHIE_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZHIE_XL_t
* Output         : Status of ZHIE_XL see LSM6DS0_ACC_GYRO_ZHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZHIE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Interrupt6D
* Description    : Write 6D
* Input          : LSM6DS0_ACC_GYRO_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Interrupt6D(void *handle, LSM6DS0_ACC_GYRO_6D_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_6D_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Interrupt6D
* Description    : Read 6D
* Input          : Pointer to LSM6DS0_ACC_GYRO_6D_t
* Output         : Status of 6D see LSM6DS0_ACC_GYRO_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Interrupt6D(void *handle, LSM6DS0_ACC_GYRO_6D_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerInterruptCombination
* Description    : Write AOI_XL
* Input          : LSM6DS0_ACC_GYRO_AOI_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerInterruptCombination(void *handle, LSM6DS0_ACC_GYRO_AOI_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_AOI_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptCombination
* Description    : Read AOI_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_AOI_XL_t
* Output         : Status of AOI_XL see LSM6DS0_ACC_GYRO_AOI_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptCombination(void *handle, LSM6DS0_ACC_GYRO_AOI_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_AOI_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisX
* Description    : Write THS_XL_X
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisX(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_THS_XL_X_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_THS_XL_X_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_X_XL, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS0_ACC_GYRO_THS_XL_X_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_X_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisX
* Description    : Read THS_XL_X
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_X
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisX(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_X_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_THS_XL_X_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_THS_XL_X_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisY
* Description    : Write THS_XL_Y
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisY(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_THS_XL_Y_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_THS_XL_Y_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Y_XL, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS0_ACC_GYRO_THS_XL_Y_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Y_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisY
* Description    : Read THS_XL_Y
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_Y
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisY(void *handle, u8_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Y_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_THS_XL_Y_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_THS_XL_Y_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisZ
* Description    : Write THS_XL_Z
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisZ(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_THS_XL_Z_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_THS_XL_Z_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Z_XL, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS0_ACC_GYRO_THS_XL_Z_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Z_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisZ
* Description    : Read THS_XL_Z
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_Z
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisZ(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_Z_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_THS_XL_Z_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_THS_XL_Z_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_InterruptDuration
* Description    : Write DUR_XL
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptDuration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_DUR_XL_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_DUR_XL_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DUR_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_InterruptDuration
* Description    : Read DUR_XL
* Input          : Pointer to u8_t
* Output         : Status of DUR_XL
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptDuration(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DUR_XL_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_DUR_XL_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_WaitFunction
* Description    : Write WAIT_XL
* Input          : LSM6DS0_ACC_GYRO_WAIT_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_WAIT_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, &value, 1 ) )
    return MEMS_ERROR;
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_WaitFunction
* Description    : Read WAIT_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_WAIT_XL_t
* Output         : Status of WAIT_XL see LSM6DS0_ACC_GYRO_WAIT_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_WAIT_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroHighPassFilterReference
* Description    : Write REF_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilterReference(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_REF_G_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_REF_G_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_REFERENCE_G, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~LSM6DS0_ACC_GYRO_REF_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_REFERENCE_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroHighPassFilterReference
* Description    : Read REF_G
* Input          : Pointer to u8_t
* Output         : Status of REF_G
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilterReference(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_REFERENCE_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_REF_G_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_REF_G_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT
* Description    : Write INT_DRDY_XL
* Input          : LSM6DS0_ACC_GYRO_INT_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_DRDY_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_XL_DataReadyOnINT
* Description    : Read INT_DRDY_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_DRDY_XL_t
* Output         : Status of INT_DRDY_XL see LSM6DS0_ACC_GYRO_INT_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_XL_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_DRDY_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT
* Description    : Write INT_DRDY_G
* Input          : LSM6DS0_ACC_GYRO_INT_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_DRDY_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_DataReadyOnINT
* Description    : Read INT_DRDY_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_DRDY_G_t
* Output         : Status of INT_DRDY_G see LSM6DS0_ACC_GYRO_INT_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_DRDY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_BOOT_DataReadyOnINT
* Description    : Write INT__BOOT
* Input          : LSM6DS0_ACC_GYRO_INT__BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_BOOT_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT__BOOT_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT__BOOT_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_BOOT_DataReadyOnINT
* Description    : Read INT__BOOT
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT__BOOT_t
* Output         : Status of INT__BOOT see LSM6DS0_ACC_GYRO_INT__BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_BOOT_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT__BOOT_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT__BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO_Threshold_OnINT
* Description    : Write INT_FTH
* Input          : LSM6DS0_ACC_GYRO_INT_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FTH_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_FTH_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Threshold_OnINT
* Description    : Read INT_FTH
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_FTH_t
* Output         : Status of INT_FTH see LSM6DS0_ACC_GYRO_INT_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FTH_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Overrun_OnINT
* Description    : Write INT_OVR
* Input          : LSM6DS0_ACC_GYRO_INT_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Overrun_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_OVR_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_OVR_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Overrun_OnINT
* Description    : Read INT_OVR
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_OVR_t
* Output         : Status of INT_OVR see LSM6DS0_ACC_GYRO_INT_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Overrun_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_OVR_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO_Full_OnINT
* Description    : Write INT_FSS5
* Input          : LSM6DS0_ACC_GYRO_INT_FSS5_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Full_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FSS5_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_FSS5_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Full_OnINT
* Description    : Read INT_FSS5
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_FSS5_t
* Output         : Status of INT_FSS5 see LSM6DS0_ACC_GYRO_INT_FSS5_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Full_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FSS5_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_FSS5_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Accelerometer_OnINT
* Description    : Write INT_IG_XL
* Input          : LSM6DS0_ACC_GYRO_INT_IG_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Accelerometer_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_IG_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Accelerometer_OnINT
* Description    : Read INT_IG_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_IG_XL_t
* Output         : Status of INT_IG_XL see LSM6DS0_ACC_GYRO_INT_IG_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Accelerometer_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_IG_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Gyroscope_OnINT
* Description    : Write INT_IG_G
* Input          : LSM6DS0_ACC_GYRO_INT_IG_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Gyroscope_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_IG_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Gyroscope_OnINT
* Description    : Read INT_IG_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_IG_G_t
* Output         : Status of INT_IG_G see LSM6DS0_ACC_GYRO_INT_IG_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Gyroscope_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_IG_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroLowPower
* Description    : Write LP_G
* Input          : LSM6DS0_ACC_GYRO_LP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroLowPower(void *handle, LSM6DS0_ACC_GYRO_LP_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_LP_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroLowPower
* Description    : Read LP_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_LP_G_t
* Output         : Status of LP_G
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroLowPower(void *handle, LSM6DS0_ACC_GYRO_LP_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_LP_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection
* Description    : Write BW_G
* Input          : LSM6DS0_ACC_GYRO_BW_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BW_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroBandwidthSelection
* Description    : Read BW_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_BW_G_t
* Output         : Status of BW_G see LSM6DS0_ACC_GYRO_BW_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroBandwidthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG1_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BW_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_OutMode
* Description    : Write OUT_SEL
* Input          : LSM6DS0_ACC_GYRO_OUT_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_OutMode(void *handle, LSM6DS0_ACC_GYRO_OUT_SEL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_OUT_SEL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_OutMode
* Description    : Read OUT_SEL
* Input          : Pointer to LSM6DS0_ACC_GYRO_OUT_SEL_t
* Output         : Status of OUT_SEL see LSM6DS0_ACC_GYRO_OUT_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_OutMode(void *handle, LSM6DS0_ACC_GYRO_OUT_SEL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_OUT_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_OutIntMode
* Description    : Write INT_SEL_G
* Input          : LSM6DS0_ACC_GYRO_INT_SEL_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_OutIntMode(void *handle, LSM6DS0_ACC_GYRO_INT_SEL_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_INT_SEL_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_OutIntMode
* Description    : Read INT_SEL_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_INT_SEL_G_t
* Output         : Status of INT_SEL_G see LSM6DS0_ACC_GYRO_INT_SEL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_OutIntMode(void *handle, LSM6DS0_ACC_GYRO_INT_SEL_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG2_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INT_SEL_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroHighPassFilterCutOffFrequency
* Description    : Write HPCF_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilterCutOffFrequency(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_HPCF_G_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_HPCF_G_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_HPCF_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroHighPassFilterCutOffFrequency
* Description    : Read HPCF_G
* Input          : Pointer to u8_t
* Output         : Status of HPCF_G
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilterCutOffFrequency(void *handle, u8_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_HPCF_G_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_HPCF_G_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroHighPassFilter
* Description    : Write HP_EN_G
* Input          : LSM6DS0_ACC_GYRO_HP_EN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilter(void *handle, LSM6DS0_ACC_GYRO_HP_EN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_HP_EN_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroHighPassFilter
* Description    : Read HP_EN_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_HP_EN_G_t
* Output         : Status of HP_EN_G see LSM6DS0_ACC_GYRO_HP_EN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilter(void *handle, LSM6DS0_ACC_GYRO_HP_EN_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG3_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_HP_EN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationX
* Description    : Write ORIENT_0
* Input          : LSM6DS0_ACC_GYRO_ORIENT_0_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationX(void *handle, LSM6DS0_ACC_GYRO_ORIENT_0_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ORIENT_0_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationX
* Description    : Read ORIENT_0
* Input          : Pointer to LSM6DS0_ACC_GYRO_ORIENT_0_t
* Output         : Status of ORIENT_0 see LSM6DS0_ACC_GYRO_ORIENT_0_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationX(void *handle, LSM6DS0_ACC_GYRO_ORIENT_0_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ORIENT_0_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationY
* Description    : Write ORIENT_1
* Input          : LSM6DS0_ACC_GYRO_ORIENT_1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationY(void *handle, LSM6DS0_ACC_GYRO_ORIENT_1_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ORIENT_1_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationY
* Description    : Read ORIENT_1
* Input          : Pointer to LSM6DS0_ACC_GYRO_ORIENT_1_t
* Output         : Status of ORIENT_1 see LSM6DS0_ACC_GYRO_ORIENT_1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationY(void *handle, LSM6DS0_ACC_GYRO_ORIENT_1_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ORIENT_1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationZ
* Description    : Write ORIENT_2
* Input          : LSM6DS0_ACC_GYRO_ORIENT_2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationZ(void *handle, LSM6DS0_ACC_GYRO_ORIENT_2_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ORIENT_2_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationZ
* Description    : Read ORIENT_2
* Input          : Pointer to LSM6DS0_ACC_GYRO_ORIENT_2_t
* Output         : Status of ORIENT_2 see LSM6DS0_ACC_GYRO_ORIENT_2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationZ(void *handle, LSM6DS0_ACC_GYRO_ORIENT_2_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ORIENT_2_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_SignZ
* Description    : Write SIGNZ_G
* Input          : LSM6DS0_ACC_GYRO_SIGNZ_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignZ(void *handle, LSM6DS0_ACC_GYRO_SIGNZ_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SIGNZ_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_SignZ
* Description    : Read SIGNZ_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_SIGNZ_G_t
* Output         : Status of SIGNZ_G see LSM6DS0_ACC_GYRO_SIGNZ_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignZ(void *handle, LSM6DS0_ACC_GYRO_SIGNZ_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SIGNZ_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_SignY
* Description    : Write SIGNY_G
* Input          : LSM6DS0_ACC_GYRO_SIGNY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignY(void *handle, LSM6DS0_ACC_GYRO_SIGNY_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SIGNY_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_SignY
* Description    : Read SIGNY_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_SIGNY_G_t
* Output         : Status of SIGNY_G see LSM6DS0_ACC_GYRO_SIGNY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignY(void *handle, LSM6DS0_ACC_GYRO_SIGNY_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SIGNY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GYRO_SignX
* Description    : Write SIGNX_G
* Input          : LSM6DS0_ACC_GYRO_SIGNX_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignX(void *handle, LSM6DS0_ACC_GYRO_SIGNX_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SIGNX_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GYRO_SignX
* Description    : Read SIGNX_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_SIGNX_G_t
* Output         : Status of SIGNX_G see LSM6DS0_ACC_GYRO_SIGNX_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignX(void *handle, LSM6DS0_ACC_GYRO_SIGNX_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SIGNX_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisX
* Description    : Read XL_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_XL_G_t
* Output         : Status of XL_G see LSM6DS0_ACC_GYRO_XL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XL_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XL_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisX
* Description    : Read XH_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_XH_G_t
* Output         : Status of XH_G see LSM6DS0_ACC_GYRO_XH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XH_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XH_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisY
* Description    : Read YL_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_YL_G_t
* Output         : Status of YL_G see LSM6DS0_ACC_GYRO_YL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YL_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YL_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisY
* Description    : Read YH_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_YH_G_t
* Output         : Status of YH_G see LSM6DS0_ACC_GYRO_YH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YH_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YH_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisZ
* Description    : Read ZL_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZL_G_t
* Output         : Status of ZL_G see LSM6DS0_ACC_GYRO_ZL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZL_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZL_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisZ
* Description    : Read ZH_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZH_G_t
* Output         : Status of ZH_G see LSM6DS0_ACC_GYRO_ZH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZH_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZH_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptGyroFlag
* Description    : Read IA_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_IA_G_t
* Output         : Status of IA_G see LSM6DS0_ACC_GYRO_IA_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag(void *handle, LSM6DS0_ACC_GYRO_IA_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_IA_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerDataReadyFlag
* Description    : Read XLDA
* Input          : Pointer to LSM6DS0_ACC_GYRO_XLDA_t
* Output         : Status of XLDA see LSM6DS0_ACC_GYRO_XLDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_XLDA_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XLDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroDataReadyFlag
* Description    : Read GDA
* Input          : Pointer to LSM6DS0_ACC_GYRO_GDA_t
* Output         : Status of GDA see LSM6DS0_ACC_GYRO_GDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_GDA_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_GDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_TemperatureDataReadyFlag
* Description    : Read TDA
* Input          : Pointer to LSM6DS0_ACC_GYRO_TDA_t
* Output         : Status of TDA see LSM6DS0_ACC_GYRO_TDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_TemperatureDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_TDA_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_TDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_BootRunningFlag
* Description    : Read BOOT_STATUS
* Input          : Pointer to LSM6DS0_ACC_GYRO_BOOT_STATUS_t
* Output         : Status of BOOT_STATUS see LSM6DS0_ACC_GYRO_BOOT_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_BootRunningFlag(void *handle, LSM6DS0_ACC_GYRO_BOOT_STATUS_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BOOT_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InactivityInterruptFlag
* Description    : Read INACT
* Input          : Pointer to LSM6DS0_ACC_GYRO_INACT_t
* Output         : Status of INACT see LSM6DS0_ACC_GYRO_INACT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InactivityInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_INACT_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_INACT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterruptFlag
* Description    : Read IG_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_IG_G_t
* Output         : Status of IG_G see LSM6DS0_ACC_GYRO_IG_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_IG_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_IG_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptAccelerometerFlag
* Description    : Read IG_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_IG_XL_t
* Output         : Status of IG_XL see LSM6DS0_ACC_GYRO_IG_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometerFlag(void *handle, LSM6DS0_ACC_GYRO_IG_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_IG_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Interrupt4D
* Description    : Write 4D_XL
* Input          : LSM6DS0_ACC_GYRO_4D_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Interrupt4D(void *handle, LSM6DS0_ACC_GYRO_4D_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_4D_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Interrupt4D
* Description    : Read 4D_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_4D_XL_t
* Output         : Status of 4D_XL see LSM6DS0_ACC_GYRO_4D_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Interrupt4D(void *handle, LSM6DS0_ACC_GYRO_4D_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_4D_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptSignalMode
* Description    : Write LIR_XL
* Input          : LSM6DS0_ACC_GYRO_LIR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptSignalMode(void *handle, LSM6DS0_ACC_GYRO_LIR_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_LIR_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptSignalMode
* Description    : Read LIR_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_LIR_XL_t
* Output         : Status of LIR_XL see LSM6DS0_ACC_GYRO_LIR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptSignalMode(void *handle, LSM6DS0_ACC_GYRO_LIR_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_LIR_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroAxisX
* Description    : Write XEN_G
* Input          : LSM6DS0_ACC_GYRO_XEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XEN_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroAxisX
* Description    : Read XEN_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_XEN_G_t
* Output         : Status of XEN_G see LSM6DS0_ACC_GYRO_XEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroAxisY
* Description    : Write YEN_G
* Input          : LSM6DS0_ACC_GYRO_YEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YEN_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroAxisY
* Description    : Read YEN_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_YEN_G_t
* Output         : Status of YEN_G see LSM6DS0_ACC_GYRO_YEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroAxisZ
* Description    : Write ZEN_G
* Input          : LSM6DS0_ACC_GYRO_ZEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZEN_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroAxisZ
* Description    : Read ZEN_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZEN_G_t
* Output         : Status of ZEN_G see LSM6DS0_ACC_GYRO_ZEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZEN_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerAxisX
* Description    : Write XEN_XL
* Input          : LSM6DS0_ACC_GYRO_XEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerAxisX
* Description    : Read XEN_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_XEN_XL_t
* Output         : Status of XEN_XL see LSM6DS0_ACC_GYRO_XEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerAxisY
* Description    : Write YEN_XL
* Input          : LSM6DS0_ACC_GYRO_YEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerAxisY
* Description    : Read YEN_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_YEN_XL_t
* Output         : Status of YEN_XL see LSM6DS0_ACC_GYRO_YEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ
* Description    : Write ZEN_XL
* Input          : LSM6DS0_ACC_GYRO_ZEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZEN_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerAxisZ
* Description    : Read ZEN_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZEN_XL_t
* Output         : Status of ZEN_XL see LSM6DS0_ACC_GYRO_ZEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZEN_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerDataDecimation
* Description    : Write DEC_XL
* Input          : LSM6DS0_ACC_GYRO_DEC_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerDataDecimation(void *handle, LSM6DS0_ACC_GYRO_DEC_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DEC_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerDataDecimation
* Description    : Read DEC_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_DEC_XL_t
* Output         : Status of DEC_XL see LSM6DS0_ACC_GYRO_DEC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataDecimation(void *handle, LSM6DS0_ACC_GYRO_DEC_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DEC_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth
* Description    : Write BW_XL
* Input          : LSM6DS0_ACC_GYRO_BW_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth(void *handle, LSM6DS0_ACC_GYRO_BW_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BW_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerFilterBandwidth
* Description    : Read BW_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_BW_XL_t
* Output         : Status of BW_XL see LSM6DS0_ACC_GYRO_BW_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFilterBandwidth(void *handle, LSM6DS0_ACC_GYRO_BW_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BW_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_AccelerometerBandWitdthSelection
* Description    : Write BW_SCAL_ODR
* Input          : LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerBandWitdthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BW_SCAL_ODR_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerBandWitdthSelection
* Description    : Read BW_SCAL_ODR
* Input          : Pointer to LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t
* Output         : Status of BW_SCAL_ODR see LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerBandWitdthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BW_SCAL_ODR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerHighPass_on_Interrupt
* Description    : Write HPIS
* Input          : LSM6DS0_ACC_GYRO_HPIS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerHighPass_on_Interrupt(void *handle, LSM6DS0_ACC_GYRO_HPIS_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_HPIS_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerHighPass_on_Interrupt
* Description    : Read HPIS
* Input          : Pointer to LSM6DS0_ACC_GYRO_HPIS_t
* Output         : Status of HPIS see LSM6DS0_ACC_GYRO_HPIS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerHighPass_on_Interrupt(void *handle, LSM6DS0_ACC_GYRO_HPIS_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_HPIS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection
* Description    : Write FDS
* Input          : LSM6DS0_ACC_GYRO_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection(void *handle, LSM6DS0_ACC_GYRO_FDS_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FDS_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerFilteredDataSelection
* Description    : Read FDS
* Input          : Pointer to LSM6DS0_ACC_GYRO_FDS_t
* Output         : Status of FDS see LSM6DS0_ACC_GYRO_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFilteredDataSelection(void *handle, LSM6DS0_ACC_GYRO_FDS_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FDS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter
* Description    : Write DCF
* Input          : LSM6DS0_ACC_GYRO_DCF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter(void *handle, LSM6DS0_ACC_GYRO_DCF_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DCF_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerCutOff_filter
* Description    : Read DCF
* Input          : Pointer to LSM6DS0_ACC_GYRO_DCF_t
* Output         : Status of DCF see LSM6DS0_ACC_GYRO_DCF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerCutOff_filter(void *handle, LSM6DS0_ACC_GYRO_DCF_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DCF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode
* Description    : Write HR
* Input          : LSM6DS0_ACC_GYRO_HR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode(void *handle, LSM6DS0_ACC_GYRO_HR_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_HR_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerHighResolutionMode
* Description    : Read HR
* Input          : Pointer to LSM6DS0_ACC_GYRO_HR_t
* Output         : Status of HR see LSM6DS0_ACC_GYRO_HR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerHighResolutionMode(void *handle, LSM6DS0_ACC_GYRO_HR_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_HR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_ResetSW
* Description    : Write SW_RESET
* Input          : LSM6DS0_ACC_GYRO_SW_RESET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_ResetSW(void *handle, LSM6DS0_ACC_GYRO_SW_RESET_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SW_RESET_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_ResetSW
* Description    : Read SW_RESET
* Input          : Pointer to LSM6DS0_ACC_GYRO_SW_RESET_t
* Output         : Status of SW_RESET see LSM6DS0_ACC_GYRO_SW_RESET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_ResetSW(void *handle, LSM6DS0_ACC_GYRO_SW_RESET_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SW_RESET_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_BigLittleEndianDataSelection
* Description    : Write BLE
* Input          : LSM6DS0_ACC_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_BigLittleEndianDataSelection(void *handle, LSM6DS0_ACC_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BLE_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_BigLittleEndianDataSelection
* Description    : Read BLE
* Input          : Pointer to LSM6DS0_ACC_GYRO_BLE_t
* Output         : Status of BLE see LSM6DS0_ACC_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_BigLittleEndianDataSelection(void *handle, LSM6DS0_ACC_GYRO_BLE_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess
* Description    : Write IF_ADD_INC
* Input          : LSM6DS0_ACC_GYRO_IF_ADD_INC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess(void *handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_IF_ADD_INC_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AutoIndexOnMultiAccess
* Description    : Read IF_ADD_INC
* Input          : Pointer to LSM6DS0_ACC_GYRO_IF_ADD_INC_t
* Output         : Status of IF_ADD_INC see LSM6DS0_ACC_GYRO_IF_ADD_INC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AutoIndexOnMultiAccess(void *handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_IF_ADD_INC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_SPI_SerialInterfaceMode
* Description    : Write SIM
* Input          : LSM6DS0_ACC_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_SPI_SerialInterfaceMode(void *handle, LSM6DS0_ACC_GYRO_SIM_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SIM_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_SPI_SerialInterfaceMode
* Description    : Read SIM
* Input          : Pointer to LSM6DS0_ACC_GYRO_SIM_t
* Output         : Status of SIM see LSM6DS0_ACC_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_SPI_SerialInterfaceMode(void *handle, LSM6DS0_ACC_GYRO_SIM_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_INT_Pin_Mode
* Description    : Write PP_OD
* Input          : LSM6DS0_ACC_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_INT_Pin_Mode(void *handle, LSM6DS0_ACC_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_PP_OD_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_INT_Pin_Mode
* Description    : Read PP_OD
* Input          : Pointer to LSM6DS0_ACC_GYRO_PP_OD_t
* Output         : Status of PP_OD see LSM6DS0_ACC_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_INT_Pin_Mode(void *handle, LSM6DS0_ACC_GYRO_PP_OD_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptActive
* Description    : Write H_LACTIVE
* Input          : LSM6DS0_ACC_GYRO_H_LACTIVE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptActive(void *handle, LSM6DS0_ACC_GYRO_H_LACTIVE_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_H_LACTIVE_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptActive
* Description    : Read H_LACTIVE
* Input          : Pointer to LSM6DS0_ACC_GYRO_H_LACTIVE_t
* Output         : Status of H_LACTIVE see LSM6DS0_ACC_GYRO_H_LACTIVE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptActive(void *handle, LSM6DS0_ACC_GYRO_H_LACTIVE_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_H_LACTIVE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Reboot
* Description    : Write BOOT
* Input          : LSM6DS0_ACC_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Reboot(void *handle, LSM6DS0_ACC_GYRO_BOOT_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_BOOT_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Reboot
* Description    : Read BOOT
* Input          : Pointer to LSM6DS0_ACC_GYRO_BOOT_t
* Output         : Status of BOOT see LSM6DS0_ACC_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Reboot(void *handle, LSM6DS0_ACC_GYRO_BOOT_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG8, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO_Threshold_Level
* Description    : Write STOP_ON_FTH
* Input          : LSM6DS0_ACC_GYRO_STOP_ON_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold_Level(void *handle, LSM6DS0_ACC_GYRO_STOP_ON_FTH_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_STOP_ON_FTH_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Threshold_Level
* Description    : Read STOP_ON_FTH
* Input          : Pointer to LSM6DS0_ACC_GYRO_STOP_ON_FTH_t
* Output         : Status of STOP_ON_FTH see LSM6DS0_ACC_GYRO_STOP_ON_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold_Level(void *handle, LSM6DS0_ACC_GYRO_STOP_ON_FTH_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_STOP_ON_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO
* Description    : Write FIFO_EN
* Input          : LSM6DS0_ACC_GYRO_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FIFO_EN_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO
* Description    : Read FIFO_EN
* Input          : Pointer to LSM6DS0_ACC_GYRO_FIFO_EN_t
* Output         : Status of FIFO_EN see LSM6DS0_ACC_GYRO_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_EN_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FIFO_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_DigitalInterface
* Description    : Write I2C_DISABLE
* Input          : LSM6DS0_ACC_GYRO_I2C_DISABLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_DigitalInterface(void *handle, LSM6DS0_ACC_GYRO_I2C_DISABLE_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_I2C_DISABLE_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_DigitalInterface
* Description    : Read I2C_DISABLE
* Input          : Pointer to LSM6DS0_ACC_GYRO_I2C_DISABLE_t
* Output         : Status of I2C_DISABLE see LSM6DS0_ACC_GYRO_I2C_DISABLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_DigitalInterface(void *handle, LSM6DS0_ACC_GYRO_I2C_DISABLE_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_I2C_DISABLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_DataReadyTimer
* Description    : Write DRDY_MASK_BIT
* Input          : LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_DataReadyTimer(void *handle, LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_DataReadyTimer
* Description    : Read DRDY_MASK_BIT
* Input          : Pointer to LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t
* Output         : Status of DRDY_MASK_BIT see LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_DataReadyTimer(void *handle, LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_Temperature_In_FIFO
* Description    : Write FIFO_TEMP_EN
* Input          : LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_Temperature_In_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_Temperature_In_FIFO
* Description    : Read FIFO_TEMP_EN
* Input          : Pointer to LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : Status of FIFO_TEMP_EN see LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_Temperature_In_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroSleepMode
* Description    : Write SLEEP_G
* Input          : LSM6DS0_ACC_GYRO_SLEEP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroSleepMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_SLEEP_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroSleepMode
* Description    : Read SLEEP_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_SLEEP_G_t
* Output         : Status of SLEEP_G see LSM6DS0_ACC_GYRO_SLEEP_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroSleepMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG9, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_SLEEP_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_AccelerometerSelfTest
* Description    : Write ST_XL
* Input          : LSM6DS0_ACC_GYRO_ST_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_XL_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ST_XL_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerSelfTest
* Description    : Read ST_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ST_XL_t
* Output         : Status of ST_XL see LSM6DS0_ACC_GYRO_ST_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_XL_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ST_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroSelfTest
* Description    : Write ST_G
* Input          : LSM6DS0_ACC_GYRO_ST_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ST_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroSelfTest
* Description    : Read ST_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ST_G_t
* Output         : Status of ST_G see LSM6DS0_ACC_GYRO_ST_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_CTRL_REG10, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ST_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_X
* Description    : Read XL_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_XL_XL_t
* Output         : Status of XL_XL see LSM6DS0_ACC_GYRO_XL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_X(void *handle, LSM6DS0_ACC_GYRO_XL_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XL_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_X
* Description    : Read XH_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_XH_XL_t
* Output         : Status of XH_XL see LSM6DS0_ACC_GYRO_XH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_X(void *handle, LSM6DS0_ACC_GYRO_XH_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XH_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Y
* Description    : Read YL_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_YL_XL_t
* Output         : Status of YL_XL see LSM6DS0_ACC_GYRO_YL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Y(void *handle, LSM6DS0_ACC_GYRO_YL_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YL_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Y
* Description    : Read YH_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_YH_XL_t
* Output         : Status of YH_XL see LSM6DS0_ACC_GYRO_YH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Y(void *handle, LSM6DS0_ACC_GYRO_YH_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YH_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Z
* Description    : Read ZL_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZL_XL_t
* Output         : Status of ZL_XL see LSM6DS0_ACC_GYRO_ZL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Z(void *handle, LSM6DS0_ACC_GYRO_ZL_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZL_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Z
* Description    : Read ZH_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZH_XL_t
* Output         : Status of ZH_XL see LSM6DS0_ACC_GYRO_ZH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Z(void *handle, LSM6DS0_ACC_GYRO_ZH_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZH_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag
* Description    : Read IA_XL
* Input          : Pointer to LSM6DS0_ACC_GYRO_IA_XL_t
* Output         : Status of IA_XL see LSM6DS0_ACC_GYRO_IA_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_IA_XL_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_IA_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO_Threshold
* Description    : Write FTH
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_FTH_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_FTH_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FTH_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Threshold
* Description    : Read FTH
* Input          : Pointer to u8_t
* Output         : Status of FTH
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FTH_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_FTH_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_FIFO_Mode
* Description    : Write FMODE
* Input          : LSM6DS0_ACC_GYRO_FMODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Mode(void *handle, LSM6DS0_ACC_GYRO_FMODE_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_FMODE_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Mode
* Description    : Read FMODE
* Input          : Pointer to LSM6DS0_ACC_GYRO_FMODE_t
* Output         : Status of FMODE see LSM6DS0_ACC_GYRO_FMODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Mode(void *handle, LSM6DS0_ACC_GYRO_FMODE_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FMODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_Samples
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_Samples(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FSS_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_FSS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_OverrunFlag
* Description    : Read OVRN
* Input          : Pointer to LSM6DS0_ACC_GYRO_OVRN_t
* Output         : Status of OVRN see LSM6DS0_ACC_GYRO_OVRN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_OverrunFlag(void *handle, LSM6DS0_ACC_GYRO_OVRN_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_OVRN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_FIFO_ThresholdFlag
* Description    : Read FTH
* Input          : Pointer to LSM6DS0_ACC_GYRO_FTH_t
* Output         : Status of FTH see LSM6DS0_ACC_GYRO_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_FIFO_ThresholdFlag(void *handle, LSM6DS0_ACC_GYRO_FTH_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_FTH_FLAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisX
* Description    : Write XLIE_G
* Input          : LSM6DS0_ACC_GYRO_XLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XLIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisX
* Description    : Read XLIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_XLIE_G_t
* Output         : Status of XLIE_G see LSM6DS0_ACC_GYRO_XLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XLIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisX
* Description    : Write XHIE_G
* Input          : LSM6DS0_ACC_GYRO_XHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_XHIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisX
* Description    : Read XHIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_XHIE_G_t
* Output         : Status of XHIE_G see LSM6DS0_ACC_GYRO_XHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_XHIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisY
* Description    : Write YLIE_G
* Input          : LSM6DS0_ACC_GYRO_YLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YLIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisY
* Description    : Read YLIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_YLIE_G_t
* Output         : Status of YLIE_G see LSM6DS0_ACC_GYRO_YLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YLIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisY
* Description    : Write YHIE_G
* Input          : LSM6DS0_ACC_GYRO_YHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_YHIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisY
* Description    : Read YHIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_YHIE_G_t
* Output         : Status of YHIE_G see LSM6DS0_ACC_GYRO_YHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_YHIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisZ
* Description    : Write ZLIE_G
* Input          : LSM6DS0_ACC_GYRO_ZLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZLIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisZ
* Description    : Read ZLIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZLIE_G_t
* Output         : Status of ZLIE_G see LSM6DS0_ACC_GYRO_ZLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZLIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisZ
* Description    : Write ZHIE_G
* Input          : LSM6DS0_ACC_GYRO_ZHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_ZHIE_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisZ
* Description    : Read ZHIE_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_ZHIE_G_t
* Output         : Status of ZHIE_G see LSM6DS0_ACC_GYRO_ZHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_ZHIE_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterruptSignalType
* Description    : Write LIR_G
* Input          : LSM6DS0_ACC_GYRO_LIR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterruptSignalType(void *handle, LSM6DS0_ACC_GYRO_LIR_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_LIR_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterruptSignalType
* Description    : Read LIR_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_LIR_G_t
* Output         : Status of LIR_G see LSM6DS0_ACC_GYRO_LIR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptSignalType(void *handle, LSM6DS0_ACC_GYRO_LIR_G_t *value)
{

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_LIR_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroInterruptCombinationEvent
* Description    : Write AOI_G
* Input          : LSM6DS0_ACC_GYRO_AOI_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroInterruptCombinationEvent(void *handle, LSM6DS0_ACC_GYRO_AOI_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_AOI_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroInterruptCombinationEvent
* Description    : Read AOI_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_AOI_G_t
* Output         : Status of AOI_G see LSM6DS0_ACC_GYRO_AOI_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptCombinationEvent(void *handle, LSM6DS0_ACC_GYRO_AOI_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_AOI_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_GyroCounterModeSelection
* Description    : Write DCRM_G
* Input          : LSM6DS0_ACC_GYRO_DCRM_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_GyroCounterModeSelection(void *handle, LSM6DS0_ACC_GYRO_DCRM_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DCRM_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_GyroCounterModeSelection
* Description    : Read DCRM_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_DCRM_G_t
* Output         : Status of DCRM_G see LSM6DS0_ACC_GYRO_DCRM_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_GyroCounterModeSelection(void *handle, LSM6DS0_ACC_GYRO_DCRM_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DCRM_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_InterruptDurationValue
* Description    : Write DUR_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_InterruptDurationValue(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM6DS0_ACC_GYRO_DUR_G_POSITION; //mask
  newValue &= LSM6DS0_ACC_GYRO_DUR_G_MASK; //coerce

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_DUR_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_InterruptDurationValue
* Description    : Read DUR_G
* Input          : Pointer to u8_t
* Output         : Status of DUR_G
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_InterruptDurationValue(void *handle, u8_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_DUR_G_MASK; //coerce
  *value = *value >> LSM6DS0_ACC_GYRO_DUR_G_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_W_WaitFunction
* Description    : Write WAIT_G
* Input          : LSM6DS0_ACC_GYRO_WAIT_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM6DS0_ACC_GYRO_W_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_G_t newValue)
{
  u8_t value;

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM6DS0_ACC_GYRO_WAIT_G_MASK;
  value |= newValue;

  if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, &value, 1 ) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM6DS0_ACC_GYRO_R_WaitFunction
* Description    : Read WAIT_G
* Input          : Pointer to LSM6DS0_ACC_GYRO_WAIT_G_t
* Output         : Status of WAIT_G see LSM6DS0_ACC_GYRO_WAIT_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_R_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_G_t *value)
{
  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_DUR_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM6DS0_ACC_GYRO_WAIT_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS0_ACC_GYRO_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_Temperature(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 2 / 1;

  k = 0;
  for (i = 0; i < 1; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_OUT_TEMP_L + k, &buff[k], 1 ))
        return MEMS_ERROR;
      k++;
    }
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS0_ACC_GYRO_Set_AngularRateThreshold(u8_t *buff)
* Description    : Set AngularRateThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Set_AngularRateThreshold(void *handle, u8_t *buff)
{
  u8_t  i, value;
  u8_t numberOfByteForDimension;


  numberOfByteForDimension = 6 / 3;
  LSM6DS0_ACC_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G, &value, 1) )
    return MEMS_ERROR;

  /*Coerce 15 bit two's complement value*/
  value &= LSM6DS0_ACC_GYRO_DCRM_G_MASK;
  buff[0] &= ~LSM6DS0_ACC_GYRO_DCRM_G_MASK;
  buff[0] |= value;
  buff[2] &= 0x7F;
  buff[4] &= 0x7F;

  for (i = 0; i < 6; i++ )
  {
    if( !LSM6DS0_ACC_GYRO_WriteReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G + i,  &buff[i], 1 ) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM6DS0_ACC_GYRO_Get_AngularRateThreshold(u8_t *buff)
* Description    : Read AngularRateThreshold output register
* Input          : pointer to [u8_t]
* Output         : AngularRateThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_AngularRateThreshold(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension = 6 / 3;

  k = 0;
  for (i = 0; i < 3; i++ )
  {
    for (j = 0; j < numberOfByteForDimension; j++ )
    {
      if( !LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G + k, &buff[k], 1 ))
        return MEMS_ERROR;
      k++;
    }
  }

  /*Sign Extension on 16 bit*/
  if (buff[0] & 0x40)
    buff[0] |= 0x80;
  if (buff[2] & 0x40)
    buff[2] |= 0x80;
  if (buff[4] & 0x40)
    buff[4] |= 0x80;

  LSM6DS0_ACC_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

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
void LSM6DS0_ACC_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
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



