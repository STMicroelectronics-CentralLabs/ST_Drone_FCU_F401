/**
 ******************************************************************************
 * @file    HTS221_Driver.c
 * @author  HESA Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   HTS221 driver file
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
#include "HTS221_Driver.h"

#ifdef  USE_FULL_ASSERT_HTS221
#include <stdio.h>
#endif


/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup HTS221_DRIVER
* @brief HTS221 DRIVER
* @{
*/

/** @defgroup HTS221_Imported_Function_Prototypes
* @{
*/

extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

/**
* @}
*/

/** @defgroup HTS221_Private_Function_Prototypes
* @{
*/

/**
* @}
*/

/** @defgroup HTS221_Private_Functions
* @{
*/

/**
* @}
*/

/** @defgroup HTS221_Public_Functions
* @{
*/

/*******************************************************************************
* Function Name   : HTS221_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data Read
* Return      : None
*******************************************************************************/
HTS221_Error_et HTS221_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{

  if ( NumByteToRead > 1 ) RegAddr |= 0x80;

  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return HTS221_ERROR;
  else
    return HTS221_OK;
}

/*******************************************************************************
* Function Name   : HTS221_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
HTS221_Error_et HTS221_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{

  if ( NumByteToWrite > 1 ) RegAddr |= 0x80;

  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return HTS221_ERROR;
  else
    return HTS221_OK;
}

/**
* @brief  Get the version of this driver.
* @param  pxVersion pointer to a HTS221_DriverVersion_st structure that contains the version information.
*         This parameter is a pointer to @ref HTS221_DriverVersion_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DriverVersion(HTS221_DriverVersion_st* version)
{
  version->Major = HTS221_DRIVER_VERSION_MAJOR;
  version->Minor = HTS221_DRIVER_VERSION_MINOR;
  version->Point = HTS221_DRIVER_VERSION_POINT;

  return HTS221_OK;
}

/**
* @brief  Get device type ID.
* @param  *handle Device handle.
* @param  deviceid pointer to the returned device type ID.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(HTS221_ReadReg(handle, HTS221_WHO_AM_I_REG, 1, deviceid))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Initializes the HTS221 with the specified parameters in HTS221_Init_st struct.
* @param  *handle Device handle.
* @param  pxInit pointer to a HTS221_Init_st structure that contains the configuration.
*         This parameter is a pointer to @ref HTS221_Init_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_InitConfig(void *handle, HTS221_Init_st* pxInit)
{
  uint8_t buffer[3];

  HTS221_assert_param(IS_HTS221_AVGH(pxInit->avg_h));
  HTS221_assert_param(IS_HTS221_AVGT(pxInit->avg_t));
  HTS221_assert_param(IS_HTS221_ODR(pxInit->odr));
  HTS221_assert_param(IS_HTS221_State(pxInit->bdu_status));
  HTS221_assert_param(IS_HTS221_State(pxInit->heater_status));

  HTS221_assert_param(IS_HTS221_DrdyLevelType(pxInit->irq_level));
  HTS221_assert_param(IS_HTS221_OutputType(pxInit->irq_output_type));
  HTS221_assert_param(IS_HTS221_State(pxInit->irq_enable));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  buffer[0] &= ~(HTS221_AVGH_MASK | HTS221_AVGT_MASK);
  buffer[0] |= (uint8_t)pxInit->avg_h;
  buffer[0] |= (uint8_t)pxInit->avg_t;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  buffer[0] &= ~(HTS221_BDU_MASK | HTS221_ODR_MASK);
  buffer[0] |= (uint8_t)pxInit->odr;
  buffer[0] |= ((uint8_t)pxInit->bdu_status) << HTS221_BDU_BIT;

  buffer[1] &= ~HTS221_HEATHER_BIT;
  buffer[1] |= ((uint8_t)pxInit->heater_status) << HTS221_HEATHER_BIT;

  buffer[2] &= ~(HTS221_DRDY_H_L_MASK | HTS221_PP_OD_MASK | HTS221_DRDY_MASK);
  buffer[2] |= ((uint8_t)pxInit->irq_level) << HTS221_DRDY_H_L_BIT;
  buffer[2] |= (uint8_t)pxInit->irq_output_type;
  buffer[2] |= ((uint8_t)pxInit->irq_enable) << HTS221_DRDY_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Returns a HTS221_Init_st struct with the actual configuration.
* @param  *handle Device handle.
* @param  pxInit pointer to a HTS221_Init_st structure.
*         This parameter is a pointer to @ref HTS221_Init_st.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_InitConfig(void *handle, HTS221_Init_st* pxInit)
{
  uint8_t buffer[3];

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, buffer))
    return HTS221_ERROR;

  pxInit->avg_h = (HTS221_Avgh_et)(buffer[0] & HTS221_AVGH_MASK);
  pxInit->avg_t = (HTS221_Avgt_et)(buffer[0] & HTS221_AVGT_MASK);

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 3, buffer))
    return HTS221_ERROR;

  pxInit->odr = (HTS221_Odr_et)(buffer[0] & HTS221_ODR_MASK);
  pxInit->bdu_status = (HTS221_State_et)((buffer[0] & HTS221_BDU_MASK) >> HTS221_BDU_BIT);
  pxInit->heater_status = (HTS221_State_et)((buffer[1] & HTS221_HEATHER_MASK) >> HTS221_HEATHER_BIT);

  pxInit->irq_level = (HTS221_DrdyLevel_et)(buffer[2] & HTS221_DRDY_H_L_MASK);
  pxInit->irq_output_type = (HTS221_OutputType_et)(buffer[2] & HTS221_PP_OD_MASK);
  pxInit->irq_enable = (HTS221_State_et)((buffer[2] & HTS221_DRDY_MASK) >> HTS221_DRDY_BIT);

  return HTS221_OK;
}

/**
* @brief  De initialization function for HTS221.
*         This function put the HTS221 in power down, make a memory boot and clear the data output flags.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_DeInit(void *handle)
{
  uint8_t buffer[4];

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 2, buffer))
    return HTS221_ERROR;

  /* HTS221 in power down */
  buffer[0] |= 0x01 << HTS221_PD_BIT;

  /* Make HTS221 boot */
  buffer[1] |= 0x01 << HTS221_BOOT_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 2, buffer))
    return HTS221_ERROR;

  /* Dump of data output */
  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 4, buffer))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 output registers, and calculate humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity value that must be divided by 10 to get the value in [%].
* @param  temperature pointer to the returned temperature value that must be divided by 10 to get the value in ['C].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Measurement(void *handle, uint16_t* humidity, int16_t* temperature)
{
  if ( HTS221_Get_Temperature( handle, temperature ) == HTS221_ERROR ) return HTS221_ERROR;
  if ( HTS221_Get_Humidity( handle, humidity ) == HTS221_ERROR ) return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 output registers. Humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity raw value.
* @param  temperature pointer to the returned temperature raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_RawMeasurement(void *handle, int16_t* humidity, int16_t* temperature)
{
  uint8_t buffer[4];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 4, buffer))
    return HTS221_ERROR;

  *humidity = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);
  *temperature = (int16_t)((((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2]);

  return HTS221_OK;
}

/**
* @brief  Read HTS221 Humidity output registers, and calculate humidity.
* @param  *handle Device handle.
* @param  Pointer to the returned humidity value that must be divided by 10 to get the value in [%].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Humidity(void *handle, uint16_t* value)
{
  int16_t H0_T0_out, H1_T0_out, H_T_out;
  int16_t H0_rh, H1_rh;
  uint8_t buffer[2];
  float   tmp_f;

  if(HTS221_ReadReg(handle, HTS221_H0_RH_X2, 2, buffer))
    return HTS221_ERROR;
  H0_rh = buffer[0] >> 1;
  H1_rh = buffer[1] >> 1;

  if(HTS221_ReadReg(handle, HTS221_H0_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H0_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_ReadReg(handle, HTS221_H1_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H1_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;
  H_T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
  tmp_f *= 10.0f;

  *value = ( tmp_f > 1000.0f ) ? 1000
           : ( tmp_f <    0.0f ) ?    0
           : ( uint16_t )tmp_f;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 humidity output registers.
* @param  *handle Device handle.
* @param  Pointer to the returned humidity raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_HumidityRaw(void *handle, int16_t* value)
{
  uint8_t buffer[2];

  if(HTS221_ReadReg(handle, HTS221_HR_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  *value = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);

  return HTS221_OK;
}

/**
* @brief  Read HTS221 temperature output registers, and calculate temperature.
* @param  *handle Device handle.
* @param  Pointer to the returned temperature value that must be divided by 10 to get the value in ['C].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Temperature(void *handle, int16_t *value)
{
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  uint8_t buffer[4], tmp;
  float   tmp_f;

  if(HTS221_ReadReg(handle, HTS221_T0_DEGC_X8, 2, buffer))
    return HTS221_ERROR;
  if(HTS221_ReadReg(handle, HTS221_T0_T1_DEGC_H2, 1, &tmp))
    return HTS221_ERROR;

  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16 >> 3;
  T1_degC = T1_degC_x8_u16 >> 3;

  if(HTS221_ReadReg(handle, HTS221_T0_OUT_L, 4, buffer))
    return HTS221_ERROR;

  T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];

  if(HTS221_ReadReg(handle, HTS221_TEMP_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;
  tmp_f *= 10.0f;

  *value = ( int16_t )tmp_f;

  return HTS221_OK;
}

/**
* @brief  Read HTS221 temperature output registers.
* @param  *handle Device handle.
* @param  Pointer to the returned temperature raw value.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_TemperatureRaw(void *handle, int16_t* value)
{
  uint8_t buffer[2];

  if(HTS221_ReadReg(handle, HTS221_TEMP_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;

  *value = (int16_t)((((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0]);

  return HTS221_OK;
}

/**
* @brief  Get the availability of new data for humidity and temperature.
* @param  *handle Device handle.
* @param  humidity pointer to the returned humidity data status [HTS221_SET/HTS221_RESET].
* @param  temperature pointer to the returned temperature data status [HTS221_SET/HTS221_RESET].
*         This parameter is a pointer to @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_DataStatus(void *handle, HTS221_BitStatus_et* humidity, HTS221_BitStatus_et* temperature)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_STATUS_REG, 1, &tmp))
    return HTS221_ERROR;

  *humidity = (HTS221_BitStatus_et)((tmp & HTS221_HDA_MASK) >> HTS221_H_DA_BIT);
  *temperature = (HTS221_BitStatus_et)(tmp & HTS221_TDA_MASK);

  return HTS221_OK;
}

/**
* @brief  Exit from power down mode.
* @param  *handle Device handle.
* @param  void.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Activate(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_PD_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Put the sensor in power down mode.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_DeActivate(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PD_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}



/**
* @brief  Check if the single measurement has completed.
* @param  *handle Device handle.
* @param  tmp is set to 1, when the measure is completed
* @retval Status [HTS221_ERROR, HTS221_OK]
*/
HTS221_Error_et HTS221_IsMeasurementCompleted(void *handle, HTS221_BitStatus_et* Is_Measurement_Completed)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_STATUS_REG, 1, &tmp))
    return HTS221_ERROR;

  if((tmp & (uint8_t)(HTS221_HDA_MASK | HTS221_TDA_MASK)) == (uint8_t)(HTS221_HDA_MASK | HTS221_TDA_MASK))
    *Is_Measurement_Completed = HTS221_SET;
  else
    *Is_Measurement_Completed = HTS221_RESET;

  return HTS221_OK;
}


/**
* @brief  Set_ humidity and temperature average mode.
* @param  *handle Device handle.
* @param  avgh is the average mode for humidity, this parameter is @ref HTS221_Avgh_et.
* @param  avgt is the average mode for temperature, this parameter is @ref HTS221_Avgt_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgHT(void *handle, HTS221_Avgh_et avgh, HTS221_Avgt_et avgt)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGH(avgh));
  HTS221_assert_param(IS_HTS221_AVGT(avgt));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~(HTS221_AVGH_MASK | HTS221_AVGT_MASK);
  tmp |= (uint8_t)avgh;
  tmp |= (uint8_t)avgt;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Set humidity average mode.
* @param  *handle Device handle.
* @param  avgh is the average mode for humidity, this parameter is @ref HTS221_Avgh_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgH(void *handle, HTS221_Avgh_et avgh)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGH(avgh));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_AVGH_MASK;
  tmp |= (uint8_t)avgh;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Set temperature average mode.
* @param  *handle Device handle.
* @param  avgt is the average mode for temperature, this parameter is @ref HTS221_Avgt_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_AvgT(void *handle, HTS221_Avgt_et avgt)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_AVGT(avgt));

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_AVGT_MASK;
  tmp |= (uint8_t)avgt;

  if(HTS221_WriteReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get humidity and temperature average mode.
* @param  *handle Device handle.
* @param  avgh pointer to the returned value with the humidity average mode.
* @param  avgt pointer to the returned value with the temperature average mode.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_AvgHT(void *handle, HTS221_Avgh_et* avgh, HTS221_Avgt_et* avgt)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_AV_CONF_REG, 1, &tmp))
    return HTS221_ERROR;

  *avgh = (HTS221_Avgh_et)(tmp & HTS221_AVGH_MASK);
  *avgt = (HTS221_Avgt_et)(tmp & HTS221_AVGT_MASK);

  return HTS221_OK;
}

/**
* @brief  Set block data update mode.
* @param  *handle Device handle.
* @param  status can be HTS221_ENABLE: enable the block data update, output data registers are updated once both MSB and LSB are read.
* @param  status can be HTS221_DISABLE: output data registers are continuously updated.
*         This parameter is a @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_BduMode(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_BDU_MASK;
  tmp |= ((uint8_t)status) << HTS221_BDU_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get block data update mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with block data update mode status.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_BduMode(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_BDU_MASK) >> HTS221_BDU_BIT);

  return HTS221_OK;
}

/**
* @brief  Enter or exit from power down mode.
* @param  *handle Device handle.
* @param  status can be HTS221_SET: HTS221 in power down mode.
* @param  status can be HTS221_REET: HTS221 in active mode.
*         This parameter is a @ref HTS221_BitStatus_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_PowerDownMode(void *handle, HTS221_BitStatus_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_BitStatus(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PD_MASK;
  tmp |= ((uint8_t)status) << HTS221_PD_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get if HTS221 is in active mode or in power down mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with HTS221 status.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_PowerDownMode(void *handle, HTS221_BitStatus_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_BitStatus_et)((tmp & HTS221_PD_MASK) >> HTS221_PD_BIT);

  return HTS221_OK;
}

/**
* @brief  Set the output data rate mode.
* @param  *handle Device handle.
* @param  odr is the output data rate mode.
*         This parameter is a @ref HTS221_Odr_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_Odr(void *handle, HTS221_Odr_et odr)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_ODR(odr));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the output data rate mode.
* @param  *handle Device handle.
* @param  Pointer to the returned value with output data rate mode.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_Odr(void *handle, HTS221_Odr_et* odr)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG1, 1, &tmp))
    return HTS221_ERROR;

  tmp &= HTS221_ODR_MASK;
  *odr = (HTS221_Odr_et)tmp;

  return HTS221_OK;
}

/**
* @brief  Reboot Memory Content.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_MemoryBoot(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_BOOT_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Configure the internal heater.
* @param  *handle Device handle.
* @param  The status of the internal heater [HTS221_ENABLE/HTS221_DISABLE].
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR]
*/
HTS221_Error_et HTS221_Set_HeaterState(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_HEATHER_MASK;
  tmp |= ((uint8_t)status) << HTS221_HEATHER_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the internal heater.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the internal heater [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_HeaterState(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_HEATHER_MASK) >> HTS221_HEATHER_BIT);

  return HTS221_OK;
}

/**
* @brief  Set ONE_SHOT bit to start a new conversion (ODR mode has to be 00).
*         Once the measurement is done, ONE_SHOT bit is self-cleared.
* @param  *handle Device handle.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_StartOneShotMeasurement(void *handle)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  tmp |= HTS221_ONE_SHOT_MASK;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;

}

/**
* @brief  Set level configuration of the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  status can be HTS221_LOW_LVL: active level is LOW.
* @param  status can be HTS221_HIGH_LVL: active level is HIGH.
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et value)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_DrdyLevelType(value));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_DRDY_H_L_MASK;
  tmp |= (uint8_t)value;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get level configuration of the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the level configuration [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et* value)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *value = (HTS221_DrdyLevel_et)(tmp & HTS221_DRDY_H_L_MASK);

  return HTS221_OK;
}

/**
* @brief  Set Push-pull/open drain configuration for the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  value is the output type configuration.
*         This parameter is a @ref HTS221_OutputType_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqOutputType(void *handle, HTS221_OutputType_et value)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_OutputType(value));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_PP_OD_MASK;
  tmp |= (uint8_t)value;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the configuration for the interrupt pin DRDY.
* @param  *handle Device handle.
* @param  Pointer to the returned value with output type configuration.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqOutputType(void *handle, HTS221_OutputType_et* value)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *value = (HTS221_OutputType_et)(tmp & HTS221_PP_OD_MASK);

  return HTS221_OK;
}

/**
* @brief  Enable/disable the interrupt mode.
* @param  *handle Device handle.
* @param  status is the enable/disable for the interrupt mode.
*         This parameter is a @ref HTS221_State_et.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Set_IrqEnable(void *handle, HTS221_State_et status)
{
  uint8_t tmp;

  HTS221_assert_param(IS_HTS221_State(status));

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  tmp &= ~HTS221_DRDY_MASK;
  tmp |= ((uint8_t)status) << HTS221_DRDY_BIT;

  if(HTS221_WriteReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  return HTS221_OK;
}

/**
* @brief  Get the interrupt mode.
* @param  *handle Device handle.
* @param  Pointer to the returned status of the interrupt mode configuration [HTS221_ENABLE/HTS221_DISABLE].
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
HTS221_Error_et HTS221_Get_IrqEnable(void *handle, HTS221_State_et* status)
{
  uint8_t tmp;

  if(HTS221_ReadReg(handle, HTS221_CTRL_REG3, 1, &tmp))
    return HTS221_ERROR;

  *status = (HTS221_State_et)((tmp & HTS221_DRDY_MASK) >> HTS221_DRDY_BIT);

  return HTS221_OK;
}


#ifdef  USE_FULL_ASSERT_HTS221
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void HTS221_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);

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
