/**
 ******************************************************************************
 * @file    HTS221_Driver.h
 * @author  HESA Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   HTS221 driver header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTS221_DRIVER__H
#define __HTS221_DRIVER__H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment the line below to expanse the "assert_param" macro in the drivers code */
#define USE_FULL_ASSERT_HTS221

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_HTS221

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports
*         the name of the source file and the source line number of the call
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define HTS221_assert_param(expr) ((expr) ? (void)0 : HTS221_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void HTS221_assert_failed(uint8_t* file, uint32_t line);
#else
#define HTS221_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_HTS221 */

/** @addtogroup Environmental_Sensor
* @{
*/

/** @addtogroup HTS221_DRIVER
* @{
*/

/* Exported Types -------------------------------------------------------------*/
/** @defgroup HTS221_Exported_Types
* @{
*/


/**
* @brief  Error code type.
*/
typedef enum {HTS221_OK = (uint8_t)0, HTS221_ERROR = !HTS221_OK} HTS221_Error_et;

/**
* @brief  State type.
*/
typedef enum {HTS221_DISABLE = (uint8_t)0, HTS221_ENABLE = !HTS221_DISABLE} HTS221_State_et;
#define IS_HTS221_State(MODE) ((MODE == HTS221_ENABLE) || (MODE == HTS221_DISABLE))

/**
* @brief  Bit status type.
*/
typedef enum {HTS221_RESET = (uint8_t)0, HTS221_SET = !HTS221_RESET} HTS221_BitStatus_et;
#define IS_HTS221_BitStatus(MODE) ((MODE == HTS221_RESET) || (MODE == HTS221_SET))

/**
* @brief  Humidity average.
*/
typedef enum
{
  HTS221_AVGH_4         = (uint8_t)0x00,         /*!< Internal average on 4 samples */
  HTS221_AVGH_8         = (uint8_t)0x01,         /*!< Internal average on 8 samples */
  HTS221_AVGH_16        = (uint8_t)0x02,         /*!< Internal average on 16 samples */
  HTS221_AVGH_32        = (uint8_t)0x03,         /*!< Internal average on 32 samples */
  HTS221_AVGH_64        = (uint8_t)0x04,         /*!< Internal average on 64 samples */
  HTS221_AVGH_128       = (uint8_t)0x05,         /*!< Internal average on 128 samples */
  HTS221_AVGH_256       = (uint8_t)0x06,         /*!< Internal average on 256 samples */
  HTS221_AVGH_512       = (uint8_t)0x07          /*!< Internal average on 512 samples */
} HTS221_Avgh_et;
#define IS_HTS221_AVGH(AVGH) ((AVGH == HTS221_AVGH_4) || (AVGH == HTS221_AVGH_8) || \
                              (AVGH == HTS221_AVGH_16) || (AVGH == HTS221_AVGH_32) || \
                              (AVGH == HTS221_AVGH_64) || (AVGH == HTS221_AVGH_128) || \
                              (AVGH == HTS221_AVGH_256) || (AVGH == HTS221_AVGH_512))

/**
* @brief  Temperature average.
*/
typedef enum
{
  HTS221_AVGT_2         = (uint8_t)0x00,        /*!< Internal average on 2 samples */
  HTS221_AVGT_4         = (uint8_t)0x08,        /*!< Internal average on 4 samples */
  HTS221_AVGT_8         = (uint8_t)0x10,        /*!< Internal average on 8 samples */
  HTS221_AVGT_16        = (uint8_t)0x18,        /*!< Internal average on 16 samples */
  HTS221_AVGT_32        = (uint8_t)0x20,        /*!< Internal average on 32 samples */
  HTS221_AVGT_64        = (uint8_t)0x28,        /*!< Internal average on 64 samples */
  HTS221_AVGT_128       = (uint8_t)0x30,        /*!< Internal average on 128 samples */
  HTS221_AVGT_256       = (uint8_t)0x38         /*!< Internal average on 256 samples */
} HTS221_Avgt_et;
#define IS_HTS221_AVGT(AVGT) ((AVGT == HTS221_AVGT_2) || (AVGT == HTS221_AVGT_4) || \
                              (AVGT == HTS221_AVGT_8) || (AVGT == HTS221_AVGT_16) || \
                              (AVGT == HTS221_AVGT_32) || (AVGT == HTS221_AVGT_64) || \
                              (AVGT == HTS221_AVGT_128) || (AVGT == HTS221_AVGT_256))

/**
* @brief  Output data rate configuration.
*/
typedef enum
{
  HTS221_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  HTS221_ODR_1HZ       = (uint8_t)0x01,         /*!< Output Data Rate: 1Hz */
  HTS221_ODR_7HZ       = (uint8_t)0x02,         /*!< Output Data Rate: 7Hz */
  HTS221_ODR_12_5HZ    = (uint8_t)0x03,         /*!< Output Data Rate: 12.5Hz */
} HTS221_Odr_et;
#define IS_HTS221_ODR(ODR) ((ODR == HTS221_ODR_ONE_SHOT) || (ODR == HTS221_ODR_1HZ) || \
                            (ODR == HTS221_ODR_7HZ) || (ODR == HTS221_ODR_12_5HZ))


/**
* @brief  Push-pull/Open Drain selection on DRDY pin.
*/
typedef enum
{
  HTS221_PUSHPULL   = (uint8_t)0x00,   /*!< DRDY pin in push pull */
  HTS221_OPENDRAIN  = (uint8_t)0x40    /*!< DRDY pin in open drain */
} HTS221_OutputType_et;
#define IS_HTS221_OutputType(MODE) ((MODE == HTS221_PUSHPULL) || (MODE == HTS221_OPENDRAIN))

/**
* @brief  Active level of DRDY pin.
*/
typedef enum
{
  HTS221_HIGH_LVL   = (uint8_t)0x00,   /*!< HIGH state level for DRDY pin */
  HTS221_LOW_LVL    = (uint8_t)0x80    /*!< LOW state level for DRDY pin */
} HTS221_DrdyLevel_et;
#define IS_HTS221_DrdyLevelType(MODE) ((MODE == HTS221_HIGH_LVL) || (MODE == HTS221_LOW_LVL))

/**
* @brief  Driver Version Info structure definition.
*/
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t   Point;
} HTS221_DriverVersion_st;


/**
* @brief  HTS221 Init structure definition.
*/
typedef struct
{
  HTS221_Avgh_et        avg_h;            /*!< Humidity average */
  HTS221_Avgt_et        avg_t;            /*!< Temperature average */
  HTS221_Odr_et         odr;              /*!< Output data rate */
  HTS221_State_et       bdu_status;       /*!< HTS221_ENABLE/HTS221_DISABLE the block data update */
  HTS221_State_et       heater_status;    /*!< HTS221_ENABLE/HTS221_DISABLE the internal heater */

  HTS221_DrdyLevel_et   irq_level;        /*!< HTS221_HIGH_LVL/HTS221_LOW_LVL the level for DRDY pin */
  HTS221_OutputType_et  irq_output_type;  /*!< Output configuration for DRDY pin */
  HTS221_State_et       irq_enable;       /*!< HTS221_ENABLE/HTS221_DISABLE interrupt on DRDY pin */
} HTS221_Init_st;

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup HTS221_Exported_Constants
* @{
*/

/**
* @brief  Bitfield positioning.
*/
#define HTS221_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
#define HTS221_I2C_ADDRESS  (uint8_t)0xBE

/**
* @brief  Driver version.
*/
#define HTS221_DRIVER_VERSION_MAJOR (uint8_t)1
#define HTS221_DRIVER_VERSION_MINOR (uint8_t)1
#define HTS221_DRIVER_VERSION_POINT (uint8_t)0

/**
* @addtogroup HTS221_Registers
* @{
*/


/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xBC
* 7:0 This read-only register contains the device identifier for HTS221.
* \endcode
*/
#define HTS221_WHO_AM_I_REG          (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC


/**
* @brief Humidity and temperature average mode register.
* \code
* Read/write
* Default value: 0x1B
* 7:6 Reserved.
* 5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
*
*      AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
*   ----------------------------------------------------
*       0    |   0   |   0   |    2
*       0    |   0   |   1   |    4
*       0    |   1   |   0   |    8
*       0    |   1   |   1   |    16
*       1    |   0   |   0   |    32
*       1    |   0   |   1   |    64
*       1    |   1   |   0   |    128
*       1    |   1   |   1   |    256
*
* 2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
*      AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
*   ------------------------------------------------------
*       0    |   0   |   0   |    4
*       0    |   0   |   1   |    8
*       0    |   1   |   0   |    16
*       0    |   1   |   1   |    32
*       1    |   0   |   0   |    64
*       1    |   0   |   1   |    128
*       1    |   1   |   0   |    256
*       1    |   1   |   1   |    512
*
* \endcode
*/
#define HTS221_AV_CONF_REG        (uint8_t)0x10

#define HTS221_AVGT_BIT           HTS221_BIT(3)
#define HTS221_AVGH_BIT           HTS221_BIT(0)

#define HTS221_AVGH_MASK          (uint8_t)0x07
#define HTS221_AVGT_MASK          (uint8_t)0x38

/**
* @brief Control register 1.
* \code
* Read/write
* Default value: 0x00
* 7 PD: power down control. 0 - power down mode; 1 - active mode.
* 6:3 Reserved.
* 2 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 1:0 ODR1, ODR0: output data rate selection.
*
*   ODR1  | ODR0  | Humidity output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*     0   |   0   |         one shot               |         one shot
*     0   |   1   |            1                   |            1
*     1   |   0   |            7                   |            7
*     1   |   1   |           12.5                 |           12.5
*
* \endcode
*/
#define HTS221_CTRL_REG1      (uint8_t)0x20

#define HTS221_PD_BIT          HTS221_BIT(7)
#define HTS221_BDU_BIT         HTS221_BIT(2)
#define HTS221_ODR_BIT         HTS221_BIT(0)

#define HTS221_PD_MASK        (uint8_t)0x80
#define HTS221_BDU_MASK       (uint8_t)0x04
#define HTS221_ODR_MASK       (uint8_t)0x03

/**
* @brief Control register 2.
* \code
* Read/write
* Default value: 0x00
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-cleared upon completation.
* 6:2 Reserved.
* 1 HEATHER: 0: heater enable; 1: heater disable.
* 0 ONE_SHOT: 0: waiting for start of conversion; 1: start for a new dataset. Self-cleared upon completation.
* \endcode
*/
#define HTS221_CTRL_REG2      (uint8_t)0x21

#define HTS221_BOOT_BIT        HTS221_BIT(7)
#define HTS221_HEATHER_BIT     HTS221_BIT(1)
#define HTS221_ONESHOT_BIT     HTS221_BIT(0)

#define HTS221_BOOT_MASK      (uint8_t)0x80
#define HTS221_HEATHER_MASK   (uint8_t)0x02
#define HTS221_ONE_SHOT_MASK  (uint8_t)0x01

/**
* @brief Control register 3.
* \code
* Read/write
* Default value: 0x00
* 7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull; 1: open drain.
* 5:3 Reserved.
* 2 DRDY: interrupt config. 0: disable, 1: enable.
* \endcode
*/
#define HTS221_CTRL_REG3      (uint8_t)0x22

#define HTS221_DRDY_H_L_BIT    HTS221_BIT(7)
#define HTS221_PP_OD_BIT       HTS221_BIT(6)
#define HTS221_DRDY_BIT        HTS221_BIT(2)

#define HTS221_DRDY_H_L_MASK  (uint8_t)0x80
#define HTS221_PP_OD_MASK     (uint8_t)0x40
#define HTS221_DRDY_MASK      (uint8_t)0x04

/**
* @brief  Status register.
* \code
* Read
* Default value: 0x00
* 7:2 Reserved.
* 1 H_DA: Humidity data available. 0: new data for humidity is not yet available; 1: new data for humidity is available.
* 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* \endcode
*/
#define HTS221_STATUS_REG    (uint8_t)0x27

#define HTS221_H_DA_BIT       HTS221_BIT(1)
#define HTS221_T_DA_BIT       HTS221_BIT(0)

#define HTS221_HDA_MASK      (uint8_t)0x02
#define HTS221_TDA_MASK      (uint8_t)0x01

/**
* @brief  Humidity data (LSB).
* \code
* Read
* Default value: 0x00.
* HOUT7 - HOUT0: Humidity data LSB (2's complement).
* \endcode
*/
#define HTS221_HR_OUT_L_REG        (uint8_t)0x28

/**
* @brief  Humidity data (MSB).
* \code
* Read
* Default value: 0x00.
* HOUT15 - HOUT8: Humidity data MSB (2's complement).
* \endcode
*/
#define HTS221_HR_OUT_H_REG        (uint8_t)0x29


/**
* @brief  Temperature data (LSB).
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* \endcode
*/
#define HTS221_TEMP_OUT_L_REG         (uint8_t)0x2A

/**
* @brief  Temperature data (MSB).
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* \endcode
*/
#define HTS221_TEMP_OUT_H_REG         (uint8_t)0x2B

/**
* @brief  Calibration registers.
* \code
* Read
* \endcode
*/
#define HTS221_H0_RH_X2        (uint8_t)0x30
#define HTS221_H1_RH_X2        (uint8_t)0x31
#define HTS221_T0_DEGC_X8      (uint8_t)0x32
#define HTS221_T1_DEGC_X8      (uint8_t)0x33
#define HTS221_T0_T1_DEGC_H2   (uint8_t)0x35
#define HTS221_H0_T0_OUT_L     (uint8_t)0x36
#define HTS221_H0_T0_OUT_H     (uint8_t)0x37
#define HTS221_H1_T0_OUT_L     (uint8_t)0x3A
#define HTS221_H1_T0_OUT_H     (uint8_t)0x3B
#define HTS221_T0_OUT_L        (uint8_t)0x3C
#define HTS221_T0_OUT_H        (uint8_t)0x3D
#define HTS221_T1_OUT_L        (uint8_t)0x3E
#define HTS221_T1_OUT_H        (uint8_t)0x3F


/**
* @}
*/


/**
* @}
*/


/* Exported Functions -------------------------------------------------------------*/
/** @defgroup HTS221_Exported_Functions
* @{
*/

HTS221_Error_et HTS221_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
HTS221_Error_et HTS221_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

HTS221_Error_et HTS221_Get_DriverVersion(HTS221_DriverVersion_st* version);
HTS221_Error_et HTS221_Get_DeviceID(void *handle, uint8_t* deviceid);

HTS221_Error_et HTS221_Set_InitConfig(void *handle, HTS221_Init_st* pxInit);
HTS221_Error_et HTS221_Get_InitConfig(void *handle, HTS221_Init_st* pxInit);
HTS221_Error_et HTS221_DeInit(void *handle);
HTS221_Error_et HTS221_IsMeasurementCompleted(void *handle, HTS221_BitStatus_et* Is_Measurement_Completed);

HTS221_Error_et HTS221_Get_Measurement(void *handle, uint16_t* humidity, int16_t* temperature);
HTS221_Error_et HTS221_Get_RawMeasurement(void *handle, int16_t* humidity, int16_t* temperature);
HTS221_Error_et HTS221_Get_Humidity(void *handle, uint16_t* value);
HTS221_Error_et HTS221_Get_HumidityRaw(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_TemperatureRaw(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_Temperature(void *handle, int16_t* value);
HTS221_Error_et HTS221_Get_DataStatus(void *handle, HTS221_BitStatus_et* humidity, HTS221_BitStatus_et* temperature);
HTS221_Error_et HTS221_Activate(void *handle);
HTS221_Error_et HTS221_DeActivate(void *handle);

HTS221_Error_et HTS221_Set_AvgHT(void *handle, HTS221_Avgh_et avgh, HTS221_Avgt_et avgt);
HTS221_Error_et HTS221_Set_AvgH(void *handle, HTS221_Avgh_et avgh);
HTS221_Error_et HTS221_Set_AvgT(void *handle, HTS221_Avgt_et avgt);
HTS221_Error_et HTS221_Get_AvgHT(void *handle, HTS221_Avgh_et* avgh, HTS221_Avgt_et* avgt);
HTS221_Error_et HTS221_Set_BduMode(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_BduMode(void *handle, HTS221_State_et* status);
HTS221_Error_et HTS221_Set_PowerDownMode(void *handle, HTS221_BitStatus_et status);
HTS221_Error_et HTS221_Get_PowerDownMode(void *handle, HTS221_BitStatus_et* status);
HTS221_Error_et HTS221_Set_Odr(void *handle, HTS221_Odr_et odr);
HTS221_Error_et HTS221_Get_Odr(void *handle, HTS221_Odr_et* odr);
HTS221_Error_et HTS221_MemoryBoot(void *handle);
HTS221_Error_et HTS221_Set_HeaterState(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_HeaterState(void *handle, HTS221_State_et* status);
HTS221_Error_et HTS221_StartOneShotMeasurement(void *handle);
HTS221_Error_et HTS221_Set_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et status);
HTS221_Error_et HTS221_Get_IrqActiveLevel(void *handle, HTS221_DrdyLevel_et* status);
HTS221_Error_et HTS221_Set_IrqOutputType(void *handle, HTS221_OutputType_et value);
HTS221_Error_et HTS221_Get_IrqOutputType(void *handle, HTS221_OutputType_et* value);
HTS221_Error_et HTS221_Set_IrqEnable(void *handle, HTS221_State_et status);
HTS221_Error_et HTS221_Get_IrqEnable(void *handle, HTS221_State_et* status);

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* __HTS221_DRIVER__H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
