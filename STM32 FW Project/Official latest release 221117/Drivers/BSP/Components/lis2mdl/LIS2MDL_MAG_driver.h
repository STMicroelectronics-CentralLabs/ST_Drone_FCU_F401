/**
 ******************************************************************************
 * @file    LIS2MDL_MAG_driver.h
 * @author  MEMS Application Team
 * @version v1.0
 * @date    13-October-2016
 * @brief   LIS2MDL Magnetometer header driver file
 ******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2MDL_MAG_DRIVER__H
#define __LIS2MDL_MAG_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union{
	i16_t i16bit[3];
	u8_t u8bit[6];
} Type3Axis16bit_U;

typedef union{
	i16_t i16bit;
	u8_t u8bit[2];
} Type1Axis16bit_U;

typedef union{
	i32_t i32bit;
	u8_t u8bit[4];
} Type1Axis32bit_U;

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00
} status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LIS2MDL_MAG_I2C_ADDRESS         0x3C

/************** Who am I  *******************/

#define LIS2MDL_MAG_WHO_AM_I_VALUE         0x40

/************** Device Register  *******************/
#define LIS2MDL_MAG_OFFSET_X_REG_L  	0X45
#define LIS2MDL_MAG_OFFSET_X_REG_H  	0X46
#define LIS2MDL_MAG_OFFSET_Y_REG_L  	0X47
#define LIS2MDL_MAG_OFFSET_Y_REG_H  	0X48
#define LIS2MDL_MAG_OFFSET_Z_REG_L  	0X49
#define LIS2MDL_MAG_OFFSET_Z_REG_H  	0X4A
#define LIS2MDL_MAG_WHO_AM_I  	0X4F
#define LIS2MDL_MAG_CFG_REG_A  	0X60
#define LIS2MDL_MAG_CFG_REG_B  	0X61
#define LIS2MDL_MAG_CFG_REG_C  	0X62
#define LIS2MDL_MAG_INT_CRTL_REG  	0X63
#define LIS2MDL_MAG_INT_SOURCE_REG  	0X64
#define LIS2MDL_MAG_INT_THS_L_REG  	0X65
#define LIS2MDL_MAG_INT_THS_H_REG  	0X66
#define LIS2MDL_MAG_STATUS_REG  	0X67
#define LIS2MDL_MAG_OUTX_L_REG  	0X68
#define LIS2MDL_MAG_OUTX_H_REG  	0X69
#define LIS2MDL_MAG_OUTY_L_REG  	0X6A
#define LIS2MDL_MAG_OUTY_H_REG  	0X6B
#define LIS2MDL_MAG_OUTZ_L_REG  	0X6C
#define LIS2MDL_MAG_OUTZ_H_REG  	0X6D
#define LIS2MDL_MAG_TEMP_OUT_L_REG  	0X6E
#define LIS2MDL_MAG_TEMP_OUT_H_REG  	0X6F

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LIS2MDL_MAG_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LIS2MDL_MAG_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X4F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LIS2MDL_MAG_WHO_AM_I_BIT_MASK  	0xFF
#define  	LIS2MDL_MAG_WHO_AM_I_BIT_POSITION  	0
status_t LIS2MDL_MAG_R_WhoAmI_Bits(void *handle, u8_t *value);

/************** Configuration Functions  *******************/

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: MD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_MD_CONTINUOUS 		 =0x00,
  	LIS2MDL_MAG_MD_SINGLE_MODE 		 =0x01,
  	//LIS2MDL_MAG_MD_IDLE 		 =0x02,
  	LIS2MDL_MAG_MD_IDLE 		 =0x03
} LIS2MDL_MAG_MD_t;

#define  	LIS2MDL_MAG_MD_MASK  	0x03
status_t  LIS2MDL_MAG_W_Operating_Mode(void *handle, LIS2MDL_MAG_MD_t newValue);
status_t LIS2MDL_MAG_R_Operating_Mode(void *handle, LIS2MDL_MAG_MD_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_ODR_10_Hz 		 =0x00,
  	LIS2MDL_MAG_ODR_20_Hz 		 =0x04,
  	LIS2MDL_MAG_ODR_50_Hz 		 =0x08,
  	LIS2MDL_MAG_ODR_100_Hz 		 =0x0C,
} LIS2MDL_MAG_ODR_t;

#define  	LIS2MDL_MAG_ODR_MASK  	0x0C
status_t  LIS2MDL_MAG_W_DataRate(void *handle, LIS2MDL_MAG_ODR_t newValue);
status_t LIS2MDL_MAG_R_DataRate(void *handle, LIS2MDL_MAG_ODR_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: LP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_LP_HIGH_RESOLUTION 		 =0x00,
  	LIS2MDL_MAG_LP_LOW_POWER 		 =0x10,
} LIS2MDL_MAG_LP_t;

#define  	LIS2MDL_MAG_LP_MASK  	0x10
status_t  LIS2MDL_MAG_W_PowerMode(void *handle, LIS2MDL_MAG_LP_t newValue);
status_t LIS2MDL_MAG_R_PowerMode(void *handle, LIS2MDL_MAG_LP_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: SOFT_RST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_SOFT_RST_IDLE 		 =0x00,
  	LIS2MDL_MAG_SOFT_RST_ACT 		 =0x20,
} LIS2MDL_MAG_SOFT_RST_t;

#define  	LIS2MDL_MAG_SOFT_RST_MASK  	0x20
status_t  LIS2MDL_MAG_W_SoftwareReset(void *handle, LIS2MDL_MAG_SOFT_RST_t newValue);
status_t LIS2MDL_MAG_R_SoftwareReset(void *handle, LIS2MDL_MAG_SOFT_RST_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: REBOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_REBOOT_IDLE 		 =0x00,
  	LIS2MDL_MAG_REBOOT_ACT 		 =0x40,
} LIS2MDL_MAG_REBOOT_t;

#define  	LIS2MDL_MAG_REBOOT_MASK  	0x40
status_t  LIS2MDL_MAG_W_Reboot(void *handle, LIS2MDL_MAG_REBOOT_t newValue);
status_t LIS2MDL_MAG_R_Reboot(void *handle, LIS2MDL_MAG_REBOOT_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: COMP_TEMP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_COMP_TEMP_DISABLE 		 =0x00,
  	LIS2MDL_MAG_COMP_TEMP_ENABLE 		 =0x80,
} LIS2MDL_MAG_COMP_TEMP_t;

#define  	LIS2MDL_MAG_COMP_TEMP_MASK  	0x80
status_t  LIS2MDL_MAG_W_TemperatureCompensation(void *handle, LIS2MDL_MAG_COMP_TEMP_t newValue);
status_t LIS2MDL_MAG_R_TemperatureCompensation(void *handle, LIS2MDL_MAG_COMP_TEMP_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: LPF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_LPF_DISABLE 		 =0x00,
  	LIS2MDL_MAG_LPF_ENABLE 		 =0x01,
} LIS2MDL_MAG_LPF_t;

#define  	LIS2MDL_MAG_LPF_MASK  	0x01
status_t  LIS2MDL_MAG_W_LowPassFilter(void *handle, LIS2MDL_MAG_LPF_t newValue);
status_t LIS2MDL_MAG_R_LowPassFilter(void *handle, LIS2MDL_MAG_LPF_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: OFF_CANC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_OFF_CANC_DISABLED 		 =0x00,
  	LIS2MDL_MAG_OFF_CANC_ENABLED 		 =0x02,
} LIS2MDL_MAG_OFF_CANC_t;

#define  	LIS2MDL_MAG_OFF_CANC_MASK  	0x02
status_t  LIS2MDL_MAG_W_OffsetCancellation(void *handle, LIS2MDL_MAG_OFF_CANC_t newValue);
status_t LIS2MDL_MAG_R_OffsetCancellation(void *handle, LIS2MDL_MAG_OFF_CANC_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: SET_FREQ
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_SET_FREQ_EVERY_63_ODR 		 =0x00,
  	LIS2MDL_MAG_SET_FREQ_ONLY_AT_POWER_ON 		 =0x04,
} LIS2MDL_MAG_SET_FREQ_t;

#define  	LIS2MDL_MAG_SET_FREQ_MASK  	0x04
status_t  LIS2MDL_MAG_W_PulseFrequancy(void *handle, LIS2MDL_MAG_SET_FREQ_t newValue);
status_t LIS2MDL_MAG_R_PulseFrequancy(void *handle, LIS2MDL_MAG_SET_FREQ_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: INT_ON_DATAOFF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_INT_ON_DATAOFF_HI_CORRECTION 		 =0x00,
  	LIS2MDL_MAG_INT_ON_DATAOFF_WITHOUT_HI_CORRECTION 		 =0x08,
} LIS2MDL_MAG_INT_ON_DATAOFF_t;

#define  	LIS2MDL_MAG_INT_ON_DATAOFF_MASK  	0x08
status_t  LIS2MDL_MAG_W_InterruptThresholdMode(void *handle, LIS2MDL_MAG_INT_ON_DATAOFF_t newValue);
status_t LIS2MDL_MAG_R_InterruptThresholdMode(void *handle, LIS2MDL_MAG_INT_ON_DATAOFF_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: OFF_CANC_ONE_SHOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_OFF_CANC_ONE_SHOT_DISABLE 		 =0x00,
  	LIS2MDL_MAG_OFF_CANC_ONE_SHOT_ENABLE 		 =0x10,
} LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t;

#define  	LIS2MDL_MAG_OFF_CANC_ONE_SHOT_MASK  	0x10
status_t  LIS2MDL_MAG_W_OffsetCancellationInSingleMode(void *handle, LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t newValue);
status_t LIS2MDL_MAG_R_OffsetCancellationInSingleMode(void *handle, LIS2MDL_MAG_OFF_CANC_ONE_SHOT_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: INT_MAG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_INT_MAG_DRDY 		 =0x00,
  	LIS2MDL_MAG_INT_MAG_DIGITAL 		 =0x01,
} LIS2MDL_MAG_INT_MAG_t;

#define  	LIS2MDL_MAG_INT_MAG_MASK  	0x01
status_t  LIS2MDL_MAG_W_PinMode(void *handle, LIS2MDL_MAG_INT_MAG_t newValue);
status_t LIS2MDL_MAG_R_PinMode(void *handle, LIS2MDL_MAG_INT_MAG_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: SELF_TEST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_SELF_TEST_DISABLE 		 =0x00,
  	LIS2MDL_MAG_SELF_TEST_ENABLE 		 =0x02,
} LIS2MDL_MAG_SELF_TEST_t;

#define  	LIS2MDL_MAG_SELF_TEST_MASK  	0x02
status_t  LIS2MDL_MAG_W_SelfTest(void *handle, LIS2MDL_MAG_SELF_TEST_t newValue);
status_t LIS2MDL_MAG_R_SelfTest(void *handle, LIS2MDL_MAG_SELF_TEST_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_BLE_L_ENDIAN 		 =0x00,
  	LIS2MDL_MAG_BLE_B_ENDIAN 		 =0x08,
} LIS2MDL_MAG_BLE_t;

#define  	LIS2MDL_MAG_BLE_MASK  	0x08
status_t  LIS2MDL_MAG_W_OutputsOrder(void *handle, LIS2MDL_MAG_BLE_t newValue);
status_t LIS2MDL_MAG_R_OutputsOrder(void *handle, LIS2MDL_MAG_BLE_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_BDU_DISABLE 		 =0x00,
  	LIS2MDL_MAG_BDU_ENABLE 		 =0x10,
} LIS2MDL_MAG_BDU_t;

#define  	LIS2MDL_MAG_BDU_MASK  	0x10
status_t  LIS2MDL_MAG_W_BlockDataUpdate(void *handle, LIS2MDL_MAG_BDU_t newValue);
status_t LIS2MDL_MAG_R_BlockDataUpdate(void *handle, LIS2MDL_MAG_BDU_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: I2C_DIS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_I2C_DIS_ENABLE 		 =0x00,
  	LIS2MDL_MAG_I2C_DIS_DISABLE 		 =0x20,
} LIS2MDL_MAG_I2C_DIS_t;

#define  	LIS2MDL_MAG_I2C_DIS_MASK  	0x20
status_t  LIS2MDL_MAG_W_I2C_InterfaceSatus(void *handle, LIS2MDL_MAG_I2C_DIS_t newValue);
status_t LIS2MDL_MAG_R_I2C_InterfaceSatus(void *handle, LIS2MDL_MAG_I2C_DIS_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: INT_MAG_PIN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_INT_MAG_PIN_INTERNAL_ONLY 		 =0x00,
  	LIS2MDL_MAG_INT_MAG_PIN_ROUTED_ON_PIN 		 =0x40,
} LIS2MDL_MAG_INT_MAG_PIN_t;

#define  	LIS2MDL_MAG_INT_MAG_PIN_MASK  	0x40
status_t  LIS2MDL_MAG_W_InterruptGenerator(void *handle, LIS2MDL_MAG_INT_MAG_PIN_t newValue);
status_t LIS2MDL_MAG_R_InterruptGenerator(void *handle, LIS2MDL_MAG_INT_MAG_PIN_t *value);

/************** Interrupt Configuration Functions  *******************/

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: IEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_IEN_DISABLE 		 =0x00,
  	LIS2MDL_MAG_IEN_ENABLE 		 =0x01,
} LIS2MDL_MAG_IEN_t;

#define  	LIS2MDL_MAG_IEN_MASK  	0x01
status_t  LIS2MDL_MAG_W_InterruptGeneration(void *handle, LIS2MDL_MAG_IEN_t newValue);
status_t LIS2MDL_MAG_R_InterruptGeneration(void *handle, LIS2MDL_MAG_IEN_t *value);

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: IEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_IEL_PULSED 		 =0x00,
  	LIS2MDL_MAG_IEL_LATCHED 		 =0x02,
} LIS2MDL_MAG_IEL_t;

#define  	LIS2MDL_MAG_IEL_MASK  	0x02
status_t  LIS2MDL_MAG_W_InterruptMode(void *handle, LIS2MDL_MAG_IEL_t newValue);
status_t LIS2MDL_MAG_R_InterruptMode(void *handle, LIS2MDL_MAG_IEL_t *value);

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: IEA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_IEA_LOW 		 =0x00,
  	LIS2MDL_MAG_IEA_HIGH 		 =0x04,
} LIS2MDL_MAG_IEA_t;

#define  	LIS2MDL_MAG_IEA_MASK  	0x04
status_t  LIS2MDL_MAG_W_InterruptPolarity(void *handle, LIS2MDL_MAG_IEA_t newValue);
status_t LIS2MDL_MAG_R_InterruptPolarity(void *handle, LIS2MDL_MAG_IEA_t *value);

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: ZIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_ZIEN_DISABLE 		 =0x00,
  	LIS2MDL_MAG_ZIEN_ENABLE 		 =0x20,
} LIS2MDL_MAG_ZIEN_t;

#define  	LIS2MDL_MAG_ZIEN_MASK  	0x20
status_t  LIS2MDL_MAG_W_InterruptOn_Zaxis(void *handle, LIS2MDL_MAG_ZIEN_t newValue);
status_t LIS2MDL_MAG_R_InterruptOn_Zaxis(void *handle, LIS2MDL_MAG_ZIEN_t *value);

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: YIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_YIEN_DISABLE 		 =0x00,
  	LIS2MDL_MAG_YIEN_ENABLE 		 =0x40,
} LIS2MDL_MAG_YIEN_t;

#define  	LIS2MDL_MAG_YIEN_MASK  	0x40
status_t  LIS2MDL_MAG_W_InterruptOn_Yaxis(void *handle, LIS2MDL_MAG_YIEN_t newValue);
status_t LIS2MDL_MAG_R_InterruptOn_Yaxis(void *handle, LIS2MDL_MAG_YIEN_t *value);

/*******************************************************************************
* Register      : INT_CRTL_REG
* Address       : 0X63
* Bit Group Name: XIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_XIEN_DISABLE 		 =0x00,
  	LIS2MDL_MAG_XIEN_ENABLE 		 =0x80,
} LIS2MDL_MAG_XIEN_t;

#define  	LIS2MDL_MAG_XIEN_MASK  	0x80
status_t  LIS2MDL_MAG_W_InterruptOn_Xaxis(void *handle, LIS2MDL_MAG_XIEN_t newValue);
status_t LIS2MDL_MAG_R_InterruptOn_Xaxis(void *handle, LIS2MDL_MAG_XIEN_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: INT
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2MDL_MAG_INT_NONE = 0x00,
  	LIS2MDL_MAG_INT_OCCURRED = 0x01,
  	LIS2MDL_MAG_INT_MROI = 0x02,
  	LIS2MDL_MAG_INT_OCCURRED_NEG_Z = 0x04,
  	LIS2MDL_MAG_INT_OCCURRED_NEG_Y = 0x08,
  	LIS2MDL_MAG_INT_OCCURRED_NEG_X = 0x10,
  	LIS2MDL_MAG_INT_OCCURRED_POS_Z = 0x20,
  	LIS2MDL_MAG_INT_OCCURRED_POS_Y = 0x40,
  	LIS2MDL_MAG_INT_OCCURRED_POS_X = 0x80,
} LIS2MDL_MAG_INT_t;

#define  	LIS2MDL_MAG_INT_MASK  	0xFF
status_t LIS2MDL_MAG_R_InterruptSources(void *handle, LIS2MDL_MAG_INT_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : InterruptThreshold
* Permission    : RW
*******************************************************************************/
status_t LIS2MDL_MAG_Set_InterruptThreshold(void *handle, u8_t *buff);
status_t LIS2MDL_MAG_Get_InterruptThreshold(void *handle, u8_t *buff);

/************** Output Functions  *******************/

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: STATUS_BIT
* Permission    : RO
*******************************************************************************/
typedef enum {
  LIS2MDL_MAG_STATUST_NONE		 = 0x00,
  LIS2MDL_MAG_STATUS_NEW_DATA_AVAILABLE_ON_X = 0x01,
  LIS2MDL_MAG_STATUS_NEW_DATA_AVAILABLE_ON_Y = 0x02,
  LIS2MDL_MAG_STATUS_NEW_DATA_AVAILABLE_ON_Z = 0x04,
  LIS2MDL_MAG_STATUS_NEW_DATA_AVAILABLE = 0x08,
  LIS2MDL_MAG_STATUS_DATA_OVERRUN_ON_X = 0x10,
  LIS2MDL_MAG_STATUS_DATA_OVERRUN_ON_Y = 0x20,
  LIS2MDL_MAG_STATUS_DATA_OVERRUN_ON_Z = 0x40,
  LIS2MDL_MAG_STATUS_DATA_OVERRUN =  0x80,
} LIS2MDL_MAG_STATUS_t;

#define  	LIS2MDL_MAG_STATUS_BIT_MASK  	0xFF
#define  	LIS2MDL_MAG_STATUS_BIT_POSITION  	0
status_t LIS2MDL_MAG_R_STATUS_bits(void *handle, LIS2MDL_MAG_STATUS_t *value);


/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : MagneticOutputs
* Permission    : RO
*******************************************************************************/
status_t LIS2MDL_MAG_Get_MagneticOutputs(void *handle, u8_t *buff);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Temperature
* Permission    : RO
*******************************************************************************/
status_t LIS2MDL_MAG_Get_Temperature(void *handle, u8_t *buff);

/************** Hard-iron Values  *******************/

/*******************************************************************************
  * Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : HI_Offset
* Permission    : RW
*******************************************************************************/
status_t LIS2MDL_MAG_Set_HI_Offsets(void *handle, u8_t *buff);
status_t LIS2MDL_MAG_Get_HI_Offsets(void *handle, u8_t *buff);

/************** Utility  *******************/

/*******************************************************************************
  * Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LIS2MDL_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension);


#ifdef __cplusplus
  }
#endif

#endif
