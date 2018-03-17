/**
 ******************************************************************************
 * @file    LSM303AGR_MAG_driver.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    25-February-2016
 * @brief   LSM303AGR Magnetometer header driver file
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
#ifndef __LSM303AGR_MAG_DRIVER__H
#define __LSM303AGR_MAG_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

typedef union
{
  i16_t i16bit[3];
  u8_t u8bit[6];
} Type3Axis16bit_U;

typedef union
{
  i16_t i16bit;
  u8_t u8bit[2];
} Type1Axis16bit_U;

typedef union
{
  i32_t i32bit;
  u8_t u8bit[4];
} Type1Axis32bit_U;

typedef enum
{
  MEMS_SUCCESS = 0x01,
  MEMS_ERROR   = 0x00
} status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LSM303AGR_MAG_I2C_ADDRESS         0x3C

/************** Who am I  *******************/

#define LSM303AGR_MAG_WHO_AM_I         0x40

/************** Device Register  *******************/
#define LSM303AGR_MAG_OFFSET_X_REG_L    0X45
#define LSM303AGR_MAG_OFFSET_X_REG_H    0X46
#define LSM303AGR_MAG_OFFSET_Y_REG_L    0X47
#define LSM303AGR_MAG_OFFSET_Y_REG_H    0X48
#define LSM303AGR_MAG_OFFSET_Z_REG_L    0X49
#define LSM303AGR_MAG_OFFSET_Z_REG_H    0X4A
#define LSM303AGR_MAG_WHO_AM_I_REG    0X4F
#define LSM303AGR_MAG_CFG_REG_A   0X60
#define LSM303AGR_MAG_CFG_REG_B   0X61
#define LSM303AGR_MAG_CFG_REG_C   0X62
#define LSM303AGR_MAG_INT_CTRL_REG    0X63
#define LSM303AGR_MAG_INT_SOURCE_REG    0X64
#define LSM303AGR_MAG_INT_THS_L_REG   0X65
#define LSM303AGR_MAG_INT_THS_H_REG   0X66
#define LSM303AGR_MAG_STATUS_REG    0X67
#define LSM303AGR_MAG_OUTX_L_REG    0X68
#define LSM303AGR_MAG_OUTX_H_REG    0X69
#define LSM303AGR_MAG_OUTY_L_REG    0X6A
#define LSM303AGR_MAG_OUTY_H_REG    0X6B
#define LSM303AGR_MAG_OUTZ_L_REG    0X6C
#define LSM303AGR_MAG_OUTZ_H_REG    0X6D

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM303AGR_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM303AGR_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I_REG
* Address       : 0X4F
* Bit Group Name: WHO_AM_I
* Permission    : RO
*******************************************************************************/
#define   LSM303AGR_MAG_WHO_AM_I_MASK   0xFF
#define   LSM303AGR_MAG_WHO_AM_I_POSITION   0
status_t LSM303AGR_MAG_R_WHO_AM_I(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_BDU_DISABLED     = 0x00,
  LSM303AGR_MAG_BDU_ENABLED      = 0x10,
} LSM303AGR_MAG_BDU_t;

#define   LSM303AGR_MAG_BDU_MASK    0x10
status_t  LSM303AGR_MAG_W_BDU(void *handle, LSM303AGR_MAG_BDU_t newValue);
status_t LSM303AGR_MAG_R_BDU(void *handle, LSM303AGR_MAG_BDU_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: MD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_MD_CONTINUOS_MODE      = 0x00,
  LSM303AGR_MAG_MD_SINGLE_MODE     = 0x01,
  LSM303AGR_MAG_MD_IDLE1_MODE      = 0x02,
  LSM303AGR_MAG_MD_IDLE2_MODE      = 0x03,
} LSM303AGR_MAG_MD_t;

#define   LSM303AGR_MAG_MD_MASK   0x03
status_t  LSM303AGR_MAG_W_MD(void *handle, LSM303AGR_MAG_MD_t newValue);
status_t LSM303AGR_MAG_R_MD(void *handle, LSM303AGR_MAG_MD_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Magnetic
* Permission    : ro
*******************************************************************************/
status_t LSM303AGR_MAG_Get_Raw_Magnetic(void *handle, u8_t *buff);
status_t LSM303AGR_MAG_Get_Magnetic(void *handle, int *buff);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ODR_10Hz     = 0x00,
  LSM303AGR_MAG_ODR_20Hz     = 0x04,
  LSM303AGR_MAG_ODR_50Hz     = 0x08,
  LSM303AGR_MAG_ODR_100Hz      = 0x0C,
} LSM303AGR_MAG_ODR_t;

#define   LSM303AGR_MAG_ODR_MASK    0x0C
status_t  LSM303AGR_MAG_W_ODR(void *handle, LSM303AGR_MAG_ODR_t newValue);
status_t LSM303AGR_MAG_R_ODR(void *handle, LSM303AGR_MAG_ODR_t *value);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : OFFSET_X_REG_L
* Address       : 0X45
* Bit Group Name: OFF_X_L
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_X_L_MASK    0xFF
#define   LSM303AGR_MAG_OFF_X_L_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_X_L(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_X_L(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFFSET_X_REG_H
* Address       : 0X46
* Bit Group Name: OFF_X_H
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_X_H_MASK    0xFF
#define   LSM303AGR_MAG_OFF_X_H_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_X_H(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_X_H(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFFSET_Y_REG_L
* Address       : 0X47
* Bit Group Name: OFF_Y_L
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_Y_L_MASK    0xFF
#define   LSM303AGR_MAG_OFF_Y_L_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_Y_L(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_Y_L(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFFSET_Y_REG_H
* Address       : 0X48
* Bit Group Name: OFF_Y_H
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_Y_H_MASK    0xFF
#define   LSM303AGR_MAG_OFF_Y_H_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_Y_H(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_Y_H(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFFSET_Z_REG_L
* Address       : 0X49
* Bit Group Name: OFF_Z_L
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_Z_L_MASK    0xFF
#define   LSM303AGR_MAG_OFF_Z_L_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_Z_L(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_Z_L(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFFSET_Z_REG_H
* Address       : 0X4A
* Bit Group Name: OFF_Z_H
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_MAG_OFF_Z_H_MASK    0xFF
#define   LSM303AGR_MAG_OFF_Z_H_POSITION    0
status_t  LSM303AGR_MAG_W_OFF_Z_H(void *handle, u8_t newValue);
status_t LSM303AGR_MAG_R_OFF_Z_H(void *handle, u8_t *value);

/*******************************************************************************
 * Set/Get the Magnetic offsets
*******************************************************************************/
status_t LSM303AGR_MAG_Get_MagOff(void *handle, u16_t *magx_off, u16_t *magy_off, u16_t *magz_off);
status_t LSM303AGR_MAG_Set_MagOff(void *handle, u16_t magx_off, u16_t magy_off, u16_t magz_off);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: LP
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_HR_MODE      = 0x00,
  LSM303AGR_MAG_LP_MODE      = 0x10,
} LSM303AGR_MAG_LP_t;

#define   LSM303AGR_MAG_LP_MASK   0x10
status_t  LSM303AGR_MAG_W_LP(void *handle, LSM303AGR_MAG_LP_t newValue);
status_t LSM303AGR_MAG_R_LP(void *handle, LSM303AGR_MAG_LP_t *value);

/*******************************************************************************
* Register      : CFG_REG_A
* Address       : 0X60
* Bit Group Name: SOFT_RST
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_SOFT_RST_DISABLED      = 0x00,
  LSM303AGR_MAG_SOFT_RST_ENABLED     = 0x20,
} LSM303AGR_MAG_SOFT_RST_t;

#define   LSM303AGR_MAG_SOFT_RST_MASK   0x20
status_t  LSM303AGR_MAG_W_SOFT_RST(void *handle, LSM303AGR_MAG_SOFT_RST_t newValue);
status_t LSM303AGR_MAG_R_SOFT_RST(void *handle, LSM303AGR_MAG_SOFT_RST_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: LPF
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_LPF_DISABLED     = 0x00,
  LSM303AGR_MAG_LPF_ENABLED      = 0x01,
} LSM303AGR_MAG_LPF_t;

#define   LSM303AGR_MAG_LPF_MASK    0x01
status_t  LSM303AGR_MAG_W_LPF(void *handle, LSM303AGR_MAG_LPF_t newValue);
status_t LSM303AGR_MAG_R_LPF(void *handle, LSM303AGR_MAG_LPF_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: OFF_CANC
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_OFF_CANC_DISABLED      = 0x00,
  LSM303AGR_MAG_OFF_CANC_ENABLED     = 0x02,
} LSM303AGR_MAG_OFF_CANC_t;

#define   LSM303AGR_MAG_OFF_CANC_MASK   0x02
status_t  LSM303AGR_MAG_W_OFF_CANC(void *handle, LSM303AGR_MAG_OFF_CANC_t newValue);
status_t LSM303AGR_MAG_R_OFF_CANC(void *handle, LSM303AGR_MAG_OFF_CANC_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: SET_FREQ
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_SET_FREQ_CONTINUOS     = 0x00,
  LSM303AGR_MAG_SET_FREQ_SINGLE      = 0x04,
} LSM303AGR_MAG_SET_FREQ_t;

#define   LSM303AGR_MAG_SET_FREQ_MASK   0x04
status_t  LSM303AGR_MAG_W_SET_FREQ(void *handle, LSM303AGR_MAG_SET_FREQ_t newValue);
status_t LSM303AGR_MAG_R_SET_FREQ(void *handle, LSM303AGR_MAG_SET_FREQ_t *value);

/*******************************************************************************
* Register      : CFG_REG_B
* Address       : 0X61
* Bit Group Name: INT_ON_DATAOFF
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_INT_ON_DATAOFF_DISABLED      = 0x00,
  LSM303AGR_MAG_INT_ON_DATAOFF_ENABLED     = 0x08,
} LSM303AGR_MAG_INT_ON_DATAOFF_t;

#define   LSM303AGR_MAG_INT_ON_DATAOFF_MASK   0x08
status_t  LSM303AGR_MAG_W_INT_ON_DATAOFF(void *handle, LSM303AGR_MAG_INT_ON_DATAOFF_t newValue);
status_t LSM303AGR_MAG_R_INT_ON_DATAOFF(void *handle, LSM303AGR_MAG_INT_ON_DATAOFF_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: INT_MAG
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_INT_MAG_DISABLED     = 0x00,
  LSM303AGR_MAG_INT_MAG_ENABLED      = 0x01,
} LSM303AGR_MAG_INT_MAG_t;

#define   LSM303AGR_MAG_INT_MAG_MASK    0x01
status_t  LSM303AGR_MAG_W_INT_MAG(void *handle, LSM303AGR_MAG_INT_MAG_t newValue);
status_t LSM303AGR_MAG_R_INT_MAG(void *handle, LSM303AGR_MAG_INT_MAG_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ST_DISABLED      = 0x00,
  LSM303AGR_MAG_ST_ENABLED     = 0x02,
} LSM303AGR_MAG_ST_t;

#define   LSM303AGR_MAG_ST_MASK   0x02
status_t  LSM303AGR_MAG_W_ST(void *handle, LSM303AGR_MAG_ST_t newValue);
status_t LSM303AGR_MAG_R_ST(void *handle, LSM303AGR_MAG_ST_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_BLE_DISABLED     = 0x00,
  LSM303AGR_MAG_BLE_ENABLED      = 0x08,
} LSM303AGR_MAG_BLE_t;

#define   LSM303AGR_MAG_BLE_MASK    0x08
status_t  LSM303AGR_MAG_W_BLE(void *handle, LSM303AGR_MAG_BLE_t newValue);
status_t LSM303AGR_MAG_R_BLE(void *handle, LSM303AGR_MAG_BLE_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: I2C_DIS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_I2C_ENABLED      = 0x00,
  LSM303AGR_MAG_I2C_DISABLED     = 0x20,
} LSM303AGR_MAG_I2C_DIS_t;

#define   LSM303AGR_MAG_I2C_DIS_MASK    0x20
status_t LSM303AGR_MAG_W_I2C_DIS(void *handle, LSM303AGR_MAG_I2C_DIS_t newValue);
status_t LSM303AGR_MAG_R_I2C_DIS(void *handle, LSM303AGR_MAG_I2C_DIS_t *value);

/*******************************************************************************
* Register      : CFG_REG_C
* Address       : 0X62
* Bit Group Name: INT_MAG_PIN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_INT_MAG_PIN_DISABLED     = 0x00,
  LSM303AGR_MAG_INT_MAG_PIN_ENABLED      = 0x40,
} LSM303AGR_MAG_INT_MAG_PIN_t;

#define   LSM303AGR_MAG_INT_MAG_PIN_MASK    0x40
status_t  LSM303AGR_MAG_W_INT_MAG_PIN(void *handle, LSM303AGR_MAG_INT_MAG_PIN_t newValue);
status_t LSM303AGR_MAG_R_INT_MAG_PIN(void *handle, LSM303AGR_MAG_INT_MAG_PIN_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: IEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_IEN_DISABLED     = 0x00,
  LSM303AGR_MAG_IEN_ENABLED      = 0x01,
} LSM303AGR_MAG_IEN_t;

#define   LSM303AGR_MAG_IEN_MASK    0x01
status_t  LSM303AGR_MAG_W_IEN(void *handle, LSM303AGR_MAG_IEN_t newValue);
status_t LSM303AGR_MAG_R_IEN(void *handle, LSM303AGR_MAG_IEN_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: IEL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_IEL_PULSED     = 0x00,
  LSM303AGR_MAG_IEL_LATCHED      = 0x02,
} LSM303AGR_MAG_IEL_t;

#define   LSM303AGR_MAG_IEL_MASK    0x02
status_t  LSM303AGR_MAG_W_IEL(void *handle, LSM303AGR_MAG_IEL_t newValue);
status_t LSM303AGR_MAG_R_IEL(void *handle, LSM303AGR_MAG_IEL_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: IEA
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_IEA_ACTIVE_LO      = 0x00,
  LSM303AGR_MAG_IEA_ACTIVE_HI      = 0x04,
} LSM303AGR_MAG_IEA_t;

#define   LSM303AGR_MAG_IEA_MASK    0x04
status_t  LSM303AGR_MAG_W_IEA(void *handle, LSM303AGR_MAG_IEA_t newValue);
status_t LSM303AGR_MAG_R_IEA(void *handle, LSM303AGR_MAG_IEA_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: ZIEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ZIEN_DISABLED      = 0x00,
  LSM303AGR_MAG_ZIEN_ENABLED     = 0x20,
} LSM303AGR_MAG_ZIEN_t;

#define   LSM303AGR_MAG_ZIEN_MASK   0x20
status_t  LSM303AGR_MAG_W_ZIEN(void *handle, LSM303AGR_MAG_ZIEN_t newValue);
status_t LSM303AGR_MAG_R_ZIEN(void *handle, LSM303AGR_MAG_ZIEN_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: YIEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_YIEN_DISABLED      = 0x00,
  LSM303AGR_MAG_YIEN_ENABLED     = 0x40,
} LSM303AGR_MAG_YIEN_t;

#define   LSM303AGR_MAG_YIEN_MASK   0x40
status_t  LSM303AGR_MAG_W_YIEN(void *handle, LSM303AGR_MAG_YIEN_t newValue);
status_t LSM303AGR_MAG_R_YIEN(void *handle, LSM303AGR_MAG_YIEN_t *value);

/*******************************************************************************
* Register      : INT_CTRL_REG
* Address       : 0X63
* Bit Group Name: XIEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_XIEN_DISABLED      = 0x00,
  LSM303AGR_MAG_XIEN_ENABLED     = 0x80,
} LSM303AGR_MAG_XIEN_t;

#define   LSM303AGR_MAG_XIEN_MASK   0x80
status_t  LSM303AGR_MAG_W_XIEN(void *handle, LSM303AGR_MAG_XIEN_t newValue);
status_t LSM303AGR_MAG_R_XIEN(void *handle, LSM303AGR_MAG_XIEN_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: INT
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_INT_EV_OFF     = 0x00,
  LSM303AGR_MAG_INT_EV_ON      = 0x01,
} LSM303AGR_MAG_INT_t;

#define   LSM303AGR_MAG_INT_MASK    0x01
status_t LSM303AGR_MAG_R_INT(void *handle, LSM303AGR_MAG_INT_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: MROI
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_MROI_EV_OFF      = 0x00,
  LSM303AGR_MAG_MROI_EV_ON     = 0x02,
} LSM303AGR_MAG_MROI_t;

#define   LSM303AGR_MAG_MROI_MASK   0x02
status_t LSM303AGR_MAG_R_MROI(void *handle, LSM303AGR_MAG_MROI_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: N_TH_S_Z
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_N_TH_S_Z_EV_OFF      = 0x00,
  LSM303AGR_MAG_N_TH_S_Z_EV_ON     = 0x04,
} LSM303AGR_MAG_N_TH_S_Z_t;

#define   LSM303AGR_MAG_N_TH_S_Z_MASK   0x04
status_t LSM303AGR_MAG_R_N_TH_S_Z(void *handle, LSM303AGR_MAG_N_TH_S_Z_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: N_TH_S_Y
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_N_TH_S_Y_EV_OFF      = 0x00,
  LSM303AGR_MAG_N_TH_S_Y_EV_ON     = 0x08,
} LSM303AGR_MAG_N_TH_S_Y_t;

#define   LSM303AGR_MAG_N_TH_S_Y_MASK   0x08
status_t LSM303AGR_MAG_R_N_TH_S_Y(void *handle, LSM303AGR_MAG_N_TH_S_Y_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: N_TH_S_X
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_N_TH_S_X_EV_OFF      = 0x00,
  LSM303AGR_MAG_N_TH_S_X_EV_ON     = 0x10,
} LSM303AGR_MAG_N_TH_S_X_t;

#define   LSM303AGR_MAG_N_TH_S_X_MASK   0x10
status_t LSM303AGR_MAG_R_N_TH_S_X(void *handle, LSM303AGR_MAG_N_TH_S_X_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: P_TH_S_Z
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_P_TH_S_Z_EV_OFF      = 0x00,
  LSM303AGR_MAG_P_TH_S_Z_EV_ON     = 0x20,
} LSM303AGR_MAG_P_TH_S_Z_t;

#define   LSM303AGR_MAG_P_TH_S_Z_MASK   0x20
status_t LSM303AGR_MAG_R_P_TH_S_Z(void *handle, LSM303AGR_MAG_P_TH_S_Z_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: P_TH_S_Y
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_P_TH_S_Y_EV_OFF      = 0x00,
  LSM303AGR_MAG_P_TH_S_Y_EV_ON     = 0x40,
} LSM303AGR_MAG_P_TH_S_Y_t;

#define   LSM303AGR_MAG_P_TH_S_Y_MASK   0x40
status_t LSM303AGR_MAG_R_P_TH_S_Y(void *handle, LSM303AGR_MAG_P_TH_S_Y_t *value);

/*******************************************************************************
* Register      : INT_SOURCE_REG
* Address       : 0X64
* Bit Group Name: P_TH_S_X
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_P_TH_S_X_EV_OFF      = 0x00,
  LSM303AGR_MAG_P_TH_S_X_EV_ON     = 0x80,
} LSM303AGR_MAG_P_TH_S_X_t;

#define   LSM303AGR_MAG_P_TH_S_X_MASK   0x80
status_t LSM303AGR_MAG_R_P_TH_S_X(void *handle, LSM303AGR_MAG_P_TH_S_X_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_XDA_EV_OFF     = 0x00,
  LSM303AGR_MAG_XDA_EV_ON      = 0x01,
} LSM303AGR_MAG_XDA_t;

#define   LSM303AGR_MAG_XDA_MASK    0x01
status_t LSM303AGR_MAG_R_XDA(void *handle, LSM303AGR_MAG_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_YDA_EV_OFF     = 0x00,
  LSM303AGR_MAG_YDA_EV_ON      = 0x02,
} LSM303AGR_MAG_YDA_t;

#define   LSM303AGR_MAG_YDA_MASK    0x02
status_t LSM303AGR_MAG_R_YDA(void *handle, LSM303AGR_MAG_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ZDA_EV_OFF     = 0x00,
  LSM303AGR_MAG_ZDA_EV_ON      = 0x04,
} LSM303AGR_MAG_ZDA_t;

#define   LSM303AGR_MAG_ZDA_MASK    0x04
status_t LSM303AGR_MAG_R_ZDA(void *handle, LSM303AGR_MAG_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ZYXDA_EV_OFF     = 0x00,
  LSM303AGR_MAG_ZYXDA_EV_ON      = 0x08,
} LSM303AGR_MAG_ZYXDA_t;

#define   LSM303AGR_MAG_ZYXDA_MASK    0x08
status_t LSM303AGR_MAG_R_ZYXDA(void *handle, LSM303AGR_MAG_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_XOR_EV_OFF     = 0x00,
  LSM303AGR_MAG_XOR_EV_ON      = 0x10,
} LSM303AGR_MAG_XOR_t;

#define   LSM303AGR_MAG_XOR_MASK    0x10
status_t LSM303AGR_MAG_R_XOR(void *handle, LSM303AGR_MAG_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_YOR_EV_OFF     = 0x00,
  LSM303AGR_MAG_YOR_EV_ON      = 0x20,
} LSM303AGR_MAG_YOR_t;

#define   LSM303AGR_MAG_YOR_MASK    0x20
status_t LSM303AGR_MAG_R_YOR(void *handle, LSM303AGR_MAG_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ZOR_EV_OFF     = 0x00,
  LSM303AGR_MAG_ZOR_EV_ON      = 0x40,
} LSM303AGR_MAG_ZOR_t;

#define   LSM303AGR_MAG_ZOR_MASK    0x40
status_t LSM303AGR_MAG_R_ZOR(void *handle, LSM303AGR_MAG_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X67
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_MAG_ZYXOR_EV_OFF     = 0x00,
  LSM303AGR_MAG_ZYXOR_EV_ON      = 0x80,
} LSM303AGR_MAG_ZYXOR_t;

#define   LSM303AGR_MAG_ZYXOR_MASK    0x80
status_t LSM303AGR_MAG_R_ZYXOR(void *handle, LSM303AGR_MAG_ZYXOR_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : IntThreshld
* Permission    : rw
*******************************************************************************/
status_t LSM303AGR_MAG_Get_IntThreshld(void *handle, u8_t *buff);
status_t LSM303AGR_MAG_Set_IntThreshld(void *handle, u8_t *buff);

#ifdef __cplusplus
}
#endif

#endif
