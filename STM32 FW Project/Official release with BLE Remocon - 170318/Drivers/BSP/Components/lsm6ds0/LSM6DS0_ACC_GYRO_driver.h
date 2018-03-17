/**
 ******************************************************************************
 * @file    LSM6DS0_ACC_GYRO_driver.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   LSM6DS0 header driver file
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
#ifndef __LSM6DS0_ACC_GYRO_DRIVER__H
#define __LSM6DS0_ACC_GYRO_DRIVER__H

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
  MEMS_SUCCESS        =   0x01,
  MEMS_ERROR        =   0x00
} status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LSM6DS0_ACC_GYRO_I2C_ADDRESS_LOW   0xD4  // SAD[0] = 0
#define LSM6DS0_ACC_GYRO_I2C_ADDRESS_HIGH  0xD6  // SAD[0] = 1

/************** Who am I  *******************/

#define LSM6DS0_ACC_GYRO_WHO_AM_I         0x68

/************** Device Register  *******************/
#define LSM6DS0_ACC_GYRO_ACT_THS    0X04
#define LSM6DS0_ACC_GYRO_ACT_DUR    0X05
#define LSM6DS0_ACC_GYRO_INT_GEN_CFG_XL   0X06
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_X_XL   0X07
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_Y_XL   0X08
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_Z_XL   0X09
#define LSM6DS0_ACC_GYRO_INT_GEN_DUR_XL   0X0A
#define LSM6DS0_ACC_GYRO_REFERENCE_G    0X0B
#define LSM6DS0_ACC_GYRO_INT_CTRL   0X0C
#define LSM6DS0_ACC_GYRO_WHO_AM_I_REG   0X0F
#define LSM6DS0_ACC_GYRO_CTRL_REG1_G    0X10
#define LSM6DS0_ACC_GYRO_CTRL_REG2_G    0X11
#define LSM6DS0_ACC_GYRO_CTRL_REG3_G    0X12
#define LSM6DS0_ACC_GYRO_ORIENT_CFG_G   0X13
#define LSM6DS0_ACC_GYRO_INT_GEN_SRC_G    0X14
#define LSM6DS0_ACC_GYRO_OUT_TEMP_L   0X15
#define LSM6DS0_ACC_GYRO_OUT_TEMP_H   0X16
#define LSM6DS0_ACC_GYRO_OUT_X_L_G    0X18
#define LSM6DS0_ACC_GYRO_OUT_X_H_G    0X19
#define LSM6DS0_ACC_GYRO_OUT_Y_L_G    0X1A
#define LSM6DS0_ACC_GYRO_OUT_Y_H_G    0X1B
#define LSM6DS0_ACC_GYRO_OUT_Z_L_G    0X1C
#define LSM6DS0_ACC_GYRO_OUT_Z_H_G    0X1D
#define LSM6DS0_ACC_GYRO_CTRL_REG4    0X1E
#define LSM6DS0_ACC_GYRO_CTRL_REG5_XL   0X1F
#define LSM6DS0_ACC_GYRO_CTRL_REG6_XL   0X20
#define LSM6DS0_ACC_GYRO_CTRL_REG7_XL   0X21
#define LSM6DS0_ACC_GYRO_CTRL_REG8    0X22
#define LSM6DS0_ACC_GYRO_CTRL_REG9    0X23
#define LSM6DS0_ACC_GYRO_CTRL_REG10   0X24
#define LSM6DS0_ACC_GYRO_INT_GEN_SRC_XL   0X26
#define LSM6DS0_ACC_GYRO_STATUS_REG   0X27
#define LSM6DS0_ACC_GYRO_OUT_X_L_XL   0X28
#define LSM6DS0_ACC_GYRO_OUT_X_H_XL   0X29
#define LSM6DS0_ACC_GYRO_OUT_Y_L_XL   0X2A
#define LSM6DS0_ACC_GYRO_OUT_Y_H_XL   0X2B
#define LSM6DS0_ACC_GYRO_OUT_Z_L_XL   0X2C
#define LSM6DS0_ACC_GYRO_OUT_Z_H_XL   0X2D
#define LSM6DS0_ACC_GYRO_FIFO_CTRL    0X2E
#define LSM6DS0_ACC_GYRO_FIFO_SRC   0X2F
#define LSM6DS0_ACC_GYRO_INT_GEN_CFG_G    0X30
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_XH_G   0X31
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_XL_G   0X32
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_YH_G   0X33
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_YL_G   0X34
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_ZH_G   0X35
#define LSM6DS0_ACC_GYRO_INT_GEN_THS_ZL_G   0X36
#define LSM6DS0_ACC_GYRO_INT_GEN_DUR_G    0X37


/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I_REG
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_MASK    0xFF
#define   LSM6DS0_ACC_GYRO_WHO_AM_I_BIT_POSITION    0
status_t LSM6DS0_ACC_GYRO_R_WHO_AM_I_(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG1_G
* Address       : 0X10
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FS_G_245dps     = 0x00,
  LSM6DS0_ACC_GYRO_FS_G_500dps     = 0x08,
  LSM6DS0_ACC_GYRO_FS_G_1000dps      = 0x10,
  LSM6DS0_ACC_GYRO_FS_G_2000dps      = 0x18,
} LSM6DS0_ACC_GYRO_FS_G_t;

#define   LSM6DS0_ACC_GYRO_FS_G_MASK    0x18
status_t  LSM6DS0_ACC_GYRO_W_GyroFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG1_G
* Address       : 0X10
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_ODR_G_15Hz      = 0x20,
  LSM6DS0_ACC_GYRO_ODR_G_60Hz      = 0x40,
  LSM6DS0_ACC_GYRO_ODR_G_119Hz     = 0x60,
  LSM6DS0_ACC_GYRO_ODR_G_238Hz     = 0x80,
  LSM6DS0_ACC_GYRO_ODR_G_476Hz     = 0xA0,
  LSM6DS0_ACC_GYRO_ODR_G_952Hz     = 0xC0,
} LSM6DS0_ACC_GYRO_ODR_G_t;

#define   LSM6DS0_ACC_GYRO_ODR_G_MASK   0xE0
status_t  LSM6DS0_ACC_GYRO_W_GyroDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG6_XL
* Address       : 0X20
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FS_XL_2g      = 0x00,
  LSM6DS0_ACC_GYRO_FS_XL_16g     = 0x08,
  LSM6DS0_ACC_GYRO_FS_XL_4g      = 0x10,
  LSM6DS0_ACC_GYRO_FS_XL_8g      = 0x18,
} LSM6DS0_ACC_GYRO_FS_XL_t;

#define   LSM6DS0_ACC_GYRO_FS_XL_MASK   0x18
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFullScale(void *handle, LSM6DS0_ACC_GYRO_FS_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG6_XL
* Address       : 0X20
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_ODR_XL_10Hz     = 0x20,
  LSM6DS0_ACC_GYRO_ODR_XL_50Hz     = 0x40,
  LSM6DS0_ACC_GYRO_ODR_XL_119Hz      = 0x60,
  LSM6DS0_ACC_GYRO_ODR_XL_238Hz      = 0x80,
  LSM6DS0_ACC_GYRO_ODR_XL_476Hz      = 0xA0,
  LSM6DS0_ACC_GYRO_ODR_XL_952Hz      = 0xC0,
} LSM6DS0_ACC_GYRO_ODR_XL_t;

#define   LSM6DS0_ACC_GYRO_ODR_XL_MASK    0xE0
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataRate(void *handle, LSM6DS0_ACC_GYRO_ODR_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BDU_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_BDU_ENABLE      = 0x40,
} LSM6DS0_ACC_GYRO_BDU_t;

#define   LSM6DS0_ACC_GYRO_BDU_MASK   0x40
status_t  LSM6DS0_ACC_GYRO_W_BlockDataUpdate(void *handle, LSM6DS0_ACC_GYRO_BDU_t newValue);
status_t LSM6DS0_ACC_GYRO_R_BlockDataUpdate(void *handle, LSM6DS0_ACC_GYRO_BDU_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : AngularRate
* Permission    : RO
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_AngularRate(void *handle, u8_t *buff);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Acceleration
* Permission    : RO
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_Acceleration(void *handle, u8_t *buff);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : ACT_THS
* Address       : 0X04
* Bit Group Name: ACT_THS
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_ACT_THS_MASK   0x7F
#define   LSM6DS0_ACC_GYRO_ACT_THS_POSITION   0
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityThreshold(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : ACT_THS
* Address       : 0X04
* Bit Group Name: SLEEP_ON_INACT_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_POWER_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_SLEEP     = 0x80,
} LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t;

#define   LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_MASK   0x80
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_ON_INACT_EN_t *value);

/*******************************************************************************
* Register      : ACT_DUR
* Address       : 0X05
* Bit Group Name: ACT_DUR
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_ACT_DUR_MASK   0xFF
#define   LSM6DS0_ACC_GYRO_ACT_DUR_POSITION   0
status_t  LSM6DS0_ACC_GYRO_W_GyroInactivityDuration(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInactivityDuration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: XLIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XLIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_XLIE_XL_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_XLIE_XL_t;

#define   LSM6DS0_ACC_GYRO_XLIE_XL_MASK   0x01
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: XHIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XHIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_XHIE_XL_ENABLE      = 0x02,
} LSM6DS0_ACC_GYRO_XHIE_XL_t;

#define   LSM6DS0_ACC_GYRO_XHIE_XL_MASK   0x02
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: YLIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YLIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_YLIE_XL_ENABLE      = 0x04,
} LSM6DS0_ACC_GYRO_YLIE_XL_t;

#define   LSM6DS0_ACC_GYRO_YLIE_XL_MASK   0x04
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: YHIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YHIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_YHIE_XL_ENABLE      = 0x08,
} LSM6DS0_ACC_GYRO_YHIE_XL_t;

#define   LSM6DS0_ACC_GYRO_YHIE_XL_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: ZLIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZLIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_ZLIE_XL_ENABLE      = 0x10,
} LSM6DS0_ACC_GYRO_ZLIE_XL_t;

#define   LSM6DS0_ACC_GYRO_ZLIE_XL_MASK   0x10
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: ZHIE_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZHIE_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_ZHIE_XL_ENABLE      = 0x20,
} LSM6DS0_ACC_GYRO_ZHIE_XL_t;

#define   LSM6DS0_ACC_GYRO_ZHIE_XL_MASK   0x20
status_t  LSM6DS0_ACC_GYRO_W_InterruptAccelerometer_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometer_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: 6D
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_6D_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_6D_ENABLE     = 0x40,
} LSM6DS0_ACC_GYRO_6D_t;

#define   LSM6DS0_ACC_GYRO_6D_MASK    0x40
status_t  LSM6DS0_ACC_GYRO_W_Interrupt6D(void *handle, LSM6DS0_ACC_GYRO_6D_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Interrupt6D(void *handle, LSM6DS0_ACC_GYRO_6D_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_XL
* Address       : 0X06
* Bit Group Name: AOI_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_AOI_XL_OR     = 0x00,
  LSM6DS0_ACC_GYRO_AOI_XL_AND      = 0x80,
} LSM6DS0_ACC_GYRO_AOI_XL_t;

#define   LSM6DS0_ACC_GYRO_AOI_XL_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerInterruptCombination(void *handle, LSM6DS0_ACC_GYRO_AOI_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptCombination(void *handle, LSM6DS0_ACC_GYRO_AOI_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_THS_X_XL
* Address       : 0X07
* Bit Group Name: THS_XL_X
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_THS_XL_X_MASK    0xFF
#define   LSM6DS0_ACC_GYRO_THS_XL_X_POSITION    0
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisX(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisX(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_THS_Y_XL
* Address       : 0X08
* Bit Group Name: THS_XL_Y
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_THS_XL_Y_MASK    0xFF
#define   LSM6DS0_ACC_GYRO_THS_XL_Y_POSITION    0
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisY(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisY(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_THS_Z_XL
* Address       : 0X09
* Bit Group Name: THS_XL_Z
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_THS_XL_Z_MASK    0xFF
#define   LSM6DS0_ACC_GYRO_THS_XL_Z_POSITION    0
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptThresholdAxisZ(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptThresholdAxisZ(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_DUR_XL
* Address       : 0X0A
* Bit Group Name: DUR_XL
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_DUR_XL_MASK    0x7F
#define   LSM6DS0_ACC_GYRO_DUR_XL_POSITION    0
status_t  LSM6DS0_ACC_GYRO_W_XL_InterruptDuration(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_InterruptDuration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_DUR_XL
* Address       : 0X0A
* Bit Group Name: WAIT_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_WAIT_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_WAIT_XL_ENABLE      = 0x80,
} LSM6DS0_ACC_GYRO_WAIT_XL_t;

#define   LSM6DS0_ACC_GYRO_WAIT_XL_MASK   0x80
status_t  LSM6DS0_ACC_GYRO_W_XL_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_XL_t *value);

/*******************************************************************************
* Register      : REFERENCE_G
* Address       : 0X0B
* Bit Group Name: REF_G
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_REF_G_MASK   0xFF
#define   LSM6DS0_ACC_GYRO_REF_G_POSITION   0
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilterReference(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilterReference(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_DRDY_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_INT_DRDY_XL_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_INT_DRDY_XL_t;

#define   LSM6DS0_ACC_GYRO_INT_DRDY_XL_MASK   0x01
status_t  LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_XL_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_DRDY_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_INT_DRDY_G_ENABLE     = 0x02,
} LSM6DS0_ACC_GYRO_INT_DRDY_G_t;

#define   LSM6DS0_ACC_GYRO_INT_DRDY_G_MASK    0x02
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_DataReadyOnINT(void *handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT__BOOT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT__BOOT_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_INT__BOOT_ENABLE      = 0x04,
} LSM6DS0_ACC_GYRO_INT__BOOT_t;

#define   LSM6DS0_ACC_GYRO_INT__BOOT_MASK   0x04
status_t  LSM6DS0_ACC_GYRO_W_BOOT_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT__BOOT_t newValue);
status_t LSM6DS0_ACC_GYRO_R_BOOT_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT__BOOT_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_FTH
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_FTH_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_INT_FTH_ENABLE      = 0x08,
} LSM6DS0_ACC_GYRO_INT_FTH_t;

#define   LSM6DS0_ACC_GYRO_INT_FTH_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FTH_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FTH_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_OVR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_OVR_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_INT_OVR_ENABLE      = 0x10,
} LSM6DS0_ACC_GYRO_INT_OVR_t;

#define   LSM6DS0_ACC_GYRO_INT_OVR_MASK   0x10
status_t  LSM6DS0_ACC_GYRO_W_Overrun_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_OVR_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Overrun_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_OVR_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_FSS5_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_INT_FSS5_ENABLE     = 0x20,
} LSM6DS0_ACC_GYRO_INT_FSS5_t;

#define   LSM6DS0_ACC_GYRO_INT_FSS5_MASK    0x20
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Full_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FSS5_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO_Full_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_FSS5_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_IG_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_IG_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_INT_IG_XL_ENABLE      = 0x40,
} LSM6DS0_ACC_GYRO_INT_IG_XL_t;

#define   LSM6DS0_ACC_GYRO_INT_IG_XL_MASK   0x40
status_t  LSM6DS0_ACC_GYRO_W_Accelerometer_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Accelerometer_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_XL_t *value);

/*******************************************************************************
* Register      : INT_CTRL
* Address       : 0X0C
* Bit Group Name: INT_IG_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_IG_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_INT_IG_G_ENABLE     = 0x80,
} LSM6DS0_ACC_GYRO_INT_IG_G_t;

#define   LSM6DS0_ACC_GYRO_INT_IG_G_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_Gyroscope_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Gyroscope_OnINT(void *handle, LSM6DS0_ACC_GYRO_INT_IG_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG1_G
* Address       : 0X10
* Bit Group Name: BW_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BW_G_LOW      = 0x00,
  LSM6DS0_ACC_GYRO_BW_G_NORMAL     = 0x01,
  LSM6DS0_ACC_GYRO_BW_G_HIGH     = 0x02,
  LSM6DS0_ACC_GYRO_BW_G_ULTRA_HIGH     = 0x03,
} LSM6DS0_ACC_GYRO_BW_G_t;

#define   LSM6DS0_ACC_GYRO_BW_G_MASK    0x03
status_t  LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroBandwidthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG2_G
* Address       : 0X11
* Bit Group Name: OUT_SEL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_OUT_SEL_BYPASS_HPF_AND_LPF2     = 0x00,
  LSM6DS0_ACC_GYRO_OUT_SEL_BYPASS_LPF2     = 0x01,
  LSM6DS0_ACC_GYRO_OUT_SEL_USE_HPF_AND_LPF2      = 0x02,
} LSM6DS0_ACC_GYRO_OUT_SEL_t;

#define   LSM6DS0_ACC_GYRO_OUT_SEL_MASK   0x03
status_t  LSM6DS0_ACC_GYRO_W_GYRO_OutMode(void *handle, LSM6DS0_ACC_GYRO_OUT_SEL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_OutMode(void *handle, LSM6DS0_ACC_GYRO_OUT_SEL_t *value);

/*******************************************************************************
* Register      : CTRL_REG2_G
* Address       : 0X11
* Bit Group Name: INT_SEL_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INT_SEL_G_BYPASS_HPF_AND_LPF2     = 0x00,
  LSM6DS0_ACC_GYRO_INT_SEL_G_BYPASS_LPF2     = 0x04,
  LSM6DS0_ACC_GYRO_INT_SEL_G_USE_HPF_AND_LPF2      = 0x08,
} LSM6DS0_ACC_GYRO_INT_SEL_G_t;

#define   LSM6DS0_ACC_GYRO_INT_SEL_G_MASK   0x0C
status_t  LSM6DS0_ACC_GYRO_W_GYRO_OutIntMode(void *handle, LSM6DS0_ACC_GYRO_INT_SEL_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_OutIntMode(void *handle, LSM6DS0_ACC_GYRO_INT_SEL_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG3_G
* Address       : 0X12
* Bit Group Name: HPCF_G
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_HPCF_G_MASK    0x0F
#define   LSM6DS0_ACC_GYRO_HPCF_G_POSITION    0
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilterCutOffFrequency(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilterCutOffFrequency(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG3_G
* Address       : 0X12
* Bit Group Name: HP_EN_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_HP_EN_G_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_HP_EN_G_ENABLE      = 0x40,
} LSM6DS0_ACC_GYRO_HP_EN_G_t;

#define   LSM6DS0_ACC_GYRO_HP_EN_G_MASK   0x40
status_t  LSM6DS0_ACC_GYRO_W_GyroHighPassFilter(void *handle, LSM6DS0_ACC_GYRO_HP_EN_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroHighPassFilter(void *handle, LSM6DS0_ACC_GYRO_HP_EN_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG3_G
* Address       : 0X12
* Bit Group Name: LP_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_LP_G_DISABLE    = 0x00,
  LSM6DS0_ACC_GYRO_LP_G_ENABLE       = 0x80,
} LSM6DS0_ACC_GYRO_LP_G_t;

#define   LSM6DS0_ACC_GYRO_LP_G_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_GyroLowPower(void *handle, LSM6DS0_ACC_GYRO_LP_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroLowPower(void *handle, LSM6DS0_ACC_GYRO_LP_G_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: ORIENT_0
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ORIENT_0_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ORIENT_0_ENABLE     = 0x01,
} LSM6DS0_ACC_GYRO_ORIENT_0_t;

#define   LSM6DS0_ACC_GYRO_ORIENT_0_MASK    0x01
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationX(void *handle, LSM6DS0_ACC_GYRO_ORIENT_0_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationX(void *handle, LSM6DS0_ACC_GYRO_ORIENT_0_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: ORIENT_1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ORIENT_1_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ORIENT_1_ENABLE     = 0x02,
} LSM6DS0_ACC_GYRO_ORIENT_1_t;

#define   LSM6DS0_ACC_GYRO_ORIENT_1_MASK    0x02
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationY(void *handle, LSM6DS0_ACC_GYRO_ORIENT_1_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationY(void *handle, LSM6DS0_ACC_GYRO_ORIENT_1_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: ORIENT_2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ORIENT_2_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ORIENT_2_ENABLE     = 0x04,
} LSM6DS0_ACC_GYRO_ORIENT_2_t;

#define   LSM6DS0_ACC_GYRO_ORIENT_2_MASK    0x04
status_t  LSM6DS0_ACC_GYRO_W_GYRO_DirectionalUserOrientationZ(void *handle, LSM6DS0_ACC_GYRO_ORIENT_2_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_DirectionalUserOrientationZ(void *handle, LSM6DS0_ACC_GYRO_ORIENT_2_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: SIGNZ_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SIGNZ_G_POSITIVE      = 0x00,
  LSM6DS0_ACC_GYRO_SIGNZ_G_NEGATIVE      = 0x08,
} LSM6DS0_ACC_GYRO_SIGNZ_G_t;

#define   LSM6DS0_ACC_GYRO_SIGNZ_G_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignZ(void *handle, LSM6DS0_ACC_GYRO_SIGNZ_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignZ(void *handle, LSM6DS0_ACC_GYRO_SIGNZ_G_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: SIGNY_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SIGNY_G_POSITIVE      = 0x00,
  LSM6DS0_ACC_GYRO_SIGNY_G_NEGATIVE      = 0x10,
} LSM6DS0_ACC_GYRO_SIGNY_G_t;

#define   LSM6DS0_ACC_GYRO_SIGNY_G_MASK   0x10
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignY(void *handle, LSM6DS0_ACC_GYRO_SIGNY_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignY(void *handle, LSM6DS0_ACC_GYRO_SIGNY_G_t *value);

/*******************************************************************************
* Register      : ORIENT_CFG_G
* Address       : 0X13
* Bit Group Name: SIGNX_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SIGNX_G_POSITIVE      = 0x00,
  LSM6DS0_ACC_GYRO_SIGNX_G_NEGATIVE      = 0x20,
} LSM6DS0_ACC_GYRO_SIGNX_G_t;

#define   LSM6DS0_ACC_GYRO_SIGNX_G_MASK   0x20
status_t  LSM6DS0_ACC_GYRO_W_GYRO_SignX(void *handle, LSM6DS0_ACC_GYRO_SIGNX_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GYRO_SignX(void *handle, LSM6DS0_ACC_GYRO_SIGNX_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: XL_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XL_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_XL_G_UP     = 0x01,
} LSM6DS0_ACC_GYRO_XL_G_t;

#define   LSM6DS0_ACC_GYRO_XL_G_MASK    0x01
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XL_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: XH_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XH_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_XH_G_UP     = 0x02,
} LSM6DS0_ACC_GYRO_XH_G_t;

#define   LSM6DS0_ACC_GYRO_XH_G_MASK    0x02
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XH_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: YL_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YL_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_YL_G_UP     = 0x04,
} LSM6DS0_ACC_GYRO_YL_G_t;

#define   LSM6DS0_ACC_GYRO_YL_G_MASK    0x04
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YL_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: YH_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YH_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_YH_G_UP     = 0x08,
} LSM6DS0_ACC_GYRO_YH_G_t;

#define   LSM6DS0_ACC_GYRO_YH_G_MASK    0x08
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YH_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: ZL_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZL_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_ZL_G_UP     = 0x10,
} LSM6DS0_ACC_GYRO_ZL_G_t;

#define   LSM6DS0_ACC_GYRO_ZL_G_MASK    0x10
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZL_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: ZH_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZH_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_ZH_G_UP     = 0x20,
} LSM6DS0_ACC_GYRO_ZH_G_t;

#define   LSM6DS0_ACC_GYRO_ZH_G_MASK    0x20
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZH_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_G
* Address       : 0X14
* Bit Group Name: IA_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_IA_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_IA_G_UP     = 0x40,
} LSM6DS0_ACC_GYRO_IA_G_t;

#define   LSM6DS0_ACC_GYRO_IA_G_MASK    0x40
status_t LSM6DS0_ACC_GYRO_R_InterruptGyroFlag(void *handle, LSM6DS0_ACC_GYRO_IA_G_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XLDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XLDA_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_XLDA_UP     = 0x01,
} LSM6DS0_ACC_GYRO_XLDA_t;

#define   LSM6DS0_ACC_GYRO_XLDA_MASK    0x01
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_XLDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: GDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_GDA_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_GDA_UP      = 0x02,
} LSM6DS0_ACC_GYRO_GDA_t;

#define   LSM6DS0_ACC_GYRO_GDA_MASK   0x02
status_t LSM6DS0_ACC_GYRO_R_GyroDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_GDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: TDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_TDA_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_TDA_UP      = 0x04,
} LSM6DS0_ACC_GYRO_TDA_t;

#define   LSM6DS0_ACC_GYRO_TDA_MASK   0x04
status_t LSM6DS0_ACC_GYRO_R_TemperatureDataReadyFlag(void *handle, LSM6DS0_ACC_GYRO_TDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: BOOT_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BOOT_STATUS_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_BOOT_STATUS_UP      = 0x08,
} LSM6DS0_ACC_GYRO_BOOT_STATUS_t;

#define   LSM6DS0_ACC_GYRO_BOOT_STATUS_MASK   0x08
status_t LSM6DS0_ACC_GYRO_R_BootRunningFlag(void *handle, LSM6DS0_ACC_GYRO_BOOT_STATUS_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: INACT
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_INACT_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_INACT_UP      = 0x10,
} LSM6DS0_ACC_GYRO_INACT_t;

#define   LSM6DS0_ACC_GYRO_INACT_MASK   0x10
status_t LSM6DS0_ACC_GYRO_R_InactivityInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_INACT_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: IG_G
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_IG_G_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_IG_G_UP     = 0x20,
} LSM6DS0_ACC_GYRO_IG_G_t;

#define   LSM6DS0_ACC_GYRO_IG_G_MASK    0x20
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_IG_G_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: IG_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_IG_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_IG_XL_UP      = 0x40,
} LSM6DS0_ACC_GYRO_IG_XL_t;

#define   LSM6DS0_ACC_GYRO_IG_XL_MASK   0x40
status_t LSM6DS0_ACC_GYRO_R_InterruptAccelerometerFlag(void *handle, LSM6DS0_ACC_GYRO_IG_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X1E
* Bit Group Name: 4D_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_4D_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_4D_XL_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_4D_XL_t;

#define   LSM6DS0_ACC_GYRO_4D_XL_MASK   0x01
status_t  LSM6DS0_ACC_GYRO_W_Interrupt4D(void *handle, LSM6DS0_ACC_GYRO_4D_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Interrupt4D(void *handle, LSM6DS0_ACC_GYRO_4D_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X1E
* Bit Group Name: LIR_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_LIR_XL_NOT_LATCHED      = 0x00,
  LSM6DS0_ACC_GYRO_LIR_XL_LATCHED      = 0x02,
} LSM6DS0_ACC_GYRO_LIR_XL_t;

#define   LSM6DS0_ACC_GYRO_LIR_XL_MASK    0x02
status_t  LSM6DS0_ACC_GYRO_W_InterruptSignalMode(void *handle, LSM6DS0_ACC_GYRO_LIR_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptSignalMode(void *handle, LSM6DS0_ACC_GYRO_LIR_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X1E
* Bit Group Name: XEN_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XEN_G_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_XEN_G_ENABLE      = 0x08,
} LSM6DS0_ACC_GYRO_XEN_G_t;

#define   LSM6DS0_ACC_GYRO_XEN_G_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X1E
* Bit Group Name: YEN_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YEN_G_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_YEN_G_ENABLE      = 0x10,
} LSM6DS0_ACC_GYRO_YEN_G_t;

#define   LSM6DS0_ACC_GYRO_YEN_G_MASK   0x10
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X1E
* Bit Group Name: ZEN_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZEN_G_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_ZEN_G_ENABLE      = 0x20,
} LSM6DS0_ACC_GYRO_ZEN_G_t;

#define   LSM6DS0_ACC_GYRO_ZEN_G_MASK   0x20
status_t  LSM6DS0_ACC_GYRO_W_GyroAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG5_XL
* Address       : 0X1F
* Bit Group Name: XEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XEN_XL_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_XEN_XL_ENABLE     = 0x08,
} LSM6DS0_ACC_GYRO_XEN_XL_t;

#define   LSM6DS0_ACC_GYRO_XEN_XL_MASK    0x08
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisX(void *handle, LSM6DS0_ACC_GYRO_XEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5_XL
* Address       : 0X1F
* Bit Group Name: YEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YEN_XL_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_YEN_XL_ENABLE     = 0x10,
} LSM6DS0_ACC_GYRO_YEN_XL_t;

#define   LSM6DS0_ACC_GYRO_YEN_XL_MASK    0x10
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisY(void *handle, LSM6DS0_ACC_GYRO_YEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5_XL
* Address       : 0X1F
* Bit Group Name: ZEN_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZEN_XL_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE     = 0x20,
} LSM6DS0_ACC_GYRO_ZEN_XL_t;

#define   LSM6DS0_ACC_GYRO_ZEN_XL_MASK    0x20
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerAxisZ(void *handle, LSM6DS0_ACC_GYRO_ZEN_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5_XL
* Address       : 0X1F
* Bit Group Name: DEC_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_DEC_XL_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_DEC_XL_2_SAMPLE     = 0x40,
  LSM6DS0_ACC_GYRO_DEC_XL_4_SAMPLE     = 0x80,
  LSM6DS0_ACC_GYRO_DEC_XL_8_SAMPLE     = 0xC0,
} LSM6DS0_ACC_GYRO_DEC_XL_t;

#define   LSM6DS0_ACC_GYRO_DEC_XL_MASK    0xC0
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerDataDecimation(void *handle, LSM6DS0_ACC_GYRO_DEC_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerDataDecimation(void *handle, LSM6DS0_ACC_GYRO_DEC_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG6_XL
* Address       : 0X20
* Bit Group Name: BW_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BW_XL_408Hz     = 0x00,
  LSM6DS0_ACC_GYRO_BW_XL_211Hz     = 0x01,
  LSM6DS0_ACC_GYRO_BW_XL_105Hz     = 0x02,
  LSM6DS0_ACC_GYRO_BW_XL_50Hz      = 0x03,
} LSM6DS0_ACC_GYRO_BW_XL_t;

#define   LSM6DS0_ACC_GYRO_BW_XL_MASK   0x03
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth(void *handle, LSM6DS0_ACC_GYRO_BW_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFilterBandwidth(void *handle, LSM6DS0_ACC_GYRO_BW_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG6_XL
* Address       : 0X20
* Bit Group Name: BW_SCAL_ODR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BW_SCAL_ODR_WITH_ODR      = 0x00,
  LSM6DS0_ACC_GYRO_BW_SCAL_ODR_WITH_BW_XL      = 0x04,
} LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t;

#define   LSM6DS0_ACC_GYRO_BW_SCAL_ODR_MASK   0x04
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerBandWitdthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerBandWitdthSelection(void *handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_t *value);

/*******************************************************************************
* Register      : CTRL_REG7_XL
* Address       : 0X21
* Bit Group Name: HPIS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_HPIS_FILTER_BYPASS      = 0x00,
  LSM6DS0_ACC_GYRO_HPIS_FILTER_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_HPIS_t;

#define   LSM6DS0_ACC_GYRO_HPIS_MASK    0x01
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerHighPass_on_Interrupt(void *handle, LSM6DS0_ACC_GYRO_HPIS_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerHighPass_on_Interrupt(void *handle, LSM6DS0_ACC_GYRO_HPIS_t *value);

/*******************************************************************************
* Register      : CTRL_REG7_XL
* Address       : 0X21
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FDS_FILTER_BYPASS     = 0x00,
  LSM6DS0_ACC_GYRO_FDS_FILTER_ENABLE     = 0x04,
} LSM6DS0_ACC_GYRO_FDS_t;

#define   LSM6DS0_ACC_GYRO_FDS_MASK   0x04
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection(void *handle, LSM6DS0_ACC_GYRO_FDS_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerFilteredDataSelection(void *handle, LSM6DS0_ACC_GYRO_FDS_t *value);

/*******************************************************************************
* Register      : CTRL_REG7_XL
* Address       : 0X21
* Bit Group Name: DCF
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_DCF_ODR_DIV_50      = 0x00,
  LSM6DS0_ACC_GYRO_DCF_ODR_DIV_100     = 0x20,
  LSM6DS0_ACC_GYRO_DCF_ODR_DIV_9     = 0x40,
  LSM6DS0_ACC_GYRO_DCF_ODR_DIV_400     = 0x60,
} LSM6DS0_ACC_GYRO_DCF_t;

#define   LSM6DS0_ACC_GYRO_DCF_MASK   0x60
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter(void *handle, LSM6DS0_ACC_GYRO_DCF_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerCutOff_filter(void *handle, LSM6DS0_ACC_GYRO_DCF_t *value);

/*******************************************************************************
* Register      : CTRL_REG7_XL
* Address       : 0X21
* Bit Group Name: HR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_HR_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_HR_ENABLE     = 0x80,
} LSM6DS0_ACC_GYRO_HR_t;

#define   LSM6DS0_ACC_GYRO_HR_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode(void *handle, LSM6DS0_ACC_GYRO_HR_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerHighResolutionMode(void *handle, LSM6DS0_ACC_GYRO_HR_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SW_RESET_NO     = 0x00,
  LSM6DS0_ACC_GYRO_SW_RESET_YES      = 0x01,
} LSM6DS0_ACC_GYRO_SW_RESET_t;

#define   LSM6DS0_ACC_GYRO_SW_RESET_MASK    0x01
status_t  LSM6DS0_ACC_GYRO_W_ResetSW(void *handle, LSM6DS0_ACC_GYRO_SW_RESET_t newValue);
status_t LSM6DS0_ACC_GYRO_R_ResetSW(void *handle, LSM6DS0_ACC_GYRO_SW_RESET_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BLE_LSB_AT_LOW      = 0x00,
  LSM6DS0_ACC_GYRO_BLE_MSB_AT_LOW      = 0x02,
} LSM6DS0_ACC_GYRO_BLE_t;

#define   LSM6DS0_ACC_GYRO_BLE_MASK   0x02
status_t  LSM6DS0_ACC_GYRO_W_BigLittleEndianDataSelection(void *handle, LSM6DS0_ACC_GYRO_BLE_t newValue);
status_t LSM6DS0_ACC_GYRO_R_BigLittleEndianDataSelection(void *handle, LSM6DS0_ACC_GYRO_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: IF_ADD_INC
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_IF_ADD_INC_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE     = 0x04,
} LSM6DS0_ACC_GYRO_IF_ADD_INC_t;

#define   LSM6DS0_ACC_GYRO_IF_ADD_INC_MASK    0x04
status_t  LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess(void *handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AutoIndexOnMultiAccess(void *handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SIM_4WIRE     = 0x00,
  LSM6DS0_ACC_GYRO_SIM_3WIRE     = 0x08,
} LSM6DS0_ACC_GYRO_SIM_t;

#define   LSM6DS0_ACC_GYRO_SIM_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_SPI_SerialInterfaceMode(void *handle, LSM6DS0_ACC_GYRO_SIM_t newValue);
status_t LSM6DS0_ACC_GYRO_R_SPI_SerialInterfaceMode(void *handle, LSM6DS0_ACC_GYRO_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_PP_OD_PUSH_PULL     = 0x00,
  LSM6DS0_ACC_GYRO_PP_OD_OPEN_DRAIN      = 0x10,
} LSM6DS0_ACC_GYRO_PP_OD_t;

#define   LSM6DS0_ACC_GYRO_PP_OD_MASK   0x10
status_t  LSM6DS0_ACC_GYRO_W_INT_Pin_Mode(void *handle, LSM6DS0_ACC_GYRO_PP_OD_t newValue);
status_t LSM6DS0_ACC_GYRO_R_INT_Pin_Mode(void *handle, LSM6DS0_ACC_GYRO_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_H_LACTIVE_HIGH      = 0x00,
  LSM6DS0_ACC_GYRO_H_LACTIVE_LOW     = 0x20,
} LSM6DS0_ACC_GYRO_H_LACTIVE_t;

#define   LSM6DS0_ACC_GYRO_H_LACTIVE_MASK   0x20
status_t  LSM6DS0_ACC_GYRO_W_InterruptActive(void *handle, LSM6DS0_ACC_GYRO_H_LACTIVE_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptActive(void *handle, LSM6DS0_ACC_GYRO_H_LACTIVE_t *value);

/*******************************************************************************
* Register      : CTRL_REG8
* Address       : 0X22
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_BOOT_NO     = 0x00,
  LSM6DS0_ACC_GYRO_BOOT_YES      = 0x80,
} LSM6DS0_ACC_GYRO_BOOT_t;

#define   LSM6DS0_ACC_GYRO_BOOT_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_Reboot(void *handle, LSM6DS0_ACC_GYRO_BOOT_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Reboot(void *handle, LSM6DS0_ACC_GYRO_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: STOP_ON_FTH
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_STOP_ON_FTH_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_STOP_ON_FTH_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_STOP_ON_FTH_t;

#define   LSM6DS0_ACC_GYRO_STOP_ON_FTH_MASK   0x01
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold_Level(void *handle, LSM6DS0_ACC_GYRO_STOP_ON_FTH_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold_Level(void *handle, LSM6DS0_ACC_GYRO_STOP_ON_FTH_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FIFO_EN_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_FIFO_EN_ENABLE      = 0x02,
} LSM6DS0_ACC_GYRO_FIFO_EN_t;

#define   LSM6DS0_ACC_GYRO_FIFO_EN_MASK   0x02
status_t  LSM6DS0_ACC_GYRO_W_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_EN_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_I2C_DISABLE_SPI_AND_I2C     = 0x00,
  LSM6DS0_ACC_GYRO_I2C_DISABLE_SPI_ONLY      = 0x04,
} LSM6DS0_ACC_GYRO_I2C_DISABLE_t;

#define   LSM6DS0_ACC_GYRO_I2C_DISABLE_MASK   0x04
status_t  LSM6DS0_ACC_GYRO_W_DigitalInterface(void *handle, LSM6DS0_ACC_GYRO_I2C_DISABLE_t newValue);
status_t LSM6DS0_ACC_GYRO_R_DigitalInterface(void *handle, LSM6DS0_ACC_GYRO_I2C_DISABLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: DRDY_MASK_BIT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_ENABLE      = 0x08,
} LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t;

#define   LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_MASK   0x08
status_t  LSM6DS0_ACC_GYRO_W_DataReadyTimer(void *handle, LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t newValue);
status_t LSM6DS0_ACC_GYRO_R_DataReadyTimer(void *handle, LSM6DS0_ACC_GYRO_DRDY_MASK_BIT_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: FIFO_TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_ENABLE     = 0x10,
} LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t;

#define   LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_MASK    0x10
status_t  LSM6DS0_ACC_GYRO_W_Temperature_In_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t newValue);
status_t LSM6DS0_ACC_GYRO_R_Temperature_In_FIFO(void *handle, LSM6DS0_ACC_GYRO_FIFO_TEMP_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG9
* Address       : 0X23
* Bit Group Name: SLEEP_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_SLEEP_G_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_SLEEP_G_ENABLE      = 0x40,
} LSM6DS0_ACC_GYRO_SLEEP_G_t;

#define   LSM6DS0_ACC_GYRO_SLEEP_G_MASK   0x40
status_t  LSM6DS0_ACC_GYRO_W_GyroSleepMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroSleepMode(void *handle, LSM6DS0_ACC_GYRO_SLEEP_G_t *value);

/*******************************************************************************
* Register      : CTRL_REG10
* Address       : 0X24
* Bit Group Name: ST_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ST_XL_DISABLE     = 0x00,
  LSM6DS0_ACC_GYRO_ST_XL_ENABLE      = 0x01,
} LSM6DS0_ACC_GYRO_ST_XL_t;

#define   LSM6DS0_ACC_GYRO_ST_XL_MASK   0x01
status_t  LSM6DS0_ACC_GYRO_W_AccelerometerSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_XL_t newValue);
status_t LSM6DS0_ACC_GYRO_R_AccelerometerSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_XL_t *value);

/*******************************************************************************
* Register      : CTRL_REG10
* Address       : 0X24
* Bit Group Name: ST_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ST_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ST_G_ENABLE     = 0x04,
} LSM6DS0_ACC_GYRO_ST_G_t;

#define   LSM6DS0_ACC_GYRO_ST_G_MASK    0x04
status_t  LSM6DS0_ACC_GYRO_W_GyroSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroSelfTest(void *handle, LSM6DS0_ACC_GYRO_ST_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: XL_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XL_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_XL_XL_UP      = 0x01,
} LSM6DS0_ACC_GYRO_XL_XL_t;

#define   LSM6DS0_ACC_GYRO_XL_XL_MASK   0x01
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_X(void *handle, LSM6DS0_ACC_GYRO_XL_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: XH_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XH_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_XH_XL_UP      = 0x02,
} LSM6DS0_ACC_GYRO_XH_XL_t;

#define   LSM6DS0_ACC_GYRO_XH_XL_MASK   0x02
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_X(void *handle, LSM6DS0_ACC_GYRO_XH_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: YL_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YL_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_YL_XL_UP      = 0x04,
} LSM6DS0_ACC_GYRO_YL_XL_t;

#define   LSM6DS0_ACC_GYRO_YL_XL_MASK   0x04
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Y(void *handle, LSM6DS0_ACC_GYRO_YL_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: YH_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YH_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_YH_XL_UP      = 0x08,
} LSM6DS0_ACC_GYRO_YH_XL_t;

#define   LSM6DS0_ACC_GYRO_YH_XL_MASK   0x08
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Y(void *handle, LSM6DS0_ACC_GYRO_YH_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: ZL_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZL_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_ZL_XL_UP      = 0x10,
} LSM6DS0_ACC_GYRO_ZL_XL_t;

#define   LSM6DS0_ACC_GYRO_ZL_XL_MASK   0x10
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Z(void *handle, LSM6DS0_ACC_GYRO_ZL_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: ZH_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZH_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_ZH_XL_UP      = 0x20,
} LSM6DS0_ACC_GYRO_ZH_XL_t;

#define   LSM6DS0_ACC_GYRO_ZH_XL_MASK   0x20
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag_High_Z(void *handle, LSM6DS0_ACC_GYRO_ZH_XL_t *value);

/*******************************************************************************
* Register      : INT_GEN_SRC_XL
* Address       : 0X26
* Bit Group Name: IA_XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_IA_XL_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_IA_XL_UP      = 0x40,
} LSM6DS0_ACC_GYRO_IA_XL_t;

#define   LSM6DS0_ACC_GYRO_IA_XL_MASK   0x40
status_t LSM6DS0_ACC_GYRO_R_AccelerometerInterruptFlag(void *handle, LSM6DS0_ACC_GYRO_IA_XL_t *value);


/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: FTH
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_FTH_MASK   0x1F
#define   LSM6DS0_ACC_GYRO_FTH_POSITION   0
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Threshold(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: FMODE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FMODE_BYPASS      = 0x00,
  LSM6DS0_ACC_GYRO_FMODE_FIFO      = 0x20,
  LSM6DS0_ACC_GYRO_FMODE_STREAM_TO_FIFO      = 0x60,
  LSM6DS0_ACC_GYRO_FMODE_BYPASS_TO_STREAM      = 0x80,
  LSM6DS0_ACC_GYRO_FMODE_STREAM    = 0xA0,
} LSM6DS0_ACC_GYRO_FMODE_t;

#define   LSM6DS0_ACC_GYRO_FMODE_MASK   0xE0
status_t  LSM6DS0_ACC_GYRO_W_FIFO_Mode(void *handle, LSM6DS0_ACC_GYRO_FMODE_t newValue);
status_t LSM6DS0_ACC_GYRO_R_FIFO_Mode(void *handle, LSM6DS0_ACC_GYRO_FMODE_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: FSS
* Permission    : RO
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_FSS_MASK   0x3F
#define   LSM6DS0_ACC_GYRO_FSS_POSITION   0
status_t LSM6DS0_ACC_GYRO_R_FIFO_Samples(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: OVRN
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_OVRN_DOWN     = 0x00,
  LSM6DS0_ACC_GYRO_OVRN_UP     = 0x40,
} LSM6DS0_ACC_GYRO_OVRN_t;

#define   LSM6DS0_ACC_GYRO_OVRN_MASK    0x40
status_t LSM6DS0_ACC_GYRO_R_FIFO_OverrunFlag(void *handle, LSM6DS0_ACC_GYRO_OVRN_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: FTH
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_FTH_DOWN      = 0x00,
  LSM6DS0_ACC_GYRO_FTH_UP      = 0x80,
} LSM6DS0_ACC_GYRO_FTH_t;

#define   LSM6DS0_ACC_GYRO_FTH_FLAG_MASK    0x80
status_t LSM6DS0_ACC_GYRO_R_FIFO_ThresholdFlag(void *handle, LSM6DS0_ACC_GYRO_FTH_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: XLIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XLIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_XLIE_G_ENABLE     = 0x01,
} LSM6DS0_ACC_GYRO_XLIE_G_t;

#define   LSM6DS0_ACC_GYRO_XLIE_G_MASK    0x01
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisX(void *handle, LSM6DS0_ACC_GYRO_XLIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: XHIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_XHIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_XHIE_G_ENABLE     = 0x02,
} LSM6DS0_ACC_GYRO_XHIE_G_t;

#define   LSM6DS0_ACC_GYRO_XHIE_G_MASK    0x02
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisX(void *handle, LSM6DS0_ACC_GYRO_XHIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: YLIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YLIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_YLIE_G_ENABLE     = 0x04,
} LSM6DS0_ACC_GYRO_YLIE_G_t;

#define   LSM6DS0_ACC_GYRO_YLIE_G_MASK    0x04
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisY(void *handle, LSM6DS0_ACC_GYRO_YLIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: YHIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_YHIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_YHIE_G_ENABLE     = 0x08,
} LSM6DS0_ACC_GYRO_YHIE_G_t;

#define   LSM6DS0_ACC_GYRO_YHIE_G_MASK    0x08
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisY(void *handle, LSM6DS0_ACC_GYRO_YHIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: ZLIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZLIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ZLIE_G_ENABLE     = 0x10,
} LSM6DS0_ACC_GYRO_ZLIE_G_t;

#define   LSM6DS0_ACC_GYRO_ZLIE_G_MASK    0x10
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_Low_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZLIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: ZHIE_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_ZHIE_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_ZHIE_G_ENABLE     = 0x20,
} LSM6DS0_ACC_GYRO_ZHIE_G_t;

#define   LSM6DS0_ACC_GYRO_ZHIE_G_MASK    0x20
status_t  LSM6DS0_ACC_GYRO_W_GyroInterrupt_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterrupt_High_AxisZ(void *handle, LSM6DS0_ACC_GYRO_ZHIE_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: LIR_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_LIR_G_NOT_LATCHED     = 0x00,
  LSM6DS0_ACC_GYRO_LIR_G_LATCHED     = 0x40,
} LSM6DS0_ACC_GYRO_LIR_G_t;

#define   LSM6DS0_ACC_GYRO_LIR_G_MASK   0x40
status_t  LSM6DS0_ACC_GYRO_W_GyroInterruptSignalType(void *handle, LSM6DS0_ACC_GYRO_LIR_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptSignalType(void *handle, LSM6DS0_ACC_GYRO_LIR_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_CFG_G
* Address       : 0X30
* Bit Group Name: AOI_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_AOI_G_OR      = 0x00,
  LSM6DS0_ACC_GYRO_AOI_G_AND     = 0x80,
} LSM6DS0_ACC_GYRO_AOI_G_t;

#define   LSM6DS0_ACC_GYRO_AOI_G_MASK   0x80
status_t  LSM6DS0_ACC_GYRO_W_GyroInterruptCombinationEvent(void *handle, LSM6DS0_ACC_GYRO_AOI_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroInterruptCombinationEvent(void *handle, LSM6DS0_ACC_GYRO_AOI_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_THS_XH_G
* Address       : 0X31
* Bit Group Name: DCRM_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_DCRM_G_RESET      = 0x00,
  LSM6DS0_ACC_GYRO_DCRM_G_DECREMENT      = 0x80,
} LSM6DS0_ACC_GYRO_DCRM_G_t;

#define   LSM6DS0_ACC_GYRO_DCRM_G_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_GyroCounterModeSelection(void *handle, LSM6DS0_ACC_GYRO_DCRM_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_GyroCounterModeSelection(void *handle, LSM6DS0_ACC_GYRO_DCRM_G_t *value);

/*******************************************************************************
* Register      : INT_GEN_DUR_G
* Address       : 0X37
* Bit Group Name: DUR_G
* Permission    : RW
*******************************************************************************/
#define   LSM6DS0_ACC_GYRO_DUR_G_MASK   0x7F
#define   LSM6DS0_ACC_GYRO_DUR_G_POSITION   0
status_t  LSM6DS0_ACC_GYRO_W_InterruptDurationValue(void *handle, u8_t newValue);
status_t LSM6DS0_ACC_GYRO_R_InterruptDurationValue(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_GEN_DUR_G
* Address       : 0X37
* Bit Group Name: WAIT_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS0_ACC_GYRO_WAIT_G_DISABLE      = 0x00,
  LSM6DS0_ACC_GYRO_WAIT_G_ENABLE     = 0x80,
} LSM6DS0_ACC_GYRO_WAIT_G_t;

#define   LSM6DS0_ACC_GYRO_WAIT_G_MASK    0x80
status_t  LSM6DS0_ACC_GYRO_W_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_G_t newValue);
status_t LSM6DS0_ACC_GYRO_R_WaitFunction(void *handle, LSM6DS0_ACC_GYRO_WAIT_G_t *value);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Temperature
* Permission    : RO
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Get_Temperature(void *handle, u8_t *buff);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : AngularRateThreshold
* Permission    : RW
*******************************************************************************/
status_t LSM6DS0_ACC_GYRO_Set_AngularRateThreshold(void *handle, u8_t *buff);
status_t LSM6DS0_ACC_GYRO_Get_AngularRateThreshold(void *handle, u8_t *buff);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LSM6DS0_ACC_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension);

#ifdef __cplusplus
}
#endif

#endif



