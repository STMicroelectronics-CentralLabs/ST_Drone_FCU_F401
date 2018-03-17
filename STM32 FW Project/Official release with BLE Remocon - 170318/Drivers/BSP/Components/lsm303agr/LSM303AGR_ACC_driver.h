/**
 ******************************************************************************
 * @file    LSM303AGR_ACC_driver.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    24-February-2016
 * @brief   LSM303AGR Accelerometer header driver file
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
#ifndef __LSM303AGR_ACC_DRIVER__H
#define __LSM303AGR_ACC_DRIVER__H

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

#define LSM303AGR_ACC_I2C_ADDRESS         0x32

/************** Who am I  *******************/

#define LSM303AGR_ACC_WHO_AM_I         0x33

/************** Device Register  *******************/
#define LSM303AGR_ACC_STATUS_REG_AUX    0X07
#define LSM303AGR_ACC_OUT_ADC1_L    0X08
#define LSM303AGR_ACC_OUT_ADC1_H    0X09
#define LSM303AGR_ACC_OUT_ADC2_L    0X0A
#define LSM303AGR_ACC_OUT_ADC2_H    0X0B
#define LSM303AGR_ACC_OUT_ADC3_L    0X0C
#define LSM303AGR_ACC_OUT_ADC3_H    0X0D
#define LSM303AGR_ACC_INT_COUNTER_REG   0X0E
#define LSM303AGR_ACC_WHO_AM_I_REG    0X0F
#define LSM303AGR_ACC_TEMP_CFG_REG    0X1F
#define LSM303AGR_ACC_CTRL_REG1   0X20
#define LSM303AGR_ACC_CTRL_REG2   0X21
#define LSM303AGR_ACC_CTRL_REG3   0X22
#define LSM303AGR_ACC_CTRL_REG4   0X23
#define LSM303AGR_ACC_CTRL_REG5   0X24
#define LSM303AGR_ACC_CTRL_REG6   0X25
#define LSM303AGR_ACC_REFERENCE   0X26
#define LSM303AGR_ACC_STATUS_REG2   0X27
#define LSM303AGR_ACC_OUT_X_L   0X28
#define LSM303AGR_ACC_OUT_X_H   0X29
#define LSM303AGR_ACC_OUT_Y_L   0X2A
#define LSM303AGR_ACC_OUT_Y_H   0X2B
#define LSM303AGR_ACC_OUT_Z_L   0X2C
#define LSM303AGR_ACC_OUT_Z_H   0X2D
#define LSM303AGR_ACC_FIFO_CTRL_REG   0X2E
#define LSM303AGR_ACC_FIFO_SRC_REG    0X2F
#define LSM303AGR_ACC_INT1_CFG    0X30
#define LSM303AGR_ACC_INT1_SOURCE   0X31
#define LSM303AGR_ACC_INT1_THS    0X32
#define LSM303AGR_ACC_INT1_DURATION   0X33
#define LSM303AGR_ACC_INT2_CFG    0X34
#define LSM303AGR_ACC_INT2_SOURCE   0X35
#define LSM303AGR_ACC_INT2_THS    0X36
#define LSM303AGR_ACC_INT2_DURATION   0X37
#define LSM303AGR_ACC_CLICK_CFG   0X38
#define LSM303AGR_ACC_CLICK_SRC   0X39
#define LSM303AGR_ACC_CLICK_THS   0X3A
#define LSM303AGR_ACC_TIME_LIMIT    0X3B
#define LSM303AGR_ACC_TIME_LATENCY    0X3C
#define LSM303AGR_ACC_TIME_WINDOW   0X3D

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM303AGR_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM303AGR_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X0F
* Bit Group Name: WHO_AM_I
* Permission    : RO
*******************************************************************************/
#define   LSM303AGR_ACC_WHO_AM_I_MASK   0xFF
#define   LSM303AGR_ACC_WHO_AM_I_POSITION   0
status_t LSM303AGR_ACC_R_WHO_AM_I(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_BDU_DISABLED     = 0x00,
  LSM303AGR_ACC_BDU_ENABLED      = 0x80,
} LSM303AGR_ACC_BDU_t;

#define   LSM303AGR_ACC_BDU_MASK    0x80
status_t  LSM303AGR_ACC_W_BlockDataUpdate(void *handle, LSM303AGR_ACC_BDU_t newValue);
status_t LSM303AGR_ACC_R_BlockDataUpdate(void *handle, LSM303AGR_ACC_BDU_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_FS_2G      = 0x00,
  LSM303AGR_ACC_FS_4G      = 0x10,
  LSM303AGR_ACC_FS_8G      = 0x20,
  LSM303AGR_ACC_FS_16G     = 0x30,
} LSM303AGR_ACC_FS_t;

#define   LSM303AGR_ACC_FS_MASK   0x30
status_t  LSM303AGR_ACC_W_FullScale(void *handle, LSM303AGR_ACC_FS_t newValue);
status_t LSM303AGR_ACC_R_FullScale(void *handle, LSM303AGR_ACC_FS_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Acceleration
* Permission    : RO
*******************************************************************************/
status_t LSM303AGR_ACC_Get_Raw_Acceleration(void *handle, u8_t *buff);
status_t LSM303AGR_ACC_Get_Acceleration(void *handle, int *buff);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ODR_DO_PWR_DOWN      = 0x00,
  LSM303AGR_ACC_ODR_DO_1Hz     = 0x10,
  LSM303AGR_ACC_ODR_DO_10Hz      = 0x20,
  LSM303AGR_ACC_ODR_DO_25Hz      = 0x30,
  LSM303AGR_ACC_ODR_DO_50Hz      = 0x40,
  LSM303AGR_ACC_ODR_DO_100Hz     = 0x50,
  LSM303AGR_ACC_ODR_DO_200Hz     = 0x60,
  LSM303AGR_ACC_ODR_DO_400Hz     = 0x70,
  LSM303AGR_ACC_ODR_DO_1_6KHz      = 0x80,
  LSM303AGR_ACC_ODR_DO_1_25KHz     = 0x90,
} LSM303AGR_ACC_ODR_t;

#define   LSM303AGR_ACC_ODR_MASK    0xF0
status_t  LSM303AGR_ACC_W_ODR(void *handle, LSM303AGR_ACC_ODR_t newValue);
status_t LSM303AGR_ACC_R_ODR(void *handle, LSM303AGR_ACC_ODR_t *value);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 1DA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_1DA_NOT_AVAILABLE      = 0x00,
  LSM303AGR_ACC_1DA_AVAILABLE      = 0x01,
} LSM303AGR_ACC_1DA_t;

#define   LSM303AGR_ACC_1DA_MASK    0x01
status_t LSM303AGR_ACC_R_x_data_avail(void *handle, LSM303AGR_ACC_1DA_t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 2DA_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_2DA__NOT_AVAILABLE     = 0x00,
  LSM303AGR_ACC_2DA__AVAILABLE     = 0x02,
} LSM303AGR_ACC_2DA__t;

#define   LSM303AGR_ACC_2DA__MASK   0x02
status_t LSM303AGR_ACC_R_y_data_avail(void *handle, LSM303AGR_ACC_2DA__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 3DA_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_3DA__NOT_AVAILABLE     = 0x00,
  LSM303AGR_ACC_3DA__AVAILABLE     = 0x04,
} LSM303AGR_ACC_3DA__t;

#define   LSM303AGR_ACC_3DA__MASK   0x04
status_t LSM303AGR_ACC_R_z_data_avail(void *handle, LSM303AGR_ACC_3DA__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 321DA_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_321DA__NOT_AVAILABLE     = 0x00,
  LSM303AGR_ACC_321DA__AVAILABLE     = 0x08,
} LSM303AGR_ACC_321DA__t;

#define   LSM303AGR_ACC_321DA__MASK   0x08
status_t LSM303AGR_ACC_R_xyz_data_avail(void *handle, LSM303AGR_ACC_321DA__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 1OR_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_1OR__NO_OVERRUN      = 0x00,
  LSM303AGR_ACC_1OR__OVERRUN     = 0x10,
} LSM303AGR_ACC_1OR__t;

#define   LSM303AGR_ACC_1OR__MASK   0x10
status_t LSM303AGR_ACC_R_DataXOverrun(void *handle, LSM303AGR_ACC_1OR__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 2OR_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_2OR__NO_OVERRUN      = 0x00,
  LSM303AGR_ACC_2OR__OVERRUN     = 0x20,
} LSM303AGR_ACC_2OR__t;

#define   LSM303AGR_ACC_2OR__MASK   0x20
status_t LSM303AGR_ACC_R_DataYOverrun(void *handle, LSM303AGR_ACC_2OR__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 3OR_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_3OR__NO_OVERRUN      = 0x00,
  LSM303AGR_ACC_3OR__OVERRUN     = 0x40,
} LSM303AGR_ACC_3OR__t;

#define   LSM303AGR_ACC_3OR__MASK   0x40
status_t LSM303AGR_ACC_R_DataZOverrun(void *handle, LSM303AGR_ACC_3OR__t *value);

/*******************************************************************************
* Register      : STATUS_REG_AUX
* Address       : 0X07
* Bit Group Name: 321OR_
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_321OR__NO_OVERRUN      = 0x00,
  LSM303AGR_ACC_321OR__OVERRUN     = 0x80,
} LSM303AGR_ACC_321OR__t;

#define   LSM303AGR_ACC_321OR__MASK   0x80
status_t LSM303AGR_ACC_R_DataXYZOverrun(void *handle, LSM303AGR_ACC_321OR__t *value);

/*******************************************************************************
* Register      : INT_COUNTER_REG
* Address       : 0X0E
* Bit Group Name: IC
* Permission    : RO
*******************************************************************************/
#define   LSM303AGR_ACC_IC_MASK   0xFF
#define   LSM303AGR_ACC_IC_POSITION   0
status_t LSM303AGR_ACC_R_int_counter(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TEMP_CFG_REG
* Address       : 0X1F
* Bit Group Name: TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_TEMP_EN_DISABLED     = 0x00,
  LSM303AGR_ACC_TEMP_EN_ENABLED      = 0x40,
} LSM303AGR_ACC_TEMP_EN_t;

#define   LSM303AGR_ACC_TEMP_EN_MASK    0x40
status_t  LSM303AGR_ACC_W_TEMP_EN_bits(void *handle, LSM303AGR_ACC_TEMP_EN_t newValue);
status_t LSM303AGR_ACC_R_TEMP_EN_bits(void *handle, LSM303AGR_ACC_TEMP_EN_t *value);

/*******************************************************************************
* Register      : TEMP_CFG_REG
* Address       : 0X1F
* Bit Group Name: ADC_PD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ADC_PD_DISABLED      = 0x00,
  LSM303AGR_ACC_ADC_PD_ENABLED     = 0x80,
} LSM303AGR_ACC_ADC_PD_t;

#define   LSM303AGR_ACC_ADC_PD_MASK   0x80
status_t  LSM303AGR_ACC_W_ADC_PD(void *handle, LSM303AGR_ACC_ADC_PD_t newValue);
status_t LSM303AGR_ACC_R_ADC_PD(void *handle, LSM303AGR_ACC_ADC_PD_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: XEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XEN_DISABLED     = 0x00,
  LSM303AGR_ACC_XEN_ENABLED      = 0x01,
} LSM303AGR_ACC_XEN_t;

#define   LSM303AGR_ACC_XEN_MASK    0x01
status_t  LSM303AGR_ACC_W_XEN(void *handle, LSM303AGR_ACC_XEN_t newValue);
status_t LSM303AGR_ACC_R_XEN(void *handle, LSM303AGR_ACC_XEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: YEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YEN_DISABLED     = 0x00,
  LSM303AGR_ACC_YEN_ENABLED      = 0x02,
} LSM303AGR_ACC_YEN_t;

#define   LSM303AGR_ACC_YEN_MASK    0x02
status_t  LSM303AGR_ACC_W_YEN(void *handle, LSM303AGR_ACC_YEN_t newValue);
status_t LSM303AGR_ACC_R_YEN(void *handle, LSM303AGR_ACC_YEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: ZEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZEN_DISABLED     = 0x00,
  LSM303AGR_ACC_ZEN_ENABLED      = 0x04,
} LSM303AGR_ACC_ZEN_t;

#define   LSM303AGR_ACC_ZEN_MASK    0x04
status_t  LSM303AGR_ACC_W_ZEN(void *handle, LSM303AGR_ACC_ZEN_t newValue);
status_t LSM303AGR_ACC_R_ZEN(void *handle, LSM303AGR_ACC_ZEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: LPEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_LPEN_DISABLED      = 0x00,
  LSM303AGR_ACC_LPEN_ENABLED     = 0x08,
} LSM303AGR_ACC_LPEN_t;

#define   LSM303AGR_ACC_LPEN_MASK   0x08
status_t  LSM303AGR_ACC_W_LOWPWR_EN(void *handle, LSM303AGR_ACC_LPEN_t newValue);
status_t LSM303AGR_ACC_R_LOWPWR_EN(void *handle, LSM303AGR_ACC_LPEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPIS1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HPIS1_DISABLED     = 0x00,
  LSM303AGR_ACC_HPIS1_ENABLED      = 0x01,
} LSM303AGR_ACC_HPIS1_t;

#define   LSM303AGR_ACC_HPIS1_MASK    0x01
status_t  LSM303AGR_ACC_W_hpf_aoi_en_int1(void *handle, LSM303AGR_ACC_HPIS1_t newValue);
status_t LSM303AGR_ACC_R_hpf_aoi_en_int1(void *handle, LSM303AGR_ACC_HPIS1_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPIS2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HPIS2_DISABLED     = 0x00,
  LSM303AGR_ACC_HPIS2_ENABLED      = 0x02,
} LSM303AGR_ACC_HPIS2_t;

#define   LSM303AGR_ACC_HPIS2_MASK    0x02
status_t  LSM303AGR_ACC_W_hpf_aoi_en_int2(void *handle, LSM303AGR_ACC_HPIS2_t newValue);
status_t LSM303AGR_ACC_R_hpf_aoi_en_int2(void *handle, LSM303AGR_ACC_HPIS2_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPCLICK
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HPCLICK_DISABLED     = 0x00,
  LSM303AGR_ACC_HPCLICK_ENABLED      = 0x04,
} LSM303AGR_ACC_HPCLICK_t;

#define   LSM303AGR_ACC_HPCLICK_MASK    0x04
status_t  LSM303AGR_ACC_W_hpf_click_en(void *handle, LSM303AGR_ACC_HPCLICK_t newValue);
status_t LSM303AGR_ACC_R_hpf_click_en(void *handle, LSM303AGR_ACC_HPCLICK_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_FDS_BYPASSED     = 0x00,
  LSM303AGR_ACC_FDS_ENABLED      = 0x08,
} LSM303AGR_ACC_FDS_t;

#define   LSM303AGR_ACC_FDS_MASK    0x08
status_t  LSM303AGR_ACC_W_Data_Filter(void *handle, LSM303AGR_ACC_FDS_t newValue);
status_t LSM303AGR_ACC_R_Data_Filter(void *handle, LSM303AGR_ACC_FDS_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPCF
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HPCF_00      = 0x00,
  LSM303AGR_ACC_HPCF_01      = 0x10,
  LSM303AGR_ACC_HPCF_10      = 0x20,
  LSM303AGR_ACC_HPCF_11      = 0x30,
} LSM303AGR_ACC_HPCF_t;

#define   LSM303AGR_ACC_HPCF_MASK   0x30
status_t  LSM303AGR_ACC_W_hpf_cutoff_freq(void *handle, LSM303AGR_ACC_HPCF_t newValue);
status_t LSM303AGR_ACC_R_hpf_cutoff_freq(void *handle, LSM303AGR_ACC_HPCF_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPM
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HPM_NORMAL     = 0x00,
  LSM303AGR_ACC_HPM_REFERENCE_SIGNAL     = 0x40,
  LSM303AGR_ACC_HPM_NORMAL_2     = 0x80,
  LSM303AGR_ACC_HPM_AUTORST_ON_INT     = 0xC0,
} LSM303AGR_ACC_HPM_t;

#define   LSM303AGR_ACC_HPM_MASK    0xC0
status_t  LSM303AGR_ACC_W_hpf_mode(void *handle, LSM303AGR_ACC_HPM_t newValue);
status_t LSM303AGR_ACC_R_hpf_mode(void *handle, LSM303AGR_ACC_HPM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_OVERRUN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_OVERRUN_DISABLED      = 0x00,
  LSM303AGR_ACC_I1_OVERRUN_ENABLED     = 0x02,
} LSM303AGR_ACC_I1_OVERRUN_t;

#define   LSM303AGR_ACC_I1_OVERRUN_MASK   0x02
status_t  LSM303AGR_ACC_W_FIFO_Overrun_on_INT1(void *handle, LSM303AGR_ACC_I1_OVERRUN_t newValue);
status_t LSM303AGR_ACC_R_FIFO_Overrun_on_INT1(void *handle, LSM303AGR_ACC_I1_OVERRUN_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_WTM
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_WTM_DISABLED      = 0x00,
  LSM303AGR_ACC_I1_WTM_ENABLED     = 0x04,
} LSM303AGR_ACC_I1_WTM_t;

#define   LSM303AGR_ACC_I1_WTM_MASK   0x04
status_t  LSM303AGR_ACC_W_FIFO_Watermark_on_INT1(void *handle, LSM303AGR_ACC_I1_WTM_t newValue);
status_t LSM303AGR_ACC_R_FIFO_Watermark_on_INT1(void *handle, LSM303AGR_ACC_I1_WTM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_DRDY2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_DRDY2_DISABLED      = 0x00,
  LSM303AGR_ACC_I1_DRDY2_ENABLED     = 0x08,
} LSM303AGR_ACC_I1_DRDY2_t;

#define   LSM303AGR_ACC_I1_DRDY2_MASK   0x08
status_t  LSM303AGR_ACC_W_FIFO_DRDY2_on_INT1(void *handle, LSM303AGR_ACC_I1_DRDY2_t newValue);
status_t LSM303AGR_ACC_R_FIFO_DRDY2_on_INT1(void *handle, LSM303AGR_ACC_I1_DRDY2_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_DRDY1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_DRDY1_DISABLED      = 0x00,
  LSM303AGR_ACC_I1_DRDY1_ENABLED     = 0x10,
} LSM303AGR_ACC_I1_DRDY1_t;

#define   LSM303AGR_ACC_I1_DRDY1_MASK   0x10
status_t  LSM303AGR_ACC_W_FIFO_DRDY1_on_INT1(void *handle, LSM303AGR_ACC_I1_DRDY1_t newValue);
status_t LSM303AGR_ACC_R_FIFO_DRDY1_on_INT1(void *handle, LSM303AGR_ACC_I1_DRDY1_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_AOI2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_AOI2_DISABLED     = 0x00,
  LSM303AGR_ACC_I1_AOI2_ENABLED      = 0x20,
} LSM303AGR_ACC_I1_AOI2_t;

#define   LSM303AGR_ACC_I1_AOI2_MASK    0x20
status_t  LSM303AGR_ACC_W_FIFO_AOL2_on_INT1(void *handle, LSM303AGR_ACC_I1_AOI2_t newValue);
status_t LSM303AGR_ACC_R_FIFO_AOL2_on_INT1(void *handle, LSM303AGR_ACC_I1_AOI2_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_AOI1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_AOI1_DISABLED     = 0x00,
  LSM303AGR_ACC_I1_AOI1_ENABLED      = 0x40,
} LSM303AGR_ACC_I1_AOI1_t;

#define   LSM303AGR_ACC_I1_AOI1_MASK    0x40
status_t  LSM303AGR_ACC_W_FIFO_AOL1_on_INT1(void *handle, LSM303AGR_ACC_I1_AOI1_t newValue);
status_t LSM303AGR_ACC_R_FIFO_AOL1_on_INT1(void *handle, LSM303AGR_ACC_I1_AOI1_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_CLICK
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I1_CLICK_DISABLED      = 0x00,
  LSM303AGR_ACC_I1_CLICK_ENABLED     = 0x80,
} LSM303AGR_ACC_I1_CLICK_t;

#define   LSM303AGR_ACC_I1_CLICK_MASK   0x80
status_t  LSM303AGR_ACC_W_FIFO_Click_on_INT1(void *handle, LSM303AGR_ACC_I1_CLICK_t newValue);
status_t LSM303AGR_ACC_R_FIFO_Click_on_INT1(void *handle, LSM303AGR_ACC_I1_CLICK_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_SIM_4_WIRES      = 0x00,
  LSM303AGR_ACC_SIM_3_WIRES      = 0x01,
} LSM303AGR_ACC_SIM_t;

#define   LSM303AGR_ACC_SIM_MASK    0x01
status_t  LSM303AGR_ACC_W_SPI_mode(void *handle, LSM303AGR_ACC_SIM_t newValue);
status_t LSM303AGR_ACC_R_SPI_mode(void *handle, LSM303AGR_ACC_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ST_DISABLED      = 0x00,
  LSM303AGR_ACC_ST_SELF_TEST_0     = 0x02,
  LSM303AGR_ACC_ST_SELF_TEST_1     = 0x04,
  LSM303AGR_ACC_ST_NOT_APPLICABLE      = 0x06,
} LSM303AGR_ACC_ST_t;

#define   LSM303AGR_ACC_ST_MASK   0x06
status_t  LSM303AGR_ACC_W_SelfTest(void *handle, LSM303AGR_ACC_ST_t newValue);
status_t LSM303AGR_ACC_R_SelfTest(void *handle, LSM303AGR_ACC_ST_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: HR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_HR_DISABLED      = 0x00,
  LSM303AGR_ACC_HR_ENABLED     = 0x08,
} LSM303AGR_ACC_HR_t;

#define   LSM303AGR_ACC_HR_MASK   0x08
status_t  LSM303AGR_ACC_W_HiRes(void *handle, LSM303AGR_ACC_HR_t newValue);
status_t LSM303AGR_ACC_R_HiRes(void *handle, LSM303AGR_ACC_HR_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_BLE_LITTLE_ENDIAN      = 0x00,
  LSM303AGR_ACC_BLE_BIG_ENDIAN     = 0x40,
} LSM303AGR_ACC_BLE_t;

#define   LSM303AGR_ACC_BLE_MASK    0x40
status_t  LSM303AGR_ACC_W_LittleBigEndian(void *handle, LSM303AGR_ACC_BLE_t newValue);
status_t LSM303AGR_ACC_R_LittleBigEndian(void *handle, LSM303AGR_ACC_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: D4D_INT2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_D4D_INT2_DISABLED      = 0x00,
  LSM303AGR_ACC_D4D_INT2_ENABLED     = 0x01,
} LSM303AGR_ACC_D4D_INT2_t;

#define   LSM303AGR_ACC_D4D_INT2_MASK   0x01
status_t  LSM303AGR_ACC_W_4D_on_INT2(void *handle, LSM303AGR_ACC_D4D_INT2_t newValue);
status_t LSM303AGR_ACC_R_4D_on_INT2(void *handle, LSM303AGR_ACC_D4D_INT2_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: LIR_INT2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_LIR_INT2_DISABLED      = 0x00,
  LSM303AGR_ACC_LIR_INT2_ENABLED     = 0x02,
} LSM303AGR_ACC_LIR_INT2_t;

#define   LSM303AGR_ACC_LIR_INT2_MASK   0x02
status_t  LSM303AGR_ACC_W_LatchInterrupt_on_INT2(void *handle, LSM303AGR_ACC_LIR_INT2_t newValue);
status_t LSM303AGR_ACC_R_LatchInterrupt_on_INT2(void *handle, LSM303AGR_ACC_LIR_INT2_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: D4D_INT1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_D4D_INT1_DISABLED      = 0x00,
  LSM303AGR_ACC_D4D_INT1_ENABLED     = 0x04,
} LSM303AGR_ACC_D4D_INT1_t;

#define   LSM303AGR_ACC_D4D_INT1_MASK   0x04
status_t  LSM303AGR_ACC_W_4D_on_INT1(void *handle, LSM303AGR_ACC_D4D_INT1_t newValue);
status_t LSM303AGR_ACC_R_4D_on_INT1(void *handle, LSM303AGR_ACC_D4D_INT1_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: LIR_INT1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_LIR_INT1_DISABLED      = 0x00,
  LSM303AGR_ACC_LIR_INT1_ENABLED     = 0x08,
} LSM303AGR_ACC_LIR_INT1_t;

#define   LSM303AGR_ACC_LIR_INT1_MASK   0x08
status_t  LSM303AGR_ACC_W_LatchInterrupt_on_INT1(void *handle, LSM303AGR_ACC_LIR_INT1_t newValue);
status_t LSM303AGR_ACC_R_LatchInterrupt_on_INT1(void *handle, LSM303AGR_ACC_LIR_INT1_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_FIFO_EN_DISABLED     = 0x00,
  LSM303AGR_ACC_FIFO_EN_ENABLED      = 0x40,
} LSM303AGR_ACC_FIFO_EN_t;

#define   LSM303AGR_ACC_FIFO_EN_MASK    0x40
status_t  LSM303AGR_ACC_W_FIFO_EN(void *handle, LSM303AGR_ACC_FIFO_EN_t newValue);
status_t LSM303AGR_ACC_R_FIFO_EN(void *handle, LSM303AGR_ACC_FIFO_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_BOOT_NORMAL_MODE     = 0x00,
  LSM303AGR_ACC_BOOT_REBOOT      = 0x80,
} LSM303AGR_ACC_BOOT_t;

#define   LSM303AGR_ACC_BOOT_MASK   0x80
status_t  LSM303AGR_ACC_W_RebootMemory(void *handle, LSM303AGR_ACC_BOOT_t newValue);
status_t LSM303AGR_ACC_R_RebootMemory(void *handle, LSM303AGR_ACC_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_H_LACTIVE_ACTIVE_HI      = 0x00,
  LSM303AGR_ACC_H_LACTIVE_ACTIVE_LO      = 0x02,
} LSM303AGR_ACC_H_LACTIVE_t;

#define   LSM303AGR_ACC_H_LACTIVE_MASK    0x02
status_t  LSM303AGR_ACC_W_IntActive(void *handle, LSM303AGR_ACC_H_LACTIVE_t newValue);
status_t LSM303AGR_ACC_R_IntActive(void *handle, LSM303AGR_ACC_H_LACTIVE_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: P2_ACT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_P2_ACT_DISABLED      = 0x00,
  LSM303AGR_ACC_P2_ACT_ENABLED     = 0x08,
} LSM303AGR_ACC_P2_ACT_t;

#define   LSM303AGR_ACC_P2_ACT_MASK   0x08
status_t  LSM303AGR_ACC_W_P2_ACT(void *handle, LSM303AGR_ACC_P2_ACT_t newValue);
status_t LSM303AGR_ACC_R_P2_ACT(void *handle, LSM303AGR_ACC_P2_ACT_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: BOOT_I1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_BOOT_I1_DISABLED     = 0x00,
  LSM303AGR_ACC_BOOT_I1_ENABLED      = 0x10,
} LSM303AGR_ACC_BOOT_I1_t;

#define   LSM303AGR_ACC_BOOT_I1_MASK    0x10
status_t  LSM303AGR_ACC_W_Boot_on_INT2(void *handle, LSM303AGR_ACC_BOOT_I1_t newValue);
status_t LSM303AGR_ACC_R_Boot_on_INT2(void *handle, LSM303AGR_ACC_BOOT_I1_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: I2_INT2
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I2_INT2_DISABLED     = 0x00,
  LSM303AGR_ACC_I2_INT2_ENABLED      = 0x20,
} LSM303AGR_ACC_I2_INT2_t;

#define   LSM303AGR_ACC_I2_INT2_MASK    0x20
status_t  LSM303AGR_ACC_W_I2_on_INT2(void *handle, LSM303AGR_ACC_I2_INT2_t newValue);
status_t LSM303AGR_ACC_R_I2_on_INT2(void *handle, LSM303AGR_ACC_I2_INT2_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: I2_INT1
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I2_INT1_DISABLED     = 0x00,
  LSM303AGR_ACC_I2_INT1_ENABLED      = 0x40,
} LSM303AGR_ACC_I2_INT1_t;

#define   LSM303AGR_ACC_I2_INT1_MASK    0x40
status_t  LSM303AGR_ACC_W_I2_on_INT1(void *handle, LSM303AGR_ACC_I2_INT1_t newValue);
status_t LSM303AGR_ACC_R_I2_on_INT1(void *handle, LSM303AGR_ACC_I2_INT1_t *value);

/*******************************************************************************
* Register      : CTRL_REG6
* Address       : 0X25
* Bit Group Name: I2_CLICKEN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_I2_CLICKEN_DISABLED      = 0x00,
  LSM303AGR_ACC_I2_CLICKEN_ENABLED     = 0x80,
} LSM303AGR_ACC_I2_CLICKEN_t;

#define   LSM303AGR_ACC_I2_CLICKEN_MASK   0x80
status_t  LSM303AGR_ACC_W_Click_on_INT2(void *handle, LSM303AGR_ACC_I2_CLICKEN_t newValue);
status_t LSM303AGR_ACC_R_Click_on_INT2(void *handle, LSM303AGR_ACC_I2_CLICKEN_t *value);

/*******************************************************************************
* Register      : REFERENCE
* Address       : 0X26
* Bit Group Name: REF
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_REF_MASK    0xFF
#define   LSM303AGR_ACC_REF_POSITION    0
status_t  LSM303AGR_ACC_W_ReferenceVal(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_ReferenceVal(void *handle, u8_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XDA_NOT_AVAILABLE      = 0x00,
  LSM303AGR_ACC_XDA_AVAILABLE      = 0x01,
} LSM303AGR_ACC_XDA_t;

#define   LSM303AGR_ACC_XDA_MASK    0x01
status_t LSM303AGR_ACC_R_XDataAvail(void *handle, LSM303AGR_ACC_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YDA_NOT_AVAILABLE      = 0x00,
  LSM303AGR_ACC_YDA_AVAILABLE      = 0x02,
} LSM303AGR_ACC_YDA_t;

#define   LSM303AGR_ACC_YDA_MASK    0x02
status_t LSM303AGR_ACC_R_YDataAvail(void *handle, LSM303AGR_ACC_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZDA_NOT_AVAILABLE      = 0x00,
  LSM303AGR_ACC_ZDA_AVAILABLE      = 0x04,
} LSM303AGR_ACC_ZDA_t;

#define   LSM303AGR_ACC_ZDA_MASK    0x04
status_t LSM303AGR_ACC_R_ZDataAvail(void *handle, LSM303AGR_ACC_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZYXDA_NOT_AVAILABLE      = 0x00,
  LSM303AGR_ACC_ZYXDA_AVAILABLE      = 0x08,
} LSM303AGR_ACC_ZYXDA_t;

#define   LSM303AGR_ACC_ZYXDA_MASK    0x08
status_t LSM303AGR_ACC_R_XYZDataAvail(void *handle, LSM303AGR_ACC_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XOR_NO_OVERRUN     = 0x00,
  LSM303AGR_ACC_XOR_OVERRUN      = 0x10,
} LSM303AGR_ACC_XOR_t;

#define   LSM303AGR_ACC_XOR_MASK    0x10
status_t LSM303AGR_ACC_R_XDataOverrun(void *handle, LSM303AGR_ACC_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YOR_NO_OVERRUN     = 0x00,
  LSM303AGR_ACC_YOR_OVERRUN      = 0x20,
} LSM303AGR_ACC_YOR_t;

#define   LSM303AGR_ACC_YOR_MASK    0x20
status_t LSM303AGR_ACC_R_YDataOverrun(void *handle, LSM303AGR_ACC_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZOR_NO_OVERRUN     = 0x00,
  LSM303AGR_ACC_ZOR_OVERRUN      = 0x40,
} LSM303AGR_ACC_ZOR_t;

#define   LSM303AGR_ACC_ZOR_MASK    0x40
status_t LSM303AGR_ACC_R_ZDataOverrun(void *handle, LSM303AGR_ACC_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG2
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZYXOR_NO_OVERRUN     = 0x00,
  LSM303AGR_ACC_ZYXOR_OVERRUN      = 0x80,
} LSM303AGR_ACC_ZYXOR_t;

#define   LSM303AGR_ACC_ZYXOR_MASK    0x80
status_t LSM303AGR_ACC_R_XYZDataOverrun(void *handle, LSM303AGR_ACC_ZYXOR_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: FTH
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_FTH_MASK    0x1F
#define   LSM303AGR_ACC_FTH_POSITION    0
status_t  LSM303AGR_ACC_W_FifoThreshold(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_FifoThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: TR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_TR_TRIGGER_ON_INT1     = 0x00,
  LSM303AGR_ACC_TR_TRIGGER_ON_INT2     = 0x20,
} LSM303AGR_ACC_TR_t;

#define   LSM303AGR_ACC_TR_MASK   0x20
status_t  LSM303AGR_ACC_W_TriggerSel(void *handle, LSM303AGR_ACC_TR_t newValue);
status_t LSM303AGR_ACC_R_TriggerSel(void *handle, LSM303AGR_ACC_TR_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: FM
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_FM_BYPASS      = 0x00,
  LSM303AGR_ACC_FM_FIFO      = 0x40,
  LSM303AGR_ACC_FM_STREAM      = 0x80,
  LSM303AGR_ACC_FM_TRIGGER     = 0xC0,
} LSM303AGR_ACC_FM_t;

#define   LSM303AGR_ACC_FM_MASK   0xC0
status_t  LSM303AGR_ACC_W_FifoMode(void *handle, LSM303AGR_ACC_FM_t newValue);
status_t LSM303AGR_ACC_R_FifoMode(void *handle, LSM303AGR_ACC_FM_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: FSS
* Permission    : RO
*******************************************************************************/
#define   LSM303AGR_ACC_FSS_MASK    0x1F
#define   LSM303AGR_ACC_FSS_POSITION    0
status_t LSM303AGR_ACC_R_FifoSamplesAvail(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_EMPTY_NOT_EMPTY      = 0x00,
  LSM303AGR_ACC_EMPTY_EMPTY      = 0x20,
} LSM303AGR_ACC_EMPTY_t;

#define   LSM303AGR_ACC_EMPTY_MASK    0x20
status_t LSM303AGR_ACC_R_FifoEmpty(void *handle, LSM303AGR_ACC_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: OVRN_FIFO
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_OVRN_FIFO_NO_OVERRUN     = 0x00,
  LSM303AGR_ACC_OVRN_FIFO_OVERRUN      = 0x40,
} LSM303AGR_ACC_OVRN_FIFO_t;

#define   LSM303AGR_ACC_OVRN_FIFO_MASK    0x40
status_t LSM303AGR_ACC_R_FifoOverrun(void *handle, LSM303AGR_ACC_OVRN_FIFO_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_WTM_NORMAL     = 0x00,
  LSM303AGR_ACC_WTM_OVERFLOW     = 0x80,
} LSM303AGR_ACC_WTM_t;

#define   LSM303AGR_ACC_WTM_MASK    0x80
status_t LSM303AGR_ACC_R_WatermarkLevel(void *handle, LSM303AGR_ACC_WTM_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: XLIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XLIE_DISABLED      = 0x00,
  LSM303AGR_ACC_XLIE_ENABLED     = 0x01,
} LSM303AGR_ACC_XLIE_t;

#define   LSM303AGR_ACC_XLIE_MASK   0x01
status_t  LSM303AGR_ACC_W_Int1EnXLo(void *handle, LSM303AGR_ACC_XLIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnXLo(void *handle, LSM303AGR_ACC_XLIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnXLo(void *handle, LSM303AGR_ACC_XLIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnXLo(void *handle, LSM303AGR_ACC_XLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: XHIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XHIE_DISABLED      = 0x00,
  LSM303AGR_ACC_XHIE_ENABLED     = 0x02,
} LSM303AGR_ACC_XHIE_t;

#define   LSM303AGR_ACC_XHIE_MASK   0x02
status_t  LSM303AGR_ACC_W_Int1EnXHi(void *handle, LSM303AGR_ACC_XHIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnXHi(void *handle, LSM303AGR_ACC_XHIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnXHi(void *handle, LSM303AGR_ACC_XHIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnXHi(void *handle, LSM303AGR_ACC_XHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: YLIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YLIE_DISABLED      = 0x00,
  LSM303AGR_ACC_YLIE_ENABLED     = 0x04,
} LSM303AGR_ACC_YLIE_t;

#define   LSM303AGR_ACC_YLIE_MASK   0x04
status_t  LSM303AGR_ACC_W_Int1EnYLo(void *handle, LSM303AGR_ACC_YLIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnYLo(void *handle, LSM303AGR_ACC_YLIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnYLo(void *handle, LSM303AGR_ACC_YLIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnYLo(void *handle, LSM303AGR_ACC_YLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: YHIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YHIE_DISABLED      = 0x00,
  LSM303AGR_ACC_YHIE_ENABLED     = 0x08,
} LSM303AGR_ACC_YHIE_t;

#define   LSM303AGR_ACC_YHIE_MASK   0x08
status_t  LSM303AGR_ACC_W_Int1EnYHi(void *handle, LSM303AGR_ACC_YHIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnYHi(void *handle, LSM303AGR_ACC_YHIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnYHi(void *handle, LSM303AGR_ACC_YHIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnYHi(void *handle, LSM303AGR_ACC_YHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: ZLIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZLIE_DISABLED      = 0x00,
  LSM303AGR_ACC_ZLIE_ENABLED     = 0x10,
} LSM303AGR_ACC_ZLIE_t;

#define   LSM303AGR_ACC_ZLIE_MASK   0x10
status_t  LSM303AGR_ACC_W_Int1EnZLo(void *handle, LSM303AGR_ACC_ZLIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnZLo(void *handle, LSM303AGR_ACC_ZLIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnZLo(void *handle, LSM303AGR_ACC_ZLIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnZLo(void *handle, LSM303AGR_ACC_ZLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: ZHIE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZHIE_DISABLED      = 0x00,
  LSM303AGR_ACC_ZHIE_ENABLED     = 0x20,
} LSM303AGR_ACC_ZHIE_t;

#define   LSM303AGR_ACC_ZHIE_MASK   0x20
status_t  LSM303AGR_ACC_W_Int1EnZHi(void *handle, LSM303AGR_ACC_ZHIE_t newValue);
status_t LSM303AGR_ACC_R_Int1EnZHi(void *handle, LSM303AGR_ACC_ZHIE_t *value);
status_t  LSM303AGR_ACC_W_Int2EnZHi(void *handle, LSM303AGR_ACC_ZHIE_t newValue);
status_t LSM303AGR_ACC_R_Int2EnZHi(void *handle, LSM303AGR_ACC_ZHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: 6D
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_6D_DISABLED      = 0x00,
  LSM303AGR_ACC_6D_ENABLED     = 0x40,
} LSM303AGR_ACC_6D_t;

#define   LSM303AGR_ACC_6D_MASK   0x40
status_t  LSM303AGR_ACC_W_Int1_6D(void *handle, LSM303AGR_ACC_6D_t newValue);
status_t LSM303AGR_ACC_R_Int1_6D(void *handle, LSM303AGR_ACC_6D_t *value);
status_t  LSM303AGR_ACC_W_Int2_6D(void *handle, LSM303AGR_ACC_6D_t newValue);
status_t LSM303AGR_ACC_R_Int2_6D(void *handle, LSM303AGR_ACC_6D_t *value);

/*******************************************************************************
* Register      : INT1_CFG/INT2_CFG
* Address       : 0X30/0x34
* Bit Group Name: AOI
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_AOI_OR     = 0x00,
  LSM303AGR_ACC_AOI_AND      = 0x80,
} LSM303AGR_ACC_AOI_t;

#define   LSM303AGR_ACC_AOI_MASK    0x80
status_t  LSM303AGR_ACC_W_Int1_AOI(void *handle, LSM303AGR_ACC_AOI_t newValue);
status_t LSM303AGR_ACC_R_Int1_AOI(void *handle, LSM303AGR_ACC_AOI_t *value);
status_t  LSM303AGR_ACC_W_Int2_AOI(void *handle, LSM303AGR_ACC_AOI_t newValue);
status_t LSM303AGR_ACC_R_Int2_AOI(void *handle, LSM303AGR_ACC_AOI_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: XL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XL_DOWN      = 0x00,
  LSM303AGR_ACC_XL_UP      = 0x01,
} LSM303AGR_ACC_XL_t;

#define   LSM303AGR_ACC_XL_MASK   0x01
status_t LSM303AGR_ACC_R_Int1_Xlo(void *handle, LSM303AGR_ACC_XL_t *value);
status_t LSM303AGR_ACC_R_Int2_Xlo(void *handle, LSM303AGR_ACC_XL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: XH
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XH_DOWN      = 0x00,
  LSM303AGR_ACC_XH_UP      = 0x02,
} LSM303AGR_ACC_XH_t;

#define   LSM303AGR_ACC_XH_MASK   0x02
status_t LSM303AGR_ACC_R_Int1_XHi(void *handle, LSM303AGR_ACC_XH_t *value);
status_t LSM303AGR_ACC_R_Int2_XHi(void *handle, LSM303AGR_ACC_XH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: YL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YL_DOWN      = 0x00,
  LSM303AGR_ACC_YL_UP      = 0x04,
} LSM303AGR_ACC_YL_t;

#define   LSM303AGR_ACC_YL_MASK   0x04
status_t LSM303AGR_ACC_R_Int1_YLo(void *handle, LSM303AGR_ACC_YL_t *value);
status_t LSM303AGR_ACC_R_Int2_YLo(void *handle, LSM303AGR_ACC_YL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: YH
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YH_DOWN      = 0x00,
  LSM303AGR_ACC_YH_UP      = 0x08,
} LSM303AGR_ACC_YH_t;

#define   LSM303AGR_ACC_YH_MASK   0x08
status_t LSM303AGR_ACC_R_Int1_YHi(void *handle, LSM303AGR_ACC_YH_t *value);
status_t LSM303AGR_ACC_R_Int2_YHi(void *handle, LSM303AGR_ACC_YH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: ZL
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZL_DOWN      = 0x00,
  LSM303AGR_ACC_ZL_UP      = 0x10,
} LSM303AGR_ACC_ZL_t;

#define   LSM303AGR_ACC_ZL_MASK   0x10
status_t LSM303AGR_ACC_R_Int1_Zlo(void *handle, LSM303AGR_ACC_ZL_t *value);
status_t LSM303AGR_ACC_R_Int2_Zlo(void *handle, LSM303AGR_ACC_ZL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: ZH
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZH_DOWN      = 0x00,
  LSM303AGR_ACC_ZH_UP      = 0x20,
} LSM303AGR_ACC_ZH_t;

#define   LSM303AGR_ACC_ZH_MASK   0x20
status_t LSM303AGR_ACC_R_Int1_ZHi(void *handle, LSM303AGR_ACC_ZH_t *value);
status_t LSM303AGR_ACC_R_Int2_ZHi(void *handle, LSM303AGR_ACC_ZH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE/INT2_SOURCE
* Address       : 0X31/0x35
* Bit Group Name: IA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_IA_DOWN      = 0x00,
  LSM303AGR_ACC_IA_UP      = 0x40,
} LSM303AGR_ACC_IA_t;

#define   LSM303AGR_ACC_IA_MASK   0x40
status_t LSM303AGR_ACC_R_Int1_IA(void *handle, LSM303AGR_ACC_IA_t *value);
status_t LSM303AGR_ACC_R_Int2_IA(void *handle, LSM303AGR_ACC_IA_t *value);

/*******************************************************************************
* Register      : INT1_THS/INT2_THS
* Address       : 0X32/0x36
* Bit Group Name: THS
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_THS_MASK    0x7F
#define   LSM303AGR_ACC_THS_POSITION    0
status_t  LSM303AGR_ACC_W_Int1_Threshold(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_Int1_Threshold(void *handle, u8_t *value);
status_t  LSM303AGR_ACC_W_Int2_Threshold(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_Int2_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT1_DURATION/INT2_DURATION
* Address       : 0X33/0x37
* Bit Group Name: D
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_D_MASK    0x7F
#define   LSM303AGR_ACC_D_POSITION    0
status_t  LSM303AGR_ACC_W_Int1_Duration(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_Int1_Duration(void *handle, u8_t *value);
status_t  LSM303AGR_ACC_W_Int2_Duration(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_Int2_Duration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: XS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XS_DISABLED      = 0x00,
  LSM303AGR_ACC_XS_ENABLED     = 0x01,
} LSM303AGR_ACC_XS_t;

#define   LSM303AGR_ACC_XS_MASK   0x01
status_t  LSM303AGR_ACC_W_XSingle(void *handle, LSM303AGR_ACC_XS_t newValue);
status_t LSM303AGR_ACC_R_XSingle(void *handle, LSM303AGR_ACC_XS_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: XD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_XD_DISABLED      = 0x00,
  LSM303AGR_ACC_XD_ENABLED     = 0x02,
} LSM303AGR_ACC_XD_t;

#define   LSM303AGR_ACC_XD_MASK   0x02
status_t  LSM303AGR_ACC_W_XDouble(void *handle, LSM303AGR_ACC_XD_t newValue);
status_t LSM303AGR_ACC_R_XDouble(void *handle, LSM303AGR_ACC_XD_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: YS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YS_DISABLED      = 0x00,
  LSM303AGR_ACC_YS_ENABLED     = 0x04,
} LSM303AGR_ACC_YS_t;

#define   LSM303AGR_ACC_YS_MASK   0x04
status_t  LSM303AGR_ACC_W_YSingle(void *handle, LSM303AGR_ACC_YS_t newValue);
status_t LSM303AGR_ACC_R_YSingle(void *handle, LSM303AGR_ACC_YS_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: YD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_YD_DISABLED      = 0x00,
  LSM303AGR_ACC_YD_ENABLED     = 0x08,
} LSM303AGR_ACC_YD_t;

#define   LSM303AGR_ACC_YD_MASK   0x08
status_t  LSM303AGR_ACC_W_YDouble(void *handle, LSM303AGR_ACC_YD_t newValue);
status_t LSM303AGR_ACC_R_YDouble(void *handle, LSM303AGR_ACC_YD_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: ZS
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZS_DISABLED      = 0x00,
  LSM303AGR_ACC_ZS_ENABLED     = 0x10,
} LSM303AGR_ACC_ZS_t;

#define   LSM303AGR_ACC_ZS_MASK   0x10
status_t  LSM303AGR_ACC_W_ZSingle(void *handle, LSM303AGR_ACC_ZS_t newValue);
status_t LSM303AGR_ACC_R_ZSingle(void *handle, LSM303AGR_ACC_ZS_t *value);

/*******************************************************************************
* Register      : CLICK_CFG
* Address       : 0X38
* Bit Group Name: ZD
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_ZD_DISABLED      = 0x00,
  LSM303AGR_ACC_ZD_ENABLED     = 0x20,
} LSM303AGR_ACC_ZD_t;

#define   LSM303AGR_ACC_ZD_MASK   0x20
status_t  LSM303AGR_ACC_W_ZDouble(void *handle, LSM303AGR_ACC_ZD_t newValue);
status_t LSM303AGR_ACC_R_ZDouble(void *handle, LSM303AGR_ACC_ZD_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: X
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_X_DOWN     = 0x00,
  LSM303AGR_ACC_X_UP     = 0x01,
} LSM303AGR_ACC_X_t;

#define   LSM303AGR_ACC_X_MASK    0x01
status_t LSM303AGR_ACC_R_ClickX(void *handle, LSM303AGR_ACC_X_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: Y
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_Y_DOWN     = 0x00,
  LSM303AGR_ACC_Y_UP     = 0x02,
} LSM303AGR_ACC_Y_t;

#define   LSM303AGR_ACC_Y_MASK    0x02
status_t LSM303AGR_ACC_R_ClickY(void *handle, LSM303AGR_ACC_Y_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: Z
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_Z_DOWN     = 0x00,
  LSM303AGR_ACC_Z_UP     = 0x04,
} LSM303AGR_ACC_Z_t;

#define   LSM303AGR_ACC_Z_MASK    0x04
status_t LSM303AGR_ACC_R_ClickZ(void *handle, LSM303AGR_ACC_Z_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: SIGN
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_SIGN_POSITIVE      = 0x00,
  LSM303AGR_ACC_SIGN_NEGATIVE      = 0x08,
} LSM303AGR_ACC_SIGN_t;

#define   LSM303AGR_ACC_SIGN_MASK   0x08
status_t LSM303AGR_ACC_R_ClickSign(void *handle, LSM303AGR_ACC_SIGN_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: SCLICK
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_SCLICK_DISABLED      = 0x00,
  LSM303AGR_ACC_SCLICK_ENABLED     = 0x10,
} LSM303AGR_ACC_SCLICK_t;

#define   LSM303AGR_ACC_SCLICK_MASK   0x10
status_t LSM303AGR_ACC_R_SingleCLICK(void *handle, LSM303AGR_ACC_SCLICK_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: DCLICK
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_DCLICK_DISABLED      = 0x00,
  LSM303AGR_ACC_DCLICK_ENABLED     = 0x20,
} LSM303AGR_ACC_DCLICK_t;

#define   LSM303AGR_ACC_DCLICK_MASK   0x20
status_t LSM303AGR_ACC_R_DoubleCLICK(void *handle, LSM303AGR_ACC_DCLICK_t *value);

/*******************************************************************************
* Register      : CLICK_SRC
* Address       : 0X39
* Bit Group Name: IA
* Permission    : RO
*******************************************************************************/
typedef enum
{
  LSM303AGR_ACC_CLICK_IA_DOWN      = 0x00,
  LSM303AGR_ACC_CLICK_IA_UP      = 0x40,
} LSM303AGR_ACC_CLICK_IA_t;

#define   LSM303AGR_ACC_IA_MASK   0x40
status_t LSM303AGR_ACC_R_CLICK_IA(void *handle, LSM303AGR_ACC_CLICK_IA_t *value);

/*******************************************************************************
* Register      : CLICK_THS
* Address       : 0X3A
* Bit Group Name: THS
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_THS_MASK    0x7F
#define   LSM303AGR_ACC_THS_POSITION    0
status_t  LSM303AGR_ACC_W_ClickThreshold(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_ClickThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TIME_LIMIT
* Address       : 0X3B
* Bit Group Name: TLI
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_TLI_MASK    0x7F
#define   LSM303AGR_ACC_TLI_POSITION    0
status_t  LSM303AGR_ACC_W_ClickTimeLimit(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_ClickTimeLimit(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TIME_LATENCY
* Address       : 0X3C
* Bit Group Name: TLA
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_TLA_MASK    0xFF
#define   LSM303AGR_ACC_TLA_POSITION    0
status_t  LSM303AGR_ACC_W_ClickTimeLatency(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_ClickTimeLatency(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TIME_WINDOW
* Address       : 0X3D
* Bit Group Name: TW
* Permission    : RW
*******************************************************************************/
#define   LSM303AGR_ACC_TW_MASK   0xFF
#define   LSM303AGR_ACC_TW_POSITION   0
status_t  LSM303AGR_ACC_W_ClickTimeWindow(void *handle, u8_t newValue);
status_t LSM303AGR_ACC_R_ClickTimeWindow(void *handle, u8_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Voltage_ADC
* Permission    : RO
*******************************************************************************/
status_t LSM303AGR_ACC_Get_Voltage_ADC(void *handle, u8_t *buff);

#ifdef __cplusplus
}
#endif

#endif
