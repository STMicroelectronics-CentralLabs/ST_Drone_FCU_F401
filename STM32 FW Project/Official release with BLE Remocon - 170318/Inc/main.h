/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};


/* Private define ------------------------------------------------------------*/

//#define LPS22HB_CS_Pin GPIO_PIN_13
//#define LPS22HB_CS_GPIO_Port GPIOC
//#define USB_Monitor_Pin GPIO_PIN_14
//#define USB_Monitor_GPIO_Port GPIOC
//#define OSC_16MHZ_IN_Pin GPIO_PIN_0
//#define OSC_16MHZ_IN_GPIO_Port GPIOH
//#define OSC_16MHZ_OUT_Pin GPIO_PIN_1
//#define OSC_16MHZ_OUT_GPIO_Port GPIOH
//#define BLE_IRQ_Pin GPIO_PIN_4
//#define BLE_IRQ_GPIO_Port GPIOA
//#define BLE_CS_Pin GPIO_PIN_0
//#define BLE_CS_GPIO_Port GPIOB
//#define VBAT_SENSE_Pin GPIO_PIN_1
//#define VBAT_SENSE_GPIO_Port GPIOB
//#define BLE_RSTN_Pin GPIO_PIN_2
//#define BLE_RSTN_GPIO_Port GPIOB
//#define LIS2MDL_CS_Pin GPIO_PIN_12
//#define LIS2MDL_CS_GPIO_Port GPIOB
//#define SPI2_CLK_Pin GPIO_PIN_13
//#define SPI2_CLK_GPIO_Port GPIOB
//#define SPI2_SDA_Pin GPIO_PIN_15
//#define SPI2_SDA_GPIO_Port GPIOB
//#define LSM6DSL_CS_Pin GPIO_PIN_8
//#define LSM6DSL_CS_GPIO_Port GPIOA
///* USER CODE BEGIN Private defines */
//
///** @defgroup LSM6DSL_G Selftest limits
// * @{
// */
//#define LSM6DSL_G_ST_X_LO_LIMIT                          ((float)20000)  //((float)200000)    /*!< Selftest X axis low limit [mdps] */
//#define LSM6DSL_G_ST_X_HI_LIMIT                          ((float)80000)  //((float)800000)    /*!< Selftest X axis high limit [mdps] */
//#define LSM6DSL_G_ST_Y_LO_LIMIT                          ((float)20000)  //((float)200000)    /*!< Selftest Y axis low limit [mdps] */
//#define LSM6DSL_G_ST_Y_HI_LIMIT                          ((float)80000)  //((float)800000)    /*!< Selftest Y axis high limit [mdps] */
//#define LSM6DSL_G_ST_Z_LO_LIMIT                          ((float)20000)  //((float)200000)    /*!< Selftest Z axis low limit [mdps] */
//#define LSM6DSL_G_ST_Z_HI_LIMIT                          ((float)80000)  //((float)800000)    /*!< Selftest Z axis high limit [mdps] */
//
///** @defgroup LSM6DSL_XL Selftest limits
// * @{
// */
//#define LSM6DSL_XL_ST_X_LO_LIMIT                          ((float)90) //((float)70)       /*!< Selftest X axis low limit [mg] */
//#define LSM6DSL_XL_ST_X_HI_LIMIT                          ((float)1700)//((float)1500)     /*!< Selftest X axis high limit [mg] */
//#define LSM6DSL_XL_ST_Y_LO_LIMIT                          ((float)90) //((float)70)       /*!< Selftest Y axis low limit [mg] */
//#define LSM6DSL_XL_ST_Y_HI_LIMIT                          ((float)1700)//((float)1500)     /*!< Selftest Y axis high limit [mg] */
//#define LSM6DSL_XL_ST_Z_LO_LIMIT                          ((float)90) //((float)70)       /*!< Selftest Z axis low limit [mg] */
//#define LSM6DSL_XL_ST_Z_HI_LIMIT                          ((float)1700)//((float)1500)     /*!< Selftest Z axis high limit [mg] */
//
///** @defgroup LPS22HB Selftest limits
// * @{
// */
//#define LPS22HB_P_ST1_LO_LIMIT           ( ( float )260 )        /*!< Selftest 1 pressure low limit [hPa] */
//#define LPS22HB_P_ST1_HI_LIMIT           ( ( float )1260 )       /*!< Selftest 1 pressure high limit [hPa] */
//#define LPS22HB_T_ST1_LO_LIMIT           ( ( float )-25 )        /*!< Selftest 1 temperature low limit [degC] */
//#define LPS22HB_T_ST1_HI_LIMIT           ( ( float )105 )        /*!< Selftest 1 temperature high limit [degC] */
//   
//#define LPS22HB_P_ST3_LO_LIMIT           ( ( float )0.000001 )    /*!< Selftest 3 pressure STDEV low limit [hPa] */
//#define LPS22HB_T_ST3_LO_LIMIT           ( ( float )0.000001 )    /*!< Selftest 3 temperature STDEV low limit [degC] */  
//   
///** @defgroup LSM303AGR_M Self-test limits
// * @{
// */
//#define SELFTEST_MAX_M    360 // 360LSB * 3.9sensitivity
//#define SELFTEST_MIN_M    17 // 17LSB * 3.9sensitivity 
//   
//#define LIS2MDL_M_ST_X_LO_LIMIT                          ((float)SELFTEST_MIN_M)     /*!< Selftest X axis low limit [LSB] */
//#define LIS2MDL_M_ST_X_HI_LIMIT                          ((float)SELFTEST_MAX_M)     /*!< Selftest X axis high limit [LSB] */
//#define LIS2MDL_M_ST_Y_LO_LIMIT                          ((float)SELFTEST_MIN_M)     /*!< Selftest Y axis low limit [LSB] */
//#define LIS2MDL_M_ST_Y_HI_LIMIT                          ((float)SELFTEST_MAX_M)     /*!< Selftest Y axis high limit [LSB] */
//#define LIS2MDL_M_ST_Z_LO_LIMIT                          ((float)SELFTEST_MIN_M)     /*!< Selftest Z axis low limit [LSB] */
//#define LIS2MDL_M_ST_Z_HI_LIMIT                          ((float)SELFTEST_MAX_M)     /*!< Selftest Z axis high limit [LSB] */
   
/** @defgroup BLE default values
* @{
*/
#define NAME_BLUEMS 'B','M','2','V','2','1','0'
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#define GAP_PERIPHERAL_ROLE_IDB05A1			(0x01)
#define GAP_BROADCASTER_ROLE_IDB05A1		        (0x02)
#define GAP_CENTRAL_ROLE_IDB05A1			(0x04)
#define GAP_OBSERVER_ROLE_IDB05A1			(0x08)
#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}


/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/**
 * @name Configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
#define CONFIG_DATA_PUBADDR_OFFSET          (0x00) /**< Bluetooth public address */
#define CONFIG_DATA_DIV_OFFSET              (0x06) /**< DIV used to derive CSRK */
#define CONFIG_DATA_ER_OFFSET               (0x08) /**< Encryption root key used to derive LTK and CSRK */
#define CONFIG_DATA_IR_OFFSET               (0x18) /**< Identity root key used to derive LTK and CSRK */
#define CONFIG_DATA_LL_WITHOUT_HOST         (0x2C) /**< Switch on/off Link Layer only mode. Set to 1 to disable Host.
 	 	 	 	 	 	 	 	 	 	 	 	 	 It can be written only if aci_hal_write_config_data() is the first command 	 	 	 	 	 	 	 	 	 	 	 	 after reset. */
#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
/**
 * @name Length for configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
#define CONFIG_DATA_PUBADDR_LEN             (6)
#define CONFIG_DATA_DIV_LEN                 (2)
#define CONFIG_DATA_ER_LEN                  (16)
#define CONFIG_DATA_IR_LEN                  (16)
#define CONFIG_DATA_LL_WITHOUT_HOST_LEN     (1)
#define CONFIG_DATA_MODE_LEN                (1)
#define CONFIG_DATA_WATCHDOG_DISABLE_LEN    (1)
   
/**
 * Select the BlueNRG mode configurations.\n
 * @li Mode 1: slave or master, 1 connection, RAM1 only (small GATT DB)
 * @li Mode 2: slave or master, 1 connection, RAM1 and RAM2 (large GATT DB)
 * @li Mode 3: master/slave, 8 connections, RAM1 and RAM2.
 * @li Mode 4: master/slave, 4 connections, RAM1 and RAM2 simultaneous scanning and advertising.
 */
#define CONFIG_DATA_MODE_OFFSET 			(0x2D)

#define CONFIG_DATA_WATCHDOG_DISABLE 		(0x2F) /**< Set to 1 to disable watchdog. It is enabled by default. */

/**
 * @anchor Auth_req
 * @name Authentication requirements
 * @{
 */
#define BONDING				            (0x01)
#define NO_BONDING				        (0x00)
/**
 * @}
 */

/**
 * @anchor MITM_req
 * @name MITM protection requirements
 * @{
 */
#define MITM_PROTECTION_NOT_REQUIRED	(0x00)
#define MITM_PROTECTION_REQUIRED        (0x01)
/**
 * @}
 */

/**
 * @anchor OOB_Data
 * @name Out-Of-Band data
 * @{
 */
#define OOB_AUTH_DATA_ABSENT		    (0x00)
#define OOB_AUTH_DATA_PRESENT      		(0x01)
/**
 * @}
 */
/**
 * @anchor Use_fixed_pin
 * @name Use fixed pin
 * @{
 */
#define USE_FIXED_PIN_FOR_PAIRING		(0x00)
#define DONOT_USE_FIXED_PIN_FOR_PAIRING	(0x01)
/**
 * @}
 */


//static int HCI_ProcessEvent=0;
//static volatile uint32_t HCI_ProcessEvent=0;
extern volatile uint32_t HCI_ProcessEvent;  
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
