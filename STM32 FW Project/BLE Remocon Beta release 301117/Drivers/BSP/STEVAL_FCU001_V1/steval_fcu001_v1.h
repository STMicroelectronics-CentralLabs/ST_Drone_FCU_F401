/**
  ******************************************************************************
  * @file    steval_fcu001_v1.h
  * @author  Competence Center Japan
  * @version V1.0.0
  * @date    12-August-2016
  * @brief   This file contains definitions for the steval_fcu001_v1.c
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
#ifndef __STEVAL_FCU001_V1_H
#define __STEVAL_FCU001_V1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Update 09/12/16 - Resistors partition for battery voltage monitoring */
#define BAT_RUP 10      /* Pull-up resistor value [Kohm] */
#define BAT_RDW 20      /* Pull-Down resistor value [Kohm] */



/* Includes ------------------------------------------------------------------*/


#include "stm32f4xx_hal.h"

#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "pressure.h"
  
#define LSM303AGR_ACC_WHO_AM_I         0x33
#define LSM303AGR_MAG_WHO_AM_I         0x40
#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STEVAL_FCU001_V1
  * @{
  */
      
  /** @addtogroup STEVAL_FCU001_V1_LOW_LEVEL
  * @{
  */

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Exported_Types STEVAL_FCU001_V1_LOW_LEVEL Exported Types
  * @{
  */

typedef struct {
    int32_t AXIS_X;
    int32_t AXIS_Y;
    int32_t AXIS_Z;
} AxesRaw_TypeDef;

typedef struct {
    float AXIS_X;
    float AXIS_Y;
    float AXIS_Z;
} AxesRaw_TypeDef_Float;
  
typedef enum {
    SPI_LSM6DSL = 0,
    SPI_LIS2MDL  = 1,
    SPI_LPS22HB  = 2,
} SPI_SENSOR_NSS_TypeDef;

#define SENSOR_SPI_NUMBER  3


typedef enum 
{
  LED1 = 0,
  LED2 = 1
}Led_TypeDef;


typedef enum
{
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LSM6DSL = 0,                  /* LSM6DSL. */
//  LSM303AGR_X,                  /* LSM303AGR Accelerometer */
//  LSM303AGR_M,                  /* LSM303AGR Magnetometer */
  LIS2MDL,                      /* LIS2MDL Magnetometer */
  LPS22HB                       /* LPS22HB */
} SPI_Device_t;

/**
  * @}
  */ 

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Exported_Constants STEVAL_FCU001_V1_LOW_LEVEL Exported Constants
  * @{
  */ 

#define LEDn                             2

#define LED1_PIN                         GPIO_PIN_5
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK_ENABLE()           __GPIOB_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()          __GPIOB_CLK_DISABLE()  

#define LED2_PIN                       GPIO_PIN_4
#define LED2_GPIO_PORT                 GPIOB
#define LED2_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); \
                                            }while(0)
												
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); \
                                            }while(0)

//#define LSM6DSL_INT2_GPIO_PORT           GPIOA
//#define LSM6DSL_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
//#define LSM6DSL_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
//#define LSM6DSL_INT2_PIN                 GPIO_PIN_2
//#define LSM6DSL_INT2_EXTI_IRQn           EXTI2_IRQn

                                              
#define STEVAL_FCU001_V1_SENSORS_SPI                    SPI2

#define STEVAL_FCU001_V1_SENSORS_SPI_Port               GPIOB
#define STEVAL_FCU001_V1_SENSORS_SPI_MOSI_Pin           GPIO_PIN_15
#define STEVAL_FCU001_V1_SENSORS_SPI_SCK_Pin            GPIO_PIN_13

#define STEVAL_FCU001_V1_SENSORS_SPI_CLK_ENABLE()       __SPI2_CLK_ENABLE()
#define STEVAL_FCU001_V1_SENSORS_SPI_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Port	          GPIOA
#define STEVAL_FCU001_V1_LSM6DSL_SPI_CS_Pin     	  GPIO_PIN_8
#define STEVAL_FCU001_V1_LSM6DSL_SPI_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
                                              
//#define STEVAL_FCU001_V1_LSM303AGR_X_SPI_CS_Port	  GPIOC
//#define STEVAL_FCU001_V1_LSM303AGR_X_SPI_CS_Pin     	  GPIO_PIN_4
//#define STEVAL_FCU001_V1_LSM303AGR_X_SPI_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()
//                                              
//#define STEVAL_FCU001_V1_LSM303AGR_M_SPI_CS_Port	  GPIOB
//#define STEVAL_FCU001_V1_LSM303AGR_M_SPI_CS_Pin     	  GPIO_PIN_1
//#define STEVAL_FCU001_V1_LSM303AGR_M_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
                                              
#define STEVAL_FCU001_V1_LPS22HB_SPI_CS_Port	          GPIOC
#define STEVAL_FCU001_V1_LPS22HB_SPI_CS_Pin     	  GPIO_PIN_13
#define STEVAL_FCU001_V1_LPS22HB_SPI_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()

#define STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Port	          GPIOB
#define STEVAL_FCU001_V1_LIS2MDL_SPI_CS_Pin     	  GPIO_PIN_12
#define STEVAL_FCU001_V1_LIS2MDL_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

                                              
                                              /* I2C clock speed configuration (in Hz) */
//#define I2C_STEVAL_FCU001_V1_TIMING_1000KHZ      0x00D00E28 /* Analog Filter ON, Rise time 120ns, Fall time 25ns */
//#define I2C_STEVAL_FCU001_V1_TIMING_100KHZ      0x10909CEC /* Analog Filter ON, Rise time 120ns, Fall time 25ns */
//                                              
//                                              /* I2C peripheral configuration defines */
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS                            I2C3
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_CLK_ENABLE()               __I2C3_CLK_ENABLE()
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_RCC_PERIPHCLK              RCC_PERIPHCLK_I2C3
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_I2CCLKSOURCE               RCC_I2C3CLKSOURCE_SYSCLK
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_SCL_SDA_AF                 GPIO_AF4_I2C3			
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_SCL_SDA_GPIO_PORT          GPIOC
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_SCL_PIN                    GPIO_PIN_0
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_SDA_PIN                    GPIO_PIN_1
//                                              
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_FORCE_RESET()              __I2C3_FORCE_RESET()
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_RELEASE_RESET()            __I2C3_RELEASE_RESET()
//
///* I2C interrupt requests */                  
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_EV_IRQn                    I2C3_EV_IRQn
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_ER_IRQn                    I2C3_ER_IRQn
//
///* Maximum Timeout values for flags waiting loops. These timeouts are not based
//   on accurate values, they just guarantee that the application will not remain
//   stuck if the SPI communication is corrupted.
//   You may modify these timeout values depending on CPU frequency and application
//   conditions (interrupts routines ...). */   
//#define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */
//
//#ifdef USE_FREERTOS
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX                      I2C3_Mutex_id
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX_TAKE()               osMutexWait(STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX, 0)
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX_RELEASE()            osMutexRelease(STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX)
//#else
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX                      0
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX_TAKE()               0
//  #define STEVAL_FCU001_V1_I2C_ONBOARD_SENSORS_MUTEX_RELEASE()            0
//#endif
                                              
                                              
///*##################### SD ###################################*/
///* Chip Select macro definition */
//#define STEVAL_FCU001_V1_SD_CS_LOW()       HAL_GPIO_WritePin(STEVAL_FCU001_V1_SD_CS_GPIO_PORT, STEVAL_FCU001_V1_SD_CS_PIN, GPIO_PIN_RESET)
//#define STEVAL_FCU001_V1_SD_CS_HIGH()      HAL_GPIO_WritePin(STEVAL_FCU001_V1_SD_CS_GPIO_PORT, STEVAL_FCU001_V1_SD_CS_PIN, GPIO_PIN_SET)
//
///**
//  * @brief  SD Control Interface pins
//  */
//#define STEVAL_FCU001_V1_SD_CS_PIN                               GPIO_PIN_12
//#define STEVAL_FCU001_V1_SD_CS_GPIO_PORT                         GPIOG
//#define STEVAL_FCU001_V1_SD_CS_GPIO_CLK_ENABLE()                 __GPIOG_CLK_ENABLE()
//#define STEVAL_FCU001_V1_SD_CS_GPIO_CLK_DISABLE()                __GPIOG_CLK_DISABLE()
//      
//#define STEVAL_FCU001_V1_SD_DUMMY_BYTE   0xFF
//#define STEVAL_FCU001_V1_SD_NO_RESPONSE_EXPECTED 0x80
//    
                                                
///*##################### SPI3 SensorTile ###################################*/
//#define STEVAL_FCU001_V1_SD_SPI                               SPI3
//#define STEVAL_FCU001_V1_SD_SPI_CLK_ENABLE()                  __SPI3_CLK_ENABLE()
//
//#define STEVAL_FCU001_V1_SD_SPI_SCK_AF                        GPIO_AF6_SPI3
//#define STEVAL_FCU001_V1_SD_SPI_SCK_GPIO_PORT                 GPIOG
//#define STEVAL_FCU001_V1_SD_SPI_SCK_PIN                       GPIO_PIN_9
//#define STEVAL_FCU001_V1_SD_SPI_SCK_GPIO_CLK_ENABLE()         __GPIOG_CLK_ENABLE()
//#define STEVAL_FCU001_V1_SD_SPI_SCK_GPIO_CLK_DISABLE()        __GPIOG_CLK_DISABLE()
//
//#define STEVAL_FCU001_V1_SD_SPI_MISO_MOSI_AF                  GPIO_AF6_SPI3
//#define STEVAL_FCU001_V1_SD_SPI_MISO_MOSI_GPIO_PORT           GPIOG
//#define STEVAL_FCU001_V1_SD_SPI_MISO_MOSI_GPIO_CLK_ENABLE()   __GPIOG_CLK_ENABLE()
//#define STEVAL_FCU001_V1_SD_SPI_MISO_MOSI_GPIO_CLK_DISABLE()  __GPIOG_CLK_DISABLE()
//#define STEVAL_FCU001_V1_SD_SPI_MISO_PIN                      GPIO_PIN_10
//#define STEVAL_FCU001_V1_SD_SPI_MOSI_PIN                      GPIO_PIN_11
///* Maximum Timeout values for flags waiting loops. These timeouts are not based
//   on accurate values, they just guarantee that the application will not remain
//   stuck if the SPI communication is corrupted.
//   You may modify these timeout values depending on CPU frequency and application
//   conditions (interrupts routines ...). */   
//#define STEVAL_FCU001_V1_SD_SPI_TIMEOUT_MAX                   1000

/**
  * @}
  */ 


/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Exported_Macros STEVAL_FCU001_V1_LOW_LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STEVAL_FCU001_V1_LOW_LEVEL_Exported_Functions STEVAL_FCU001_V1_LOW_LEVEL Exported Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
DrvStatusTypeDef Sensor_IO_I2C_Init( void );
DrvStatusTypeDef Sensor_IO_SPI_Init( void );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_I2C_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_I2C_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_CS_Init_All(void);
uint8_t Sensor_IO_SPI_CS_Init(void *handle);
uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
uint8_t Sensor_IO_SPI_CS_Disable(void *handle);
DrvStatusTypeDef LSM6DSM_Sensor_IO_ITConfig( void );

void SD_IO_CS_Init(void);
void SD_IO_CS_DeInit(void);

DrvStatusTypeDef Sensor_IO_Init( void );
DrvStatusTypeDef LSM6DSL_Sensor_IO_ITConfig( void );

/**
  * @}
  */

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

#endif /* __STEVAL_FCU001_V1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
