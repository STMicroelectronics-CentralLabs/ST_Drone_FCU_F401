/**
  ******************************************************************************
  * @file    motenv1_config.h
  * @author  Central LAB
  * @version V2.2.0
  * @date    24-November-2016
  * @brief   MotEnv1 configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
#ifndef __MOTENV1_CONFIG_H
#define __MOTENV1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* Define the MOTENV1 MAC address, otherwise it will create a MAC related to STM32 UID */
//#define MAC_MOTENV 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#ifndef MAC_MOTENV
  /* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
  #define MAC_STM32UID_MOTENV
#endif /* MAC_MOTENV */

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#ifdef STM32_SENSORTILE
  /* Enabling this define for SensorTile..
   * it will introduce a delay of 10Seconds before starting the application
   * for having time to open the Terminal
   * for looking the BlueMicrosystem Initialization phase */
  //#define MOTENV_ENABLE_PRINTF
#else /* STM32_SENSORTILE */
  #ifndef USE_STM32L0XX_NUCLEO
    /* For Nucleo F401RE/L476RG it's enable by default */
    #define MOTENV_ENABLE_PRINTF
  #endif /* USE_STM32L0XX_NUCLEO */
#endif /* STM32_SENSORTILE */

/* For enabling connection and notification subscriptions debug */
#define MOTENV_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define MOTENV_DEBUG_NOTIFY_TRAMISSION


/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define DRN_VERSION_MAJOR '1'
#define DRN_VERSION_MINOR '1'
#define DRN_VERSION_PATCH '0'

/* Define the MOTENV1 Name MUST be 7 char long */
#define NAME_DRN 'D','R','N','1',DRN_VERSION_MAJOR,DRN_VERSION_MINOR,DRN_VERSION_PATCH

/* Package Name */
#define DRN_PACKAGENAME "DRN1"

#ifdef MOTENV_ENABLE_PRINTF
  #ifdef STM32_NUCLEO
    #define MOTENV_PRINTF(...) printf(__VA_ARGS__)
  #elif STM32_SENSORTILE
    #include "usbd_cdc_interface.h"
    #define MOTENV_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
  #endif /* STM32_NUCLEO */
#else /* MOTENV_ENABLE_PRINTF */
  #define MOTENV_PRINTF(...)
#endif /* MOTENV_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FF80050)
#endif /* USE_STM32L0XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

#endif /* __MOTENV1_CONFIG_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
