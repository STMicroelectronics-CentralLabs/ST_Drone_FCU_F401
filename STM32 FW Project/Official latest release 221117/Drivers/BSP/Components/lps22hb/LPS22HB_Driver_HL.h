/**
 ******************************************************************************
 * @file    LPS22HB_Driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2016
 * @brief   This file contains definitions for the LPS22HB_Driver_HL.c firmware driver
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
#ifndef __LPS22HB_DRIVER_HL_H
#define __LPS22HB_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "pressure.h"
#include "temperature.h"

/* Include pressure sensor component drivers. */
#include "LPS22HB_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup LPS22HB LPS22HB
 * @{
 */

/** @addtogroup LPS22HB_Public_Constants Public constants
 * @{
 */

#define LPS22HB_SENSORS_MAX_NUM  1     /**< LPS22HB max number of instances */

/** @addtogroup LPS22HB_I2C_Addresses LPS22HB I2C Addresses
 * @{
 */

#define LPS22HB_ADDRESS_LOW      0xB8  /**< LPS22HB I2C Address Low */
#define LPS22HB_ADDRESS_HIGH     0xBA  /**< LPS22HB I2C Address High */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup LPS22HB_Public_Types LPS22HB Public Types
 * @{
 */

/**
 * @brief LPS22HB pressure extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Ovr_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Fth_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Stop_On_Fth         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Usage               ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data            ) ( DrvContextTypeDef*, float*, float* );
  DrvStatusTypeDef ( *FIFO_Get_Mode            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Watermark_Level ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Watermark_Usage     ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Interrupt       ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Reset_Interrupt     ) ( DrvContextTypeDef*, uint8_t );
} LPS22HB_P_ExtDrv_t;

/**
 * @brief LPS22HB temperature extended features driver internal structure definition
 */
typedef struct
{
  DrvStatusTypeDef ( *FIFO_Get_Empty_Status    ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Full_Status     ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Ovr_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Fth_Status      ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Stop_On_Fth         ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Usage               ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Num_Of_Samples  ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Get_Data            ) ( DrvContextTypeDef*, float*, float* );
  DrvStatusTypeDef ( *FIFO_Get_Mode            ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Mode            ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Get_Watermark_Level ) ( DrvContextTypeDef*, uint8_t* );
  DrvStatusTypeDef ( *FIFO_Set_Watermark_Level ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Watermark_Usage     ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Set_Interrupt       ) ( DrvContextTypeDef*, uint8_t );
  DrvStatusTypeDef ( *FIFO_Reset_Interrupt     ) ( DrvContextTypeDef*, uint8_t );
} LPS22HB_T_ExtDrv_t;

/**
 * @brief LPS22HB combo specific data internal structure definition
 */
typedef struct
{
  uint8_t isPressInitialized;
  uint8_t isTempInitialized;
  uint8_t isPressEnabled;
  uint8_t isTempEnabled;
  float Last_ODR;
} LPS22HB_Combo_Data_t;

/**
 * @brief LPS22HB pressure specific data internal structure definition
 */
typedef struct
{
  LPS22HB_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HB_P_Data_t;

/**
 * @brief LPS22HB temperature specific data internal structure definition
 */
typedef struct
{
  LPS22HB_Combo_Data_t *comboData;       /* Combo data to manage in software enable/disable of the combo sensors */
} LPS22HB_T_Data_t;

/**
 * @}
 */

/** @addtogroup LPS22HB_Public_Variables Public variables
 * @{
 */

extern PRESSURE_Drv_t LPS22HB_P_Drv;
extern TEMPERATURE_Drv_t LPS22HB_T_Drv;
extern LPS22HB_Combo_Data_t LPS22HB_Combo_Data[LPS22HB_SENSORS_MAX_NUM];
extern LPS22HB_P_ExtDrv_t LPS22HB_P_ExtDrv;
extern LPS22HB_T_ExtDrv_t LPS22HB_T_ExtDrv;

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

#endif /* __LPS22HB_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
