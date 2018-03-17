#ifndef _BOARD_H_
#define _BOARD_H_

#include "stm32f4xx_hal.h"
#include "imu_6axes.h"
#include "magneto.h"
#include "pressure.h"

#define SPI_TIMEOUT     1000

//#define LSM6DS33_NSS_PORT   GPIOA
//#define LIS3MDL_NSS_PORT    GPIOB
//#define LPS25HB_NSS_PORT    GPIOB
#define LSM6DSL_NSS_PORT   GPIOA
#define LIS2MDL_NSS_PORT    GPIOB
#define LPS22HB_NSS_PORT    GPIOC


//#define LSM6DS33_NSS_PIN    GPIO_PIN_8
//#define LIS3MDL_NSS_PIN     GPIO_PIN_12
//#define LPS25HB_NSS_PIN     GPIO_PIN_10
#define LSM6DSL_NSS_PIN    GPIO_PIN_8
#define LIS2MDL_NSS_PIN     GPIO_PIN_12
#define LPS22HB_NSS_PIN     GPIO_PIN_13

#define LSM6DSL_SPI_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()
#define LIS2MDL_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define LPS22HB_SPI_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()



/* Update 09/12/16 - Resistors partition for battery voltage monitoring */
#define BAT_RUP 10      /* Pull-up resistor value [Kohm] */
#define BAT_RDW 20      /* Pull-Down resistor value [Kohm] */

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
    SPI_LSM6DS33 = 0,
    SPI_LIS3MDL  = 1,
    SPI_LPS25HB  = 2,
} SPI_SENSOR_NSS_TypeDef;

#define SENSOR_SPI_NUMBER  3


IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Init(void);
uint8_t BSP_IMU_6AXES_isInitialized(void);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Read_XG_ID(uint8_t *xg_id);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Check_XG_ID(void);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_X_GetAxesRaw(AxesRaw_TypeDef *pData);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_G_GetAxesRaw(AxesRaw_TypeDef *pData);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_X_GetSensitivity( float *pfData );
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_G_GetSensitivity( float *pfData );
IMU_6AXES_ComponentTypeDef BSP_IMU_6AXES_GetComponentType( void );
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Enable_Free_Fall_Detection_Ext(void);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Disable_Free_Fall_Detection_Ext(void);
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Get_Status_Free_Fall_Detection_Ext(uint8_t *status);




MAGNETO_StatusTypeDef BSP_MAGNETO_Init(void);
uint8_t BSP_MAGNETO_isInitialized(void);
MAGNETO_StatusTypeDef BSP_MAGNETO_Read_M_ID(uint8_t *m_id);
MAGNETO_StatusTypeDef BSP_MAGNETO_Check_M_ID(void);
MAGNETO_StatusTypeDef BSP_MAGNETO_M_GetAxesRaw(AxesRaw_TypeDef *pData);
MAGNETO_ComponentTypeDef BSP_MAGNETO_GetComponentType(void);





PRESSURE_StatusTypeDef BSP_PRESSURE_Init(void);
uint8_t BSP_PRESSURE_isInitialized(void);
PRESSURE_StatusTypeDef BSP_PRESSURE_Reset(void);
PRESSURE_StatusTypeDef BSP_PRESSURE_ReadID(uint8_t *p_id);
PRESSURE_StatusTypeDef BSP_PRESSURE_CheckID(void);
PRESSURE_StatusTypeDef BSP_PRESSURE_GetPressure(float* pfData);
PRESSURE_StatusTypeDef BSP_PRESSURE_GetTemperature(float* pfData);
PRESSURE_ComponentTypeDef BSP_PRESSURE_GetComponentType(void);


void Sensor_SPI_NSS_Set(SPI_SENSOR_NSS_TypeDef sensor);
void Sensor_SPI_NSS_Reset(SPI_SENSOR_NSS_TypeDef sensor);

uint8_t Sensor_IO_SPI_CS_Init_All(void);

#endif /* _BOARD_H_ */