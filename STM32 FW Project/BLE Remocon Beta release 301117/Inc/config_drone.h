

#ifndef _CONFIG_DRONE_H_
#define _CONFIG_DRONE_H_

#include "stm32f4xx_hal.h"



// RC Connection timeout value in ms
// If there is no new data received after this specified time, it will go to
// disconnected status
#define RC_TIMEOUT_VALUE    30

// Define the GPIO port for R/C input capture channels
#define RC_CHANNEL1_PORT    GPIOA
#define RC_CHANNEL1_PIN     GPIO_PIN_0
#define RC_CHANNEL2_PORT    GPIOA
#define RC_CHANNEL2_PIN     GPIO_PIN_1
#define RC_CHANNEL3_PORT    GPIOA
#define RC_CHANNEL3_PIN     GPIO_PIN_2
#define RC_CHANNEL4_PORT    GPIOA
#define RC_CHANNEL4_PIN     GPIO_PIN_3

// Choose coordinate system defined in sensor_data.c
#define COORDINATE_SYSTEM   3
// Use magnetic sensor or not 1 = use
#define USE_MAG_SENSOR      0
// Use presure sensor or not 1 = use
#define USE_PRESSURE_SENSOR  1

#endif /* __CONFIG_DRONE_H_ */
