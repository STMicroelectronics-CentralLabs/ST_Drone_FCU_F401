#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include "stm32f4xx_hal.h"
#include "config_drone.h"
//#include "imu_6axes.h"
//#include "magneto.h"
//#include "pressure.h"
//#include "board.h"
#include "steval_fcu001_v1.h"

void ReadSensorRawData(void *ACC_handle, void *GYR_handle, void *MAG_handle, void *PRE_handle, AxesRaw_TypeDef *acc, AxesRaw_TypeDef *gyro, AxesRaw_TypeDef *mag, float *pre);


#endif /* _SENSOR_DATA_H_ */
