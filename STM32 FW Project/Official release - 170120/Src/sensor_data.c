/*
 *  The purpose of this file is to convert the sensor data to proper coordinate
 *  X-axis and Y-axis orientation is in line with Accelerometer sensor mounted on board
 * 
 *  STEVAL-FCU001V1
 *
 *              +---------------------+
 *              |      MOTOR      BAT |
 *              |                     |
 *              |I2C      ^        USB|
 *              |         |           |
 *              |         |       JTAG|
 *              |RC       |           |
 *              |                     |
 *              |    BLE          UART|
 *              +---------------------+
 *       
 *                        ^ (ACC X)
 *                        |          
 *              (ACC Y)<--O(z)
 *  
 *   (1)   Drone FWD direction V (ACC -X)
 *   (2)   Drone FWD direction > (ACC -Y)
 *   (3)   Drone FWD direction ^ (ACC X)
 *   (4)   Drone FWD direction < (ACC Y)
 *  
 * Translation of coordinates for AHRS 
 *  (1) Drone FWD direction > (ACC -X)
 *          
 *           (x)<--O(z)
 *                 |
 *                 V(y)
 * 
 *  (2) Drone FWD direction > (ACC -Y)
 *          
 *                 ^(x)
 *                 |                       
 *           (y)<--O(z)
 *       
 *  (3) Drone FWD direction ^ (ACC X)
 *          
 *                 ^(y)
 *                 |                       
 *              (z)O-->(x)              
 *
 *  (4) Drone FWD direction < (ACC Y)
 *                                       
 *              (z)O-->(y)
 *                 |
 *                 V(x)
 */


#include "sensor_data.h"
#include "steval_fcu001_v1_accelero.h"  
#include "steval_fcu001_v1_gyro.h"  
#include "steval_fcu001_v1_magneto.h"  
#include "steval_fcu001_v1_pressure.h"  


/*
 * This function read sensor data and prepare data for proper coordinate system
 * according to definition of COORDINATE_SYSTEM
 * The unit of each data are:
 *      Acc - mg
 *      Gyro - mdps
 *      Mag - mguass
 */
void ReadSensorRawData(void *ACC_handle, void *GYR_handle, void *MAG_handle, void *PRE_handle, AxesRaw_TypeDef *acc, AxesRaw_TypeDef *gyro, AxesRaw_TypeDef *mag, float *pre)
{
    int32_t t1;
    SensorAxes_t acc_temp_int16, gyro_temp_int16, mag_temp_int16;            /* Data Type int16_t */
    AxesRaw_TypeDef acc_temp, gyro_temp; 
    /* Data Type int32_t */
    // Read data is in mg unit
    BSP_ACCELERO_Get_Axes(ACC_handle, &acc_temp_int16);
    acc_temp.AXIS_X = (int32_t) acc_temp_int16.AXIS_X;                /* Casting data to int32_t */
    acc_temp.AXIS_Y = (int32_t) acc_temp_int16.AXIS_Y;
    acc_temp.AXIS_Z = (int32_t) acc_temp_int16.AXIS_Z;
    // Read data is in mdps unit
    BSP_GYRO_Get_Axes(GYR_handle, &gyro_temp_int16);
    gyro_temp.AXIS_X = (int32_t) gyro_temp_int16.AXIS_X;                /* Casting data to int32_t */
    gyro_temp.AXIS_Y = (int32_t) gyro_temp_int16.AXIS_Y;
    gyro_temp.AXIS_Z = (int32_t) gyro_temp_int16.AXIS_Z;
    // Read data is in mg unit
    if (USE_MAG_SENSOR){
        BSP_MAGNETO_Get_Axes(MAG_handle, &mag_temp_int16);
        mag->AXIS_X = (int32_t) mag_temp_int16.AXIS_X;
        mag->AXIS_Y = (int32_t) mag_temp_int16.AXIS_Y;
        mag->AXIS_Z = (int32_t) mag_temp_int16.AXIS_Z;
    }
    else
    {
        mag->AXIS_X = 0;
        mag->AXIS_Y = 0;
        mag->AXIS_Z = 0;
    }
    
    if (USE_PRESSURE_SENSOR)
        BSP_PRESSURE_Get_Press(PRE_handle, pre);
    else
        pre = 0;
    
    
    if (COORDINATE_SYSTEM == 1)
    {
        // convert acc
        t1 = acc->AXIS_X;
        acc->AXIS_X = acc->AXIS_Y;
        acc->AXIS_Y = -t1;
        // convert gyro
        t1 = gyro->AXIS_X;
        gyro->AXIS_X = gyro->AXIS_Y;
        gyro->AXIS_Y = -t1;
        // convert mag
        t1 = mag->AXIS_X;
        mag->AXIS_X = mag->AXIS_Y;
        mag->AXIS_Y = -t1;
    }
    else if (COORDINATE_SYSTEM == 2)
    {
        // No need to convert in this case
    }
    else if (COORDINATE_SYSTEM == 3)
    {
     
      acc->AXIS_X = -acc_temp.AXIS_Y;
      acc->AXIS_Y = acc_temp.AXIS_X;
      acc->AXIS_Z = acc_temp.AXIS_Z;
      
      gyro->AXIS_X = -gyro_temp.AXIS_Y;
      gyro->AXIS_Y = gyro_temp.AXIS_X;
      gyro->AXIS_Z = gyro_temp.AXIS_Z;
      
      // convert mag
      t1 = mag->AXIS_X;
      mag->AXIS_X = - mag->AXIS_Y;
      mag->AXIS_Y = t1;
    }
    else if (COORDINATE_SYSTEM == 4)
    {
        // convert acc
        acc->AXIS_X = - acc->AXIS_X;
        acc->AXIS_Y = - acc->AXIS_Y;
        // convert gyro
        gyro->AXIS_X = - gyro->AXIS_X;
        gyro->AXIS_Y = - gyro->AXIS_Y;
        // convert mag
        mag->AXIS_X = - mag->AXIS_X;
        mag->AXIS_Y = - mag->AXIS_Y;
    }
}
