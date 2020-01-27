
//#include "matrix.h"
#include "ahrs.h"
//#include "magnet_cal.h"
#include "basic_math.h"
#include "flight_control.h"


float offset[3];
float cor[3][3];

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float gx_off, gy_off, gz_off;
float mx_mag, my_mag, mz_mag;
float wbx = 0, wby = 0, wbz = 0;
float by = 1, bz = 0;
float exInt = 0, eyInt = 0, ezInt = 0;

int count;
int ahrs_init_flag = 0;
int acc_over = 0;
extern int16_t gTHR;
float ahrs_kp;

void ahrs_fusion_ag(AxesRaw_TypeDef_Float *acc, AxesRaw_TypeDef_Float *gyro, AHRS_State_TypeDef *ahrs)
{
  float axf,ayf,azf,gxf,gyf,gzf;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
  float q0q0, q0q1, q0q2, /*q0q3,*/ q1q1, /*q1q2,*/ q1q3, q2q2, q2q3, q3q3;
  float halfT;
 
  
  if(gTHR<MIN_THR)
  {
    ahrs_kp = AHRS_KP_BIG;
  }
  else
  {
    ahrs_kp = AHRS_KP_NORM;
  }

  axf = acc->AXIS_X;
  ayf = acc->AXIS_Y;
  azf = acc->AXIS_Z;

  // mdps convert to rad/s
  gxf = ((float)gyro->AXIS_X) * ((float)COE_MDPS_TO_RADPS);  
  gyf = ((float)gyro->AXIS_Y) * ((float)COE_MDPS_TO_RADPS);  
  gzf = ((float)gyro->AXIS_Z) * ((float)COE_MDPS_TO_RADPS); 


  // auxiliary variables to reduce number of repeated operations
  q0q0 = q0*q0;
  q0q1 = q0*q1;
  q0q2 = q0*q2;
  //q0q3 = q0*q3;
  q1q1 = q1*q1;
  //q1q2 = q1*q2;
  q1q3 = q1*q3;
  q2q2 = q2*q2;
  q2q3 = q2*q3;
  q3q3 = q3*q3;

  // normalise the accelerometer measurement
  norm = invSqrt(axf*axf+ayf*ayf+azf*azf);

  axf = axf * norm;
  ayf = ayf * norm;
  azf = azf * norm;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  ex = (ayf*vz - azf*vy);
  ey = (azf*vx - axf*vz);
  ez = (axf*vy - ayf*vx);

  // integral error scaled integral gain
  exInt = exInt + ex*AHRS_KI*SENSOR_SAMPLING_TIME;
  eyInt = eyInt + ey*AHRS_KI*SENSOR_SAMPLING_TIME;
  ezInt = ezInt + ez*AHRS_KI*SENSOR_SAMPLING_TIME;

  // adjusted gyroscope measurements
  gxf = gxf + ahrs_kp*ex + exInt;
  gyf = gyf + ahrs_kp*ey + eyInt;
  gzf = gzf + ahrs_kp*ez + ezInt;

  // integrate quaternion rate and normalise
  halfT = 0.5f*SENSOR_SAMPLING_TIME;
  q0 = q0 + (-q1*gxf - q2*gyf - q3*gzf)*halfT;
  q1 = q1 + (q0*gxf + q2*gzf - q3*gyf)*halfT;
  q2 = q2 + (q0*gyf - q1*gzf + q3*gxf)*halfT;
  q3 = q3 + (q0*gzf + q1*gyf - q2*gxf)*halfT;

  // normalise quaternion
  norm = invSqrt(q0q0 + q1q1 + q2q2 + q3q3); 
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;

  ahrs->q.q0 = q0;
  ahrs->q.q1 = q1;
  ahrs->q.q2 = q2;
  ahrs->q.q3 = q3;

}
