#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx_hal.h"

#define MOTOR_DC
//#define MOTOR_ESC


#ifdef MOTOR_DC
  #define MOTOR_MAX_PWM_VALUE     1900.0f    /* DC motor configuration */
  #define MOTOR_MIN_PWM_VALUE     0.0f     /* DC motor configuration */
#endif
#ifdef MOTOR_ESC
  #define MOTOR_MAX_PWM_VALUE     1700.0f    /* External ESC configuration */
  #define MOTOR_MIN_PWM_VALUE     850.0f     /* 850 for 1msec (Fine tuned for Freq 400Hz), FCU ON motors not ARMED*/
#endif

typedef struct
{
  float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
}MotorControlTypeDef;

void set_motor_pwm(MotorControlTypeDef *motor_pwm);
void set_motor_pwm_zero(MotorControlTypeDef *motor_pwm);

#endif /* _MOTOR_H_ */
