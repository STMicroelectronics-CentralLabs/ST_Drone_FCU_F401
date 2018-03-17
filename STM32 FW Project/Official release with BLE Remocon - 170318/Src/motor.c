#include "motor.h"

/*                                  Note
 * The PWM is handled by TIM4. 
 * In case of DC motor configuration:
 * - the master clock for TIM4 is 1MHz
 * - the counter counts up to 2000, result in 2ms of PWM period (500Hz)
 * - the PWM pulse width data can to 0~1999, coresponding to 0~100% duty cycle
*/

extern TIM_HandleTypeDef htim4;

/*
 * Setup the driving power for 4 motors. p1~p4 data range is 0~1999, which equals
 * to 0~100% duty cycle (for DC motor configuration)
 */
void set_motor_pwm(MotorControlTypeDef *motor_pwm)
{
  if (motor_pwm->motor1_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR1 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor1_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR1 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR1 = (uint32_t) motor_pwm->motor1_pwm; 
  
  if (motor_pwm->motor2_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR2 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor2_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR2 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR2 = (uint32_t) motor_pwm->motor2_pwm;
  
  if (motor_pwm->motor3_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR3 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor3_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR3 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR3 = (uint32_t) motor_pwm->motor3_pwm;
  
  if (motor_pwm->motor4_pwm >= MOTOR_MAX_PWM_VALUE)
    htim4.Instance->CCR4 = MOTOR_MAX_PWM_VALUE;
  else if (motor_pwm->motor4_pwm <= MOTOR_MIN_PWM_VALUE)
    htim4.Instance->CCR4 = MOTOR_MIN_PWM_VALUE;
  else
    htim4.Instance->CCR4 = (uint32_t) motor_pwm->motor4_pwm;
}


void set_motor_pwm_zero(MotorControlTypeDef *motor_pwm)
{
  motor_pwm->motor1_pwm = 0;
  motor_pwm->motor2_pwm = 0;
  motor_pwm->motor3_pwm = 0;
  motor_pwm->motor4_pwm = 0;
}

