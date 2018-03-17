#ifndef _RC_H_
#define _RC_H_

#include "stm32f4xx_hal.h"
#include "config_drone.h"
#include "timer.h"
#include "quaternion.h"
#include "ahrs.h"

//#define REMOCON_PWM                     // External Remocon RX 4 channel PWM
#define REMOCON_BLE                   // BLE Remocon App


/* Definition for R/C Timing (1 LSB = 250us) */

// Definition for AIL(Roll) Channel
#define AIL_LEFT    8088
//#define AIL_MIDDLE  6030                /* Calibrated TGY-i6 remocon */
//#define AIL_MIDDLE  6120              /* Calibrated Radiolink remocon */
#define AIL_MIDDLE  5876              /* Calibrated Devo7E remocon */
#define AIL_RIGHT   4126
// Definition for ELE(Pitch) Channel
#define ELE_BOTTOM  4472
//#define ELE_MIDDLE  6050                /* Calibrated TGY-i6 remocon */
//#define ELE_MIDDLE  6316                /* Calibrated Radiolink remocon */
#define ELE_MIDDLE  6151                /* Calibrated Devo7E remocon */
#define ELE_TOP     8088
// Definition for THR Channel
//#define THR_BOTTOM  4337                /* Calibrated TGY-i6 remocon */
//#define THR_BOTTOM  4337                /* Calibrated Radiolink remocon */
#define THR_BOTTOM  4450                /* Calibrated Devo7E remocon */
#define THR_TOP     8080
// Definition for RUD(Yaw) Channel
#define RUD_LEFT    8088
//#define RUD_MIDDLE  6020                /* Calibrated TGY-i6 remocon */
//#define RUD_MIDDLE  6210                /* Calibrated Radiolink remocon */
#define RUD_MIDDLE  6414                /* Calibrated Devo7E remocon */
#define RUD_RIGHT   4216

#define RC_FULLSCALE        1800
#define RC_CAL_THRESHOLD    1200


#define PI  3.141592654f

// Maximum roll/pitch 35deg
//#define PITCH_MAX_DEG   45
//#define ROLL_MAX_DEG    45
#define PITCH_MAX_DEG   30
#define ROLL_MAX_DEG    30

//#define YAW_MAX_DEG     (180.0*SENSOR_SAMPLING_TIME)
#define YAW_MAX_DEG     (60.0f*SENSOR_SAMPLING_TIME)
#define YAW_MIN_RAD     0.0872

#define EULER_Z_TH      600


void init_remote_control(void);
void update_rc_data(int32_t idx);
void GetTargetEulerAngle(EulerAngleTypeDef *euler_rc, EulerAngleTypeDef *euler_ahrs);

// Below queue is for debug purpose
#define QUEUE_LENGTH        16

typedef struct
{
  int16_t               header;
  int16_t               tail;
  int16_t               length;
  int16_t               full;
  int16_t               empty;
  int16_t               buffer[QUEUE_LENGTH][2];
}Queue_TypeDef;


void init_queue(Queue_TypeDef *q);
void add_queue(Queue_TypeDef *q, int16_t idx, int16_t value);
int32_t get_queue(Queue_TypeDef *q, int16_t *idx, int16_t *value);


#endif /* _RC_H_ */
