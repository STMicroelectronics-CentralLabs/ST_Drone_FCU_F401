#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "stm32f4xx_hal.h"
#include "basic_math.h"

// Type define for quaternion
typedef struct
{
    float q0, q1, q2, q3;
}QuaternionTypeDef;

// Type define for Euler angle
typedef struct
{
    float thx, thy, thz;
}EulerAngleTypeDef;


void QuaternionNorm(QuaternionTypeDef *q);
void QuaternionMult(QuaternionTypeDef *qa, QuaternionTypeDef *qb, QuaternionTypeDef *qo);
void QuaternionRotation(QuaternionTypeDef *qr, QuaternionTypeDef *qv, QuaternionTypeDef *qo);
void QuaternionConj(QuaternionTypeDef *qa, QuaternionTypeDef *qo);
void QuaternionToEuler(QuaternionTypeDef *qr, EulerAngleTypeDef *ea);

#define MAX_RAD    1.5


#endif /* _QUATERNION_H_ */
