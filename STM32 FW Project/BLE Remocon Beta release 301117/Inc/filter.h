#ifndef _FILTER_H_
#define _FILTER_H_

#include "ahrs.h"

void two_order_IIR(AxesRaw_TypeDef *y, AxesRaw_TypeDef *y_p1, AxesRaw_TypeDef *y_p2, AxesRaw_TypeDef *x, AxesRaw_TypeDef *x_p1, AxesRaw_TypeDef *x_p2);

#endif

