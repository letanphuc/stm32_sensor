#ifndef __KALMANFILTER_H__
#define __KALMANFILTER_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

typedef struct {
    float q; /*< process noise covariance */
    float r; /*< measurement noise covariance */
    float x; /*< value */
    float p; /*< estimation error covariance */
    float k; /*< kalman gain */
    uint8_t is_initialized; /*< is init or not */
} kalman_state;

void kalman_init(kalman_state* state, float q, float r, float p, float intial_value);
void kalman_update(kalman_state* state, float measurement);


#endif /* __KALMANFILTER_H__ */

