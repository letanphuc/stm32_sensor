#ifndef __KALMANFILTER_H__
#define __KALMANFILTER_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

typedef struct {
    double q; /*< process noise covariance */
    double r; /*< measurement noise covariance */
    double x; /*< value */
    double p; /*< estimation error covariance */
    double k; /*< kalman gain */
    uint8_t is_initialized; /*< is init or not */
} kalman_state;

void kalman_init(kalman_state* state, double q, double r, double p, double intial_value);
void kalman_update(kalman_state* state, double measurement);


#endif /* __KALMANFILTER_H__ */

