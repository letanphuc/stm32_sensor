#ifndef __SRF_05_H__
#define __SRF_05_H__

#include "stm32f1xx_hal.h"

#define SRF05_ECHO_Pin GPIO_PIN_6
#define SRF05_ECHO_GPIO_Port GPIOA
#define SRF05_TRIGGER_Pin GPIO_PIN_7
#define SRF05_TRIGGER_GPIO_Port GPIOA
#define __HAL_RCC_GPIOSRF05_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE

typedef enum {
	ECHO_GOES_HIGH,
	ECHO_GOES_LOW,
	TIMEOUT, /*< time out 1 ms, if we are in state S1 and timeout, increase counter */
} event_t;

typedef enum {
	STATE_S0, /*< initialized state, wait for echo goes high */
	STATE_S1, /*< wait for echo goes low */
	STATE_S2 /*< if total time is less than 10 ms, wait for enoght 10 ms,
	 because we can not read sensor more than 100 sample/s */
} state_t;

struct _srf05 {
	state_t state; /*< current state, please refer design document */
	uint8_t new_data; /*< is there new data or not */
	uint8_t need_perform; /*< need main function run perform */
	float value; /*< current value */
	uint32_t timer_counter; /*< counter in us to calculate distance */
	uint32_t ms_counter; /*< counter in ms to calculate time out, data rate */
};

#endif /* __SRF_05_H__ */
