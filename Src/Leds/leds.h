#ifndef __LEDS_H__
#define __LEDS_H__

#include "stm32f1xx_hal.h"

typedef enum
{
	LED_RED,
	LED_GREEN,
	MAX_LEDS
} led_id_t;


#define SIGNAL_LED_Pin GPIO_PIN_0
#define SIGNAL_LED_GPIO_Port GPIOA
#define __HAL_RCC_PORTLEDS_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE

void led_init(void);
void led_toggle(led_id_t led);

#endif /* __SENSOR_H__ */
