#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f1xx_hal_def.h"

#define MINIMUM_DELAY 30
#define CALIB_TIMER 999

/**
 * a struct to communicate with main function.
 */
typedef struct _sensor
{
	uint8_t * type; /*!< type of sensor like "Ultrasonic" or "Temp" ... */
	void (*init)(void); /*!< initializing function will be called in main */
	uint8_t (*new_data)(void); /*!< check if new data is available or not. Use to reply to Raspberry query command */
	float (*get_data)(void); /*!< get the latest data */
	uint8_t (*perform)(void); /*!< will be call in main loop, it is need to perform filter because this
							activity will take time, can not call in interrupt function */
} sensor_t;

extern const sensor_t sensor;
extern void (*timer_callback) (void);
extern void (*gpio_callback) (void);

void StartSensorTask(void const * argument);

#endif /* __SENSOR_H__ */
