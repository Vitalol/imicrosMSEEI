/*
 * gpio_adc.h
 *
 *  Created on: 3 may 2022
 *      Author: Victor
 */

#ifndef MAIN_GPIO_ADC_H_
#define MAIN_GPIO_ADC_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"


float GPIO_adcRead(void);
void gpio_adcInit(void);
void gpio_adcChangePeriod(float seconds);
void gpio_adcGetQueueHandle(QueueHandle_t *ptrQueue);
#endif /* MAIN_GPIO_ADC_H_ */
