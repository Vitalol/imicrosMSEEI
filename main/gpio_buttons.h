/*
 * gpio_buttons.h
 *
 *  Created on: 2 may 2022
 *      Author: Victor
 */

#ifndef MAIN_GPIO_BUTTONS_H_
#define MAIN_GPIO_BUTTONS_H_
#include <stdbool.h>
#include <stdint.h>

//Include FreeRTOS headers
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#define BOTON_1 	27
#define BOTON_2 	35

extern void GB_initGPIO(void);
void GB_initButtons(void);
uint8_t GB_getButtonState(uint8_t button);
void GB_getQueue(QueueHandle_t *ptrQueue);

typedef struct btnState{
	uint8_t buttonId;
	bool state;
}btnState;

#endif /* MAIN_GPIO_BUTTONS_H_ */
