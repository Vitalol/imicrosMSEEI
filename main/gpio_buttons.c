/*
 * gpio_buttons.c
 *
 *  Created on: 2 may 2022
 *      Author: Victor
 */

#include "gpio_buttons.h"
//#include "mqtt.h"
btnState Button1;
btnState Button2;


// Cola de mensaje
QueueHandle_t buttonQueue;
#define BTN_QUEUE_SIZE 10
//Prototipos
static void IRAM_ATTR Button1PressedISR(void* arg) // Boton 27
{

	Button1.buttonId = 1;
	Button1.state = (gpio_get_level(BOTON_1) > 0);

	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	xQueueSendFromISR(buttonQueue, &Button1, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

static void IRAM_ATTR Button2PressedISR(void* arg) // boton 35
{

	Button2.buttonId = 2;
	Button2.state = (gpio_get_level(BOTON_2) > 0);
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	xQueueSendFromISR(buttonQueue, &Button2, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GB_initGPIO(void){
	gpio_config_t io_conf; // Solucion EF41
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << BOTON_1) | (1ULL << BOTON_2);
	io_conf.pull_down_en = 0; //disable pull-down mode
	io_conf.pull_up_en = 0; //disable pull-up mode
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BOTON_1, Button1PressedISR, (void*) BOTON_1);
	gpio_isr_handler_add(BOTON_2, Button2PressedISR, (void*) BOTON_2);
	gpio_intr_disable(BOTON_1);
	gpio_intr_disable(BOTON_2);

}

void GB_initButtons(void){
	// iniciar gpios
	GB_initGPIO();
	// iniciar cola de mensajes
	buttonQueue = xQueueCreate(  10, sizeof(struct btnState) );
}
void GB_getQueue(QueueHandle_t *ptrQueue){
	//para async
	*ptrQueue = buttonQueue;
}
uint8_t GB_getButtonState(uint8_t button){
	return gpio_get_level(button);
}

