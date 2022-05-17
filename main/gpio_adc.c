/*
 * gpio_adc.c
 *
 *  Created on: 3 may 2022
 *      Author: Victor
 */

#include "gpio_adc.h"

#include "driver/adc.h"



TimerHandle_t timerADC = NULL;
QueueHandle_t adcQueue;

void vTimerADCCallback( TimerHandle_t pxTimer )
{
	uint16_t adcValue= (int16_t) adc1_get_raw(ADC_CHANNEL_6);
	xQueueSend(adcQueue, &adcValue, 0);
}
 void gpio_adcGetQueueHandle(QueueHandle_t *ptrQueue){
	 *ptrQueue = adcQueue;
}
float gpio_adcRead(void)
{
// Unused
	int muestra;
	float voltaje;
	muestra=adc1_get_raw(ADC_CHANNEL_6); //Toma la muestra
	voltaje=(float)muestra*3.6/4096.0;	//Pasa a voltios (muestra de 12 bits con rango de 3,6V)

	return voltaje;
}

void gpio_adcChangePeriod(float seconds){
	xTimerStop(timerADC, portMAX_DELAY);
	xTimerChangePeriod(timerADC,  seconds * configTICK_RATE_HZ, portMAX_DELAY);
	xTimerStart(timerADC, portMAX_DELAY);
}

void gpio_adcInit(void){
	//Inicializa el ADC... 12bits con rango de 3,6V (atenuaci�n 11dB)
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11); //GPIO34 if ADC1. Rango de 3,6 V
	adcQueue = xQueueCreate(10, sizeof(uint16_t));
    timerADC = xTimerCreate("TmrADC", configTICK_RATE_HZ, pdTRUE, ( void * ) 1, vTimerADCCallback);
    gpio_adcChangePeriod(5);
    xTimerStart(timerADC,portMAX_DELAY);
	 if (timerADC==NULL){
		 printf("No se pudo crear el timer ADC");
	 }

}


