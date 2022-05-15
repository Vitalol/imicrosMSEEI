/*
 * protocol_sensors.h
 *
 *  Created on: 8 may 2022
 *      Author: Victor
 */

#ifndef MAIN_PROTOCOL_SENSORS_H_
#define MAIN_PROTOCOL_SENSORS_H_

#include <stdint.h>

typedef enum {BUTTONS, ADC} TipoSensor;

typedef struct {
    uint8_t comando;
    union {	/* Ojo, que esto es una UNION y no una estructura */
    	uint16_t adc;
     } parametro;
} TipoPaquete;


#endif /* MAIN_PROTOCOL_SENSORS_H_ */
