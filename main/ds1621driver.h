// EJERCICIO EF11
// MSEEI-UMA

#ifndef __DS1621_H__
#define __DS1621_H__

#include "driver/i2c.h"
#include "esp_err.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

// CONSTANTES
#define DS1621_BASE_ADDRESS         0x4F
#define DS1621_CONVERSION_TIME_MS   750
#define DS1621_BUS_FREE_TIME_MS     5
#define DS1621_WAIT_TIME_MS         500

// COMANDOS
#define DS1621_CMD_READ_TEMP        0xAA
#define DS1621_CMD_ACCESS_TH        0xA1
#define DS1621_CMD_ACCESS_TL        0xA2
#define DS1621_CMD_ACCESS_CONFIG    0xAC
#define DS1621_CMD_READ_COUNTER     0xA8
#define DS1621_CMD_READ_SLOPE       0xA9
#define DS1621_CMD_START_CONVERT    0xEE
#define DS1621_CMD_STOP_CONVERT     0x22

// CONTROL
#define DS1621_CTRL_POL_HIGH        0x02
#define DS1621_CTRL_POL_LOW         0x00
#define DS1621_CTRL_1SHOT           0x01
#define DS1621_CTRL_CONTINUOUS      0x00

//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES
//*****************************************************************************


extern esp_err_t ds1621_i2c_master_init(uint8_t address, i2c_port_t i2c_master_port);

extern esp_err_t ds1621_config(uint8_t config);

extern esp_err_t ds1621_read_counter(uint8_t* counter);

extern esp_err_t ds1621_read_slope(uint8_t* slope);

extern esp_err_t ds1621_start_conversion(void);

extern esp_err_t ds1621_stop_conversion(void);

extern esp_err_t ds1621_read_temperature_low_resolution(float* temperature);

extern esp_err_t ds1621_read_temperature_high_resolution(float* temperature);

//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES del timer
//*****************************************************************************
void ds1621_GetQueueHandle(QueueHandle_t *ptrQueue);
void ds1621_TimerChangePeriod(float seconds);
void ds1621_stop_timer();
void ds1621_start_timer();
#endif //  __DS1621_H__
