// EJERCICIO EF11
// MSEEI-UMA

#include "ds1621driver.h"

//****************************************************************************
//      VARIABLES
//****************************************************************************

static unsigned char ds1621_slave_address = 0;
static i2c_port_t ds1621_i2c_port;

static TimerHandle_t timerDS1621 = NULL;
static QueueHandle_t DS1621Queue;

/***
 *  Prototipos
 */

void vTimerDS1621Callback( TimerHandle_t pxTimer );
//****************************************************************************
//     FUNCIONES PROPIAS
//****************************************************************************

static esp_err_t ds1621_read_n_reg(uint8_t* regValue, uint8_t regAddress, uint8_t N)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddress, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, regValue, N, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, configTICK_RATE_HZ);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(0.2*configTICK_RATE_HZ);
    return ret;
};

static esp_err_t ds1621_write_one_command(uint8_t Command)
{
    unsigned char data_buffer[1];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = Command;
    i2c_master_write(cmd, data_buffer, 1, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, DS1621_WAIT_TIME_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(DS1621_BUS_FREE_TIME_MS / portTICK_RATE_MS);
    return ret;
};



//****************************************************************************
//     FUNCION EXAMEN
//****************************************************************************
static esp_err_t ds1621_write_two_command(uint8_t Command, uint8_t data_msb, uint8_t data_lsb)
{
    unsigned char data_buffer[3];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = Command;
    data_buffer[1] = data_msb;
    data_buffer[2] = data_lsb;
    i2c_master_write(cmd, data_buffer, 3, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, DS1621_WAIT_TIME_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(DS1621_BUS_FREE_TIME_MS / portTICK_RATE_MS);
    return ret;
};

esp_err_t ds1621_write_TH(float temperature)
{
       uint8_t data_buffer[2];
       int16_t value;
// Conversión de la temperatura a valores a escribir con el comando ACCESS_TH
       value= (int16_t)(temperature*2.0);
       data_buffer[0]=(value>>1)&0xFF;  // 8 bits más significativos
       data_buffer[1]= (value & 0x01)<<7;  // noveno bit
       esp_err_t ret = ds1621_write_two_command(DS1621_CMD_ACCESS_TH ,data_buffer[0], data_buffer[1]);

      return ret;
};

esp_err_t ds1621_read_TH(float* TH)
{
	uint8_t TH_buffer[2] = {0};
	esp_err_t ret = ds1621_read_n_reg(TH_buffer, DS1621_CMD_ACCESS_TH, 2);
	* TH = (TH_buffer[1]==0 ? (float) TH_buffer[0] : (float) TH_buffer[0] + 0.5);

	return ret; 	  // Definir como en la función ds1621_config()
	 	 	 	  // una variable de retorno de la función i2c_master_cmd_begin()
};


//****************************************************************************
//     FUNCIONES
//****************************************************************************

esp_err_t ds1621_i2c_master_init(uint8_t address, i2c_port_t i2c_master_port)
{
    ds1621_slave_address = DS1621_BASE_ADDRESS | (address & 0x7);
    ds1621_i2c_port = i2c_master_port;
    return 0;
}

esp_err_t ds1621_config(uint8_t config)
{
    unsigned char data_buffer[2];
    if(ds1621_slave_address == 0)
        return ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ds1621_slave_address << 1) | I2C_MASTER_WRITE, true);
    data_buffer[0] = DS1621_CMD_ACCESS_CONFIG;
    data_buffer[1] = config;
    i2c_master_write(cmd, data_buffer, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ds1621_i2c_port, cmd, DS1621_WAIT_TIME_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(DS1621_BUS_FREE_TIME_MS / portTICK_RATE_MS);

	DS1621Queue = xQueueCreate(10, sizeof(float));
    timerDS1621 = xTimerCreate("timerDS1621", configTICK_RATE_HZ, pdTRUE, ( void * ) 1, vTimerDS1621Callback);
	 if (timerDS1621==NULL){
		 printf("No se pudo crear el timer DS1621");
	 }

    return ret;
};

esp_err_t ds1621_read_temperature_low_resolution(float* temperature)
{
	uint8_t temp[2] = {0};
	esp_err_t ret = ds1621_read_n_reg(temp, DS1621_CMD_READ_TEMP, 2);
	* temperature = (temp[1]==0 ? (float) temp[0] : (float) temp[0] + 0.5);
	return ret; 	  // Definir como en la función ds1621_config()
	 	 	 	  // una variable de retorno de la función i2c_master_cmd_begin()
};

esp_err_t ds1621_read_counter(uint8_t* counter)
{
	esp_err_t ret = ds1621_read_n_reg(counter, DS1621_CMD_READ_COUNTER, 1);
	return ret;
    //return ret; 	// Definir como en la función ds1621_config()
	 	 	 	 	// una variable de retorno de la función i2c_master_cmd_begin()
};

esp_err_t ds1621_read_slope(uint8_t* slope)
{
	esp_err_t ret = ds1621_read_n_reg(slope, DS1621_CMD_READ_SLOPE, 1);
	return ret;   // Definir como en la función ds1621_config()
	 	 	 	  // una variable de retorno de la función i2c_master_cmd_begin()
};

esp_err_t ds1621_start_conversion(void)
{
	esp_err_t ret = ds1621_write_one_command(DS1621_CMD_START_CONVERT);
	return ret; 	// Definir como en la función ds1621_config()
	 	 	 	  	// una variable de retorno de la función i2c_master_cmd_begin()
};

esp_err_t ds1621_stop_conversion(void)
{
	esp_err_t ret = ds1621_write_one_command(DS1621_CMD_STOP_CONVERT);
	return ret; 	  // Definir como en la función ds1621_config()
	 	 	 	  // una variable de retorno de la función i2c_master_cmd_begin()
};

esp_err_t ds1621_read_temperature_high_resolution(float* temperature)
{
	uint8_t counter = 0;
	uint8_t slope 	= 0;
	uint8_t temp[2] = {0};


	ds1621_read_counter(&counter);
	ds1621_read_slope(&slope);

	esp_err_t ret = ds1621_read_n_reg(temp, DS1621_CMD_READ_TEMP, 2);
	* temperature = (temp[0]-0.25+((float)(slope-counter)/(float)slope));
	return ret; 	// Definir como en la función ds1621_config()
	 	 	 	  // una variable de retorno de la función i2c_master_cmd_begin()
};

void ds1621_start_timer(){
	xTimerStart(timerDS1621, portMAX_DELAY);
}

void ds1621_stop_timer(){
	xTimerStop(timerDS1621, portMAX_DELAY);
}

void ds1621_TimerChangePeriod(float seconds){
	ds1621_stop_timer();
	xTimerChangePeriod(timerDS1621,  seconds * configTICK_RATE_HZ, portMAX_DELAY);
	ds1621_start_timer();
}


void ds1621_GetQueueHandle(QueueHandle_t *ptrQueue){
	 *ptrQueue = DS1621Queue;
}
void vTimerDS1621Callback( TimerHandle_t pxTimer ){
	float grados;
	ds1621_read_temperature_high_resolution(&grados);
	xQueueSend(DS1621Queue, &grados, 0); //No tiene sentido esperar en el envio de un lectura, si se solapa se prefiere la mas reciente
}

