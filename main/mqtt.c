//Include standard lib headers
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


//Include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

//Include ESP submodules headers.
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

//Include own project  headers
#include "gpio_leds.h"
#include "gpio_buttons.h"
#include "gpio_adc.h"
#include "mqtt.h"
#include "ds1621driver.h"

//FROZEN JSON parsing/formatting library header
#include "frozen.h"


//****************************************************************************
//      VARIABLES GLOBALES STATIC
//****************************************************************************

static const char *TAG = "MQTT_CLIENT";
static esp_mqtt_client_handle_t client=NULL;
static TaskHandle_t senderTaskHandler=NULL;
static TaskHandle_t sensorsTaskHandler=NULL;
static QueueHandle_t mqttSendQueue = NULL; // Cola de mensajes hacia sender


//****************************************************************************
// Funciones.
//****************************************************************************
static void mqtt_sensorSubscription(void *pvParameters);
static void mqtt_sender_task(void *pvParameters);

void mqtt_sed(mqttMessage* ptrMsg){
	xQueueSend(mqttSendQueue, (void *) &(ptrMsg), portMAX_DELAY);
}


// callback that will handle MQTT events. Will be called by  the MQTT internal task.
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUBSCRIBE_BASE, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            mqttSendQueue = xQueueCreate(  10, sizeof(struct mqttMessage *) ); // VMTO la cola va a contener punteros a las cadenas de textos que forman el msj mqtt
            xTaskCreate(mqtt_sender_task, "mqtt_sender", 4096, NULL, 5, &senderTaskHandler); //Crea la tarea MQTT sender
            xTaskCreate(mqtt_sensorSubscription, "mqtt_sensors", 4096, NULL, 0, &sensorsTaskHandler); // VMTO Crea la tarea para subscribirse a los sensores/gpios

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            //Deberíamos destruir la tarea que envía....
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
        {
        	//Para poder imprimir el nombre del topic lo tengo que copiar en una cadena correctamente terminada....
        	char topic_name[event->topic_len+1]; //Esto va a la pila y es potencialmente peligroso si el nombre del topic es grande....
        	mqttMessage * ptrAnswer = pvPortMalloc(sizeof(mqttMessage));

        	struct json_out out1 = JSON_OUT_BUF(ptrAnswer->buffer, sizeof(ptrAnswer->buffer));
        	strncpy(topic_name,event->topic,event->topic_len);
        	topic_name[event->topic_len]=0; //añade caracter de terminacion al final.

        	ESP_LOGI(TAG, "MQTT_EVENT_DATA: Topic %s",topic_name);
        	bool booleano;
        	uint8_t rgbLeds[3] = {0};
        	float tempPeriod;

        	// RED LED
        	if(json_scanf(event->data, event->data_len, "{ redLed: %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "redLed: %s", booleano ? "true":"false");

        		gpio_set_level(BLINK_GPIO_1, booleano);
        	}
        	// GREEN LED
        	if(json_scanf(event->data, event->data_len, "{ greenLed: %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "greenLed: %s", booleano ? "true":"false");

        		gpio_set_level(BLINK_GPIO_2, booleano);
        	}
        	// BLUE LED
        	if(json_scanf(event->data, event->data_len, "{ blueLed: %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "blueLed: %s", booleano ? "true":"false");

        		gpio_set_level(BLINK_GPIO_3, booleano);
        	}
        	// PING
        	if(json_scanf(event->data, event->data_len, "{ pingRequest : %B }", &booleano)==1)
        	{
        		if(booleano){
					ESP_LOGI(TAG, "ping: %s", booleano ? "true":"false");
					// formar mensaje json
					json_printf(&out1,"{ ping : %B }",booleano);
					// Enviar mensaje a la cola
					xQueueSend(mqttSendQueue, (void *) &(ptrAnswer), portMAX_DELAY);
        		}
        	}
        	// SONDEO
        	if(json_scanf(event->data, event->data_len, "{ sondeo : %B }", &booleano)==1)
        	{
        		if(booleano){
        		ESP_LOGI(TAG, "sondeo: %s", booleano ? "true":"false");
        		bool btn1 = (GB_getButtonState(BOTON_1) > 0);
        		bool btn2 = (GB_getButtonState(BOTON_2) > 0);
        		json_printf(&out1,"{ button1 : %B, button2: %B }", btn1, btn2);
        		xQueueSend(mqttSendQueue, (void *) &(ptrAnswer), portMAX_DELAY);
        		}

        	}
        	// Modo PWM
        	if(json_scanf(event->data, event->data_len, "{ ledPWM : %B }", &booleano)==1)
        	{
				ESP_LOGI(TAG, "ledPWM: %s", booleano ? "true":"false");
        		if(booleano){
        			GL_initLEDC();
        			ESP_LOGI(TAG, "Inicio modo PWM");
        		}else{
        			GL_stopLEDC();
        			ESP_LOGI(TAG, "Parada modo PWM");
        		}

        	}
        	// LEDS PWM
        	if(json_scanf(event->data, event->data_len, "{ redLedPWM:%d, greenLedPWM:%d, blueLedPWM:%d}", &rgbLeds[0], &rgbLeds[1], &rgbLeds[2])>=1){
        		ESP_LOGI(TAG, "{ redLedPWM : %d, greenLedPWM : %d, blueLedPWM : %d}", rgbLeds[0], rgbLeds[1], rgbLeds[2]);
        		GL_setRGB(rgbLeds);
        	}
        	// ASYNC
        	if(json_scanf(event->data, event->data_len, "{ ASYNC:%B }", &booleano)==1){

        		ESP_LOGI(TAG, "ASYNC: %s", booleano ? "true":"false");
        		if(booleano){
        			gpio_intr_enable(BOTON_1);
        			gpio_intr_enable(BOTON_2);

        		}else{
        			gpio_intr_disable(BOTON_1);
        			gpio_intr_disable(BOTON_2);

        		}

        	}
        	// TEMP
        	// Iniciar
        	if(json_scanf(event->data, event->data_len, "{ tempRequest : %B }", &booleano)==1)
        	{
        		ESP_LOGI(TAG, "tempRequest: %s", booleano ? "true":"false");
        		if(booleano){
        			ds1621_start_timer();
        		}else{
        			ds1621_stop_timer();
        		}

        	}
        	// Cambiar periodo

        	if(json_scanf(event->data, event->data_len, "{ tempPeriod : %f }", &tempPeriod)==1)
        	{
        		ESP_LOGI(TAG, "tempPeriod: %f", tempPeriod);
        		ds1621_TimerChangePeriod(tempPeriod);
        	}


        }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}


static void mqtt_sender_task(void *pvParameters)
{
	mqttMessage* msj; //"buffer" para guardar el mensaje. Me debo asegurar que quepa...

	while (1)
	{
		xQueueReceive(mqttSendQueue, &msj, portMAX_DELAY);
		ESP_LOGI(TAG,"%s",msj->buffer);

		// Esperar la cola de mensaje

		int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_PUBLISH_BASE, msj->buffer, 0, 0, 1); //al utilizar la biblioteca Frozen, buffer es una cadena correctamente terminada con el caracter 0.
																								//así que puedo no indicar la longitud (cuarto parámetro vale 0).
																								// Si fuese un puntero a datos binarios arbitrários, tendría que indicar la longitud de los datos en el cuarto parámetro de la función.
		ESP_LOGI(TAG, "sent successful, msg_id=%d: %s", msg_id, msj->buffer);
		vPortFree(msj);
	}
}

esp_err_t mqtt_app_start(const char* url)
{
	esp_err_t error;

	if (client==NULL){

		esp_mqtt_client_config_t mqtt_cfg = {
				.uri = MQTT_BROKER_URL,
				.lwt_topic = MQTT_TOPIC_LAST_WILL,
				.lwt_msg = "{\"ping\": false}",
				.lwt_retain = 1,
				.keepalive = 20
		};
		if(url[0] != '\0'){
			mqtt_cfg.uri= url;
		}


		client = esp_mqtt_client_init(&mqtt_cfg);
		esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
		error=esp_mqtt_client_start(client);
		return error;
	}
	else {

		ESP_LOGE(TAG,"MQTT client already running");
		return ESP_FAIL;
	}
}

/**
 * Esta tarea se subscribe a las colas de los diferentes componentes
 * analiza las publicaciones y las envia por mqtt
 */
static void mqtt_sensorSubscription(void *pvParameters){
	// Variables del set de colas
	QueueSetHandle_t xQueueSet;
	QueueSetMemberHandle_t xActivatedMember;

	// Obtener colas
	QueueHandle_t adcQueueHandle; // length 10
	QueueHandle_t btnQueuehandle; // length 10
	QueueHandle_t ds1621Queuehandle; // length 10

	gpio_adcGetQueueHandle(&adcQueueHandle);
	GB_getQueue(&btnQueuehandle);
	ds1621_GetQueueHandle(&ds1621Queuehandle);

	xQueueSet = xQueueCreateSet(10+10+10);

	xQueueAddToSet( adcQueueHandle, xQueueSet );
	xQueueAddToSet( btnQueuehandle, xQueueSet );
	xQueueAddToSet( ds1621Queuehandle, xQueueSet );

	for(;;){
		 xActivatedMember = xQueueSelectFromSet( xQueueSet, 15000 / portTICK_PERIOD_MS ); // ADC deberia enviar cada 10 segundos, si en 15 segundos no ha pasado nada algo está roto

		 mqttMessage * ptrAnswer = pvPortMalloc(sizeof(mqttMessage));
		 struct json_out out1 = JSON_OUT_BUF(ptrAnswer->buffer, sizeof(ptrAnswer->buffer));

		 if( xActivatedMember == adcQueueHandle )
	        {
			 	uint16_t adcValue;
	            xQueueReceive( xActivatedMember, &adcValue, 0 );
	            // ESP_LOGI(TAG, "ADC Value %d", adcValue);  // Debug
				// formar mensaje json
				json_printf(&out1,"{ ADC : %d }", adcValue);
				// Enviar mensaje a la cola
				xQueueSend(mqttSendQueue, (void *) &(ptrAnswer), portMAX_DELAY);


	        }
	        else if( xActivatedMember == btnQueuehandle )
	        {
	        	btnState btnsState;
	            xQueueReceive( xActivatedMember, &btnsState, 0 );
	            if(btnsState.buttonId == 1)
	            	json_printf(&out1,"{ button1 : %B }", btnsState.state);
	            if(btnsState.buttonId == 2)
	            	json_printf(&out1,"{ button2 : %B }", btnsState.state);
	            xQueueSend(mqttSendQueue, (void *) &(ptrAnswer), portMAX_DELAY);

	        }else if (xActivatedMember == ds1621Queuehandle) {
	        	float grados;
	        	xQueueReceive( xActivatedMember, &grados, 0 );
	        	json_printf(&out1,"{ temp : %f }", grados);
	        	xQueueSend(mqttSendQueue, (void *) &(ptrAnswer), portMAX_DELAY);
	        }
	        else
	        {
	        	ESP_LOGI(TAG, "No sensor events in 15 seconds");
	        }

	}
}
