#ifndef __MQTT_H__
#define __MQTT_H__

//*****************************************************************************
//      DEFINICIONES
//*****************************************************************************

#define MQTT_BROKER_URL      		CONFIG_EXAMPLE_MQTT_BROKER_URI
#define MQTT_TOPIC_SUBSCRIBE_BASE   CONFIG_EXAMPLE_MQTT_TOPIC_SUBSCRIBE_BASE
#define MQTT_TOPIC_PUBLISH_BASE  	CONFIG_EXAMPLE_MQTT_TOPIC_PUBLISH_BASE
#define MQTT_TOPIC_LAST_WILL		CONFIG_EXAMPLE_MQTT_TOPIC_PUBLISH_BASE "/LastWill"



//*****************************************************************************
//      Estructuras
//*****************************************************************************
// Estructura utilizada para enviar mensaje mqtt
typedef struct mqttMessage{
	char buffer[100];
}mqttMessage;


//*****************************************************************************
//      PROTOTIPOS DE FUNCIONES
//*****************************************************************************

esp_err_t mqtt_app_start(const char* url);

/** API para faciitar el envio de mensajes mqtt
 *  encola el mensaje para que la tarea de publicación los publique
 */
void mqtt_sed(mqttMessage* msj);

#endif //  __MQTT_H__
