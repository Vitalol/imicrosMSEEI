# Ejemplo de partida bloque 4

Este ejemplo est� basado y combina varios ejemplos de Espressiff, junto con c�digo propio y blibliotecas de terceros. 

Implementa un servidor TCP con un protocolo "binario" sencillo (tramas de tama�o constante). De momento permite modificar el estado del LED rojo 
desde un cliente de interfaz gr�fico implementado con Qt.

Tambien implementa una consola de comandos de terminal y una tarea que gestiona un reloj por el LCD.

Sobre el se monta un ejemplo de uso de MQTT en el que se utiliza cadenas json para intercambiar mensajes con una aplicación QT