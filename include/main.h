#include <Arduino.h>
#include <MCP3XXX.h>
#include <AdcSwitch.h>
#include <Switch.h>
#include <ShiftOutput.h>
#include <Motor.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <credentials.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6
#define CS_ADC_PIN D0
#define CS_SHIFT_PIN D8

#define MOTOR0_0 D2
#define MOTOR0_1 D1
#define MOTOR0_SPEED_CHANNEL 0
#define MOTOR1_0 D4
#define MOTOR1_1 D3
#define MOTOR1_SPEED_CHANNEL 1

#define MILLIS_BETWEEN_TOGGLE 1000
#define MILLIS_AFTER_POWER_SUPPLY_ACTIVATED 800
#define MILLIS_POWER_SUPPLY_SHUTDOWN_TIME 1000

#define SHIFT_PIN_RELAY_4_POWER_SUPPLY 0
#define SHIFT_PIN_RELAY_3_NC 1
#define SHIFT_PIN_RELAY_2_NC 2
#define SHIFT_PIN_RELAY_1_NC 3
#define SHIFT_PIN_LED_LEARN 4
#define SHIFT_PIN_LED_ERROR 5
#define SHIFT_PIN_LED_0 6
#define SHIFT_PIN_LED_1 7

// toggle gate state from button, radio or wifi
void toggleGateState();

// press for learning mode
void learnPressed();

// not connected
void btn2Callback();
void btn3Callback();

// if a motor has an error
void errorCallback();

void debugInfos(bool mqtt);

void mqtt_reconnect();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void mqtt_send_status(boolean full);
void mqtt_send_adebar_carport_gate(boolean full);
void mqtt_send_adebar_carport_ip_address(boolean full);