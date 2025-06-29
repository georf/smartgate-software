#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <Arduino.h>
#include <credentials.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <LedDriver.h>

// Wiederverbindungsinterval
#define RECONNECTING_INTERVAL 10000

// WIFI und MQTT starten
void wifiAndMqttStartup();

// WIFI und MQTT regelmäßig
void wifiAndMqttLoop(const unsigned long currentMillis);

// Wifi verbinden
void connectToWiFi();

// MQTT verbinden
void connectToMqtt();

// MQTT senden
bool mqttPublish(const char *topic, const char *message);

// Alle Status senden
void mqttSendStatus(boolean full);

// in main.cpp Callback für Subscriptions
extern void mqttCallback(char *topic, byte *payload, unsigned int length);
extern void mqttSendAdebarCarportGate(bool full);
extern void mqttSendAdebarCarportIpAddress(bool full);

extern LedsHandler leds; // LEDs Interface

#endif