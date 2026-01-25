
#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <credentials.h>
#include "MCP23017Controller.h"
#include <Adafruit_ADS1X15.h>
#include "Relay.h"
#include <ArduinoJson.h>
#include "mqtt_helper.h"
#include "Motor.h"

enum motor_target_or_state
{
  open = 1,
  opening = 2,
  close = 3,
  closing = 4,
  stop = 5,
};

void gateToggle();
void gateOpen();
void gateClose();
void gateStop();
void gateError(uint32_t milliWatt);
bool gateRunning();