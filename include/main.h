#include <Arduino.h>
#include <MCP3XXX.h>
#include <AdcSwitch.h>
#include <AdcTribleSwitch.h>
#include <ShiftOutput.h>
#include <Motor.h>
#include <LedDriver.h>
#include "wifi_mqtt.h"

#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6
#define CS_ADC_PIN D8
#define CS_SHIFT_PIN D0

// MOTOR0 links von Hause gesehen
#define MOTOR_L_OPEN D1
#define MOTOR_L_CLOSE D2
#define MOTOR_L_SPEED_CHANNEL 1
#define MOTOR_L_HALL_CHANNEL 3

// MOTOR0 rechts von Hause gesehen
#define MOTOR_R_OPEN D3
#define MOTOR_R_CLOSE D4
#define MOTOR_R_SPEED_CHANNEL 0
#define MOTOR_R_HALL_CHANNEL 4

#define MILLIS_BETWEEN_TOGGLE 1000

#define SHIFT_PIN_RELAY_4_POWER_SUPPLY 2
#define SHIFT_PIN_RELAY_3_NC 3
#define SHIFT_PIN_RELAY_2_NC 4
#define SHIFT_PIN_RELAY_1_NC 5

enum motor_target_or_state
{
  open = 1,
  opening = 2,
  close = 3,
  closing = 4,
  stop = 5,
  unknown = 6,
};

// toggle gate state from button, radio or wifi
void gateToggle();
void gateOpen();
void gateClose();
void gateStop();
void gateError(uint32_t milliWatt);
bool gateRunning();

void btn0Callback();
void btn1Callback();
void btn2Callback();

void radioBCallback();
void radioCCallback();
void radioDCallback();

void mqttCallback(char *topic, byte *payload, unsigned int length);
void mqttSendAdebarCarportGate(bool full);
void mqttSendAdebarCarportIpAddress(bool full);
