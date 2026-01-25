#ifndef RELAIS_H
#define RELAIS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "MCP23017Controller.h"
#include "mqtt_helper.h"

#define RELAY_ON 0
#define RELAY_OFF 1

class Relay
{
private:
  // store pointer to avoid copying the MCP controller object
  MCP23017Controller *_mcp;
  uint8_t _index;
  uint8_t _mcpPin;
  char *_name;
  uint16_t _watt;
  unsigned long _lastStartTime = 0;
  unsigned long _lastStopTime = 0;
  uint16_t _timeoutSeconds = 0;
  unsigned long _timeout = 0;

public:
  // accept MCP by reference to make intent clear
  Relay(MCP23017Controller &mcp, uint8_t index, char *name, uint16_t watt, uint8_t timeoutMinutes);
  void set(bool state, unsigned long now);
  void triggerOrSet(bool state, unsigned long now);
  bool read();
  void mqttPublishState(bool full);
  void loop(unsigned long now);
  void triggerTimeout(unsigned long now);
  // Accessors for coordination logic
  unsigned long getLastStopTime();
  unsigned long getLastStartTime();
  uint8_t getIndex();
};

#endif
