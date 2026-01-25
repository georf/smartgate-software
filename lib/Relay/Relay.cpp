#include "Relay.h"

Relay::Relay(MCP23017Controller &mcp, uint8_t index, char *name, uint16_t watt, uint8_t timeoutMinutes)
{
  // store pointer to the passed controller to avoid copying
  _mcp = &mcp;
  _index = index;
  _mcpPin = index + 8; // Relais auf Port B
  _name = name;
  _watt = watt;
  _timeoutSeconds = timeoutMinutes * 60;
}

void Relay::set(bool state, unsigned long now)
{
  if (state)
  {
    if (read())
      return; // Bereits an

    _mcp->digitalWrite(_mcpPin, RELAY_ON);
    _lastStartTime = now;
    _timeout = 0;

    if (_watt != 0) {
      char bufferTopic[64];
      snprintf(bufferTopic, sizeof(bufferTopic), "adebar/carport/relay%d/energy", _index);
      mqttPublish(bufferTopic, "0");
    }
  }
  else
  {
    if (!read())
      return; // Bereits aus

    _mcp->digitalWrite(_mcpPin, RELAY_OFF);
    _lastStopTime = now;

    // Wenn das Relais ausgeschaltet wird, Energieverbrauch berechnen
    if (_lastStartTime != 0 && _watt != 0)
    {
      unsigned long wasRunningMillis = now - _lastStartTime;
      unsigned long wasRunningSeconds = (wasRunningMillis / 1000);
      float energy = ((float)_watt / 3600) * wasRunningSeconds; // Wattsekunden to Wattstunden

      char bufferSmall[64];
      char bufferTopic[64];
      snprintf(bufferSmall, sizeof(bufferSmall), "%f", energy);
      snprintf(bufferTopic, sizeof(bufferTopic), "adebar/carport/relay%d/energy", _index);
      mqttPublish(bufferTopic, bufferSmall);
    }
    _lastStartTime = 0;
    _timeout = 0;
  }
  mqttPublishState(false);
}

void Relay::triggerOrSet(bool state, unsigned long now)
{
  if (state && _timeoutSeconds)
    triggerTimeout(now);
  else
    set(state, now);
}

bool Relay::read()
{
  return _mcp->digitalRead(_mcpPin) == RELAY_ON;
}

void Relay::triggerTimeout(unsigned long now)
{
  set(true, now);
  _timeout = now + ((unsigned long)_timeoutSeconds * 1000UL);
}

void Relay::loop(unsigned long now)
{
  // Handle normal timeout
  if (_timeout != 0 && now >= _timeout)
  {
    set(false, now);
    mqttDebug("Relay timeout triggered");
    _timeout = 0;
  }
}

unsigned long Relay::getLastStopTime()
{
  return _lastStopTime;
}

unsigned long Relay::getLastStartTime()
{
  return _lastStartTime;
}

uint8_t Relay::getIndex()
{
  return _index;
}

void Relay::mqttPublishState(bool full)
{
  char bufferSmall[128];
  char bufferBig[512];

  if (full)
  {
    // see https://www.home-assistant.io/integrations/switch.mqtt/
    JsonDocument discoveryConfig;

    snprintf(bufferSmall, sizeof(bufferSmall), "Carport-Schalter %d %s", _index, _name);
    discoveryConfig["name"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar/carport/relay%d/state", _index);
    discoveryConfig["stat_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar/carport/relay%d/set", _index);
    discoveryConfig["cmd_t"] = bufferSmall;

    snprintf(bufferSmall, sizeof(bufferSmall), "adebar_carport_relay%d", _index);
    discoveryConfig["uniq_id"] = bufferSmall;

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = "adebar_carport";
    device["name"] = "Carport";

    serializeJson(discoveryConfig, bufferBig);
    snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/switch/adebar_carport_relay%d/config", _index);
    mqttPublish(bufferSmall, bufferBig);

    if (_watt != 0)
    {
      discoveryConfig.clear();

      snprintf(bufferSmall, sizeof(bufferSmall), "Carport-Schalter %d %s Verbrauch", _index, _name);
      discoveryConfig["name"] = bufferSmall;

      discoveryConfig["unit_of_meas"] = "Wh";
      discoveryConfig["dev_cla"] = "energy";
      discoveryConfig["stat_cla"] = "total_increasing";

      snprintf(bufferSmall, sizeof(bufferSmall), "adebar/carport/relay%d/energy", _index);
      discoveryConfig["stat_t"] = bufferSmall;

      snprintf(bufferSmall, sizeof(bufferSmall), "adebar_carport_relay%d_energy", _index);
      discoveryConfig["uniq_id"] = bufferSmall;

      JsonObject device = discoveryConfig["dev"].to<JsonObject>();
      device["identifiers"][0] = "adebar_carport";
      device["name"] = "Carport";

      serializeJson(discoveryConfig, bufferBig);
      snprintf(bufferSmall, sizeof(bufferSmall), "homeassistant/sensor/adebar_carport_relay%d_energy/config", _index);
      mqttPublish(bufferSmall, bufferBig);
    }
  }

  snprintf(bufferSmall, sizeof(bufferSmall), "adebar/carport/relay%d/state", _index);
  mqttPublish(bufferSmall, read() ? "ON" : "OFF");
}
