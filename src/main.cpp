#include "main.h"

// Aktuelle Zeit immer nur einmal pro Loop lesen
unsigned long now;

#define PIN_SCL D1
#define PIN_SDA D4

// ----------------------------------------------------------
// MCP23017
// ----------------------------------------------------------
#define MCP_ADDR 0x20
MCP23017Controller mcp;

// GPA
#define LED_GATE_1 0
#define LED_GATE_2 1
#define SW_24V 2
#define SW_BTN 3
#define SW_REED_RIGHT 4
#define SW_REED_LEFT 5

// GPB
#define SW_RADIO2 8 + 4
#define SW_RADIO1 8 + 5
#define SW_RADIO0 8 + 6

// Relais Backend
#define RELAY_0_GATE_MOTOR 0
#define RELAY_1_UNUSED 1
#define RELAY_2_UNUSED 2
#define RELAY_3_UNUSED 3

Relay relays[4] = {
    Relay(mcp, RELAY_0_GATE_MOTOR, (char *)"Tor-Motor", 20, 0),
    Relay(mcp, RELAY_1_UNUSED, (char *)"Unbenutzt", 0, 0),
    Relay(mcp, RELAY_2_UNUSED, (char *)"Unbenutzt", 0, 0),
    Relay(mcp, RELAY_3_UNUSED, (char *)"Unbenutzt", 0, 0),
};

// ----------------------------------------------------------
// Motor Pins
// ----------------------------------------------------------

#define MOTOR_LEFT_COUNT D7
#define MOTOR_LEFT_PWM_OPEN D3
#define MOTOR_LEFT_PWM_CLOSE D5
#define MOTOR_LEFT_CURRENT_CHANNEL 0

#define MOTOR_RIGHT_COUNT D2
#define MOTOR_RIGHT_PWM_OPEN D6
#define MOTOR_RIGHT_PWM_CLOSE D8
#define MOTOR_RIGHT_CURRENT_CHANNEL 1

Motor motorLeft;  // motor left handler
Motor motorRight; // motor right handler

motor_target_or_state wantedTarget = close;
motor_target_or_state lastTarget = close;
motor_target_or_state state = closing;
unsigned long lastToggle = 0; // last toggle millis
#define MILLIS_BETWEEN_TOGGLE 1000

// ----------------------------------------------------------
// WLAN & MQTT
// ----------------------------------------------------------
#define WLAN_RECONNECT_TIME 10000           // 10 Sekunden
#define MQTT_RECONNECT_TIME 5000            // 5 Sekunden
#define MQTT_STATUS_INTERVAL 10 * 60 * 1000 // 10 Minuten
WiFiClient espClient;
PubSubClient mqttClient(espClient);

#define MAX_MQTT_BUFFER_ENTRIES 32
struct mqttBuffer
{
  char topic[56];
  char payload[64];
};
mqttBuffer mqttSendBuffer[MAX_MQTT_BUFFER_ENTRIES];
unsigned int mqttSendBufferIndex = 0;

unsigned long lastWifiReconnect = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastMqttStatusUpdate = 0;
void mqttSendStatus(bool full);

void mqttCallback(char *topic, byte *payload, unsigned int length)
{

  char buffer[56];

  if (!strcmp(topic, "adebar/carport/gate/set"))
  {
    if (!strncmp((char *)payload, "STOP", length))
      gateStop();
    else if (!strncmp((char *)payload, "CLOSE", length))
      gateClose();
    else if (!strncmp((char *)payload, "OPEN", length))
      gateOpen();
    else if (!strncmp((char *)payload, "TOGGLE", length))
      gateToggle();
    else if (!strncmp((char *)payload, "CLOSE_A_BIT_LEFT", length))
      motorLeft.doCloseABit(now);
    else if (!strncmp((char *)payload, "CLOSE_A_BIT_RIGHT", length))
      motorRight.doCloseABit(now);
    return;
  }

  if (!strcmp(topic, "adebar/carport/system/set"))
  {
    if (!strncmp((char *)payload, "RESTART", length))
    {
      ESP.restart();
    }
    return;
  }
  else if (!strncmp((char *)payload, "DEBUG", length))
  {
    char debugBuffer[128];
    sprintf(debugBuffer, "State: %d, Wanted: %d, LeftSteps: %ld, RightSteps: %ld", state, wantedTarget, motorLeft.currentSteps, motorRight.currentSteps);
    mqttDebug(debugBuffer);
    sprintf(debugBuffer, "LeftRunning: %d, RightRunning: %d", motorLeft._running ? 1 : 0, motorRight._running ? 1 : 0);
    mqttDebug(debugBuffer);
    mqttSendStatus(false);
  }

  for (uint8_t relay = 0; relay < 8; relay++)
  {
    sprintf(buffer, "adebar/carport/relay%d/set", relay);
    if (!strcmp(topic, buffer))
    {
      if (!strncmp((char *)payload, "ON", length))
        relays[relay].triggerOrSet(true, now);
      else if (!strncmp((char *)payload, "OFF", length))
        relays[relay].set(false, now);
      return;
    }
  }
}

bool connectWifiNonBlocking()
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  if (now - lastWifiReconnect < WLAN_RECONNECT_TIME)
    return false;

  lastWifiReconnect = now;
  WiFi.begin(wifiSsid, wifiPassword);
  return false;
}

// MQTT senden
bool mqttPublish(const char *topic, const char *message)
{
  if (mqttClient.connected())
  {
    mqttClient.publish(topic, message);
    return true;
  }
  else
  {
    if (mqttSendBufferIndex < MAX_MQTT_BUFFER_ENTRIES)
    {
      // Kopiere Nachricht in Buffer für späteres Senden
      strncpy(mqttSendBuffer[mqttSendBufferIndex].topic, topic, 56);
      strncpy(mqttSendBuffer[mqttSendBufferIndex].payload, message, 64);
      mqttSendBufferIndex++;
    }
    return false;
  }
}

// MQTT Debug senden
bool mqttDebug(const char *message)
{
  return mqttPublish("adebar/carport/system/debug", message);
}

void mqttSendAdebarCarportIpAddress(boolean full)
{
  if (full)
  {

    // see https://www.home-assistant.io/integrations/sensor.mqtt/
    const char *discoveryConfig = "{"
                                  "\"name\":\"Carport IP-Adresse\","
                                  "\"stat_t\":\"adebar/carport/ip_address/state\","
                                  "\"uniq_id\":\"adebar_carport_ip_address\","
                                  "\"dev\":{"
                                  "\"identifiers\":[\"adebar_carport\"],"
                                  "\"name\":\"Carport\""
                                  "}"
                                  "}";
    mqttPublish("homeassistant/sensor/adebar_carport_ip_address/config", discoveryConfig);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    String ip = WiFi.localIP().toString();
    char ip_char[ip.length() + 1];
    ip.toCharArray(ip_char, ip.length() + 1);

    mqttPublish("adebar/carport/ip_address/state", ip_char);
  }
}

void mqttSendAdebarCarportGate(boolean full)
{
  if (full)
  {
    // see https://www.home-assistant.io/integrations/cover.mqtt/
    JsonDocument discoveryConfig;
    discoveryConfig["name"] = "Hoftor";
    discoveryConfig["dev_cla"] = "gate";
    discoveryConfig["cmd_t"] = "adebar/carport/gate/set";
    discoveryConfig["stat_t"] = "adebar/carport/gate/state";
    discoveryConfig["uniq_id"] = "adebar_carport_gate";

    JsonObject device = discoveryConfig["dev"].to<JsonObject>();
    device["identifiers"][0] = "adebar_carport";
    device["name"] = "Carport";

    char buffer[256];
    serializeJson(discoveryConfig, buffer);
    mqttPublish("homeassistant/cover/adebar_carport/config", buffer);
  }

  if (state == open)
    mqttPublish("adebar/carport/gate/state", "open");
  else if (state == close)
    mqttPublish("adebar/carport/gate/state", "closed");
  else if (state == opening)
    mqttPublish("adebar/carport/gate/state", "opening");
  else if (state == closing)
    mqttPublish("adebar/carport/gate/state", "closing");
  else
    mqttPublish("adebar/carport/gate/state", "stopped");
}

void mqttSendAdebarCarportRestartButton(boolean full)
{
  if (!full)
    return;

  // see https://www.home-assistant.io/integrations/button.mqtt/
  const char *discoveryConfig = "{"
                                "\"name\":\"Carport Neustart\","
                                "\"uniq_id\":\"adebar_carport_system_restart\","
                                "\"cmd_t\":\"adebar/carport/system/set\","
                                "\"payload_press\":\"RESTART\","
                                "\"dev\":{"
                                "\"identifiers\":[\"adebar_carport\"],"
                                "\"name\":\"Carport\""
                                "}"
                                "}";
  mqttPublish("homeassistant/button/adebar_carport_system_restart/config", discoveryConfig);
}

void mqttSendStatus(boolean full)
{
  for (int i = 0; i < 4; i++)
    relays[i].mqttPublishState(full);

  mqttSendAdebarCarportIpAddress(full);
  mqttSendAdebarCarportGate(full);
  mqttSendAdebarCarportRestartButton(full);

  lastMqttStatusUpdate = now;
}

bool connectMQTT()
{
  if (mqttClient.connected())
    return true;

  if (now - lastMqttReconnect < MQTT_RECONNECT_TIME)
    return false;

  lastMqttReconnect = now;

  if (mqttClient.connect("carport3", mqttUser, mqttPassword))
  {
    mqttClient.subscribe("adebar/carport/+/set");
    mqttClient.publish("adebar/carport/system/state", "connected");
    mqttSendStatus(true);

    // Sende alle Nachrichten aus den Buffer
    for (uint8_t i = 0; i < mqttSendBufferIndex; i++)
    {
      mqttClient.publish(mqttSendBuffer[i].topic, mqttSendBuffer[i].payload);
    }
    mqttSendBufferIndex = 0;
    return true;
  }
  else
  {
    Serial.print("Fehler beim MQTT-Verbindungsversuch: ");
    Serial.println(mqttClient.state());
    return false;
  }
}

// ----------------------------------------------------------
// OTA
// ----------------------------------------------------------
void setupOTA()
{
  ArduinoOTA.setHostname("garage-controller");
  ArduinoOTA.begin();
}

Adafruit_ADS1015 ads; /* Use thi for the 12-bit version */

void clickLid()
{
  mqttPublish("adebar/garage/cover/set", "TOGGLE");
}

void gateToggle()
{
  if (now - lastToggle < MILLIS_BETWEEN_TOGGLE)
    return;

  lastToggle = now;

  if (gateRunning())
    gateStop();
  else if (lastTarget == open)
    gateClose();
  else
    gateOpen();
}

void gateOpen()
{
  wantedTarget = open;
  lastTarget = open;
  motorLeft.doOpen(now);
  motorRight.doOpen(now);
}

void gateClose()
{
  wantedTarget = close;
  lastTarget = close;
  motorLeft.doClose(now);
  motorRight.doClose(now);
}

void gateStop()
{
  wantedTarget = stop;
  motorLeft.doStop(now);
  motorRight.doStop(now);
}

void gateError(uint32_t milliWatt)
{
  char buffer[100];

  sprintf(buffer, "gate-error %d", milliWatt);
  Serial.println(buffer);
  gateStop();
  mqttPublish("adebar/carport/debug", buffer);
}

bool gateRunning()
{
  return motorLeft._running || motorRight._running;
}

void clickBell()
{
  mqttPublish("adebar/klingelbox/bell_button/set", "SET");
}

// ----------------------------------------------------------
// SETUP
// ----------------------------------------------------------
void setup()
{
  now = 0;

  Serial.begin(115200);

  // MCP23017 zurücksetzen
  pinMode(D0, OUTPUT);
  digitalWrite(D0, HIGH);
  delay(10);
  digitalWrite(D0, LOW);
  delay(100);
  digitalWrite(D0, HIGH);

  // I2C starten
  Wire.begin(PIN_SDA, PIN_SCL);

  bool error = true;
  bool addressFound = false;
  for (int i = 1; i < 128; i++)
  {
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
    if (error == 0)
    {
      addressFound = true;
      Serial.print("0x");
      Serial.println(i, HEX);
    }
  }
  if (!addressFound)
  {
    Serial.println("Keine Adresse erkannt");
  }
  Serial.println();

  // MCP einrichten
  mcp.begin(MCP_ADDR);

  mcp.setPinMode(LED_GATE_1, OUTPUT);
  mcp.digitalWrite(LED_GATE_1, HIGH); // LED aus
  mcp.setPinMode(LED_GATE_2, OUTPUT);
  mcp.digitalWrite(LED_GATE_2, HIGH); // LED aus
  mcp.setPinMode(SW_24V, INPUT);
  mcp.configureClick(SW_BTN, INPUT_PULLUP, gateToggle, HIGH);
  mcp.setPinMode(SW_REED_LEFT, INPUT_PULLUP);
  mcp.setPinMode(SW_REED_RIGHT, INPUT_PULLUP);

  mcp.setPinMode(8 + RELAY_0_GATE_MOTOR, OUTPUT);
  mcp.digitalWrite(8 + RELAY_0_GATE_MOTOR, HIGH); // Relais aus
  mcp.setPinMode(8 + RELAY_1_UNUSED, OUTPUT);
  mcp.digitalWrite(8 + RELAY_1_UNUSED, HIGH); // Relais aus
  mcp.setPinMode(8 + RELAY_2_UNUSED, OUTPUT);
  mcp.digitalWrite(8 + RELAY_2_UNUSED, HIGH); // Relais aus
  mcp.setPinMode(8 + RELAY_3_UNUSED, OUTPUT);
  mcp.digitalWrite(8 + RELAY_3_UNUSED, HIGH); // Relais aus

  mcp.configureClick(SW_RADIO2, INPUT_PULLUP, clickLid, LOW);
  mcp.configureClick(SW_RADIO1, INPUT_PULLUP, gateToggle, LOW);
  mcp.configureClick(SW_RADIO0, INPUT_PULLUP, clickBell, LOW);

  // Analog-Digital-Wandler einrichten
  ads.setGain(GAIN_ONE);
  ads.begin();

  now = millis();

  // Blickrichtung zur Garage hin
  // Motor einrichtens
  motorLeft.begin(now, MOTOR_LEFT_PWM_OPEN, MOTOR_LEFT_PWM_CLOSE, &mcp, SW_REED_LEFT, SW_24V, &ads, MOTOR_LEFT_CURRENT_CHANNEL, 4500);
  motorLeft.errorCallback = &gateError;
  motorLeft.mqttDebugCallback = &mqttDebug;
  Motor::leftInstance = &motorLeft;
  pinMode(MOTOR_LEFT_COUNT, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(MOTOR_LEFT_COUNT),
      Motor::isrLeft,
      FALLING);

  motorRight.begin(now, MOTOR_RIGHT_PWM_OPEN, MOTOR_RIGHT_PWM_CLOSE, &mcp, SW_REED_RIGHT, SW_24V, &ads, MOTOR_RIGHT_CURRENT_CHANNEL, 3790);
  motorRight.errorCallback = &gateError;
  motorRight.mqttDebugCallback = &mqttDebug;
  Motor::rightInstance = &motorRight;
  pinMode(MOTOR_RIGHT_COUNT, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(MOTOR_RIGHT_COUNT),
      Motor::isrRight,
      FALLING);

  state = stop;

  // WLAN und MQTT vorbereiten
  WiFi.mode(WIFI_STA);
  mqttClient.setBufferSize(1024);
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  // Over the Air updates
  setupOTA();

  // Alle Relais aus
  for (int i = 0; i < 4; i++)
    relays[i].set(false, now);
}

// ----------------------------------------------------------
// LOOP
// ----------------------------------------------------------
void loop()
{
  // Aktuelle Zeit nur einmal berechnen
  now = millis();

  // WLAN und MQTT-Stuff
  if (connectWifiNonBlocking())
  {
    ArduinoOTA.handle();
    if (connectMQTT())
    {
      mqttClient.loop();

      if (now - lastMqttStatusUpdate >= MQTT_STATUS_INTERVAL)
        mqttSendStatus(true);
    }
  }

  // MCPs auslesen und Handler ausführen
  mcp.loop(now);

  // handle changes at both motors
  motorLeft.handle(now);
  motorRight.handle(now);

  // Alle Relais
  for (uint8_t i = 0; i < 4; i++)
    relays[i].loop(now);

  if (gateRunning())
  {
    mcp.setOutputModus(LED_GATE_1, fastBlink);
    mcp.setOutputModus(LED_GATE_2, reverseFastBlink);
    relays[RELAY_0_GATE_MOTOR].set(true, now);

    if (wantedTarget == open && state != opening)
    {
      state = opening;
      mqttSendAdebarCarportGate(false);
    }
    else if (wantedTarget == close && state != closing)
    {
      state = closing;
      mqttSendAdebarCarportGate(false);
    }
  }
  else
  {
    mcp.setOutputModus(LED_GATE_1, off);
    mcp.setOutputModus(LED_GATE_2, off);
    relays[RELAY_0_GATE_MOTOR].set(false, now);

    if (wantedTarget != state)
    {
      if (wantedTarget == open &&
          motorLeft.isOpenPosition() &&
          motorRight.isOpenPosition())
      {
        state = open;
        mqttSendAdebarCarportGate(false);
      }
      else if (wantedTarget == close &&
               motorLeft.isClosePosition() &&
               motorRight.isClosePosition())
      {
        state = close;
        mqttSendAdebarCarportGate(false);
      }
      else
      {
        wantedTarget = stop;
        state = stop;
        mqttSendAdebarCarportGate(false);
      }
    }
  }
}
