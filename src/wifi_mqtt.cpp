#include <wifi_mqtt.h>


// Verbindungen
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Verbindungszeitpunkte
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastMqttStatusUpdate = 0;

void wifiAndMqttStartup()
{
  // wifi startup
  WiFi.mode(WIFI_STA);
  connectToWiFi();

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  ArduinoOTA.begin();
}

void wifiAndMqttLoop(const unsigned long currentMillis)
{
  // OTA
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED)
  {
    // WLAN Reconnect
    if (currentMillis - lastWifiAttempt > RECONNECTING_INTERVAL)
    {
      lastWifiAttempt = currentMillis;
      connectToWiFi();
    }

    // WLAN verbunden
  }
  else
  {

    // MQTT verbunden
    if (mqttClient.connected())
    {
      mqttClient.loop();

      if ((lastMqttStatusUpdate + 10 * 60 * 1000) < currentMillis)
      {
        mqttSendStatus(true);
      }
    }
    else
    {
      if (currentMillis - lastMqttAttempt > RECONNECTING_INTERVAL)
      {
        lastMqttAttempt = currentMillis;
        connectToMqtt();
      }
    }
  }
}

// WLAN verbinden
void connectToWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    leds.set(LED_GREEN, fastBlink);
    Serial.println("Verbindung zu WLAN...");
    WiFi.begin(wifiSsid, wifiPassword);
  }
}

// MQTT verbinden
void connectToMqtt()
{
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected())
  {
    leds.set(LED_GREEN, blink);
    Serial.println("Verbindung zu MQTT...");
    if (mqttClient.connect("smartgate2", mqttUser, mqttPassword))
    {
      leds.set(LED_GREEN, off);
      Serial.println("MQTT verbunden");
      mqttClient.subscribe("adebar/carport/+/set");
      mqttClient.publish("adebar/carport/system/state", "connected");
      mqttSendStatus(true);
    }
    else
    {
      Serial.print("Fehler beim MQTT-Verbindungsversuch: ");
      Serial.println(mqttClient.state());
    }
  }
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
    return false;
  }
}

void mqttSendStatus(boolean full)
{
  mqttSendAdebarCarportGate(full);
  mqttSendAdebarCarportIpAddress(full);

  lastMqttStatusUpdate = millis();
}