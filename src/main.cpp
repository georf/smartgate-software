#include "main.h"

ShiftOutput shiftOutput; // shift register controller
MCP3008 adc;             // analog digital converter
Motor motor0;            // motor 0 handler
Motor motor1;            // motor 1 handler

// gate states
motor_target_or_state target = close;
motor_target_or_state lastTarget = close;
motor_target_or_state wantedTarget = close;
motor_target_or_state state = stop;

unsigned long lastToggle = 0;             // last toggle millis
unsigned long powerSupplyActivatedAt = 0; // 24v power supply activated
unsigned long powerSupplyShutdownAt = 0;  // 24v power supply shutdown time
uint8_t learnMode = 0;                    // hold the learning mode state
uint8_t loopCount = 0;

AdcSwitch btn0(&adc, 7, 600, LOW); // btn on board
AdcSwitch btn1(&adc, 2, 600, LOW); // btn0 connected to box button
AdcSwitch btn2(&adc, 3, 600, LOW); // btn1 not connected
AdcSwitch btn3(&adc, 4, 600, LOW); // btn2 not connected
Switch radio(A0, 950, LOW);        // radio modul

// wifi and mqtt stuff
WiFiClient espClient;
PubSubClient client(espClient);
uint32_t lastMqttStatusUpdate = 0;

void setup()
{
  // Start serial for debugging
  Serial.begin(115200);

  // Load 256 bytes from EEPROM
  EEPROM.begin(256);

  // initialize adc and shift register
  adc.begin(CS_ADC_PIN, MOSI_PIN, MISO_PIN, CLOCK_PIN);
  shiftOutput.begin(MOSI_PIN, CS_SHIFT_PIN, CLOCK_PIN);

  // enable ERROR LED
  shiftOutput.digitalWrite(SHIFT_PIN_LED_ERROR, HIGH);

  // load last settings from EEPROM and initialize motors
  MotorSetting setting0, setting1;
  EEPROM.get(0, setting0);
  motor0.begin(MOTOR0_0, MOTOR0_1, &adc, MOTOR0_SPEED_CHANNEL, setting0);
  motor0.errorCallback = &errorCallback;
  EEPROM.get(128, setting1);
  motor1.begin(MOTOR1_0, MOTOR1_1, &adc, MOTOR1_SPEED_CHANNEL, setting1);
  motor1.errorCallback = &errorCallback;

  // set callbacks for buttons and radio modul
  btn0.onLowCallback = &learnPressed;
  btn0.pressTimeMillis = 3000; // 3s long press
  btn1.onLowCallback = &toggleGateState;
  btn2.onLowCallback = &btn2Callback;
  btn3.onLowCallback = &btn3Callback;
  radio.onLowCallback = &toggleGateState;

  // wifi startup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(100);
  }
  Serial.println("Connected to Wi-Fi sucessfully.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqtt_callback);

  debugInfos();

  // disable ERROR LED
  shiftOutput.digitalWrite(SHIFT_PIN_LED_ERROR, LOW);

  ArduinoOTA.begin();
}
void loop()
{
  loopCount++;
  uint8_t loop15 = loopCount % 15;
  unsigned long currentMillis = millis();

  ArduinoOTA.handle();

  if (!client.connected())
    mqtt_reconnect();
  client.loop();

  if ((lastMqttStatusUpdate + MQTT_STATUS_UPDATE_TIME) < currentMillis)
    mqtt_send_status();

  // handle changes at both motors
  motor0.handle(currentMillis);
  motor1.handle(currentMillis);

  // handle button changes
  // but not every loop
  // see: https://arduino.stackexchange.com/questions/19787/esp8266-analog-read-interferes-with-wifi
  if (loop15 == 0)
    btn0.read(currentMillis);
  else if (loop15 == 1)
    btn1.read(currentMillis);
  else if (loop15 == 2)
    btn2.read(currentMillis);
  else if (loop15 == 3)
    btn3.read(currentMillis);
  else if (loop15 == 4)
    radio.read(currentMillis);
  else if (loop15 == 5 && powerSupplyShutdownAt != 0 && powerSupplyShutdownAt < currentMillis)
  {
    shiftOutput.digitalWrite(SHIFT_PIN_RELAY_4_POWER_SUPPLY, HIGH);
    powerSupplyShutdownAt = 0;
    mqtt_send_status();
    debugInfos();
  }

  if (wantedTarget != target)
  {
    if (wantedTarget == open || wantedTarget == close)
    {
      if (shiftOutput.digitalRead(SHIFT_PIN_RELAY_4_POWER_SUPPLY))
      {
        shiftOutput.digitalWrite(SHIFT_PIN_RELAY_4_POWER_SUPPLY, LOW);
        powerSupplyActivatedAt = currentMillis + MILLIS_AFTER_POWER_SUPPLY_ACTIVATED;
      }
      else if (powerSupplyActivatedAt < currentMillis)
      {
        mqtt_send_status();
        motor0.target = motor1.target = target = wantedTarget;
      }
    }
    else if (wantedTarget == stop)
    {
      motor0.target = motor1.target = target = wantedTarget;
      powerSupplyShutdownAt = currentMillis + MILLIS_POWER_SUPPLY_SHUTDOWN_TIME;
    }
  }
  else if (target != state)
  {
    if ((motor0.state == stop && motor1.state == stop) ||
        (motor0.state == open && motor1.state == open) ||
        (motor0.state == close && motor1.state == close))
    {
      // set global state from motors
      state = motor0.state;

      if (state == open || state == close)
      {
        // disable error led if full opened or closed
        shiftOutput.digitalSet(SHIFT_PIN_LED_ERROR, LOW);

        // and save state
        EEPROM.put(0, motor0.setting);
        EEPROM.put(128, motor1.setting);
        EEPROM.commit();

        powerSupplyShutdownAt = currentMillis + MILLIS_POWER_SUPPLY_SHUTDOWN_TIME;
      }

      // disable warning leds
      shiftOutput.digitalSet(SHIFT_PIN_LED_0, LOW);
      shiftOutput.digitalSet(SHIFT_PIN_LED_1, LOW);
      shiftOutput.write();

      mqtt_send_status();
    }
    else if (motor0.state == closing || motor0.state == closing_soft || motor1.state == closing || motor1.state == closing_soft)
      state = closing;
    else if (motor0.state == opening || motor0.state == opening_soft || motor1.state == opening || motor1.state == opening_soft)
      state = opening;
  }

  // show warning led while moving
  if (loop15 == 10)
  {

    if (wantedTarget != target || state == opening || state == closing)
    {
      if (currentMillis / 100 % 2 == 0)
      {
        shiftOutput.digitalSet(SHIFT_PIN_LED_0, HIGH);
        shiftOutput.digitalSet(SHIFT_PIN_LED_1, LOW);
        shiftOutput.write();
      }
      else
      {
        shiftOutput.digitalSet(SHIFT_PIN_LED_0, LOW);
        shiftOutput.digitalSet(SHIFT_PIN_LED_1, HIGH);
        shiftOutput.write();
      }
    }

    // show warning led while learning
    if (learnMode != 0)
    {
      if (currentMillis / 100 % 2 == 0)
      {

        shiftOutput.digitalSet(SHIFT_PIN_LED_LEARN, HIGH);
        shiftOutput.digitalSet(SHIFT_PIN_LED_0, HIGH);
        shiftOutput.digitalSet(SHIFT_PIN_LED_1, LOW);
        shiftOutput.write();
      }
      else
      {
        shiftOutput.digitalSet(SHIFT_PIN_LED_LEARN, LOW);
        shiftOutput.digitalSet(SHIFT_PIN_LED_0, LOW);
        shiftOutput.digitalSet(SHIFT_PIN_LED_1, HIGH);
        shiftOutput.write();
      }
    }
  }
}

void toggleGateState()
{
  unsigned long currentMillis = millis();
  if (lastToggle < currentMillis && currentMillis < lastToggle + MILLIS_BETWEEN_TOGGLE)
    return;

  lastToggle = currentMillis;

  if (state == open)
    lastTarget = wantedTarget = close;
  else if (state == close)
    lastTarget = wantedTarget = open;
  else if (state == opening || state == closing)
    wantedTarget = stop;
  else if (state == stop && lastTarget == close)
    lastTarget = wantedTarget = open;
  else if (state == stop && lastTarget == open)
    lastTarget = wantedTarget = close;

  mqtt_send_status();
}

void learnPressed()
{
  unsigned long currentMillis = millis();
  if (learnMode == 0)
  {
    Serial.println("LEARN MODE");
    btn0.pressTimeMillis = 50;                                     // normal press time
    shiftOutput.digitalWrite(SHIFT_PIN_RELAY_4_POWER_SUPPLY, LOW); // enable power supply
    powerSupplyShutdownAt = 0;
    learnMode = 1;
  }
  else if (learnMode == 1)
  {
    Serial.println("LEARN MODE motor 0 open");
    motor0.learnStartOpen();
    motor0.handle(currentMillis);
    learnMode = 2;
  }
  else if (learnMode == 2)
  {
    motor0.learnOpen();
    motor0.handle(currentMillis);
    learnMode = 3;
  }
  else if (learnMode == 3)
  {
    Serial.println("LEARN MODE motor 0 close");
    motor0.learnStartClose();
    motor0.handle(currentMillis);
    learnMode = 4;
  }
  else if (learnMode == 4)
  {
    motor0.learnClose();
    motor0.handle(currentMillis);

    Serial.print("LEARN MODE motor 0 steps: ");
    Serial.println(motor0.setting.closeAt);
    learnMode = 5;
  }
  else if (learnMode == 5)
  {
    Serial.println("LEARN MODE motor 1 open");
    motor1.learnStartOpen();
    motor1.handle(currentMillis);
    learnMode = 6;
  }
  else if (learnMode == 6)
  {
    motor1.learnOpen();
    motor1.handle(currentMillis);
    learnMode = 7;
  }
  else if (learnMode == 7)
  {
    Serial.println("LEARN MODE motor 1 close");
    motor1.learnStartClose();
    motor1.handle(currentMillis);
    learnMode = 8;
  }
  else if (learnMode == 8)
  {
    motor1.learnClose();
    motor1.handle(currentMillis);

    Serial.print("LEARN MODE motor 1 steps: ");
    Serial.println(motor0.setting.closeAt);

    Serial.println("LEARN MODE OFF");

    // save state
    EEPROM.put(0, motor0.setting);
    EEPROM.put(128, motor1.setting);
    EEPROM.commit();

    btn0.pressTimeMillis = 3000; // long press time
    shiftOutput.digitalSet(SHIFT_PIN_LED_LEARN, LOW);
    shiftOutput.digitalSet(SHIFT_PIN_LED_0, LOW);
    shiftOutput.digitalSet(SHIFT_PIN_LED_1, LOW);
    shiftOutput.write();
    powerSupplyShutdownAt = currentMillis + MILLIS_POWER_SUPPLY_SHUTDOWN_TIME; // disable power supply
    learnMode = 0;
  }
}

void btn2Callback()
{
}

void btn3Callback()
{
}

void errorCallback()
{
  // If an error occurs in the learning mode, we evaluate it as a button press.
  if (learnMode != 0)
    return learnPressed();

  // If it's the second error, we will stop.
  if (lastTarget != wantedTarget)
    wantedTarget = stop;
  // If the old target was open, then close the gate.
  else if (wantedTarget == open)
    wantedTarget = close;
  // If the old target was close, then open the gate.
  else if (wantedTarget == close)
    wantedTarget = open;
  // Otherwise stop
  else
    wantedTarget = stop;

  // enable error led
  shiftOutput.digitalWrite(SHIFT_PIN_LED_ERROR, HIGH);
}

void mqtt_reconnect()
{
  Serial.println(WiFi.status());
  if (!client.connected())
  {
    Serial.println("Reconnecting MQTT...");

    if (!client.connect(MQTT_ID, mqttUser, mqttPassword))
    {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
    else
    {
      client.subscribe(MQTT_SMARTGATE_TOGGLE);
      client.subscribe(MQTT_SMARTGATE_OPEN);
      client.subscribe(MQTT_SMARTGATE_CLOSE);
      Serial.println("MQTT Connected...");
    }
  }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("MQTT: receive ");
  Serial.println(topic);
  if (strcmp(topic, MQTT_SMARTGATE_TOGGLE) == 0)
    toggleGateState();
  else if (strcmp(topic, MQTT_SMARTGATE_OPEN) == 0)
  {
    lastTarget = wantedTarget = open;
  }
  else if (strcmp(topic, MQTT_SMARTGATE_CLOSE) == 0)
  {
    lastTarget = wantedTarget = close;
  }
}

void mqtt_send_status()
{
  if (wantedTarget != target)
  {

    if (wantedTarget == close)
      client.publish(MQTT_SMARTGATE_CHANNEL, "closing");
    else if (wantedTarget == open)
      client.publish(MQTT_SMARTGATE_CHANNEL, "opening");
  }
  else if (state == open)
    client.publish(MQTT_SMARTGATE_CHANNEL, "opened");
  else if (state == close)
    client.publish(MQTT_SMARTGATE_CHANNEL, "closed");
  else if (state == stop)
    client.publish(MQTT_SMARTGATE_CHANNEL, "stopped");
  else if (state == opening)
    client.publish(MQTT_SMARTGATE_CHANNEL, "opening");
  else if (state == closing)
    client.publish(MQTT_SMARTGATE_CHANNEL, "closing");
  else if (state == unknown)
    client.publish(MQTT_SMARTGATE_CHANNEL, "unknown");

  lastMqttStatusUpdate = millis();
}

void debugInfos()
{
  Serial.print("Motor 0 openAt:");
  Serial.println(motor0.setting.openAt);
  Serial.print("Motor 0 closeAt:");
  Serial.println(motor0.setting.closeAt);
  Serial.print("Motor 0 currentSteps:");
  Serial.println(motor0.setting.currentSteps);

  Serial.print("Motor 1 openAt:");
  Serial.println(motor1.setting.openAt);
  Serial.print("Motor 1 closeAt:");
  Serial.println(motor1.setting.closeAt);
  Serial.print("Motor 1 currentSteps:");
  Serial.println(motor1.setting.currentSteps);
}