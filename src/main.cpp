#include "main.h"

ShiftOutput shiftOutput; // shift register controller
MCP3008 adc;             // analog digital converter
Motor motorL;            // motor left handler
Motor motorR;            // motor right handler

unsigned long lastToggle = 0; // last toggle millis
uint8_t loopCount = 0;
uint8_t voltageCounter = 0;
uint32_t avarageVoltage = 5;

AdcTribleSwitch btns(&adc, 2);       // btns
AdcSwitch radioB(&adc, 6, 800, LOW); // connected to radio B
AdcSwitch radioC(&adc, 7, 800, LOW); // connected to radio C
AdcSwitch radioD(&adc, 5, 800, LOW); // connected to radio D

LedsHandler leds(&shiftOutput); // LEDs Interface

motor_target_or_state wantedTarget = close;
motor_target_or_state lastTarget = close;
motor_target_or_state state = closing;

unsigned long nextDebugPrint = 0; // last toggle millis

void setup()
{
  
  pinMode(MOTOR_L_OPEN, OUTPUT);
  analogWrite(MOTOR_L_OPEN, 0);
  pinMode(MOTOR_L_CLOSE, OUTPUT);
  analogWrite(MOTOR_L_CLOSE, 0);
  pinMode(MOTOR_R_OPEN, OUTPUT);
  analogWrite(MOTOR_R_OPEN, 0);
  pinMode(MOTOR_R_CLOSE, OUTPUT);
  analogWrite(MOTOR_R_CLOSE, 0);

  // Start serial for debugging
  Serial.begin(115200);

  // warte eine Sekunde für eine stabile Spannungsversorgung
  delay(1000);

  // A0 für die Spannungsüberwachung
  pinMode(A0, INPUT);

  // initialize adc and shift register
  adc.begin(CS_ADC_PIN, MOSI_PIN, MISO_PIN, CLOCK_PIN);
  shiftOutput.begin(MOSI_PIN, CS_SHIFT_PIN, CLOCK_PIN);

  shiftOutput.digitalSet(SHIFT_PIN_RELAY_1_NC, HIGH);
  shiftOutput.digitalSet(SHIFT_PIN_RELAY_2_NC, HIGH);
  shiftOutput.digitalSet(SHIFT_PIN_RELAY_3_NC, HIGH);
  shiftOutput.digitalSet(SHIFT_PIN_RELAY_4_POWER_SUPPLY, HIGH);

  // initialize motors
  motorL.begin(MOTOR_L_OPEN, MOTOR_L_CLOSE, &adc, MOTOR_L_SPEED_CHANNEL, MOTOR_L_HALL_CHANNEL, 6250, 0); // eigentlich 5000
  motorL.errorCallback = &gateError;
  motorR.begin(MOTOR_R_OPEN, MOTOR_R_CLOSE, &adc, MOTOR_R_SPEED_CHANNEL, MOTOR_R_HALL_CHANNEL, 6200, 1000);
  motorR.errorCallback = &gateError;

  // set callbacks for buttons and radio modul
  btns.onBtn0Callback = &btn0Callback;
  btns.onBtn1Callback = &btn1Callback;
  btns.onBtn2Callback = &btn2Callback;
  radioB.onHighCallback = &radioBCallback;
  radioC.onHighCallback = &radioCCallback;
  radioD.onHighCallback = &radioDCallback;

  wifiAndMqttStartup();

}
void loop()
{
  loopCount++;
  uint8_t loop15 = loopCount % 15;
  const unsigned long currentMillis = millis();

  wifiAndMqttLoop(currentMillis);

  // handle changes at both motors
  motorL.handle(currentMillis);
  motorR.handle(currentMillis);

  if (loop15 == 0)
  {
    if (gateRunning())
    {
      leds.set(LED_WARN0, fastBlink);
      leds.set(LED_WARN1, fastBlinkReverse);
      
      shiftOutput.digitalWrite(SHIFT_PIN_RELAY_4_POWER_SUPPLY, LOW);
      if (wantedTarget == open && state != opening)
      {
        state = opening;
        mqttSendStatus(false);
      }
      else if (wantedTarget == close && state != closing)
      {
        state = closing;
        mqttSendStatus(false);
      }
    }
    else
    {
      leds.set(LED_WARN0, off);
      leds.set(LED_WARN1, off);
      
      shiftOutput.digitalWrite(SHIFT_PIN_RELAY_4_POWER_SUPPLY, HIGH);
      if (wantedTarget != state)
      {
        if (wantedTarget == open && motorL.isOpenPosition() && motorR.isOpenPosition())
        {
          state = open;
          mqttSendStatus(false);
        }
        else if (wantedTarget == close && motorL.isClosePosition() && motorR.isClosePosition())
        {
          state = close;
          mqttSendStatus(false);
        }
        else
        {
          state = stop;
          mqttSendStatus(false);
        }
      }
    }
  }
  
  // handle button changes
  else if (loop15 == 2)
    btns.read(currentMillis);
  else if (loop15 == 3)
    radioB.read(currentMillis);
  else if (loop15 == 4)
    radioC.read(currentMillis);
  else if (loop15 == 5)
    radioD.read(currentMillis);
  else if (loop15 == 6)
    leds.handle(currentMillis);
  else if (loop15 == 13)
  {
    voltageCounter++;

    if (voltageCounter > 40)
    {
      int milliVoltage = analogRead(A0) * 35;
      avarageVoltage = (3 * avarageVoltage + milliVoltage) / 4;
      motorL.milliVoltage = avarageVoltage;
      motorR.milliVoltage = avarageVoltage;
      voltageCounter = 0;
    }
  }
  else if (loop15 == 14 && currentMillis > nextDebugPrint)
  {
    // nextDebugPrint = currentMillis + 200;

    // Serial.print("0: ");
    // Serial.println(adc.analogRead(0));

    // Serial.print("1: ");
    // Serial.println(adc.analogRead(1));

    // Serial.print("2: ");
    // Serial.println(adc.analogRead(2));

    // Serial.print("3: ");
    // Serial.println(adc.analogRead(3));

    // Serial.print("4: ");
    // Serial.println(adc.analogRead(4));

    // Serial.print("5: ");
    // Serial.println(adc.analogRead(5));

    // Serial.print("6: ");
    // Serial.println(adc.analogRead(6));

    // Serial.print("7: ");
    // Serial.println(adc.analogRead(7));

    // Serial.println();
  }
}

void btn0Callback()
{
  Serial.println("btn0Callback");
  // intern btn
}

void btn1Callback()
{
  gateToggle();
}

void btn2Callback()
{
  // not connected
}

void radioBCallback()
{
  gateToggle();
}

void radioCCallback()
{
  mqttPublish("adebar/garage/cover/set", "TOGGLE");
}

void radioDCallback()
{
  mqttPublish("adebar/klingelbox/bell_button/set", "SET");
}

void gateToggle()
{
  if (millis() < lastToggle + MILLIS_BETWEEN_TOGGLE)
    return;

  lastToggle = millis();

  if (gateRunning())
  {
    gateStop();
  }
  else if (lastTarget == open)
  {
    gateClose();
  }
  else
  {
    gateOpen();
  }
}

void gateOpen()
{
  wantedTarget = open;
  lastTarget = open;
  motorL.doOpen();
  motorR.doOpen();
}

void gateClose()
{
  wantedTarget = close;
  lastTarget = close;
  motorL.doClose();
  motorR.doClose();
}

void gateStop()
{
  wantedTarget = stop;
  motorL.doStop();
  motorR.doStop();
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
  return motorL._running || motorR._running;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  if (!strcmp(topic, "adebar/carport/gate/set"))
  {
    if (!strncmp((char *)payload, "STOP", length))
      gateStop();
    else if (!strncmp((char *)payload, "CLOSE", length))
      gateClose();
    else if (!strncmp((char *)payload, "OPEN", length))
      gateOpen();
    return;
  }
  if (!strcmp(topic, "adebar/carport/system/set"))
  {
    if (!strncmp((char *)payload, "RESTART", length))
    {
      ESP.restart();
    }
    else if (!strncmp((char *)payload, "DEBUG", length))
    {
      char buffer[100];
      radioB.debug(buffer);
      mqttPublish("adebar/carport/debug", buffer);
      radioC.debug(buffer);
      mqttPublish("adebar/carport/debug", buffer);
      radioD.debug(buffer);
      mqttPublish("adebar/carport/debug", buffer);
    }
    return;
  }
}

void mqttSendAdebarCarportGate(boolean full)
{
  if (full)
  {
    // see https://www.home-assistant.io/integrations/cover.mqtt/
    const char *discoveryConfig = "{"
                                  "\"name\":\"Hoftor\","
                                  "\"dev_cla\":\"gate\","
                                  "\"cmd_t\":\"adebar/carport/gate/set\","
                                  "\"stat_t\":\"adebar/carport/gate/state\","
                                  "\"uniq_id\":\"adebar_carport_gate\","

                                  "\"dev\":{"
                                  "\"identifiers\":[\"adebar_carport\"],"
                                  "\"name\":\"Carport\""
                                  "}"
                                  "}";
    mqttPublish("homeassistant/cover/adebar_carport_gate/config", discoveryConfig);
  }

  if (state == open)
    mqttPublish("adebar/carport/gate/state", "open");
  else if (state == close)
    mqttPublish("adebar/carport/gate/state", "closed");
  else if (state == stop)
    mqttPublish("adebar/carport/gate/state", "stopped");
  else if (state == opening)
    mqttPublish("adebar/carport/gate/state", "opening");
  else if (state == closing)
    mqttPublish("adebar/carport/gate/state", "closing");
  else if (state == unknown)
    mqttPublish("adebar/carport/gate/state", "unknown");
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
