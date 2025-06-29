#include "AdcTribleSwitch.h"

AdcTribleSwitch::AdcTribleSwitch(MCP3008 *adc, const uint8_t channel)
{
  _adc = adc;
  _channel = channel;
  _state = nothing;
  _lastValue = nothing;
  _lastDebounceTime = 0;
  _firstCallback = false;
}

void AdcTribleSwitch::read(const unsigned long currentMillis)
{
  const uint32_t tolerance = 50;
  const uint32_t btn0Value = 531;
  const uint32_t btn1Value = 295; 
  const uint32_t btn2Value = 160; 

  uint32_t analogValue = _adc->analogRead(_channel);
  AdcTribleSwitchState value = nothing;

  if (((btn0Value - tolerance) < analogValue) && (analogValue < (btn0Value + tolerance)))
    value = btn0;
  else if (((btn1Value - tolerance) < analogValue) && (analogValue < (btn1Value + tolerance)))
    value = btn1;
  else if (((btn2Value - tolerance) < analogValue) && (analogValue < (btn2Value + tolerance)))
    value = btn2;

  if (value != _lastValue)
    _lastDebounceTime = currentMillis;

  if (((currentMillis - _lastDebounceTime) > pressTimeMillis && value != _state) || !_firstCallback)
  {
    _firstCallback = true;
    _state = value;
    if (value == btn0 && onBtn0Callback)
      onBtn0Callback();
    else if (value == btn1 && onBtn1Callback)
      onBtn1Callback();
    else if (value == btn2 && onBtn2Callback)
      onBtn2Callback();
  }
  _lastValue = value;
}

void AdcTribleSwitch::debug()
{
  Serial.print("adc switch ");
  Serial.print(_channel);
  Serial.print(": ");
  Serial.print(_adc->analogRead(_channel));
}