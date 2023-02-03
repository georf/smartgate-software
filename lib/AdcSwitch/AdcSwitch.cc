#include "AdcSwitch.h"

AdcSwitch::AdcSwitch(MCP3008 *adc, const uint8_t channel, const uint16_t threshold, const bool normal)
{
    _adc = adc;
    _channel = channel;
    _threshold = threshold;
    _state = normal;
    _lastValue = normal;
    _lastDebounceTime = 0;
    _firstCallback = false;
}

void AdcSwitch::read(unsigned long currentMillis)
{
    bool value = _adc->analogRead(_channel) > _threshold;
    if (value != _lastValue)
        _lastDebounceTime = currentMillis;

    if (((currentMillis - _lastDebounceTime) > pressTimeMillis && value != _state) || !_firstCallback)
    {
        _firstCallback = true;
        _state = value;
        if (value && onHighCallback)
            onHighCallback();
        else if (!value && onLowCallback)
            onLowCallback();
    }
    _lastValue = value;
}

void AdcSwitch::debug()
{
    Serial.print("adc switch ");
    Serial.print(_channel);
    Serial.print(": ");
    Serial.print(_adc->analogRead(_channel));
    Serial.print(" (");
    Serial.print(_threshold);
    Serial.println(")");
}