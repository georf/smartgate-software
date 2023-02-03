#include "Switch.h"

Switch::Switch(const uint8_t pin, const uint16_t threshold, const bool normal)
{
    _pin = pin;
    _state = normal;
    _threshold = threshold;
    _lastValue = normal;
    _lastDebounceTime = 0;
    _firstCallback = false;

    pinMode(pin, INPUT);
}

void Switch::read(unsigned long currentMillis)
{
    bool value = analogRead(_pin) > _threshold;
    if (value != _lastValue)
        _lastDebounceTime = currentMillis;

    if (((currentMillis - _lastDebounceTime) > 10 && value != _state) || !_firstCallback)
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

void Switch::debug()
{
    Serial.print("switch ");
    Serial.print(_pin);
    Serial.print(": ");
    Serial.print(analogRead(_pin));
    Serial.print(" (");
    Serial.print(_threshold);
    Serial.println(")");
}