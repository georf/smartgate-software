#include "ShiftOutput.h"

void ShiftOutput::begin(uint8_t data_pin, uint8_t load_pin, uint8_t clock_pin)
{
    dataState = 0b00001111; // relais output is inverted

    _data_pin = data_pin;
    _load_pin = load_pin;
    _clock_pin = clock_pin;

    pinMode(_data_pin, OUTPUT);
    pinMode(_load_pin, OUTPUT);
    pinMode(_clock_pin, OUTPUT);

    // write all states to 0
    write();
}

void ShiftOutput::write()
{
    if (_dirty)
    {
        ::digitalWrite(_load_pin, LOW);
        shiftOut(_data_pin, _clock_pin, MSBFIRST, dataState);
        ::digitalWrite(_load_pin, HIGH);
        _dirty = false;
    }
}

void ShiftOutput::digitalSet(uint8_t pin, uint8_t newValue)
{
    uint8_t oldValue = bitRead(dataState, pin);
    if (oldValue != newValue)
    {
        bitWrite(dataState, pin, newValue);
        _dirty = true;
    }
}

void ShiftOutput::digitalWrite(uint8_t pin, uint8_t newValue)
{
    digitalSet(pin, newValue);
    write();
}

uint8_t ShiftOutput::digitalRead(uint8_t pin)
{
    return bitRead(dataState, pin);
}
