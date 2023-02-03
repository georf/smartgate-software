#include "Arduino.h"

class ShiftOutput
{
private:
    // Shift register pins
    uint8_t _data_pin;
    uint8_t _load_pin;
    uint8_t _clock_pin;
    bool _dirty = true;


public:
    // hold current state
    byte dataState;

    void begin(uint8_t data_pin, uint8_t load_pin, uint8_t clock_pin);
    void write();
    void digitalSet(uint8_t pin, uint8_t value);
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);
};
