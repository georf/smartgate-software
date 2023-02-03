#include <Arduino.h>

class Switch
{
private:
    unsigned long _lastDebounceTime;
    void (*_pCallback)();
    bool _lastValue;
    uint8_t _pin;
    bool _state;
    uint16_t _threshold;
    bool _firstCallback;

public:
    void (*onHighCallback)();
    void (*onLowCallback)();
    void read(unsigned long currentMillis);
    void debug();
    Switch(const uint8_t pin, const uint16_t threshold, const bool normal);
};
