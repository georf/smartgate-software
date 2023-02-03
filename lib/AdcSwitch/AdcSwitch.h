#include <Arduino.h>
#include <MCP3XXX.h>

class AdcSwitch
{
private:
    unsigned long _lastDebounceTime;
    void (*_pCallback)();
    bool _lastValue;
    uint8_t _channel;
    bool _state;
    uint16_t _threshold;
    bool _firstCallback;

public:
    MCP3008 *_adc;
    void (*onHighCallback)();
    void (*onLowCallback)();
    void read(unsigned long currentMillis);
    void debug();
    uint16_t pressTimeMillis = 100;
    AdcSwitch(MCP3008 *adc, const uint8_t channel, const uint16_t threshold, const bool normal);
};
