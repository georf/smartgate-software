#include <Arduino.h>
#include <MCP3XXX.h>

enum AdcTribleSwitchState
{
  nothing = 0,
  btn0 = 1,
  btn1 = 2,
  btn2 = 3
};

class AdcTribleSwitch
{
private:
  unsigned long _lastDebounceTime;
  void (*_pCallback)();
  AdcTribleSwitchState _lastValue;
  uint8_t _channel;
  AdcTribleSwitchState _state;
  uint16_t _threshold;
  bool _firstCallback;

public:
  MCP3008 *_adc;
  void (*onBtn0Callback)();
  void (*onBtn1Callback)();
  void (*onBtn2Callback)();
  void read(const unsigned long currentMillis);
  void debug();
  uint16_t pressTimeMillis = 50;
  AdcTribleSwitch(MCP3008 *adc, const uint8_t channel);
};
