#ifndef LedDriver_h
#define LedDriver_h

#include <Arduino.h>
#include <ShiftOutput.h>

#define SHIFT_PIN_LED_RED 0
#define LED_RED 0
#define SHIFT_PIN_LED_GREEN 1
#define LED_GREEN 1
#define SHIFT_PIN_LED_WARN0 6
#define LED_WARN0 2
#define SHIFT_PIN_LED_WARN1 7
#define LED_WARN1 3

enum LedDriverState
{
  off = 0,
  on = 1,
  blink = 2,
  fastBlink = 3,
  fastBlinkReverse = 4,
};

class LedDriver
{
private:
  LedDriverState _state = off;
  ShiftOutput *_shiftOutput;
  uint8_t _pin;
  unsigned long _turnOffSeconds;

public:
  void set(LedDriverState newState);
  void set(LedDriverState newState, uint16_t seconds);
  void handle(const unsigned long currentMillis);
  LedDriver(ShiftOutput *shiftOutput, const uint8_t pin);
};

class LedsHandler
{
private:
  LedDriver *_leds[4];
  ShiftOutput *_shiftOutput;

public:
  void set(const uint8_t led, LedDriverState newState);
  void set(const uint8_t led, LedDriverState newState, uint16_t seconds);
  void handle(const unsigned long currentMillis);
  LedsHandler(ShiftOutput *shiftOutput);
};

#endif