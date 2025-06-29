#include "LedDriver.h"

LedDriver::LedDriver(ShiftOutput *shiftOutput, const uint8_t pin)
{
  _pin = pin;
  _shiftOutput = shiftOutput;
}

void LedDriver::set(LedDriverState newState)
{
  _state = newState;
  _turnOffSeconds = UINT32_MAX;
}

void LedDriver::set(LedDriverState newState, uint16_t seconds)
{
  _state = newState;
  _turnOffSeconds = (millis() / 1000) + seconds;
}

void LedDriver::handle(const unsigned long currentMillis)
{
  if (_turnOffSeconds != UINT32_MAX && currentMillis / 1000 > _turnOffSeconds)
  {
    _turnOffSeconds = UINT32_MAX;
    _state = off;
  }

  if (_state == off)
    _shiftOutput->digitalSet(_pin, LOW);
  else if (_state == on)
    _shiftOutput->digitalSet(_pin, HIGH);
  else if (_state == blink)
    _shiftOutput->digitalSet(_pin, currentMillis % 1000 < 500);
  else if (_state == fastBlink)
  _shiftOutput->digitalSet(_pin, currentMillis % 250 < 125);
  else if (_state == fastBlinkReverse)
  _shiftOutput->digitalSet(_pin, currentMillis % 250 >= 125);
}

LedsHandler::LedsHandler(ShiftOutput *shiftOutput)
{
  _shiftOutput = shiftOutput;
  _leds[0] = new LedDriver(_shiftOutput, SHIFT_PIN_LED_RED);
  _leds[1] = new LedDriver(_shiftOutput, SHIFT_PIN_LED_GREEN);
  _leds[2] = new LedDriver(_shiftOutput, SHIFT_PIN_LED_WARN0);
  _leds[3] = new LedDriver(_shiftOutput, SHIFT_PIN_LED_WARN1);
}

void LedsHandler::set(const uint8_t led, LedDriverState newState)
{
  _leds[led]->set(newState);
}
void LedsHandler::set(const uint8_t led, LedDriverState newState, uint16_t seconds)
{
  _leds[led]->set(newState, seconds);
}
void LedsHandler::handle(const unsigned long currentMillis)
{
  for (uint8_t i = 0; i < 4; i++)
    _leds[i]->handle(currentMillis);
  _shiftOutput->write();
}
