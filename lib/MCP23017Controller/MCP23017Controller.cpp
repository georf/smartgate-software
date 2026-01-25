#include "MCP23017Controller.h"

void MCP23017Controller::begin(uint8_t addr)
{
  _addr = addr;
  _gpioA = _gpioB = 0;
  _olatA = _olatB = 0;

  // Pins initialisieren (vermeidet undefiniertes Verhalten)
  for (int i = 0; i < 16; i++)
  {
    _pins[i].number = i;
    _pins[i].isInput = true;
    _pins[i].state = HIGH;
    _pins[i].debouncedState = HIGH;
    _pins[i].writeState = LOW;
    _pins[i].pendingWrite = false;
    _pins[i].lastDebounce = 0;
    _pins[i].onClick = NULL;
    _pins[i].onChange = NULL;
  }

  // Default: alle Input
  writeRegister(0x00, 0xFF); // IODIRA
  writeRegister(0x01, 0xFF); // IODIRB

  // Pullups aktivieren
  writeRegister(0x0C, 0xFF); // Port A
  writeRegister(0x0D, 0xFF); // Port B
}

void MCP23017Controller::setPinMode(uint8_t pin, uint8_t mode, int8_t initialState)
{
  bool input = mode != OUTPUT;
  _pins[pin].isInput = input;

  // Keep original pin number for internal array indexing
  uint8_t origPin = pin;
  uint8_t addr = 0x00; // iodirA
  uint8_t pinIndex = pin;
  if (pin >= 8)
  {
    addr = 0x01; // iodirB
    pinIndex = pin - 8;
  }

  uint8_t iodir = readRegister(addr);
  iodir = input ? (iodir | (1 << pinIndex)) : (iodir & ~(1 << pinIndex));
  writeRegister(addr, iodir);

  if (input)
  {
    addr = addr + 0x0C; // A => 0x0C ; B => 0x0D
    bool pullup = mode == INPUT_PULLUP;
    iodir = readRegister(addr);
    iodir = iodir | (pullup << pinIndex);
    writeRegister(addr, iodir);

    // If an initial state was provided, set internal state/debouncedState
    // to that value so callbacks aren't triggered immediately at startup.
    if (initialState != -1)
    {
      bool st = initialState ? HIGH : LOW;
      _pins[origPin].state = st;
      _pins[origPin].debouncedState = st;
      // Start debounce window from now to avoid immediate callbacks
      _pins[origPin].lastDebounce = millis();
    }
  }
}

void MCP23017Controller::digitalWrite(uint8_t pin, bool newState)
{
  if (_pins[pin].isInput)
    return;

  _pins[pin].writeState = newState;
  _pins[pin].pendingWrite = true;
}

bool MCP23017Controller::digitalRead(uint8_t pin)
{
  if (_pins[pin].pendingWrite)
    return _pins[pin].writeState;

  return _pins[pin].debouncedState;
}

void MCP23017Controller::setCallbackClick(uint8_t pin, void (*callback)())
{
  _pins[pin].onClick = callback;
}

void MCP23017Controller::setCallbackChange(uint8_t pin, void (*callback)())
{
  _pins[pin].onChange = callback;
}

void MCP23017Controller::configureClick(uint8_t pin, uint8_t mode, void (*callback)(), int8_t initialState)
{
  setPinMode(pin, mode, initialState);
  setCallbackClick(pin, callback);
}

void MCP23017Controller::setOutputModus(uint8_t pin, OutputModus modus)
{
  _pins[pin].outputModus = modus;
}

void MCP23017Controller::loop(unsigned long now)
{
  for (int i = 0; i < 16; i++)
  {
    if (!_pins[i].isInput && _pins[i].outputModus != disabled)
      handleOutputModus(i, now);
  }

  readRegisters(now);
  writeRegisters();
}

void MCP23017Controller::handleOutputModus(uint8_t pin, unsigned long now)
{
  if (_pins[pin].outputModus == off)
  {
    digitalWrite(pin, LOW);
  }
  else if (_pins[pin].outputModus == on)
  {
    digitalWrite(pin, HIGH);
  }
  else if (_pins[pin].outputModus == blink || _pins[pin].outputModus == fastBlink || _pins[pin].outputModus == reverseFastBlink)
  {
    unsigned long interval = (_pins[pin].outputModus == blink) ? 500 : 100; // Blink intervals
    bool state = ((now / interval) % 2) == 0;
    if (_pins[pin].outputModus == reverseFastBlink)
      state = !state;
    digitalWrite(pin, state);
  }
}

void MCP23017Controller::readRegisters(unsigned long now)
{
  _gpioA = readRegister(0x12);
  _gpioB = readRegister(0x13);

  for (int i = 0; i < 16; i++)
  {
    bool newState = i < 8 ? (_gpioA >> i) & 1 : (_gpioB >> (i - 8)) & 1;
    bool lastState = _pins[i].state;

    _pins[i].state = newState;

    if (!_pins[i].isInput)
      continue;

    if (newState != lastState)
      _pins[i].lastDebounce = now;

    if (_pins[i].debouncedState != newState && now - _pins[i].lastDebounce > DEBOUNCE_MS)
    {
      _pins[i].debouncedState = newState;
      if (_pins[i].onChange)
        _pins[i].onChange();
      if (!newState && _pins[i].onClick) // 0 heißt gedrückt
        _pins[i].onClick();
    }
  }
}

void MCP23017Controller::writeRegisters()
{
  uint8_t outA = 0, outB = 0;
  for (int i = 0; i < 8; i++)
  {
    if (!_pins[i].isInput)
    {
      if (_pins[i].pendingWrite)
      {
        outA |= (_pins[i].writeState << i);
        _pins[i].pendingWrite = false;
        _pins[i].debouncedState = _pins[i].writeState;
      }
      else
      {
        outA |= (_pins[i].state << i);
      }
    }
  }
  for (int i = 0; i < 8; i++)
  {
    if (!_pins[i + 8].isInput)
    {
      if (_pins[i + 8].pendingWrite)
      {
        outB |= (_pins[i + 8].writeState << i);
        _pins[i + 8].pendingWrite = false;
        _pins[i + 8].debouncedState = _pins[i + 8].writeState;
      }
      else
      {
        outB |= (_pins[i + 8].state << i);
      }
    }
  }
  // OLAT-Register für Ausgänge schreiben (besser als direkt GPIO)
  writeRegister(0x14, outA); // OLATA
  writeRegister(0x15, outB); // OLATB
}

void MCP23017Controller::writeRegister(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t MCP23017Controller::readRegister(uint8_t reg)
{
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_addr, (int)1);
  if (Wire.available())
    return Wire.read();
  return 0;
}
