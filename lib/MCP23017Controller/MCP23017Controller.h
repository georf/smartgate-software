#ifndef MCP23017CONTROLLER_H
#define MCP23017CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>

#define DEBOUNCE_MS 50

enum OutputModus
{
  disabled,
  off,
  on,
  blink,
  fastBlink,
  reverseFastBlink,
};

class MCP23017Controller
{
public:
  MCP23017Controller() {}
  void begin(uint8_t addr);
  void loop(unsigned long now);

  // Set pin mode. Optional: initialState (-1 = ignore) sets the initial input state
  // and debounced state to avoid triggering callbacks immediately after startup.
  void setPinMode(uint8_t pin, uint8_t mode, int8_t initialState = -1);
  void digitalWrite(uint8_t pin, bool newState);
  bool digitalRead(uint8_t pin);

  void setCallbackClick(uint8_t pin, void (*callback)());
  void setCallbackChange(uint8_t pin, void (*callback)());
  // configureClick: like setPinMode+setCallbackClick. Optional initialState as above.
  void configureClick(uint8_t pin, uint8_t mode, void (*callback)(), int8_t initialState = -1);
  void setOutputModus(uint8_t pin, OutputModus modus);

private:
  uint8_t _addr;
  uint8_t _gpioA, _gpioB;
  uint8_t _olatA, _olatB;


  struct Pin
  {
    uint8_t number;
    bool isInput;
    bool state;
    bool debouncedState;
    bool writeState;
    bool pendingWrite;
    unsigned long lastDebounce;
    void (*onClick)();
    void (*onChange)();
    OutputModus outputModus = disabled;
  } _pins[16];

  void readRegisters(unsigned long now);
  void writeRegisters();
  void writeRegister(uint8_t reg, uint8_t val);
  void handleOutputModus(uint8_t pin, unsigned long now);
  uint8_t readRegister(uint8_t reg);
};

#endif
