#include "Arduino.h"
#include <MCP3XXX.h>

#define STEP_TOLERANCE 50
#define STEP_END_SOFTING 200
#define STEP_START_SOFTING 100
#define STEP_THRESHOLD 1000
#define SPEED_FULL 255
#define SPEED_SOFT 255
#define SPEED_STOP 0
#define SPEED_STARTUP 30
#define CURRENT_ZERO 770 // 0 Ampere
#define CURRENT_REAL_FACTOR 57 // 57 => 1 Ampere
#define POWER_THRESHOLD_END 20000
#define POWER_THRESHOLD 29000
#define POWER_THRESHOLD_STARTUP 55000
#define MIN_VOLTAGE 22000
#define STARTUP_CURRENT_TIME 256*7

class Motor
{
private:
  // Pins auf ESP8266
  uint8_t _motorPinOpen;
  uint8_t _motorPinClose;

  // Handle für ADC
  MCP3008 *_adc;
  uint8_t _adcSpeedChannel;
  uint8_t _adcHallChannel;

  
  // Letzte Drehung war oben
  bool _lastStepHigh = false;
  
  // Gewünschte Richtung
  bool _targetOpening = false;

  unsigned long _runningStart; 
  unsigned long _runningStop; 
  
  
  // Umdrehungen für geschlossenen Zustand
  int _closeAt = 1000;
  // Undrehungen für offenen Zustand
  int _openAt = 6000;
  // Aktuelle Umdrehungen
  int _currentSteps = 1100;
  // Durchschnittswert hall
  uint32_t _avarageCurrent = 0;

  unsigned long _startDelay = 0; 
  
  // Interne Methoden
  
  // Zähle Umdrehungen
  void handleStepCounting();
  
  // Setze die Geschwindigket
  void handleSpeedZone(unsigned long currentMillis);

  // Beachte die Strombegrenzung
  void handleSafetyCurrent(unsigned long currentMillis);

public:
  // Callback für Fehlerzustand
  void (*errorCallback)(uint32_t milliWatt);
  
  // Letzer Messwert Volt
  uint32_t milliVoltage = 5000;
  
  // Läuft gerade
  bool _running = false;

  // Start-Modus
  bool _startup = true;
  
  // Konstruktur mit allen Einstallungen
  void begin(uint8_t motorPinOpen, uint8_t motorPinClose, MCP3008 *adc, uint8_t adcSpeedChannel, uint8_t adcHallChannel, uint32_t openAt, unsigned long startDelay);
  
  // Loop-Methode
  void handle(unsigned long currentMillis);

  // Befehle
  void doOpen();
  void doClose();
  void doStop();
  
  // Zustandsprüfungen
  bool isOpenPosition();
  bool isClosePosition();
};