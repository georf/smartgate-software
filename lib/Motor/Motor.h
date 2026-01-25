#include "Arduino.h"
#include "MCP23017Controller.h"
#include <Adafruit_ADS1X15.h>
#define STEP_TOLERANCE 50
#define SPEED_FULL 255
#define SPEED_STOP 0
#define FORCED_DOWNTIME 1000
#define STARTUP_TIME 500
#define SAFETY_CURRENT_WINDOW 8
#define SAFETY_SUSTAIN_MS 100
#define CURRENT_ZERO 70   // ~ 0 Ampere
#define CURRENT_END_ERROR 750 // ~ 0.5 Ampere
#define CURRENT_RUN_ERROR 850 // ~ 0.5 Ampere

#define MOTOR_MAX_RUN_MS 60000UL                     // Motor darf nie länger als 1 Minute laufen
#define MOTOR_REED_DEADLINE_MS (MOTOR_MAX_RUN_MS / 2) // Reed muss spätestens nach der Hälfte erreicht werden

class Motor
{
private:
  // Pins auf ESP8266
  uint8_t _motorPinOpen;
  uint8_t _motorPinClose;

  MCP23017Controller *_mcp;
  uint8_t _reedPin;
  uint8_t _voltagePin;
  Adafruit_ADS1015 *_ads;
  uint8_t _adsChannel;

  // Gewünschte Richtung
  // Wird von ISR gelesen — als volatile markieren, damit die ISR immer den
  // aktuellen Wert sieht.
  volatile bool _targetOpening = false;
  
  // Start-Modus
  bool _startup = true;

  // Zeitpunkte für Laufzeitüberwachung
  unsigned long _runningStart;
  unsigned long _runningStop;

  // Umdrehungen für geschlossenen Zustand
  int _closeAt = 1000;
  // Undrehungen für offenen Zustand
  int _openAt = 6000;

  // Ringpuffer für zuletzt gemessene Stromwerte (raw ADC)
  int32_t _currentSum; // laufende Summe für schnellen Durchschnitt
  int16_t _currentSamples[SAFETY_CURRENT_WINDOW];
  uint8_t _currentSampleIndex;
  // Zeitpunkt, ab dem ein Überschreiten des Schwellenwertes andauert
  unsigned long _overThresholdSince;

  // Laufzeit- / Reed Überwachung
  bool _expectReedCheck = false; // prüfen ob Reed erwartet wird (wenn vorher vollständig offen/geschlossen)
  bool _reedSeen = false;        // Reed wurde einmal LOW gesehen

  // Interne Methoden

  // Setze die Geschwindigket
  void handleSpeedZone(unsigned long currentMillis);

  // Beachte die Strombegrenzung
  void handleSafetyCurrent(unsigned long currentMillis);

  // Zurücksetzen der Stromüberwachung
  void resetSafetyCurrent();

public:
  // Aktuelle Umdrehungen
  volatile long currentSteps = 1100;

  static void IRAM_ATTR isrLeft();
  static void IRAM_ATTR isrRight();

  static Motor *leftInstance;
  static Motor *rightInstance;

  // Callback für Fehlerzustand
  void (*errorCallback)(uint32_t milliWatt);

  // Läuft gerade
  bool _running = false;


  // Konstruktur mit allen Einstellungen
  void begin(unsigned long now,
             uint8_t motorPinOpen, uint8_t motorPinClose,
             MCP23017Controller *mcp, uint8_t reedPin, uint8_t voltagePin,
             Adafruit_ADS1015 *ads, uint8_t adsChannel,
             uint32_t openAt);

  // Loop-Methode
  void handle(unsigned long now);

  // Befehle
  void doOpen(unsigned long now);
  void doClose(unsigned long now);
  void doStop(unsigned long now);
  void doCloseABit(unsigned long now);

  // Zustandsprüfungen
  bool isOpenPosition();
  bool isClosePosition();
};