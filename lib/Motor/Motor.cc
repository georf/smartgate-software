#include "Motor.h"

Motor *Motor::leftInstance = nullptr;
Motor *Motor::rightInstance = nullptr;

void IRAM_ATTR Motor::isrLeft()
{
  if (leftInstance && leftInstance->_running)
  {
    if (leftInstance->_targetOpening)
      leftInstance->currentSteps++;
    else
      leftInstance->currentSteps--;
  }
}

void IRAM_ATTR Motor::isrRight()
{
  if (rightInstance && rightInstance->_running)
  {
    if (rightInstance->_targetOpening)
      rightInstance->currentSteps++;
    else
      rightInstance->currentSteps--;
  }
}

void Motor::begin(unsigned long now,
                  uint8_t motorPinOpen, uint8_t motorPinClose,
                  MCP23017Controller *mcp, uint8_t reedPin, uint8_t voltagePin,
                  Adafruit_ADS1015 *ads, uint8_t adsChannel,
                  uint32_t openAt)
{

  // Einstellungen übernehmen
  _motorPinOpen = motorPinOpen;
  _motorPinClose = motorPinClose;
  _mcp = mcp;
  _reedPin = reedPin;
  _voltagePin = voltagePin;
  _ads = ads;
  _adsChannel = adsChannel;
  _openAt = openAt;

  // Ausgabe-PINs konfigurieren
  pinMode(_motorPinOpen, OUTPUT);
  pinMode(_motorPinClose, OUTPUT);

  // Erstmal alle stoppen
  doStop(now);

  resetSafetyCurrent();

  // Zum Starten öffnen wir Stück
  // currentSteps wird von einer ISR verändert; Schreibzugriffen im
  // Hauptkontext müssen atomar erfolgen, daher Interrupts kurz sperren.
  noInterrupts();
  currentSteps = _openAt - STEP_TOLERANCE - STEP_TOLERANCE;
  interrupts();
  doOpen(now);
}

void Motor::handle(unsigned long now)
{
  // Wenn wir nicht laufen, brauchen wir auch nichts machen
  if (!_running)
    return;

  // Wenn sich die Spanung noch nicht aufgebaut hat, machen wir auch nichts
  if (_mcp->digitalRead(_voltagePin) == LOW)
  {
    analogWrite(_motorPinOpen, SPEED_STOP);
    analogWrite(_motorPinClose, SPEED_STOP);

    _runningStart = now; // Startzeitpunkt zurücksetzen

    return;
  }

  // Wenn letzter Stopp gerade erst war, warten wir kurz, um Induktivität abzubauen
  if (_runningStop + FORCED_DOWNTIME > now)
  {
    _runningStart = now; // Startzeitpunkt zurücksetzen
    return;
  }

  handleSpeedZone(now);
  handleSafetyCurrent(now);

  // --- Laufzeit-Timeout: Motor nie länger als MOTOR_MAX_RUN_MS laufen lassen
  if (now - _runningStart >= MOTOR_MAX_RUN_MS)
  {
    Serial.println("Motor-Laufzeit überschritten: Stop");
    doStop(now);
    if (errorCallback)
      errorCallback(0); // 0 als Indikator für Timeout
    return;
  }

  // --- Reed-Prüfung: wenn beim Start von Endlage erwartet, dann muss der Reed
  // spätestens bis zur Hälfte der Max-Laufzeit einmal an geschaltet
  // worden sein (auf LOW gezogen). Andernfalls Fehler.
  if (_expectReedCheck && !_reedSeen)
  {
    bool currentReed = _mcp->digitalRead(_reedPin);

    if (currentReed == LOW)
      _reedSeen = true;

    if (now >= MOTOR_REED_DEADLINE_MS + _runningStart && !_reedSeen)
    {
      Serial.println("Reed-Mittelpunkts-Schalter nicht rechtzeitig erreicht: Stop");
      doStop(now);
      if (errorCallback)
        errorCallback(1); // 1 als Indikator für Reed-Fehler
      return;
    }
  }

  long currentStepsCopy;
  noInterrupts();
  currentStepsCopy = currentSteps;
  interrupts();

  // Offen, also stoppen
  if (_targetOpening && currentStepsCopy >= _openAt)
  {
    doStop(now);

    // Beim ersten Öffnen, war das nur ein kleines Stück. Wir schließen.
    if (_startup)
    {
      _startup = false;

      doCloseABit(now);
    }
  }
}

void Motor::doOpen(unsigned long now)
{
  if (isOpenPosition())
    return;

  _targetOpening = true;
  _running = true;
  _runningStart = now;

  // Setup Reed / Laufzeit checks: only expect reed if we started from an end position
  _expectReedCheck = isOpenPosition() || isClosePosition();
  _reedSeen = false;
}

void Motor::doClose(unsigned long now)
{

  if (isClosePosition())
    return;

  _targetOpening = false;
  _running = true;
  _runningStart = now;

  // Setup Reed / Laufzeit checks: only expect reed if we started from an end position
  _expectReedCheck = isOpenPosition() || isClosePosition();
  _reedSeen = false;
}

void Motor::doStop(unsigned long now)
{
  analogWrite(_motorPinOpen, SPEED_STOP);
  analogWrite(_motorPinClose, SPEED_STOP);
  resetSafetyCurrent();
  _expectReedCheck = false;
  _running = false;
  _runningStop = now;
}

void Motor::doCloseABit(unsigned long now)
{
  // Schließe vielleicht bis Anschlag
  noInterrupts();
  currentSteps = _closeAt + STEP_TOLERANCE + STEP_TOLERANCE;
  interrupts();
  doClose(now);
}

bool Motor::isOpenPosition()
{
  long currentStepsCopy;
  noInterrupts();
  currentStepsCopy = currentSteps;
  interrupts();
  return currentStepsCopy >= _openAt - STEP_TOLERANCE;
}

bool Motor::isClosePosition()
{
  long currentStepsCopy;
  noInterrupts();
  currentStepsCopy = currentSteps;
  interrupts();
  return currentStepsCopy <= _closeAt + STEP_TOLERANCE;
}

void Motor::handleSpeedZone(unsigned long now)
{
  uint8_t speed = SPEED_FULL;

  unsigned long millisSinceStart = (now - _runningStart);
  if (millisSinceStart < STARTUP_TIME)
  {
    float x = (float)millisSinceStart / (float)STARTUP_TIME; // 0.0 ... 1.0
    float s = x * x * x * (x * (x * 6 - 15) + 10);           // smootherstep
    speed = (s * SPEED_FULL);
  }

  analogWrite(_motorPinOpen, _targetOpening ? speed : 0);
  analogWrite(_motorPinClose, !_targetOpening ? speed : 0);
}

void Motor::handleSafetyCurrent(unsigned long now)
{
  // Lese aktuellen Rohwert
  const int16_t rawCurrent = _ads->readADC_SingleEnded(_adsChannel);

  long currentStepsCopy;
  noInterrupts();
  currentStepsCopy = currentSteps;
  interrupts();
  boolean closed = !_targetOpening && currentStepsCopy < (_closeAt + STEP_TOLERANCE + STEP_TOLERANCE);

  // Aktualisiere Ringpuffer und laufende Summe
  _currentSum -= _currentSamples[_currentSampleIndex];
  _currentSamples[_currentSampleIndex] = rawCurrent;
  _currentSum += rawCurrent;
  _currentSampleIndex = (_currentSampleIndex + 1) % SAFETY_CURRENT_WINDOW;

  int32_t avg = _currentSum / SAFETY_CURRENT_WINDOW;

  // Bestimme Schwellenwert (gleich wie vorher, abhängig von Startphase / Endlage)
  int32_t threshold;
  if (closed)
    threshold = CURRENT_END_ERROR;
  else
    threshold = _runningStart + STARTUP_TIME * 2.5 > now ? CURRENT_RUN_ERROR * 2 : CURRENT_RUN_ERROR;

  // Bedingungen: a) aktueller Messwert ist dauerhaft (> SAFETY_SUSTAIN_MS) über threshold
  //              b) oder der Durchschnitt der letzten N Werte liegt über threshold
  bool triggerByAvg = (avg > threshold);
  bool triggerBySustain = false;

  if (rawCurrent > threshold)
  {
    if (_overThresholdSince == 0)
      _overThresholdSince = now;
    else if (now - _overThresholdSince >= SAFETY_SUSTAIN_MS)
      triggerBySustain = true;
  }
  else
  {
    _overThresholdSince = 0;
  }

  if (triggerByAvg || triggerBySustain)
  {

    doStop(now);
    
    // Beim ersten Öffnen irgendwo gegen gefahren (vielleicht schon ganz offen gewesen), war das nur ein kleines Stück. Wir schließen.
    if (_targetOpening && _startup)
    {
      _startup = false;

      doCloseABit(now);
    }

    // Ist komplett geschlossen, also okay
    else if (closed)
    {
      // Beim Erkennen als geschlossen: Stell den Zähler atomar auf die
      // definierte Endlage.
      noInterrupts();
      currentSteps = _closeAt;
      interrupts();
    }
    else
    {
      // Fehler-Callback mit Durchschnitt (oder roher Messung) aufrufen
      if (errorCallback)
        errorCallback((uint32_t)avg);
      mqttDebug("safety current triggered");
    }
    // Sicherheits-Reset: verhindern, dass wir sofort wieder triggern
    _overThresholdSince = 0;
  }
}

void Motor::resetSafetyCurrent()
{
  // Initialisiere Ringpuffer für Stromüberwachung
  _currentSum = 0;
  for (uint8_t i = 0; i < SAFETY_CURRENT_WINDOW; i++)
  {
    _currentSamples[i] = CURRENT_ZERO;
    _currentSum += CURRENT_ZERO;
  }
  _currentSampleIndex = 0;
  _overThresholdSince = 0;
}

void Motor::mqttDebug(const char *message)
{
  if (mqttDebugCallback)
  {
    long currentStepsCopy;
    noInterrupts();
    currentStepsCopy = currentSteps;
    interrupts();

    char buffer[50];
    _motorPinOpen == D6 ? snprintf(buffer, sizeof(buffer), "MotorL: %s, steps: %ld", message, currentStepsCopy)
                        : snprintf(buffer, sizeof(buffer), "MotorR: %s, steps: %ld", message, currentStepsCopy);
    mqttDebugCallback(buffer);
  }
}