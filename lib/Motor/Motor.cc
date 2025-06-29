#include "Motor.h"

void Motor::begin(uint8_t motorPinOpen, uint8_t motorPinClose, MCP3008 *adc, uint8_t adcSpeedChannel, uint8_t adcHallChannel, uint32_t openAt, unsigned long startDelay)
{

  // Einstellungen übernehmen
  _motorPinOpen = motorPinOpen;
  _motorPinClose = motorPinClose;
  _adc = adc;
  _adcSpeedChannel = adcSpeedChannel;
  _adcHallChannel = adcHallChannel;
  _openAt = openAt;
  _startDelay = startDelay;

  // Ausgabe-PINs konfigurieren
  pinMode(_motorPinOpen, OUTPUT);
  pinMode(_motorPinClose, OUTPUT);

  // Erstmal alle stoppen
  doStop();

  // Zum Starten öffnen wir Stück
  _currentSteps = _openAt - STEP_TOLERANCE - STEP_TOLERANCE;
  doOpen();
}

void Motor::handle(unsigned long currentMillis)
{
  // Wenn wir nicht laufen, brauchen wir auch nichts machen
  if (!_running)
    return;

  // Wenn sich die Spanung noch nicht aufgebaut hat, machen wir auch nichts
  if (milliVoltage < MIN_VOLTAGE)
  {
    analogWrite(_motorPinOpen, SPEED_STOP);
    analogWrite(_motorPinClose, SPEED_STOP);

    _runningStart = currentMillis; // Startzeitpunkt zurücksetzen

    return;
  }

  // Wenn letzter Stopp gerade erst war, warten wir kurz, um Induktivität abzubauen
  if (_runningStop + STARTUP_CURRENT_TIME*2 + _startDelay > currentMillis)
  {
    _runningStart = currentMillis; // Startzeitpunkt zurücksetzen
    return;
  }

  handleStepCounting();
  handleSpeedZone(currentMillis);
  handleSafetyCurrent(currentMillis);

  // Offen, also stoppen
  if (_targetOpening && _currentSteps >= _openAt)
  {
    Serial.print("_currentSteps");
    Serial.println(_currentSteps);
    doStop();

    // Beim ersten Öffnen, war das nur ein kleines Stück. Wir schließen.
    if (_startup)
    {
      _startup = false;
      _currentSteps = _closeAt + STEP_TOLERANCE + STEP_TOLERANCE;
      doClose();
    }
  }
}

void Motor::doOpen()
{
  if (isOpenPosition())
    return;

  Serial.print(_motorPinOpen);
  Serial.println(" doOpen");

  _avarageCurrent = CURRENT_ZERO;

  _targetOpening = true;
  _running = true;
  _runningStart = millis();
}

void Motor::doClose()
{

  if (isClosePosition())
    return;

  _targetOpening = false;
  _running = true;
  _runningStart = millis();

  _avarageCurrent = CURRENT_ZERO;

  Serial.print(_motorPinOpen);
  Serial.println(" doClose");
}

void Motor::doStop()
{
  Serial.print(_motorPinOpen);
  Serial.println(" doStop");

  analogWrite(_motorPinOpen, SPEED_STOP);
  analogWrite(_motorPinClose, SPEED_STOP);
  _avarageCurrent = CURRENT_ZERO;
  _running = false;
  _runningStop = millis();
}

bool Motor::isOpenPosition()
{
  return _currentSteps >= _openAt - STEP_TOLERANCE;
}

bool Motor::isClosePosition()
{
  return _currentSteps <= _closeAt + STEP_TOLERANCE;
}

void Motor::handleSpeedZone(unsigned long currentMillis)
{
  int stepsToTarget = _targetOpening ? (_openAt - _currentSteps) : (_currentSteps - _closeAt);
  uint8_t speed = (stepsToTarget <= 100) ? SPEED_SOFT : SPEED_FULL;

  unsigned long millisSinceStart = (currentMillis - _runningStart);
  if (millisSinceStart < 256 * 7)
    speed = millisSinceStart / 7;

  analogWrite(_motorPinOpen, _targetOpening ? speed : 0);
  analogWrite(_motorPinClose, !_targetOpening ? speed : 0);
}

void Motor::handleSafetyCurrent(unsigned long currentMillis)
{
  const uint32_t rawCurrent = _adc->analogRead(_adcHallChannel);
  _avarageCurrent = (10 * _avarageCurrent + rawCurrent) / 11;

  uint32_t realMilliCurrent;
  if (_avarageCurrent < CURRENT_ZERO)
    realMilliCurrent = (CURRENT_ZERO - _avarageCurrent) * 1000 / CURRENT_REAL_FACTOR;
  else
    realMilliCurrent = (_avarageCurrent - CURRENT_ZERO) * 1000 / CURRENT_REAL_FACTOR;

  uint32_t milliWatt = realMilliCurrent * milliVoltage / 1000;

  boolean closed = !_targetOpening && _currentSteps < (_closeAt + STEP_TOLERANCE);

  // Serial.printf("volt: %06d   watt: %06d  current: %05d, orig: %05d\n", milliVoltage, milliWatt, realMilliCurrent, _avarageCurrent);

  // Wenn Wert zu hoch, ist er irgendwo gegen gefahren
  uint32_t threshold;

  if (closed)
    threshold = POWER_THRESHOLD_END;
  else
    threshold = _runningStart + STARTUP_CURRENT_TIME > currentMillis ? POWER_THRESHOLD_STARTUP : POWER_THRESHOLD;

  if (milliWatt > threshold)
  {
    doStop();
    Serial.print("watt: ");
    Serial.println(milliWatt);
    Serial.println(threshold);

    // Ist komplett geschlossen, also okay
    if (closed)
    {
      Serial.print("geschlossen bei: ");
      Serial.print(_currentSteps);
      _currentSteps = _closeAt;
      Serial.print("ersetzt durch: ");
      Serial.println(_currentSteps);
    }
    else
    {
      errorCallback(milliWatt);
    }
  }
}

// Zählen wenn eine Umdrehung gemacht wurde
void Motor::handleStepCounting()
{
  uint32_t value = _adc->analogRead(_adcSpeedChannel);
  if ((value > STEP_THRESHOLD) != _lastStepHigh)
  {
    _lastStepHigh = !_lastStepHigh;

    if (_targetOpening)
      _currentSteps++;
    else
      _currentSteps--;

    // Serial.print("Channel: ");
    // Serial.print(_adcSpeedChannel);
    // Serial.print(" Steps: ");
    // Serial.println(_currentSteps);
  }
}
