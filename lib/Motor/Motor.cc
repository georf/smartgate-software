#include "Motor.h"
#include <PubSubClient.h>

/*

at closing:

1000 _open
1010 setting.openAt  + STEP_TOLERANCE
1100 setting.openAt  + STEP_START_SOFTING
7000 setting.closeAt - STEP_END_SOFTING
7990 setting.closeAt - STEP_TOLERANCE
8000 setting.closeAt

at opening:

8000 setting.closeAt
7990 setting.closeAt - STEP_TOLERANCE
7900 setting.closeAt - STEP_START_SOFTING
2000 setting.openAt  + STEP_END_SOFTING
1010 setting.openAt  + STEP_TOLERANCE
1000 _open

*/

void Motor::begin(uint8_t motor_pin_open, uint8_t motor_pin_close, MCP3008 *adc, uint8_t adc_channel, MotorSetting &loaded)
{
    _motor_pin_open = motor_pin_open;
    _motor_pin_close = motor_pin_close;
    _adc = adc;
    _adc_channel = adc_channel;

    setting = loaded;

    // for startup
    target = close;
    state = unknown;

    pinMode(_motor_pin_open, OUTPUT);
    pinMode(_motor_pin_close, OUTPUT);
}

void Motor::handle(unsigned long current_millis)
{
    if ((_adc->analogRead(_adc_channel) > STEP_THRESHOLD) != _last_step_high)
    {
        _last_step_high = !_last_step_high;
        if (_last_target_opening)
            setting.currentSteps--;
        else
            setting.currentSteps++;

        if (target != stop)
        {
            unsigned long last3 = (current_millis - last_step_millis[2]);
            if (state == opening_soft || state == closing_soft)
            {
                if (last3 > SPEED_SOFT_MILLIS_LAST3)
                {
                    Serial.print("ERROR! SOFT ");
                    Serial.print(last3);
                    error();
                }
            }
            else if (state == opening || state == closing)
            {
                if (last3 > SPEED_FULL_MILLIS_LAST3)
                {
                    Serial.print("ERROR! FULL ");
                    Serial.print(last3);
                    error();
                }
            }
            last_step_millis[2] = last_step_millis[1];
            last_step_millis[1] = last_step_millis[0];
            last_step_millis[0] = current_millis;
        }
    }

    if (target == state)
        return;

    if (target == stop)
    {
        analogWrite(_motor_pin_open, SPEED_STOP);
        analogWrite(_motor_pin_close, SPEED_STOP);

        if (setting.currentSteps + STEP_TOLERANCE > setting.closeAt && setting.currentSteps - STEP_TOLERANCE < setting.closeAt)
        {
            state = close;
        }
        else if (setting.currentSteps + STEP_TOLERANCE > setting.openAt && setting.currentSteps - STEP_TOLERANCE < setting.openAt)
        {
            state = open;
        }
        else
        {
            state = stop;
        }
    }

    if (!_learn && (((state == opening || state == opening_soft) && setting.currentSteps < (setting.openAt + STEP_TOLERANCE)) ||
                    ((state == closing || state == closing_soft) && setting.currentSteps > (setting.closeAt - STEP_TOLERANCE))))
    {
        Serial.println("Debug: stopping");
        target = stop;
        analogWrite(_motor_pin_open, SPEED_STOP);
        analogWrite(_motor_pin_close, SPEED_STOP);
    }
    else if (target == open)
    {
        _last_target_opening = true;
        analogWrite(_motor_pin_close, SPEED_STOP);
        if (_learn ||
            ((state == opening || state == opening_soft) && setting.currentSteps < (setting.openAt + STEP_END_SOFTING)))
        {
            if (state != opening_soft)
            {
                Serial.println("Debug: opening_soft");
                state = opening_soft;
                analogWrite(_motor_pin_open, SPEED_SOFT);

                last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
            }
        }
        else if (state == close || state == stop || state == opening_soft)
        {
            if (setting.currentSteps > (setting.closeAt - STEP_START_SOFTING))
            {
                if (state != opening_soft)
                {
                    Serial.println("Debug: opening_soft");
                    state = opening_soft;
                    analogWrite(_motor_pin_open, SPEED_SOFT);

                    last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
                }
            }
            else
            {
                if (state != opening)
                {
                    Serial.println("Debug: opening");
                    state = opening;
                    analogWrite(_motor_pin_open, SPEED_FULL);

                    last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
                }
            }
        }
    }
    else if (target == close)
    {
        _last_target_opening = false;
        analogWrite(_motor_pin_open, SPEED_STOP);
        if (_learn ||
            ((state == closing || state == closing_soft) && setting.currentSteps > (setting.closeAt - STEP_END_SOFTING)))
        {
            if (state != closing_soft)
            {
                Serial.println("Debug: closing_soft");
                state = closing_soft;
                analogWrite(_motor_pin_close, SPEED_SOFT);

                last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
            }
        }
        else if (state == open || state == stop || state == closing_soft)
        {
            if (setting.currentSteps < (setting.openAt + STEP_START_SOFTING))
            {
                if (state != closing_soft)
                {
                    Serial.println("Debug: closing_soft");
                    state = closing_soft;
                    analogWrite(_motor_pin_close, SPEED_SOFT);

                    last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
                }
            }
            else
            {
                if (state != closing)
                {
                    Serial.println("Debug: closing");
                    state = closing;
                    analogWrite(_motor_pin_close, SPEED_FULL);

                    last_step_millis[0] = last_step_millis[1] = last_step_millis[2] = current_millis;
                }
            }
        }
    }
}

void Motor::learnStartOpen()
{
    _learn = true;
    setting.currentSteps = 100000;
    setting.openAt = 1000;
    setting.closeAt = 200000;
    target = open;
    state = unknown;
}

void Motor::learnOpen()
{
    setting.currentSteps = 1000;
    target = stop;
    state = unknown;
}

void Motor::learnStartClose()
{
    target = close;
    state = unknown;
}

void Motor::learnClose()
{
    setting.closeAt = setting.currentSteps;
    _learn = false;
    target = stop;
}


void Motor::error()
{
  if (startup) {
    setting.currentSteps = setting.closeAt;
    _learn = false;
    startup = false;
    target = stop;
  } else {
    errorCallback();
  }
}
