#include "Arduino.h"
#include <MCP3XXX.h>

enum motor_target_or_state
{
    open = 1,
    opening = 2,
    opening_soft = 3,
    close = 4,
    closing = 5,
    closing_soft = 6,
    stop = 7,
    unknown = 8,
};

typedef struct
{
    uint32_t closeAt = 200000;
    uint32_t openAt = 1000;
    uint32_t currentSteps = 10000;
    motor_target_or_state state = unknown;
} MotorSetting;

#define STEP_TOLERANCE 10
#define STEP_END_SOFTING 200
#define STEP_START_SOFTING 100
#define STEP_THRESHOLD 1000
#define SPEED_FULL 255
#define SPEED_FULL_MILLIS_LAST3 100
#define SPEED_SOFT 100
#define SPEED_SOFT_MILLIS_LAST3 100
#define SPEED_STOP 0

class Motor
{
private:
    uint8_t _motor_pin_open;
    uint8_t _motor_pin_close;
    MCP3008 *_adc;
    uint8_t _adc_channel;
    bool _last_step_high = false;
    bool _last_target_opening = false;
    unsigned long last_step_millis[3] = {0, 0, 0};
    bool _learn = true;

public:
    MotorSetting setting;
    motor_target_or_state target = close;
    motor_target_or_state state = close;
    bool startup = true;
    void (*errorCallback)();
    void begin(uint8_t motor_pin_open, uint8_t motor_pin_close, MCP3008 *adc, uint8_t adc_channel, MotorSetting &loaded);
    void handle(unsigned long current_millis);
    void learnStartOpen();
    void learnOpen();
    void learnStartClose();
    void learnClose();
    void error();
};