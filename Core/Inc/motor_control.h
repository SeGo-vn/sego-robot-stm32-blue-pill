#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef enum {
    MOTOR_DIRECTION_STOP = 0,
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_REVERSE
} MotorDirection;

void MotorControl_Set(uint8_t motor_index, MotorDirection direction);
void MotorControl_SetAll(MotorDirection motor1, MotorDirection motor2, MotorDirection motor3);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
