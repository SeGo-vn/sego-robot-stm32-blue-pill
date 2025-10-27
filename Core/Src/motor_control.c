#include "motor_control.h"
#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

static uint16_t motor_pwm_period = 0;

static void MotorControl_WritePins(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b, MotorDirection direction)
{
    switch (direction) {
    case MOTOR_DIRECTION_FORWARD:
        HAL_GPIO_WritePin(port, pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port, pin_b, GPIO_PIN_RESET);
        break;
    case MOTOR_DIRECTION_REVERSE:
        HAL_GPIO_WritePin(port, pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, pin_b, GPIO_PIN_SET);
        break;
    case MOTOR_DIRECTION_STOP:
    default:
        HAL_GPIO_WritePin(port, pin_a | pin_b, GPIO_PIN_RESET);
        break;
    }
}

void MotorControl_InitPwm(void)
{
    motor_pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

static void MotorControl_SetDuty(uint8_t motor_index, float duty)
{
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }
    uint16_t compare = (uint16_t)(duty * motor_pwm_period);
    switch (motor_index) {
    case 1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
        break;
    case 2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare);
        break;
    case 3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compare);
        break;
    default:
        break;
    }
}

void MotorControl_Set(uint8_t motor_index, MotorDirection direction)
{
    float duty = (direction == MOTOR_DIRECTION_STOP) ? 0.0f : 1.0f;
    switch (motor_index) {
    case 1:
        MotorControl_WritePins(W1_A_GPIO_Port, W1_A_Pin, W1_B_Pin, direction);
        MotorControl_SetDuty(1, duty);
        break;
    case 2:
        MotorControl_WritePins(W2_A_GPIO_Port, W2_A_Pin, W2_B_Pin, direction);
        MotorControl_SetDuty(2, duty);
        break;
    case 3:
        MotorControl_WritePins(W3_A_GPIO_Port, W3_A_Pin, W3_B_Pin, direction);
        MotorControl_SetDuty(3, duty);
        break;
    default:
        break;
    }
}

void MotorControl_SetAll(MotorDirection motor1, MotorDirection motor2, MotorDirection motor3)
{
    MotorControl_Set(1, motor1);
    MotorControl_Set(2, motor2);
    MotorControl_Set(3, motor3);
}

void MotorControl_Command(uint8_t motor_index, float speed)
{
    MotorDirection direction = MOTOR_DIRECTION_STOP;
    if (speed > 0.0f) {
        direction = MOTOR_DIRECTION_FORWARD;
    } else if (speed < 0.0f) {
        direction = MOTOR_DIRECTION_REVERSE;
    }
    MotorControl_Set(motor_index, direction);
    MotorControl_SetDuty(motor_index, fabsf(speed));
}
