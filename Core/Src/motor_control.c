#include "motor_control.h"
#include "main.h"

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

void MotorControl_Set(uint8_t motor_index, MotorDirection direction)
{
    switch (motor_index) {
    case 1:
        MotorControl_WritePins(W1_A_GPIO_Port, W1_A_Pin, W1_B_Pin, direction);
        break;
    case 2:
        MotorControl_WritePins(W2_A_GPIO_Port, W2_A_Pin, W2_B_Pin, direction);
        break;
    case 3:
        MotorControl_WritePins(W3_A_GPIO_Port, W3_A_Pin, W3_B_Pin, direction);
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
