/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//THEANH
#include "motor_control.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//THEANH END
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//THEANH
#define ENCODER_CPR             52      // Counts per motor shaft revolution
#define GEAR_RATIO              34      // Gear ratio (34:1 or 75:1)
#define COUNTS_PER_WHEEL_REV    (ENCODER_CPR * GEAR_RATIO)  // 1768 for 1:34
#define WHEEL_DIAMETER_M        0.058   // 58mm wheel diameter
#define WHEEL_CIRCUMFERENCE_M   (M_PI * WHEEL_DIAMETER_M)  // 0.182 meters
#define ROBOT_RADIUS_M          0.105   // 10.5cm from center to wheel
// Empirical scale so encoder odometry matches 10/20cm tape tests (actual ≈ 77-86% of requested travel)
#define ODOM_DISTANCE_SCALE     0.902521f  // Updated from MOVE calibration (~0.885m actual for 1.0m odom)
#define METERS_PER_COUNT        ((WHEEL_CIRCUMFERENCE_M / COUNTS_PER_WHEEL_REV) * ODOM_DISTANCE_SCALE)
#define ODOMETRY_UPDATE_HZ      50      // Calculate odometry 50 times per second
#define ODOMETRY_UPDATE_MS      (1000 / ODOMETRY_UPDATE_HZ)
#define UART_BAUD_RATE          115200  // Baud rate for Raspberry Pi communication
#define DEG2RAD(x)              ((x) * (float)M_PI / 180.0f)
#define ANGULAR_RATE_TOL        0.05f   // rad/s tolerance for declaring rotation complete

// Wheel geometry mapped to motor indices (motor1 right/back, motor2 front, motor3 left/back)
static const float wheel_position_angles_deg[3] = { 60.0f, 180.0f, 300.0f };
static const float wheel_drive_angles_deg[3]    = { 150.0f, 270.0f, 30.0f };
//THEANH END
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//THEANH
typedef struct {
    int32_t count;          // Current encoder count
    int32_t prev_count;     // Previous encoder count
    int32_t delta_count;    // Change in count
    float velocity;         // Wheel velocity (m/s)
} EncoderData;

EncoderData motor1, motor2, motor3;
typedef struct {
    float x;                // X position (meters)
    float y;                // Y position (meters)
    float theta;            // Orientation (radians)
    float vx;               // Linear velocity X (m/s)
    float vy;               // Linear velocity Y (m/s)
    float omega;            // Angular velocity (rad/s)
} RobotOdometry;

RobotOdometry robot = {0};
uint32_t last_odometry_time = 0;
char uart_buffer[200];
uint8_t uart_rx_byte = 0;
volatile uint8_t uart_command_ready = 0;
size_t uart_command_length = 0;
char command_buffer[64];

typedef struct {
    uint8_t active;
    MotorDirection direction;
    float duty;  // PWM duty cycle (0.0 to 1.0)
    uint32_t end_time;
} MotorTimer;

MotorTimer motor_timers[3] = {0};

typedef struct {
    uint8_t active;
    float target_x;
    float target_y; 
    float target_theta;
    float tolerance_distance;  // meters
    float tolerance_angle;     // radians
    uint32_t timeout_ms;
    uint32_t start_time;
    uint8_t debug_logged_start;
    uint32_t last_debug_time;
} ClosedLoopTarget;

ClosedLoopTarget closed_loop_target = {0};
typedef struct {
    uint8_t active;
    uint32_t end_time;
    float target_theta;
    float base_pwm;
    uint32_t last_debug_time;
} TimedForwardTarget;

TimedForwardTarget timed_forward_target = {0};

typedef struct {
    uint8_t active;
    float target_distance;
    float start_x;
    float start_y;
    float base_pwm;
    uint32_t timeout_ms;
    uint32_t start_time;
    // PID state
    float pid_integral;
    float pid_previous_error;
    float pid_filtered_output;
} MoveTarget;

MoveTarget move_target = {0};

static const float TIMED_HEADING_GAIN = 0.6f;
float wheel_to_body_mat[3][3];   // Maps chassis twist to wheel speeds
float body_from_wheel_mat[3][3]; // Maps wheel speeds back to chassis twist
//THEANH END
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//THEANH
void Read_Encoders(void);
void Calculate_Odometry(float dt);
void Send_Odometry_Data(void);
void Process_Command(const char *command);
void MotorTimers_Update(void);
static int32_t Encoder_ComputeDelta(int32_t current_count, int32_t previous_count);
uint8_t IsAtTarget(void);
void UpdateClosedLoopControl(void);
void UpdateTimedForwardControl(void);
void UpdateMoveControl(void);
void Kinematics_Init(void);
static void Mat3MulVec(const float m[3][3], const float v[3], float out[3]);
static uint8_t Mat3Inverse(const float m[3][3], float inv[3][3]);
//THEANH END
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//THEANH
static int32_t Encoder_ComputeDelta(int32_t current_count, int32_t previous_count)
{
    int32_t delta = current_count - previous_count;
    if (delta > 32768) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }
    return delta;
}

void Read_Encoders(void)
{
    // Read current encoder counts
    motor1.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    motor2.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    motor3.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    // Calculate change in counts
    motor1.delta_count = Encoder_ComputeDelta(motor1.count, motor1.prev_count);
    motor2.delta_count = Encoder_ComputeDelta(motor2.count, motor2.prev_count);
    motor3.delta_count = Encoder_ComputeDelta(motor3.count, motor3.prev_count);

    // Update previous counts
    motor1.prev_count = motor1.count;
    motor2.prev_count = motor2.count;
    motor3.prev_count = motor3.count;
}

static void Mat3MulVec(const float m[3][3], const float v[3], float out[3])
{
    for (uint8_t i = 0; i < 3; ++i) {
        out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
    }
}

static uint8_t Mat3Inverse(const float m[3][3], float inv[3][3])
{
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
              - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
              + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabsf(det) < 1e-6f) {
        return 0;
    }

    float inv_det = 1.0f / det;
    inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * inv_det;
    inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;

    inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * inv_det;
    inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * inv_det;

    inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * inv_det;
    inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    return 1;
}

void Kinematics_Init(void)
{
    // Build kinematic rows in physical order: front (motor2), left (motor3), right (motor1)
    const uint8_t order[3] = {1, 2, 0}; // indices into wheel_*_angles arrays
    for (uint8_t row = 0; row < 3; ++row) {
        uint8_t i = order[row];
        float drive_rad = DEG2RAD(wheel_drive_angles_deg[i]);
        float pos_rad = DEG2RAD(wheel_position_angles_deg[i]);
        float tx = cosf(drive_rad);
        float ty = sinf(drive_rad);
        float rx = ROBOT_RADIUS_M * cosf(pos_rad);
        float ry = ROBOT_RADIUS_M * sinf(pos_rad);

        wheel_to_body_mat[row][0] = tx;
        wheel_to_body_mat[row][1] = ty;
        wheel_to_body_mat[row][2] = -ry * tx + rx * ty;
    }

    if (!Mat3Inverse(wheel_to_body_mat, body_from_wheel_mat)) {
        for (uint8_t r = 0; r < 3; ++r) {
            for (uint8_t c = 0; c < 3; ++c) {
                body_from_wheel_mat[r][c] = (r == c) ? 1.0f : 0.0f;
            }
        }
    }
}
void Calculate_Odometry(float dt)
{
    if (dt <= 0) return;  // Avoid division by zero

    // Calculate wheel velocities (m/s)
    float v1 = (motor1.delta_count * METERS_PER_COUNT) / dt;
    float v2 = (motor2.delta_count * METERS_PER_COUNT) / dt;
    float v3 = (motor3.delta_count * METERS_PER_COUNT) / dt;

    motor1.velocity = v1;
    motor2.velocity = v2;
    motor3.velocity = v3;

    float wheel_vector[3] = {v1, v2, v3};
    float body_vector[3] = {0.0f};
    Mat3MulVec(body_from_wheel_mat, wheel_vector, body_vector);

    robot.vx = body_vector[0];
    robot.vy = body_vector[1];
    robot.omega = body_vector[2];

    robot.theta += robot.omega * dt;

    while (robot.theta > M_PI) robot.theta -= 2.0f * M_PI;
    while (robot.theta < -M_PI) robot.theta += 2.0f * M_PI;

    float cos_theta = cosf(robot.theta);
    float sin_theta = sinf(robot.theta);

    float vx_world = robot.vx * cos_theta - robot.vy * sin_theta;
    float vy_world = robot.vx * sin_theta + robot.vy * cos_theta;

    robot.x += vx_world * dt;
    robot.y += vy_world * dt;
}
void Send_Odometry_Data(void)
{
    // Format: x,y,theta,vx,vy,omega,m1,m2,m3
    // Position in meters, angles in radians, velocities in m/s and rad/s
    sprintf(uart_buffer,
            "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%ld,%ld,%ld\r\n",
            robot.x, robot.y, robot.theta,
            robot.vx, robot.vy, robot.omega,
            motor1.delta_count, motor2.delta_count, motor3.delta_count);

    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

static MotorDirection Direction_FromToken(const char *token, uint8_t *valid)
{
    if (valid) {
        *valid = 0;
    }
    if (token == NULL || token[0] == '\0') {
        return MOTOR_DIRECTION_STOP;
    }
    char c = (char)toupper((unsigned char)token[0]);
    MotorDirection direction = MOTOR_DIRECTION_STOP;
    switch (c) {
    case 'F':
        direction = MOTOR_DIRECTION_FORWARD;
        break;
    case 'R':
        direction = MOTOR_DIRECTION_REVERSE;
        break;
    case 'S':
        direction = MOTOR_DIRECTION_STOP;
        break;
    default:
        return MOTOR_DIRECTION_STOP;
    }
    if (valid) {
        *valid = 1;
    }
    return direction;
}

static uint8_t ParseDurationMs(const char *token, uint32_t *duration_ms)
{
    if (token == NULL || duration_ms == NULL) {
        return 0;
    }
    char *endptr = NULL;
    unsigned long value = strtoul(token, &endptr, 10);
    if (endptr == token || (*endptr != '\0' && *endptr != '\r' && *endptr != '\n')) {
        return 0;
    }
    *duration_ms = (uint32_t)value;
    return 1;
}

static void MotorTimers_ClearAll(void)
{
    for (size_t i = 0; i < 3; ++i) {
        motor_timers[i].active = 0;
        motor_timers[i].direction = MOTOR_DIRECTION_STOP;
        motor_timers[i].end_time = 0;
    }
}

static void MotorTimer_Set(uint8_t motor_index, MotorDirection direction, uint32_t duration_ms)
{
    if (motor_index >= 3) {
        return;
    }
    if (direction == MOTOR_DIRECTION_STOP || duration_ms == 0) {
        motor_timers[motor_index].active = 0;
    } else {
        motor_timers[motor_index].active = 1;
        motor_timers[motor_index].direction = direction;
        motor_timers[motor_index].duty = 1.0f;  // Full power by default
        motor_timers[motor_index].end_time = HAL_GetTick() + duration_ms;
    }
    MotorControl_Set(motor_index + 1, direction);
}

static void MotorTimer_SetPwm(uint8_t motor_index, MotorDirection direction, float duty, uint32_t duration_ms)
{
    if (motor_index >= 3) {
        return;
    }
    if (direction == MOTOR_DIRECTION_STOP || duration_ms == 0) {
        motor_timers[motor_index].active = 0;
    } else {
        motor_timers[motor_index].active = 1;
        motor_timers[motor_index].direction = direction;
        motor_timers[motor_index].duty = duty;  // Custom PWM
        motor_timers[motor_index].end_time = HAL_GetTick() + duration_ms;
    }
    
    // Set direction and PWM directly
    switch (motor_index) {
        case 0:
            MotorControl_WritePins(W1_A_GPIO_Port, W1_A_Pin, W1_B_Pin, direction);
            MotorControl_SetDuty(1, duty);
            break;
        case 1:
            MotorControl_WritePins(W2_A_GPIO_Port, W2_A_Pin, W2_B_Pin, direction);
            MotorControl_SetDuty(2, duty);
            break;
        case 2:
            MotorControl_WritePins(W3_A_GPIO_Port, W3_A_Pin, W3_B_Pin, direction);
            MotorControl_SetDuty(3, duty);
            break;
    }
}

static void Send_Response(const char *message)
{
    if (message == NULL) {
        return;
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), 100);
}

//THEANH - Closed Loop Control Functions
uint8_t IsAtTarget(void) {
    if (!closed_loop_target.active) return 1;
    
    float dx = robot.x - closed_loop_target.target_x;
    float dy = robot.y - closed_loop_target.target_y;
    float distance_error = sqrtf(dx*dx + dy*dy);
    
    float angle_error = robot.theta - closed_loop_target.target_theta;
    // Normalize angle error to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0f * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0f * M_PI;
    angle_error = fabsf(angle_error);

    float angular_rate = fabsf(robot.omega);
    
    return (distance_error < closed_loop_target.tolerance_distance && 
            angle_error < closed_loop_target.tolerance_angle &&
            angular_rate < ANGULAR_RATE_TOL);
}

void UpdateClosedLoopControl(void) {
    if (!closed_loop_target.active) return;

    uint32_t now = HAL_GetTick();

    // Check timeout
    if ((now - closed_loop_target.start_time) > closed_loop_target.timeout_ms) {
        closed_loop_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP);
        Send_Response("TIMEOUT\r\n");
        return;
    }

    // Check if target reached
    if (IsAtTarget()) {
        closed_loop_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP);
        Send_Response("TARGET_REACHED\r\n");
        return;
    }

    // Proportional error
    float dx = closed_loop_target.target_x - robot.x;
    float dy = closed_loop_target.target_y - robot.y;
    float dtheta = closed_loop_target.target_theta - robot.theta;
    while (dtheta > M_PI) dtheta -= 2.0f * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0f * M_PI;
    float distance_error = sqrtf(dx*dx + dy*dy);

    // Convert world error into robot frame
    float cos_theta = cosf(robot.theta);
    float sin_theta = sinf(robot.theta);
    float dx_robot = dx * cos_theta + dy * sin_theta;
    float dy_robot = -dx * sin_theta + dy * cos_theta;

    // Gain settings
    const float linear_k = 1.0f;
    const float angular_k = 0.55f;
    const float max_speed = 0.8f;
    const float min_duty_linear = 0.45f; // From user's manual test
    const float min_duty_rotate = 0.48f; // Slightly above friction threshold
    const float min_duty_brake = 0.18f;

    float vx = linear_k * dx_robot;
    float vy = linear_k * dy_robot;
    float omega = angular_k * dtheta;

    // Clamp commands
    vx = fmaxf(fminf(vx, max_speed), -max_speed);
    vy = fmaxf(fminf(vy, max_speed), -max_speed);
    omega = fmaxf(fminf(omega, max_speed), -max_speed);

    if (!closed_loop_target.debug_logged_start) {
        sprintf(uart_buffer,
                "CL START tgt=(%.3f,%.3f,%.3f) cur=(%.3f,%.3f,%.3f)\r\n",
                closed_loop_target.target_x,
                closed_loop_target.target_y,
                closed_loop_target.target_theta,
                robot.x, robot.y, robot.theta);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        closed_loop_target.debug_logged_start = 1;
        closed_loop_target.last_debug_time = now;
    }

    float twist_cmd[3] = {vx, vy, omega};
    float wheel_cmds[3] = {0.0f};
    Mat3MulVec(wheel_to_body_mat, twist_cmd, wheel_cmds);


    float max_mag = fmaxf(fmaxf(fabsf(wheel_cmds[0]), fabsf(wheel_cmds[1])), fabsf(wheel_cmds[2]));
    if (max_mag > 1.0f) {
        wheel_cmds[0] /= max_mag;
        wheel_cmds[1] /= max_mag;
        wheel_cmds[2] /= max_mag;
    }

    // Apply deadband and remap to [min_duty, 1.0]
    float min_duty = min_duty_linear;
    if (distance_error < 0.05f) {
        float angle_mag = fabsf(dtheta);
        if (angle_mag >= 0.4f) {
            min_duty = min_duty_rotate;
        } else {
            float frac = angle_mag / 0.4f;
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            min_duty = min_duty_brake + frac * (min_duty_rotate - min_duty_brake);
        }
    }
    for (int i = 0; i < 3; i++) {
        float speed = wheel_cmds[i];
        if (fabsf(speed) > 0.01f) { // If not in deadband
            // Remap [0, 1] output to [min_duty, 1.0]
            float remapped_speed = min_duty + fabsf(speed) * (1.0f - min_duty);
            wheel_cmds[i] = (speed > 0) ? remapped_speed : -remapped_speed;
        }
    }

    if (now - closed_loop_target.last_debug_time >= 200) {
        sprintf(uart_buffer,
                "CL CMD dist=%.3f dth=%.3f body=(%.3f,%.3f,%.3f) wheels=(%.2f,%.2f,%.2f)\r\n",
                distance_error,
                dtheta,
                vx, vy, omega,
                wheel_cmds[0], wheel_cmds[1], wheel_cmds[2]);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        closed_loop_target.last_debug_time = now;
    }

    MotorControl_Command(2, wheel_cmds[0]); // row0: front wheel (motor 2)
    MotorControl_Command(3, wheel_cmds[1]); // row1: left wheel (motor 3)
    MotorControl_Command(1, wheel_cmds[2]); // row2: right wheel (motor 1)
}
//THEANH END

void UpdateTimedForwardControl(void) {
    if (!timed_forward_target.active) return;
    uint32_t now = HAL_GetTick();
    if (now >= timed_forward_target.end_time) {
        timed_forward_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP);
        Send_Response("TIMED_DONE\r\n");
        return;
    }

    float dtheta = timed_forward_target.target_theta - robot.theta;
    while (dtheta > M_PI) dtheta -= 2.0f * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0f * M_PI;

    float correction = TIMED_HEADING_GAIN * dtheta;
    float motor1_cmd = -timed_forward_target.base_pwm + correction;
    float motor2_cmd = 0.0f;
    float motor3_cmd =  timed_forward_target.base_pwm + correction;

    if (motor1_cmd > 1.0f) motor1_cmd = 1.0f;
    if (motor1_cmd < -1.0f) motor1_cmd = -1.0f;
    if (motor3_cmd > 1.0f) motor3_cmd = 1.0f;
    if (motor3_cmd < -1.0f) motor3_cmd = -1.0f;

    const float min_duty = 0.20f;
    if (fabsf(motor1_cmd) > 0.0f && fabsf(motor1_cmd) < min_duty)
        motor1_cmd = copysignf(min_duty, motor1_cmd);
    if (fabsf(motor3_cmd) > 0.0f && fabsf(motor3_cmd) < min_duty)
        motor3_cmd = copysignf(min_duty, motor3_cmd);

    MotorControl_Command(1, motor1_cmd);
    MotorControl_Command(2, motor2_cmd);
    MotorControl_Command(3, motor3_cmd);

    if (now - timed_forward_target.last_debug_time >= 250) {
        sprintf(uart_buffer,
                "TIMED CMD theta_err=%.3f cmds=(%.2f,%.2f,%.2f)\r\n",
                dtheta, motor1_cmd, motor2_cmd, motor3_cmd);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        timed_forward_target.last_debug_time = now;
    }
}

void UpdateMoveControl(void) {
    if (!move_target.active) return;

    uint32_t now = HAL_GetTick();

    // Check timeout
    if ((now - move_target.start_time) > move_target.timeout_ms) {
        move_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP);
        Send_Response("MOVE TIMEOUT\r\n");
        return;
    }

    float dx = robot.x - move_target.start_x;
    float dy = robot.y - move_target.start_y;
    float distance_traveled = sqrtf(dx*dx + dy*dy);

    if (distance_traveled >= move_target.target_distance) {
        move_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP, MOTOR_DIRECTION_STOP);
        Send_Response("MOVE TARGET_REACHED\r\n");
        return;
    }

    // PID controller for drift correction
    const float Kp = -0.0040f;
    const float Ki = -0.00015f;
    const float Kd = -0.0008f;
    const float MIN_PWM = 0.46f;
    const float MAX_PWM = 0.70f;
    const float OUTPUT_SMOOTHING = 0.25f;
    const float BASE_DRIVE_SKEW = 0.015f; // Bias to counter left drift

    int32_t error = motor1.delta_count + motor2.delta_count + motor3.delta_count;

    // Proportional term
    float p_term = Kp * (float)error;

    // Integral term
    move_target.pid_integral += (float)error;
    float i_term = Ki * move_target.pid_integral;

    // Derivative term
    float derivative = (float)error - move_target.pid_previous_error;
    float d_term = Kd * derivative;
    move_target.pid_previous_error = (float)error;

    float correction_pwm = p_term + i_term + d_term;
    // Low-pass filter the PID output so the correction wheel ramps instead of slamming
    correction_pwm = move_target.pid_filtered_output +
                     OUTPUT_SMOOTHING * (correction_pwm - move_target.pid_filtered_output);
    move_target.pid_filtered_output = correction_pwm;

    // Apply deadband compensation
    if (fabsf(correction_pwm) > 0.01f) { // Only apply correction if error is significant
        if (correction_pwm > 1.0f) correction_pwm = 1.0f;
        if (correction_pwm < -1.0f) correction_pwm = -1.0f;
        float output_pwm = MIN_PWM + fabsf(correction_pwm) * (MAX_PWM - MIN_PWM);
        if (output_pwm > MAX_PWM) {
            output_pwm = MAX_PWM;
        }
        correction_pwm = copysignf(output_pwm, correction_pwm);
    }

    // Apply base PWM to Motor 1 (Reverse) and Motor 3 (Forward) with slight skew for lateral bias
    float motor1_cmd = -(move_target.base_pwm + BASE_DRIVE_SKEW); // Motor 1 Reverse harder
    float motor3_cmd =  (move_target.base_pwm - BASE_DRIVE_SKEW); // Motor 3 Forward slightly softer

    if (motor1_cmd < -1.0f) motor1_cmd = -1.0f;
    if (motor3_cmd > 1.0f)  motor3_cmd = 1.0f;

    MotorControl_Command(1, motor1_cmd);
    MotorControl_Command(3, motor3_cmd);
    
    // Apply correction to Motor 2
    MotorControl_Command(2, correction_pwm);
}
void Process_Command(const char *command)
{
    if (command == NULL) {
        return;
    }

    char working_copy[64];
    strncpy(working_copy, command, sizeof(working_copy));
    working_copy[sizeof(working_copy) - 1] = '\0';

    char *token = strtok(working_copy, " ,");
    if (token == NULL) {
        return;
    }

    if (strcmp(token, "SET") == 0) {
        char *dir_tokens[3];
        for (int i = 0; i < 3; ++i) {
            dir_tokens[i] = strtok(NULL, " ,");
            if (dir_tokens[i] == NULL) {
                Send_Response("ERR SET params\r\n");
                return;
            }
        }
        uint8_t valid = 0;
        MotorDirection directions[3];
        for (int i = 0; i < 3; ++i) {
            directions[i] = Direction_FromToken(dir_tokens[i], &valid);
            if (!valid) {
                Send_Response("ERR SET dir\r\n");
                return;
            }
        }
        MotorTimers_ClearAll();
        MotorControl_SetAll(directions[0], directions[1], directions[2]);
        Send_Response("OK SET\r\n");
        return;
    }

    if (strcmp(token, "RUN") == 0) {
        char *param1 = strtok(NULL, " ,");
        char *param2 = strtok(NULL, " ,");
        char *param3 = strtok(NULL, " ,");
        char *param4 = strtok(NULL, " ,");
        if (param1 == NULL || param2 == NULL) {
            Send_Response("ERR RUN params\r\n");
            return;
        }
        MotorDirection directions[3];
        uint32_t duration_ms = 0;
        uint8_t valid = 0;
        if (param3 == NULL) {
            directions[0] = Direction_FromToken(param1, &valid);
            if (!valid || !ParseDurationMs(param2, &duration_ms)) {
                Send_Response("ERR RUN data\r\n");
                return;
            }
            directions[1] = directions[0];
            directions[2] = directions[0];
        } else if (param4 != NULL && strtok(NULL, " ,") == NULL) {
            directions[0] = Direction_FromToken(param1, &valid);
            if (!valid) {
                Send_Response("ERR RUN dir1\r\n");
                return;
            }
            directions[1] = Direction_FromToken(param2, &valid);
            if (!valid) {
                Send_Response("ERR RUN dir2\r\n");
                return;
            }
            directions[2] = Direction_FromToken(param3, &valid);
            if (!valid || !ParseDurationMs(param4, &duration_ms)) {
                Send_Response("ERR RUN data\r\n");
                return;
            }
        } else {
            Send_Response("ERR RUN format\r\n");
            return;
        }
        MotorTimers_ClearAll();
        for (uint8_t i = 0; i < 3; ++i) {
            MotorTimer_Set(i, directions[i], duration_ms);
        }
        Send_Response("OK RUN\r\n");
        return;
    }

    if (strcmp(token, "RUNM") == 0) {
        char *motor_idx_token = strtok(NULL, " ,");
        char *dir_token = strtok(NULL, " ,");
        char *duration_token = strtok(NULL, " ,");
        if (motor_idx_token == NULL || dir_token == NULL || duration_token == NULL || strtok(NULL, " ,") != NULL) {
            Send_Response("ERR RUNM params\r\n");
            return;
        }
        int motor_idx = atoi(motor_idx_token);
        if (motor_idx < 1 || motor_idx > 3) {
            Send_Response("ERR RUNM motor\r\n");
            return;
        }
        uint8_t valid = 0;
        MotorDirection direction = Direction_FromToken(dir_token, &valid);
        uint32_t duration_ms = 0;
        if (!valid || !ParseDurationMs(duration_token, &duration_ms)) {
            Send_Response("ERR RUNM data\r\n");
            return;
        }
        MotorTimer_Set((uint8_t)(motor_idx - 1), direction, duration_ms);
        Send_Response("OK RUNM\r\n");
        return;
    }

    if (strcmp(token, "RUNP") == 0) {
        // RUNP command: RUNP dir1 dir2 dir3 pwm duration_ms
        char *param1 = strtok(NULL, " ,");
        char *param2 = strtok(NULL, " ,");
        char *param3 = strtok(NULL, " ,");
        char *param4 = strtok(NULL, " ,");
        char *param5 = strtok(NULL, " ,");
        if (param1 == NULL || param2 == NULL) {
            Send_Response("ERR RUNP params\r\n");
            return;
        }
        MotorDirection directions[3];
        float duty = 0.0f;
        uint32_t duration_ms = 0;
        uint8_t valid = 0;
        
        if (param3 != NULL && param4 != NULL && param5 != NULL && strtok(NULL, " ,") == NULL) {
            // 5-param version: dir1 dir2 dir3 pwm duration
            directions[0] = Direction_FromToken(param1, &valid);
            if (!valid) {
                Send_Response("ERR RUNP dir1\r\n");
                return;
            }
            directions[1] = Direction_FromToken(param2, &valid);
            if (!valid) {
                Send_Response("ERR RUNP dir2\r\n");
                return;
            }
            directions[2] = Direction_FromToken(param3, &valid);
            if (!valid) {
                Send_Response("ERR RUNP dir3\r\n");
                return;
            }
            duty = atof(param4);
            if (duty <= 0.0f || duty > 1.0f) {
                Send_Response("ERR RUNP pwm\r\n");
                return;
            }
            if (!ParseDurationMs(param5, &duration_ms)) {
                Send_Response("ERR RUNP duration\r\n");
                return;
            }
        } else if (param3 != NULL && param4 == NULL) {
            // 3-param version: dir pwm duration
            directions[0] = Direction_FromToken(param1, &valid);
            if (!valid) {
                Send_Response("ERR RUNP dir\r\n");
                return;
            }
            duty = atof(param2);
            if (duty <= 0.0f || duty > 1.0f) {
                Send_Response("ERR RUNP pwm\r\n");
                return;
            }
            if (!ParseDurationMs(param3, &duration_ms)) {
                Send_Response("ERR RUNP duration\r\n");
                return;
            }
            directions[1] = directions[0];
            directions[2] = directions[0];
        } else {
            Send_Response("ERR RUNP format\r\n");
            return;
        }
        MotorTimers_ClearAll();
        for (uint8_t i = 0; i < 3; ++i) {
            MotorTimer_SetPwm(i, directions[i], duty, duration_ms);
        }
        Send_Response("OK RUNP\r\n");
        return;
    }

    if (strcmp(token, "STOP") == 0) {
        MotorTimers_ClearAll();
        closed_loop_target.active = 0; // Stop any active closed-loop control
        timed_forward_target.active = 0;
        move_target.active = 0;
        MotorControl_SetAll(MOTOR_DIRECTION_STOP,
                            MOTOR_DIRECTION_STOP,
                            MOTOR_DIRECTION_STOP);
        Send_Response("OK STOP\r\n");
        return;
    }

    if (strcmp(token, "RESET_ODOM") == 0) {
        MotorTimers_ClearAll();
        closed_loop_target.active = 0;
        timed_forward_target.active = 0;
        move_target.active = 0;

        robot.x = 0.0f;
        robot.y = 0.0f;
        robot.theta = 0.0f;
        robot.vx = 0.0f;
        robot.vy = 0.0f;
        robot.omega = 0.0f;
        last_odometry_time = HAL_GetTick();

        motor1.prev_count = motor1.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        motor2.prev_count = motor2.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        motor3.prev_count = motor3.count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        motor1.delta_count = 0;
        motor2.delta_count = 0;
        motor3.delta_count = 0;

        move_target.pid_integral = 0.0f;
        move_target.pid_previous_error = 0.0f;
        move_target.pid_filtered_output = 0.0f;

        Send_Response("OK RESET_ODOM\r\n");
        return;
    }

    if (strcmp(token, "MOVE") == 0) {
		char *distance_token = strtok(NULL, " ,");
		if (distance_token == NULL) {
			Send_Response("ERR MOVE params\r\n");
			return;
		}
		float distance = atof(distance_token);
		if (distance <= 0.0f) {
			Send_Response("ERR MOVE distance\r\n");
			return;
		}

		move_target.active = 1;
		move_target.target_distance = distance;
		move_target.start_x = robot.x;
		move_target.start_y = robot.y;
		move_target.base_pwm = 0.58f; // Slightly higher to hit full meter
		move_target.timeout_ms = 20000; // 20s timeout
		move_target.start_time = HAL_GetTick();
		move_target.pid_integral = 0.0f;
		move_target.pid_previous_error = 0.0f;
		move_target.pid_filtered_output = 0.0f;

		Send_Response("OK MOVE\r\n");
		return;
	}

    //THEANH - New Closed Loop Commands
    if (strcmp(token, "MOVE_DIST") == 0) {
        char *distance_token = strtok(NULL, " ,");
        if (distance_token == NULL || strtok(NULL, " ,") != NULL) {
            Send_Response("ERR MOVE_DIST params\r\n");
            return;
        }
        
        float distance = atof(distance_token);
        
        // Set target relative to current position (forward direction)
        float cos_theta = cosf(robot.theta);
        float sin_theta = sinf(robot.theta);
        
        closed_loop_target.active = 1;
        closed_loop_target.target_x = robot.x + distance * cos_theta;
        closed_loop_target.target_y = robot.y + distance * sin_theta;
        closed_loop_target.target_theta = robot.theta;
        closed_loop_target.tolerance_distance = 0.01; // 1cm tolerance
        closed_loop_target.tolerance_angle = 0.05;    // ~3 degree tolerance
        closed_loop_target.timeout_ms = 10000;        // 10 second timeout
        closed_loop_target.start_time = HAL_GetTick();
        closed_loop_target.debug_logged_start = 0;
        closed_loop_target.last_debug_time = HAL_GetTick();
Send_Response("Version 18 MOVE_DIST armed\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        
        Send_Response("OK MOVE_DIST\r\n");
        return;
    }

    if (strcmp(token, "MOVE_TIME") == 0) {
        char *duration_token = strtok(NULL, " ,");
        char *pwm_token = strtok(NULL, " ,");
        if (duration_token == NULL || strtok(NULL, " ,") != NULL) {
            Send_Response("ERR MOVE_TIME params\r\n");
            return;
        }
        float duration_s = atof(duration_token);
        if (duration_s <= 0.0f) {
            Send_Response("ERR MOVE_TIME duration\r\n");
            return;
        }
        float base_pwm = (pwm_token) ? atof(pwm_token) : 0.45f;
        if (base_pwm <= 0.0f || base_pwm > 1.0f) {
            Send_Response("ERR MOVE_TIME pwm\r\n");
            return;
        }

        MotorTimers_ClearAll();
        closed_loop_target.active = 0;
        timed_forward_target.active = 1;
        timed_forward_target.target_theta = robot.theta;
        timed_forward_target.base_pwm = base_pwm;
        timed_forward_target.end_time = HAL_GetTick() + (uint32_t)(duration_s * 1000.0f);
        timed_forward_target.last_debug_time = HAL_GetTick();

        sprintf(uart_buffer, "Version 13 MOVE_TIME %.2fs pwm=%.2f\r\n", duration_s, base_pwm);
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

        Send_Response("OK MOVE_TIME\r\n");
        return;
    }
    
    if (strcmp(token, "ROTATE_DEG") == 0) {
        char *angle_token = strtok(NULL, " ,");
        if (angle_token == NULL || strtok(NULL, " ,") != NULL) {
            Send_Response("ERR ROTATE_DEG params\r\n");
            return;
        }
        
        float angle_deg = atof(angle_token);
        float angle_rad = angle_deg * M_PI / 180.0f;
        
        closed_loop_target.active = 1;
        closed_loop_target.target_x = robot.x;  // Don't move position
        closed_loop_target.target_y = robot.y;
        closed_loop_target.target_theta = robot.theta + angle_rad;
        // Normalize target angle
        while (closed_loop_target.target_theta > M_PI) 
            closed_loop_target.target_theta -= 2.0f * M_PI;
        while (closed_loop_target.target_theta < -M_PI) 
            closed_loop_target.target_theta += 2.0f * M_PI;
            
        closed_loop_target.tolerance_distance = 0.02; // 2cm tolerance (allow some drift)
        closed_loop_target.tolerance_angle = 0.02;    // ~1 degree tolerance
        closed_loop_target.timeout_ms = 10000;
        closed_loop_target.start_time = HAL_GetTick();
        closed_loop_target.debug_logged_start = 0;
        closed_loop_target.last_debug_time = HAL_GetTick();
        
        Send_Response("OK ROTATE_DEG\r\n");
        return;
    }
    //THEANH END

    Send_Response("ERR UNKNOWN\r\n");
}

void MotorTimers_Update(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < 3; ++i) {
        if (motor_timers[i].active) {
            int32_t remaining = (int32_t)(motor_timers[i].end_time - now);
            if (remaining <= 0) {
                motor_timers[i].active = 0;
                MotorControl_Set(i + 1, MOTOR_DIRECTION_STOP);
            }
        }
    }
}
//THEANH END
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //THEANH
  MotorControl_InitPwm();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 32768);
  __HAL_TIM_SET_COUNTER(&htim3, 32768);
  __HAL_TIM_SET_COUNTER(&htim4, 32768);
  motor1.prev_count = 32768;
  motor2.prev_count = 32768;
  motor3.prev_count = 32768;
  sprintf(uart_buffer, "STM32 Omni Robot Odometry Started\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  last_odometry_time = HAL_GetTick();
  Kinematics_Init();
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
  MotorControl_SetAll(MOTOR_DIRECTION_STOP,
                      MOTOR_DIRECTION_STOP,
                      MOTOR_DIRECTION_STOP);
  //THEANH END
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //THEANH
	  if (uart_command_ready)
	  {
		  Process_Command(command_buffer);
		  uart_command_ready = 0;
	  }

	  MotorTimers_Update();
	  UpdateClosedLoopControl(); // New closed-loop control update
	  UpdateTimedForwardControl();
	  UpdateMoveControl();

	  uint32_t current_time = HAL_GetTick();
	  if (current_time - last_odometry_time >= ODOMETRY_UPDATE_MS)
	  {
		  float dt = (current_time - last_odometry_time) / 1000.0f; // Convert to seconds

		  Read_Encoders();
		  Calculate_Odometry(dt);
		  Send_Odometry_Data();

		  last_odometry_time = current_time;
	  }
	  //THEANH END
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, W1_A_Pin|W1_B_Pin|W2_B_Pin|W2_A_Pin
                          |W3_A_Pin|W3_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : W1_A_Pin W1_B_Pin W2_B_Pin W2_A_Pin
                           W3_A_Pin W3_B_Pin */
  GPIO_InitStruct.Pin = W1_A_Pin|W1_B_Pin|W2_B_Pin|W2_A_Pin
                          |W3_A_Pin|W3_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (!uart_command_ready)
    {
      if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
      {
        if (uart_command_length > 0)
        {
          if (uart_command_length >= sizeof(command_buffer))
          {
            command_buffer[sizeof(command_buffer) - 1] = '\0';
          }
          else
          {
            command_buffer[uart_command_length] = '\0';
          }
          uart_command_ready = 1;
        }
        uart_command_length = 0;
      }
      else
      {
        if (uart_command_length < sizeof(command_buffer) - 1)
        {
          command_buffer[uart_command_length++] = (char)uart_rx_byte;
        }
        else
        {
          uart_command_length = 0;
        }
      }
    }
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
