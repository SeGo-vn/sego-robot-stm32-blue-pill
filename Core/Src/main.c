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
#define ODOM_DISTANCE_SCALE     0.872947f  // Calibrated from 0.187 m / 0.1778 m
#define METERS_PER_COUNT        ((WHEEL_CIRCUMFERENCE_M / COUNTS_PER_WHEEL_REV) * ODOM_DISTANCE_SCALE)
#define ODOMETRY_UPDATE_HZ      50      // Calculate odometry 50 times per second
#define ODOMETRY_UPDATE_MS      (1000 / ODOMETRY_UPDATE_HZ)
#define UART_BAUD_RATE          115200  // Baud rate for Raspberry Pi communication
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
} ClosedLoopTarget;

ClosedLoopTarget closed_loop_target = {0};
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

    robot.vx = (2.0f * v1 - v2 - v3) / 3.0f;
    robot.vy = (v2 - v3) / sqrtf(3.0f);
    robot.omega = (v1 + v2 + v3) / (3.0f * ROBOT_RADIUS_M);

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
        motor_timers[motor_index].end_time = HAL_GetTick() + duration_ms;
    }
    MotorControl_Set(motor_index + 1, direction);
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
    
    return (distance_error < closed_loop_target.tolerance_distance && 
            angle_error < closed_loop_target.tolerance_angle);
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

    // Convert world error into robot frame
    float cos_theta = cosf(robot.theta);
    float sin_theta = sinf(robot.theta);
    float dx_robot = dx * cos_theta + dy * sin_theta;
    float dy_robot = -dx * sin_theta + dy * cos_theta;

    // Gain settings (tune experimentally)
    const float linear_k = 1.0f;   // proportional gain for translation
    const float angular_k = 0.8f;  // proportional gain for heading
    const float max_speed = 1.0f;  // clamp speed commands to [-1, 1]
    const float deadband = 0.01f;  // ignore very small commands
    const float min_duty_linear = 0.18f;   // minimum duty for translation
    const float min_duty_rotate = 0.08f;   // minimum duty for pure rotation

    float vx = linear_k * dx_robot;
    float vy = linear_k * dy_robot;
    float omega = angular_k * dtheta;

    // Clamp commands
    vx = fmaxf(fminf(vx, max_speed), -max_speed);
    vy = fmaxf(fminf(vy, max_speed), -max_speed);
    omega = fmaxf(fminf(omega, max_speed), -max_speed);

    // Omni wheel inverse kinematics (same formulas as ROS bridge)
    float w1 = vx + ROBOT_RADIUS_M * omega;
    float w2 = -0.5f * vx + (sqrtf(3.0f) / 2.0f) * vy + ROBOT_RADIUS_M * omega;
    float w3 = -0.5f * vx - (sqrtf(3.0f) / 2.0f) * vy + ROBOT_RADIUS_M * omega;

    // Normalize so largest magnitude is 1
    float max_mag = fmaxf(fmaxf(fabsf(w1), fabsf(w2)), fabsf(w3));
    if (max_mag > 1.0f) {
        w1 /= max_mag;
        w2 /= max_mag;
        w3 /= max_mag;
    }

    int rotating_only = (fabsf(vx) < deadband && fabsf(vy) < deadband && fabsf(omega) >= deadband);
    float min_duty = rotating_only ? min_duty_rotate : min_duty_linear;

    if (fabsf(w1) < deadband) {
        w1 = 0.0f;
    } else if (fabsf(w1) < min_duty) {
        w1 = copysignf(min_duty, w1);
    }
    if (fabsf(w2) < deadband) {
        w2 = 0.0f;
    } else if (fabsf(w2) < min_duty) {
        w2 = copysignf(min_duty, w2);
    }
    if (fabsf(w3) < deadband) {
        w3 = 0.0f;
    } else if (fabsf(w3) < min_duty) {
        w3 = copysignf(min_duty, w3);
    }

    MotorControl_Command(1, w1);
    MotorControl_Command(2, w2);
    MotorControl_Command(3, w3);
}
//THEANH END

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

    if (strcmp(token, "STOP") == 0) {
        MotorTimers_ClearAll();
        closed_loop_target.active = 0; // Stop any active closed-loop control
        MotorControl_SetAll(MOTOR_DIRECTION_STOP,
                            MOTOR_DIRECTION_STOP,
                            MOTOR_DIRECTION_STOP);
        Send_Response("OK STOP\r\n");
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
        
        Send_Response("OK MOVE_DIST\r\n");
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
