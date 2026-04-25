#include "app.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* Robot State/ Starting ON/OFF with USERB1 */
typedef enum {
    ROBOT_IDLE = 0,
    ROBOT_ACTIVE = 1
} Robot_State;

volatile Robot_State robot_state = ROBOT_IDLE;

/* HC-SR04 Ultra-sonic Sensor  */
#define TRIG_PORT GPIOA
#define TRIG_PIN  GPIO_PIN_6

#define ECHO_PORT GPIOB
#define ECHO_PIN  GPIO_PIN_9

/* Obstacle Collision Parameters */
#define OBSTACLE_DISTANCE_CM  8
#define SLOW_DISTANCE_CM      20

/* Motor Pins */
#define IN1_PORT GPIOB
#define IN1_PIN  GPIO_PIN_0

#define IN2_PORT GPIOB
#define IN2_PIN  GPIO_PIN_1

#define IN3_PORT GPIOB
#define IN3_PIN  GPIO_PIN_2

#define IN4_PORT GPIOB
#define IN4_PIN  GPIO_PIN_10

/* Three Line Following Sensors */
#define SENSOR_LEFT_PORT    GPIOC
#define SENSOR_LEFT_PIN     GPIO_PIN_0

#define SENSOR_CENTER_PORT  GPIOC
#define SENSOR_CENTER_PIN   GPIO_PIN_1

#define SENSOR_RIGHT_PORT   GPIOC
#define SENSOR_RIGHT_PIN    GPIO_PIN_3

/* Defined Speeds */
#define SPEED_FULL          360
#define SPEED_TURN_HARD     460
#define SPEED_RECOVER       350
#define SPEED_SOFT_TURN     420

#define RAMP_STEP_MS        8
#define RAMP_STEP_SIZE      8

/* Motor Control */
static void Set_Speed(uint32_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
}

static void Motors_Stop(void)
{
    Set_Speed(0);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
}

static void Drive_Forward(uint32_t speed)
{
    Set_Speed(speed);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
}

static void Turn_Left(uint32_t speed)
{
    Set_Speed(speed);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
}

static void Turn_Right(uint32_t speed)
{
    Set_Speed(speed);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
}

static void Soft_Left(uint32_t speed)  { Drive_Forward(speed); }
static void Soft_Right(uint32_t speed) { Drive_Forward(speed); }

static void Brake(void)
{
    Set_Speed(0);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    Motors_Stop();
}

/* Ultra-sonic Sensor Control */
static uint32_t measure_distance_cm(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    uint32_t timeout = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
        if (HAL_GetTick() - timeout > 30) return 999;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    timeout = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
        if (HAL_GetTick() - timeout > 30) return 999;

    return __HAL_TIM_GET_COUNTER(&htim1) / 58;
}

/* App_Init */
void App_Init(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    Motors_Stop();
    HAL_Delay(2000);
}

/* PC13EXTI: Button Interrupts Functionality, "Pausing" Logic */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        if (robot_state == ROBOT_IDLE)
        {
            robot_state = ROBOT_ACTIVE;
        }
        else
        {
            robot_state = ROBOT_IDLE;
            Motors_Stop();
        }
    }
}

/* ── MAIN LOOP ────────────────────────────── */
void App_MainLoop(void)
{
    /* ── SAFETY LOCK ── */
    if (robot_state == ROBOT_IDLE)
    {
        Motors_Stop();
        return;
    }

    static uint8_t has_moved       = 0;
    static uint8_t last_action     = 0;
    static uint8_t obstacle_active = 0;

    static uint32_t clear_timer  = 0;
    static uint32_t ramp_timer   = 0;
    static uint32_t current_speed = 0;

    /* Sensors */
    uint8_t left   = HAL_GPIO_ReadPin(SENSOR_LEFT_PORT, SENSOR_LEFT_PIN);
    uint8_t center = HAL_GPIO_ReadPin(SENSOR_CENTER_PORT, SENSOR_CENTER_PIN);
    uint8_t right  = HAL_GPIO_ReadPin(SENSOR_RIGHT_PORT, SENSOR_RIGHT_PIN);

    /* Begins on Black */
    if (left == GPIO_PIN_SET && center == GPIO_PIN_SET && right == GPIO_PIN_SET)
    {
        robot_state = ROBOT_ACTIVE;
    }

    /* Ultra-sonic Control */
    uint32_t distance = measure_distance_cm();

    if (distance < SLOW_DISTANCE_CM)
    {
        obstacle_active = 1;
        clear_timer = HAL_GetTick();

        if (HAL_GetTick() - ramp_timer >= RAMP_STEP_MS)
        {
            ramp_timer = HAL_GetTick();
            current_speed = (current_speed > RAMP_STEP_SIZE)
                ? current_speed - RAMP_STEP_SIZE
                : 0;
        }

        if (current_speed == 0) Motors_Stop();
        else Drive_Forward(current_speed);

        return;
    }

    if (obstacle_active)
    {
        if (HAL_GetTick() - clear_timer < 150)
        {
            Motors_Stop();
            return;
        }

        if (HAL_GetTick() - ramp_timer >= RAMP_STEP_MS)
        {
            ramp_timer = HAL_GetTick();
            current_speed = (current_speed + RAMP_STEP_SIZE < SPEED_FULL)
                ? current_speed + RAMP_STEP_SIZE
                : SPEED_FULL;
        }

        Drive_Forward(current_speed);

        if (current_speed >= SPEED_FULL)
            obstacle_active = 0;

        return;
    }

    current_speed = SPEED_FULL;

    /* Line-Following Logic, Cases of Each Three Modules in Combination */
    if (left == GPIO_PIN_RESET && center == GPIO_PIN_RESET && right == GPIO_PIN_RESET)
    {
        if (!has_moved) { Motors_Stop(); return; }

        if      (last_action == 1) Drive_Forward(SPEED_RECOVER);
        else if (last_action == 2) Turn_Left(SPEED_TURN_HARD);
        else if (last_action == 3) Turn_Right(SPEED_TURN_HARD);
        else Motors_Stop();
        return;
    }

    if (left == GPIO_PIN_RESET && center == GPIO_PIN_SET && right == GPIO_PIN_RESET)
    {
        Drive_Forward(SPEED_FULL);
        has_moved = 1; last_action = 1;
    }
    else if (left == GPIO_PIN_SET && center == GPIO_PIN_SET && right == GPIO_PIN_RESET)
    {
        Soft_Right(SPEED_SOFT_TURN);
        has_moved = 1; last_action = 3;
    }
    else if (left == GPIO_PIN_SET && center == GPIO_PIN_RESET && right == GPIO_PIN_RESET)
    {
        if (last_action == 1) Brake();
        Turn_Right(SPEED_TURN_HARD);
        has_moved = 1; last_action = 3;
    }
    else if (left == GPIO_PIN_RESET && center == GPIO_PIN_RESET && right == GPIO_PIN_SET)
    {
        if (last_action == 1) Brake();
        Turn_Left(SPEED_TURN_HARD);
        has_moved = 1; last_action = 2;
    }
    else if (left == GPIO_PIN_RESET && center == GPIO_PIN_SET && right == GPIO_PIN_SET)
    {
        Soft_Left(SPEED_SOFT_TURN);
        has_moved = 1; last_action = 2;
    }
    else if (left == GPIO_PIN_SET && center == GPIO_PIN_SET && right == GPIO_PIN_SET)
    {
        Drive_Forward(SPEED_FULL);
        has_moved = 1; last_action = 1;
    }
    else
    {
        Motors_Stop();
    }
}
