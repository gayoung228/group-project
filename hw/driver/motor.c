#include "main.h"
#include "motor.h"

/* ------------------------------------------------------------------
 * 하드웨어 배선 (Pin Map 문서 기준)
 *
 *   모터 L PWM  : PB10  TIM2_CH3   -> TB6612FNG PWMA
 *   모터 L IN1  : PB5   GPIO Out   -> TB6612FNG AIN1
 *   모터 L IN2  : PB4   GPIO Out   -> TB6612FNG AIN2
 *   모터 R PWM  : PC7   TIM3_CH2   -> TB6612FNG PWMB
 *   모터 R IN1  : PA8   GPIO Out   -> TB6612FNG BIN1
 *   모터 R IN2  : PA9   GPIO Out   -> TB6612FNG BIN2
 *   드라이버 STBY: PB3  GPIO Out   -> TB6612FNG STBY
 *
 * ------------------------------------------------------------------ */

#define MOTOR_L_IN1_PORT    GPIOB
#define MOTOR_L_IN1_PIN     GPIO_PIN_5
#define MOTOR_L_IN2_PORT    GPIOB
#define MOTOR_L_IN2_PIN     GPIO_PIN_4

#define MOTOR_R_IN1_PORT    GPIOA
#define MOTOR_R_IN1_PIN     GPIO_PIN_8
#define MOTOR_R_IN2_PORT    GPIOA
#define MOTOR_R_IN2_PIN     GPIO_PIN_9

#define MOTOR_STBY_PORT     GPIOB
#define MOTOR_STBY_PIN      GPIO_PIN_3

/* 좌우 모터는 서로 마주 보게 장착되므로 한쪽은 배선상 방향이 반대다.
 * 실제로 조립한 뒤 전진 명령에서 한쪽만 반대로 돌면 이 값을 뒤집는다. */
#define MOTOR_L_INVERT      0
#define MOTOR_R_INVERT      0

/* 속도 지령의 최대값 [%] */
#define MOTOR_SPEED_MAX     100

/* 모터 개수 */
#define MOTOR_COUNT         2


/* CubeMX 가 생성한 타이머 핸들 */
extern TIM_HandleTypeDef htim2;   /* 좌측 모터 PWM */
extern TIM_HandleTypeDef htim3;   /* 우측 모터 PWM */


/* 모터 한 개의 하드웨어 정보를 묶어 두는 구조체 */
typedef struct
{
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
    TIM_HandleTypeDef *pwm_tim;
    uint32_t           pwm_channel;
    uint8_t            invert;
} motor_hw_t;


/* 좌우 모터의 배선 정보 테이블 */
static const motor_hw_t motor_hw[MOTOR_COUNT] =
{
    /* MOTOR_LEFT */
    {
        MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN,
        MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN,
        &htim2, TIM_CHANNEL_3,
        MOTOR_L_INVERT
    },
    /* MOTOR_RIGHT */
    {
        MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN,
        MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN,
        &htim3, TIM_CHANNEL_2,
        MOTOR_R_INVERT
    }
};

/* 현재 설정된 상태를 기억해 두는 변수 (get 함수에서 사용) */
static motor_direction_t motor_direction[MOTOR_COUNT];
static uint8_t           motor_speed[MOTOR_COUNT];
static bool              motor_ready = false;


/* 0~100% 속도값을 타이머 비교값으로 바꿔 PWM 듀티에 적용하는 내부 함수 */
static void motor_write_pwm(motor_t motor, uint8_t speed)
{
    uint32_t period;
    uint32_t compare;

    if (speed > MOTOR_SPEED_MAX)
    {
        speed = MOTOR_SPEED_MAX;
    }

    /* ARR 값을 읽어서 계산하므로 PWM 주파수를 바꿔도 코드 수정이 필요 없다 */
    period  = __HAL_TIM_GET_AUTORELOAD(motor_hw[motor].pwm_tim) + 1;
    compare = (period * speed) / MOTOR_SPEED_MAX;

    if (compare > 0)
    {
        compare = compare - 1;
    }

    __HAL_TIM_SET_COMPARE(motor_hw[motor].pwm_tim,
                          motor_hw[motor].pwm_channel,
                          compare);
}

/* IN1 / IN2 핀을 실제로 출력하는 내부 함수
 * invert 가 1 인 모터는 여기서 두 핀의 역할을 서로 바꾼다. */
static void motor_write_direction(motor_t motor, GPIO_PinState in1, GPIO_PinState in2)
{
    GPIO_PinState out1 = in1;
    GPIO_PinState out2 = in2;

    if (motor_hw[motor].invert != 0)
    {
        out1 = in2;
        out2 = in1;
    }

    HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, out1);
    HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, out2);
}


/* 모터 방향 핀을 정지 상태로 만들고 좌우 PWM을 시작한다. */
bool motor_init(void)
{
    motor_t motor;
    bool pwm_started = true;

    /* 초기화가 끝날 때까지 드라이버 출력을 막아 둔다 */
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

    for (motor = MOTOR_LEFT; motor < MOTOR_COUNT; motor++)
    {
        motor_direction[motor] = MOTOR_STOP;
        motor_speed[motor]     = 0;

        motor_write_direction(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);

        if (HAL_TIM_PWM_Start(motor_hw[motor].pwm_tim,
                              motor_hw[motor].pwm_channel) != HAL_OK)
        {
            pwm_started = false;
        }
        motor_write_pwm(motor, 0);
    }

    /* 두 PWM이 모두 준비됐을 때만 드라이버를 활성화한다. */
    motor_ready = pwm_started;
    HAL_GPIO_WritePin(MOTOR_STBY_PORT,
                      MOTOR_STBY_PIN,
                      motor_ready ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return motor_ready;
}

/* 모터 PWM 초기화 결과를 반환한다. */
bool motor_is_ready(void)
{
    return motor_ready;
}

/* 선택한 모터의 회전 방향을 설정한다. */
void motor_set_direction(motor_t motor, motor_direction_t direction)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    motor_direction[motor] = direction;

    if (direction == MOTOR_FORWARD)
    {
        motor_write_direction(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
    }
    else if (direction == MOTOR_REVERSE)
    {
        motor_write_direction(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
    }
    else
    {
        /* IN1 = IN2 = LOW : 출력을 끊어 관성으로 굴러가게 둔다 */
        motor_write_direction(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
}

/* 선택한 모터의 속도를 0~100% 범위로 설정한다. */
void motor_set_speed(motor_t motor, uint8_t speed)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    if (speed > MOTOR_SPEED_MAX)
    {
        speed = MOTOR_SPEED_MAX;
    }

    motor_speed[motor] = speed;
    motor_write_pwm(motor, speed);
}

/* 선택한 모터의 방향과 속도를 함께 설정한다. */
void motor_control(motor_t motor, motor_direction_t direction, uint8_t speed)
{
    motor_set_direction(motor, direction);
    motor_set_speed(motor, speed);
}

/* 좌우 모터를 모두 정지한다. */
void motor_stop_all(void)
{
    motor_stop(MOTOR_LEFT);
    motor_stop(MOTOR_RIGHT);
}

/* 부호를 포함한 -100~100 값으로 모터 방향과 속도를 함께 설정 */
void motor_set_output(motor_t motor, int16_t output)
{
    if (output > MOTOR_SPEED_MAX)
    {
        output = MOTOR_SPEED_MAX;
    }
    if (output < -MOTOR_SPEED_MAX)
    {
        output = -MOTOR_SPEED_MAX;
    }

    if (output > 0)
    {
        motor_control(motor, MOTOR_FORWARD, (uint8_t)output);
    }
    else if (output < 0)
    {
        motor_control(motor, MOTOR_REVERSE, (uint8_t)(-output));
    }
    else
    {
        motor_stop(motor);
    }
}

/* 선택한 모터 하나를 정지 */
void motor_stop(motor_t motor)
{
    motor_control(motor, MOTOR_STOP, 0);
}

/* 선택한 모터에 설정된 속도를 반환 */
uint8_t motor_get_speed(motor_t motor)
{
    if (motor >= MOTOR_COUNT)
    {
        return 0;
    }

    return motor_speed[motor];
}

/* 선택한 모터에 설정된 회전 방향을 반환 */
motor_direction_t motor_get_direction(motor_t motor)
{
    if (motor >= MOTOR_COUNT)
    {
        return MOTOR_STOP;
    }

    return motor_direction[motor];
}
