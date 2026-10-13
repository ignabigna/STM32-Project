/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_IDLE = 0,
  STATE_SCANNING,
  STATE_HOMING_LOCKED,
  STATE_GO_TO_HOME,
  STATE_ERROR_LIMIT
} SystemState_t;

typedef enum {
  SCAN_PHASE_FORWARD = 0,   /* Girando hacia adelante (+2 vueltas) */
  SCAN_PHASE_PAUSE_FWD,     /* Pausa de 1s con motor frenado */
  SCAN_PHASE_REVERSE,       /* Girando hacia atrás (-2 vueltas) */
  SCAN_PHASE_PAUSE_REV      /* Pausa de 1s con motor frenado */
} ScanPhase_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- PARÁMETROS DEL ENCODER Y BARRIDO --- */
#define ENCODER_PPR            30     /* Estimación de pulsos por revolución del encoder */
/* En modo TI1 (x2) se generan 2 cuentas en el timer por cada pulso */
#define TICKS_PER_REV          (ENCODER_PPR * 2) /* 60 cuentas por vuelta */
#define SWEEP_REVS             2      /* 2 vueltas hacia cada sentido */
#define SWEEP_TARGET_TICKS     (TICKS_PER_REV * SWEEP_REVS) /* 120 pulsos para 2 vueltas completas */
#define SWEEP_PAUSE_MS         1000   /* 1 segundo de retardo antes de cambiar de sentido */
#define SWEEP_DUTY             350    /* Fuerza restaurada a 350 (sobre 999) */

/* Límite de seguridad amplio para no trabar pruebas de banco */
#define MAX_ENCODER_LIMIT      10000

/* --- PARÁMETROS DEL CONTROLADOR DE POSICIÓN (IR A HOME) --- */
#define POS_DEADBAND_TICKS     5      /* Tolerancia de llegada en pulsos */
#define POS_MIN_DUTY           250    /* Fuerza mínima para vencer fricción en retorno */
#define POS_MAX_DUTY           350    /* Velocidad máxima de retorno */

/* Inversión de signo si el encoder cuenta en sentido contrario al motor */
#define ENCODER_POLARITY       1      /* Usar 1 o -1 según el cableado */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Estado del sistema y variables de posición */
volatile SystemState_t current_state = STATE_IDLE;
volatile int32_t total_encoder_ticks = 0;   /* Posición acumulada en 32 bits (anti-desbordamiento) */
volatile int16_t last_timer_cnt = 0;        /* Última lectura directa del registro CNT (16 bits) */
volatile int32_t home_position = 0;         /* Posición de referencia fijada */

/* Variables para el barrido oscilante por PPR */
volatile ScanPhase_t scan_phase = SCAN_PHASE_FORWARD;
int32_t leg_start_ticks = 0;                /* Posición de inicio de cada tramo de giro */
uint32_t scan_pause_timer = 0;              /* Temporizador no bloqueante para el delay de 1s */

/* Variables para control inteligente y anti-embalamiento de Go to Home */
volatile int8_t motor_dir_sign = 1;         /* Signo de dirección de motor (+1 o -1 auto-calibrado) */
int32_t go_home_initial_error = 0;
uint32_t go_home_start_time = 0;

/* Estados de anti-rebote no bloqueante para los 4 pulsadores */
uint8_t btn1_last = GPIO_PIN_SET;
uint8_t btn2_last = GPIO_PIN_SET;
uint8_t btn3_last = GPIO_PIN_SET;
uint8_t btn4_last = GPIO_PIN_SET;
uint32_t btn1_debounce_time = 0;
uint32_t btn2_debounce_time = 0;
uint32_t btn3_debounce_time = 0;
uint32_t btn4_debounce_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_SetSpeed(int16_t speed);
void Laser_Set(uint8_t state);
void Encoder_Update(void);
uint8_t Button_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_state, uint32_t *last_time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Ajusta el sentido de giro y la velocidad del motor DC con el puente H L298N
  * @param  speed: Valor entre -999 y +999 (el período de TIM3 es 999)
  *                speed > 0 : Giro adelante (IN1=1, IN2=0)
  *                speed < 0 : Giro atrás    (IN1=0, IN2=1)
  *                speed == 0: Freno activo  (IN1=0, IN2=0, PWM=0)
  */
void Motor_SetSpeed(int16_t speed)
{
  if (speed > 999) speed = 999;
  if (speed < -999) speed = -999;

  if (speed > 0)
  {
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)speed);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(-speed));
  }
  else
  {
    /* Freno activo */
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  }
}

/**
  * @brief  Enciende o apaga el puntero láser montado en el eje
  * @param  state: 1 para encender, 0 para apagar
  */
void Laser_Set(uint8_t state)
{
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  Calcula los pulsos acumulados en 32 bits a partir de los 16 bits de TIM2
  */
void Encoder_Update(void)
{
  int16_t current_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta = current_cnt - last_timer_cnt;
  last_timer_cnt = current_cnt;
  total_encoder_ticks += (ENCODER_POLARITY * delta);
}

/**
  * @brief  Detector de pulsación (flanco de bajada) con anti-rebote no bloqueante
  * @retval 1 si se detectó pulsación, 0 en caso contrario
  */
uint8_t Button_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_state, uint32_t *last_time)
{
  uint8_t current = HAL_GPIO_ReadPin(port, pin);
  uint8_t triggered = 0;

  if (current != *last_state)
  {
    if ((HAL_GetTick() - *last_time) > 40) /* 40 ms de ventana anti-rebote */
    {
      *last_time = HAL_GetTick();
      if (current == GPIO_PIN_RESET && *last_state == GPIO_PIN_SET)
      {
        triggered = 1; /* Pulsado (nivel bajo) */
      }
      *last_state = current;
    }
  }
  return triggered;
}
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
  /* USER CODE BEGIN 2 */
  /* Iniciar PWM de velocidad de motor (TIM3 Canal 1 en PA6 -> ENA del L298N) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* Iniciar lectura de Encoder en TIM2 (PA0 y PA1) */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /* Asegurar actuadores apagados al arrancar */
  Motor_SetSpeed(0);
  Laser_Set(0);

  /* Resetear contadores de posición y variables de referencia */
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  last_timer_cnt = 0;
  total_encoder_ticks = 0;
  home_position = 0;
  current_state = STATE_IDLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Actualizar el conteo continuo del encoder (32 bits anti-desbordamiento) */
    Encoder_Update();

    /* 2. Lectura no bloqueante con anti-rebote de los 4 pulsadores (activos en nivel bajo) */
    uint8_t btn1 = Button_Pressed(BTN1_SCAN_GPIO_Port, BTN1_SCAN_Pin, &btn1_last, &btn1_debounce_time);
    uint8_t btn2 = Button_Pressed(BTN2_SET_H_GPIO_Port, BTN2_SET_H_Pin, &btn2_last, &btn2_debounce_time);
    uint8_t btn3 = Button_Pressed(BTN3_GOH_GPIO_Port, BTN3_GOH_Pin, &btn3_last, &btn3_debounce_time);
    uint8_t btn4 = Button_Pressed(BTN4_STOP_GPIO_Port, BTN4_STOP_Pin, &btn4_last, &btn4_debounce_time);

    /* 3. PARADA DE EMERGENCIA / STOP (BTN4 tiene prioridad sobre cualquier estado) */
    if (btn4)
    {
      Motor_SetSpeed(0);
      Laser_Set(0);
      current_state = STATE_IDLE;
    }

    /* 4. VERIFICACIÓN DE LÍMITE DE SEGURIDAD (Protección contra corte de cables del láser) */
    if (current_state == STATE_SCANNING || current_state == STATE_GO_TO_HOME)
    {
      if (abs(total_encoder_ticks - home_position) >= MAX_ENCODER_LIMIT)
      {
        Motor_SetSpeed(0);
        Laser_Set(0);
        current_state = STATE_ERROR_LIMIT;
      }
    }

    /* 5. MÁQUINA DE ESTADOS */
    switch (current_state)
    {
      case STATE_IDLE:
        Motor_SetSpeed(0);
        Laser_Set(0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED apagado en reposo */

        /* BTN1: Iniciar barrido oscilante basado en vueltas (PPR) */
        if (btn1)
        {
          leg_start_ticks = total_encoder_ticks; /* Origen de este tramo */
          scan_phase = SCAN_PHASE_FORWARD;
          Laser_Set(1); /* Encender LED testigo en PB0 */
          Motor_SetSpeed(SWEEP_DUTY);
          current_state = STATE_SCANNING;
        }
        /* BTN2: Fijar Home en reposo si se desea calibrar aquí */
        else if (btn2)
        {
          total_encoder_ticks = 0;
          home_position = 0;
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          last_timer_cnt = 0;
          current_state = STATE_HOMING_LOCKED;
        }
        /* BTN3: Regresar a la posición de Home memorizada */
        else if (btn3)
        {
          go_home_initial_error = abs(home_position - total_encoder_ticks);
          go_home_start_time = HAL_GetTick();
          current_state = STATE_GO_TO_HOME;
        }
        break;

      case STATE_SCANNING:
        Laser_Set(1); /* LED testigo encendido mientras escanea */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED PC13 encendido */

        /* Sub-máquina de estados de barrido por vueltas (PPR) */
        switch (scan_phase)
        {
          case SCAN_PHASE_FORWARD:
            Motor_SetSpeed(SWEEP_DUTY);
            /* Verificar si completó 2 vueltas relativas hacia adelante */
            if (abs(total_encoder_ticks - leg_start_ticks) >= SWEEP_TARGET_TICKS)
            {
              Motor_SetSpeed(0); /* Freno activo */
              scan_pause_timer = HAL_GetTick();
              scan_phase = SCAN_PHASE_PAUSE_FWD;
            }
            break;

          case SCAN_PHASE_PAUSE_FWD:
            Motor_SetSpeed(0); /* Freno durante el delay de 1 segundo */
            if (HAL_GetTick() - scan_pause_timer >= SWEEP_PAUSE_MS)
            {
              leg_start_ticks = total_encoder_ticks; /* Origen para el tramo en reversa */
              Motor_SetSpeed(-SWEEP_DUTY); /* Iniciar giro en reversa con fuerza */
              scan_phase = SCAN_PHASE_REVERSE;
            }
            break;

          case SCAN_PHASE_REVERSE:
            Motor_SetSpeed(-SWEEP_DUTY);
            /* Verificar si completó 2 vueltas relativas hacia atrás */
            if (abs(total_encoder_ticks - leg_start_ticks) >= SWEEP_TARGET_TICKS)
            {
              Motor_SetSpeed(0); /* Freno activo */
              scan_pause_timer = HAL_GetTick();
              scan_phase = SCAN_PHASE_PAUSE_REV;
            }
            break;

          case SCAN_PHASE_PAUSE_REV:
            Motor_SetSpeed(0); /* Freno durante el delay de 1 segundo */
            if (HAL_GetTick() - scan_pause_timer >= SWEEP_PAUSE_MS)
            {
              leg_start_ticks = total_encoder_ticks; /* Origen para el siguiente tramo adelante */
              Motor_SetSpeed(SWEEP_DUTY); /* Reiniciar giro hacia adelante */
              scan_phase = SCAN_PHASE_FORWARD;
            }
            break;
        }

        /* BTN2: Fijar Home (Simula el impacto del láser en el sensor óptico) */
        if (btn2)
        {
          Motor_SetSpeed(0);
          Laser_Set(0); /* Apaga LED testigo */

          /* Fijar el origen de coordenadas exactamente en este punto */
          total_encoder_ticks = 0;
          home_position = 0;
          __HAL_TIM_SET_COUNTER(&htim2, 0);
          last_timer_cnt = 0;

          current_state = STATE_HOMING_LOCKED;
        }
        break;

      case STATE_HOMING_LOCKED:
        /* Señalización visual: destello breve del LED integrado */
        Motor_SetSpeed(0);
        Laser_Set(0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        current_state = STATE_IDLE; /* Regresa a reposo con el cero memorizado */
        break;

      case STATE_GO_TO_HOME:
        Laser_Set(0); /* Láser apagado durante el reposicionamiento */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        {
          /* Error de posición respecto al origen fijado (0) */
          int32_t error = home_position - total_encoder_ticks;
          int32_t abs_err = abs(error);

          /* 1. Verificación de llegada dentro de la zona muerta */
          if (abs_err <= POS_DEADBAND_TICKS)
          {
            Motor_SetSpeed(0);
            current_state = STATE_IDLE; /* Posición alcanzada con precisión */
          }
          else
          {
            /* 2. Detección y auto-corrección de sentido invertido (anti-embalamiento):
             * Si el error crece más de 30 ticks en vez de achicarse, el motor está
             * girando en sentido opuesto al origen. Se invierte el signo de inmediato. */
            if (abs_err > (go_home_initial_error + 30))
            {
              motor_dir_sign = -motor_dir_sign;
              go_home_initial_error = abs_err;
            }

            /* 3. Timeout de seguridad: si tras 5 segundos no llega, frenar */
            if (HAL_GetTick() - go_home_start_time > 5000)
            {
              Motor_SetSpeed(0);
              current_state = STATE_IDLE;
            }

            /* 4. Velocidad proporcional: 250 suave cerca, 350 si está más lejos */
            int16_t speed = POS_MIN_DUTY; /* 250 */
            if (abs_err > 40)
            {
              speed = POS_MAX_DUTY; /* 350 para trayectos largos */
            }

            /* 5. Comandar motor con corrección de signo */
            if (error > 0)
            {
              Motor_SetSpeed(motor_dir_sign * speed);
            }
            else
            {
              Motor_SetSpeed(-motor_dir_sign * speed);
            }
          }
        }
        break;

      case STATE_ERROR_LIMIT:
        Motor_SetSpeed(0);
        Laser_Set(0);
        /* Parpadeo continuo de advertencia */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);

        /* Salir del estado de error al presionar STOP (BTN4) o BTN1 */
        if (btn4 || btn1)
        {
          total_encoder_ticks = 0;
          home_position = 0;
          current_state = STATE_IDLE;
        }
        break;

      default:
        current_state = STATE_IDLE;
        break;
    }

    /* Pequeño retardo de 5 ms para temporización estable */
    HAL_Delay(5);

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

/* USER CODE BEGIN 4 */

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
