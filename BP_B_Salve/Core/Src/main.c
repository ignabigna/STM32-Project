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
#include "can.h"
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
/* --- PARÁMETROS DEL ENCODER, TORNILLO SIN FIN Y ENGRANAJE --- */
#define ENCODER_PPR            30     /* Pulsos por revolución del eje del motor */
/* En modo TI1 (x2) se generan 2 cuentas en el timer por cada pulso */
#define TICKS_PER_MOTOR_REV    (ENCODER_PPR * 2) /* 60 cuentas por 1 vuelta del motor */

/* Transmisión: Tornillo sin fin (1 entrada) -> Engranaje de 40 dientes (Radio = 15 mm)
 * 1 vuelta del tornillo sin fin avanza 1 diente del engranaje (relación 40:1).
 * Por lo tanto, 1 vuelta del engranaje = 40 vueltas del motor = 2400 ticks. */
#define WORM_STARTS            1      /* Entradas del tornillo sin fin */
#define GEAR_TEETH             40     /* Dientes del engranaje conducido */
#define GEAR_RADIUS_MM         15     /* Radio del engranaje (centro a cabeza de diente): 15 mm */
#define TICKS_PER_GEAR_REV     ((TICKS_PER_MOTOR_REV * GEAR_TEETH) / WORM_STARTS) /* 2400 cuentas por vuelta de engranaje */

#define SWEEP_GEAR_REVS        2      /* 2 vueltas completas del ENGRANAJE hacia cada sentido */
#define SWEEP_TARGET_TICKS     (TICKS_PER_GEAR_REV * SWEEP_GEAR_REVS) /* 4800 ticks (80 vueltas de motor) */
#define SWEEP_PAUSE_MS         1000   /* 1 segundo de retardo antes de cambiar de sentido */
#define SWEEP_DUTY             700    /* Fuerza de barrido en 700 (70% sobre 999) */

/* --- IMPULSO DE ARRANQUE (KICK-START PARA VENCER FRICCIÓN ESTÁTICA) --- */
#define KICK_DUTY              950    /* Golpe inicial de fuerza (95% sobre 999) al salir del reposo */
#define KICK_TIME_MS           35     /* Duración del empujón inicial en milisegundos */

/* Límite de seguridad amplio para permitir las 80 vueltas de motor (4800 ticks) con holgura */
#define MAX_ENCODER_LIMIT      25000

/* --- PARÁMETROS DEL CONTROLADOR DE POSICIÓN (IR A HOME) --- */
#define POS_DEADBAND_TICKS     5      /* Tolerancia de llegada en pulsos */
#define POS_MIN_DUTY           550    /* Fuerza mínima para vencer fricción en retorno */
#define POS_MAX_DUTY           700    /* Velocidad máxima de retorno */
#define GO_HOME_TIMEOUT_MS     30000  /* 30 s de tiempo máximo para desandar hasta 2 vueltas de engranaje */

#define ENCODER_POLARITY       1      /* Polaridad de encoder */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Estado del sistema y variables de posición */
volatile SystemState_t current_state = STATE_IDLE;
volatile int32_t total_encoder_ticks = 0;   /* Posición acumulada en 32 bits (anti-desbordamiento) */
volatile int16_t last_timer_cnt = 0;        /* Última lectura directa de CNT (16 bits) */
volatile int32_t home_position = 0;         /* Posición de referencia fijada */

/* Variables para el barrido oscilante por PPR */
volatile ScanPhase_t scan_phase = SCAN_PHASE_FORWARD;
int32_t leg_start_ticks = 0;                /* Posición de inicio de cada tramo de giro */
uint32_t scan_pause_timer = 0;              /* Temporizador no bloqueante para el delay de 1s */

/* Variables para control inteligente y anti-embalamiento de Go to Home */
volatile int8_t motor_dir_sign = 1;         /* Signo de dirección de motor (+1 o -1 auto-calibrado) */
int32_t go_home_initial_error = 0;
uint32_t go_home_start_time = 0;

/* Seguimiento del sentido actual del motor para el impulso de arranque (kick-start) */
int8_t current_motor_dir = 0;               /* 0: frenado, 1: adelante, -1: atrás */

/* Estados de anti-rebote no bloqueante para los 4 pulsadores locales */
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
void Encoder_Update(void);
uint8_t Button_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_state, uint32_t *last_time);
void CAN_Filter_Config(void);
void CAN_ProcessRxMessages(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Ajusta el sentido de giro y la velocidad del motor DC con el puente H L298N.
  *         Incluye un "empujón" (kick-start) automático al arrancar desde reposo o cambiar
  *         de sentido para vencer la fricción estática del motor.
  * @param  speed: Valor entre -999 y +999
  */
void Motor_SetSpeed(int16_t speed)
{
  if (speed > 999) speed = 999;
  if (speed < -999) speed = -999;

  if (speed > 0)
  {
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_RESET);

    /* Si estaba detenido o girando al revés, dar golpe de arranque para vencer inercia */
    if (current_motor_dir != 1)
    {
      current_motor_dir = 1;
      if (speed < KICK_DUTY && KICK_TIME_MS > 0)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        HAL_Delay(KICK_TIME_MS);
      }
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)speed);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_SET);

    /* Si estaba detenido o girando al revés, dar golpe de arranque para vencer inercia */
    if (current_motor_dir != -1)
    {
      current_motor_dir = -1;
      if ((-speed) < KICK_DUTY && KICK_TIME_MS > 0)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        HAL_Delay(KICK_TIME_MS);
      }
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(-speed));
  }
  else
  {
    /* Freno activo */
    current_motor_dir = 0;
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  }
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
  */
uint8_t Button_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_state, uint32_t *last_time)
{
  uint8_t current = HAL_GPIO_ReadPin(port, pin);
  uint8_t triggered = 0;

  if (current != *last_state)
  {
    if ((HAL_GetTick() - *last_time) > 40)
    {
      *last_time = HAL_GetTick();
      if (current == GPIO_PIN_RESET && *last_state == GPIO_PIN_SET)
      {
        triggered = 1;
      }
      *last_state = current;
    }
  }
  return triggered;
}

/**
  * @brief  Configuración del filtro CAN para recibir todos los mensajes en FIFO0
  */
void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Procesa todos los mensajes pendientes en el FIFO0 de recepción CAN
  */
void CAN_ProcessRxMessages(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      /* Identificador 0x020 enviado por la Blue Pill A */
      if (rxHeader.StdId == 0x020 && rxHeader.DLC >= 1)
      {
        if (rxData[0] == 0x01)
        {
          HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);   /* Encender LED PB0 */
        }
        else if (rxData[0] == 0x00)
        {
          HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET); /* Apagar LED PB0 */
        }
      }
    }
    else
    {
      break;
    }
  }
}

/**
  * @brief  Callback de interrupción al recibir un frame CAN en el FIFO0
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanInstance)
{
  UNUSED(hcanInstance);
  CAN_ProcessRxMessages();
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* Iniciar PWM de velocidad de motor (TIM3 Canal 1 en PA6 -> ENA del L298N) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* Iniciar lectura de Encoder en TIM2 (PA0 y PA1) */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /* Iniciar periférico CAN y activar interrupciones de recepción */
  CAN_Filter_Config();
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* Asegurar actuadores apagados al arrancar */
  Motor_SetSpeed(0);
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);

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
    /* 0. Verificar si llegaron mensajes por CAN en el FIFO0 (respaldo por polling + interrupción) */
    CAN_ProcessRxMessages();

    /* 1. Actualizar el conteo continuo del encoder (32 bits anti-desbordamiento) */
    Encoder_Update();

    /* 2. Lectura no bloqueante con anti-rebote de los 4 pulsadores locales */
    uint8_t btn1 = Button_Pressed(BTN1_SCAN_GPIO_Port, BTN1_SCAN_Pin, &btn1_last, &btn1_debounce_time);
    uint8_t btn2 = Button_Pressed(BTN2_SET_H_GPIO_Port, BTN2_SET_H_Pin, &btn2_last, &btn2_debounce_time);
    uint8_t btn3 = Button_Pressed(BTN3_GOH_GPIO_Port, BTN3_GOH_Pin, &btn3_last, &btn3_debounce_time);
    uint8_t btn4 = Button_Pressed(BTN4_STOP_GPIO_Port, BTN4_STOP_Pin, &btn4_last, &btn4_debounce_time);

    /* 3. PARADA DE EMERGENCIA / STOP (BTN4 tiene prioridad sobre cualquier estado) */
    if (btn4)
    {
      Motor_SetSpeed(0);
      current_state = STATE_IDLE;
    }

    /* 4. VERIFICACIÓN DE LÍMITE DE SEGURIDAD */
    if (current_state == STATE_SCANNING || current_state == STATE_GO_TO_HOME)
    {
      if (abs(total_encoder_ticks - home_position) >= MAX_ENCODER_LIMIT)
      {
        Motor_SetSpeed(0);
        current_state = STATE_ERROR_LIMIT;
      }
    }

    /* 5. MÁQUINA DE ESTADOS */
    switch (current_state)
    {
      case STATE_IDLE:
        Motor_SetSpeed(0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED PC13 apagado en reposo */

        /* BTN1: Iniciar barrido oscilante basado en vueltas (PPR) */
        if (btn1)
        {
          leg_start_ticks = total_encoder_ticks; /* Origen de este tramo */
          scan_phase = SCAN_PHASE_FORWARD;
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
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED PC13 encendido mientras escanea */

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

        /* BTN2: Fijar Home */
        if (btn2)
        {
          Motor_SetSpeed(0);

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
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        current_state = STATE_IDLE; /* Regresa a reposo con el cero memorizado */
        break;

      case STATE_GO_TO_HOME:
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
            /* 2. Detección y auto-corrección de sentido invertido (anti-embalamiento) */
            if (abs_err > (go_home_initial_error + 30))
            {
              motor_dir_sign = -motor_dir_sign;
              go_home_initial_error = abs_err;
            }

            /* 3. Timeout de seguridad: tiempo suficiente para desandar las 2 vueltas del engranaje */
            if (HAL_GetTick() - go_home_start_time > GO_HOME_TIMEOUT_MS)
            {
              Motor_SetSpeed(0);
              current_state = STATE_IDLE;
            }

            /* 4. Velocidad proporcional: POS_MIN_DUTY cerca, POS_MAX_DUTY si está más lejos */
            int16_t speed = POS_MIN_DUTY;
            if (abs_err > 40)
            {
              speed = POS_MAX_DUTY;
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
