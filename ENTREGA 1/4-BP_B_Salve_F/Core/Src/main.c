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
  STATE_IDLE        = 0,  /* Detenido fuera de la zona Home (|pos| > +-8 grados) */
  STATE_SCANNING    = 1,  /* Barrido oscilante de +-2 vueltas */
  STATE_HOMED       = 2,  /* Detenido dentro de la zona Home (+-8 grados / +-56 ticks) */
  STATE_HOMING      = 3,  /* Retornando activamente al origen Home (0 grados) */
  STATE_ERROR_LIMIT = 4,  /* Falla por limite mecanico o timeout de movimiento */
  STATE_POSITIONING = 5   /* Posicionamiento relativo por angulo (+-30/90/180/360 grados) */
} SystemState_t;

typedef enum {
  SCAN_PHASE_FORWARD = 0,   /* Girando hacia adelante (+2 vueltas) */
  SCAN_PHASE_PAUSE_FWD,     /* Pausa de 1s con motor frenado */
  SCAN_PHASE_REVERSE,       /* Girando hacia atras (-2 vueltas) */
  SCAN_PHASE_PAUSE_REV      /* Pausa de 1s con motor frenado */
} ScanPhase_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- PARAMETROS DEL ENCODER, TORNILLO SIN FIN Y ENGRANAJE --- */
#define ENCODER_PPR            30     /* Pulsos por revolucion del eje del motor */
/* En modo TI1 (x2) se generan 2 cuentas en el timer por cada pulso */
#define TICKS_PER_MOTOR_REV    (ENCODER_PPR * 2) /* 60 cuentas por 1 vuelta del motor */

/* Transmision: Tornillo sin fin (1 entrada) -> Engranaje de 42 dientes (Radio = 15 mm)
 * 1 vuelta del tornillo sin fin avanza 1 diente del engranaje (relacion 42:1).
 * Por lo tanto, 1 vuelta del engranaje = 42 vueltas del motor = 2520 ticks. */
#define WORM_STARTS            1      /* Entradas del tornillo sin fin */
#define GEAR_TEETH             42     /* Dientes del engranaje conducido */
#define GEAR_RADIUS_MM         15     /* Radio del engranaje (centro a cabeza de diente): 15 mm */
#define TICKS_PER_GEAR_REV     ((TICKS_PER_MOTOR_REV * GEAR_TEETH) / WORM_STARTS) /* 2520 cuentas por vuelta de engranaje */

#define SWEEP_GEAR_REVS        2      /* 2 vueltas completas del ENGRANAJE hacia cada sentido */
#define SWEEP_TARGET_TICKS     (TICKS_PER_GEAR_REV * SWEEP_GEAR_REVS) /* 5040 ticks (84 vueltas de motor) */
#define SWEEP_PAUSE_MS         1000   /* 1 segundo de retardo antes de cambiar de sentido */
#define SWEEP_DUTY             700    /* Fuerza de barrido en 700 (70% sobre 999) */

/* --- IMPULSO DE ARRANQUE (KICK-START PARA VENCER FRICCION ESTATICA) --- */
#define KICK_DUTY              950    /* Golpe inicial de fuerza (95% sobre 999) al salir del reposo */
#define KICK_TIME_MS           35     /* Duracion del empujon inicial en milisegundos */

/* Limite de ticks para permitir las 84 vueltas de motor (5040 ticks) con holgura */
#define MAX_ENCODER_LIMIT      25000

/* --- PARAMETROS DEL CONTROLADOR DE POSICION (IR A HOME) --- */
#define POS_DEADBAND_TICKS     5      /* Tolerancia de llegada en pulsos */
#define HOMED_TOLERANCE_TICKS  ((8 * TICKS_PER_GEAR_REV) / 360) /* Zona Home: +-8 grados = +-56 ticks (contempla inercia) */
#define POS_MIN_DUTY           550    /* Fuerza minima para vencer friccion en retorno */
#define POS_MAX_DUTY           700    /* Velocidad maxima de retorno */
#define GO_HOME_TIMEOUT_MS     30000  /* 30 s de tiempo maximo para volver a Home */
#define MOTOR_STALL_TIMEOUT_MS 1200   /* 1.2 s sin pulsos de encoder con motor energizado -> Falla motor / L298N apagado */
#define MOTOR_STALL_MIN_TICKS  4      /* Avance minimo en ticks para confirmar que el motor responde */

#define ENCODER_POLARITY       1      /* Polaridad de encoder */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Estado del sistema y variables de posicion */
volatile SystemState_t current_state = STATE_HOMED;
volatile int32_t total_encoder_ticks = 0;   /* Posicion acumulada en 32 bits */
volatile int16_t last_timer_cnt = 0;        /* Ultima lectura directa de CNT (16 bits) */
volatile int32_t home_position = 0;         /* Posicion de referencia fijada */
volatile int32_t target_position = 0;       /* Posicion objetivo para Ir a Home o giros por angulo */

/* Variables para el barrido oscilante por PPR */
volatile ScanPhase_t scan_phase = SCAN_PHASE_FORWARD;
int32_t leg_start_ticks = 0;                /* Posicion de inicio de cada tramo de giro */
uint32_t scan_pause_timer = 0;              /* Temporizador para el delay de 1s */

/* Variables para inversion de sentido y timeout en Go to Home */
volatile int8_t motor_dir_sign = 1;         /* Signo de direccion de motor (+1 o -1) */
int32_t go_home_initial_error = 0;
uint32_t go_home_start_time = 0;

/* Seguimiento del sentido actual del motor para el impulso de arranque (kick-start) */
int8_t current_motor_dir = 0;               /* 0: frenado, 1: adelante, -1: atras */
uint8_t kick_active = 0;                    /* 1 mientras dura el pulso de arranque de 35 ms */
uint32_t kick_start_tick = 0;               /* Marca de tiempo de inicio del kick-start */
uint16_t target_pwm_duty = 0;               /* Ciclo util nominal al que baja tras el kick-start */

/* Banderas de eventos de botones recibidos por bus CAN desde BP_A_Master_F */
volatile uint8_t can_btn1_flag = 0;
volatile uint8_t can_btn2_flag = 0;
volatile uint8_t can_btn3_flag = 0;
volatile uint8_t can_btn4_flag = 0;
volatile int32_t can_step_ticks = 0;        /* Pasos relativos solicitados por CAN (+ o - ticks) */

/* Temporizadores para FSM (5 ms), telemetria CAN (50 ms), blink Home y falla */
uint32_t last_fsm_tick = 0;
uint32_t last_telemetry_tick = 0;
uint32_t error_blink_tick = 0;
uint8_t home_blink_active = 0;
uint32_t home_blink_tick = 0;
int32_t stall_ref_ticks = 0;                /* Posicion de referencia para deteccion de motor desconectado / L298N sin energia */
uint32_t stall_timer_tick = 0;              /* Marca de tiempo de ultimo avance de encoder con motor activo */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_SetSpeed(int16_t speed);
void Motor_ServiceKickStart(void);
void Encoder_Update(void);
void CAN_Filter_Config(void);
void CAN_ProcessRxMessages(void);
void CAN_SendTelemetry(void);
void Slave_EnterState(SystemState_t next_state);
void Slave_TransitionToRest(void);
void Slave_CalibrateHome(void);
void Slave_StartMoveToTarget(int32_t new_target, SystemState_t move_state);
void Slave_RunPositionControl(int32_t step_req, uint8_t btn2, uint8_t btn3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Ajusta el sentido de giro y la velocidad del motor DC con el puente H L298N.
  *         Incluye un "empujon" (kick-start) al arrancar desde reposo o cambiar
  *         de sentido para vencer la friccion estatica del motor.
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
    target_pwm_duty = (uint16_t)speed;

    /* Si estaba detenido o girando al reves, iniciar golpe de arranque */
    if (current_motor_dir != 1)
    {
      current_motor_dir = 1;
      if (target_pwm_duty < KICK_DUTY && KICK_TIME_MS > 0)
      {
        kick_active = 1;
        kick_start_tick = HAL_GetTick();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        return;
      }
    }
    if (kick_active)
    {
      if ((HAL_GetTick() - kick_start_tick) < KICK_TIME_MS)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        return;
      }
      kick_active = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target_pwm_duty);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_SET);
    target_pwm_duty = (uint16_t)(-speed);

    /* Si estaba detenido o girando al reves, iniciar golpe de arranque */
    if (current_motor_dir != -1)
    {
      current_motor_dir = -1;
      if (target_pwm_duty < KICK_DUTY && KICK_TIME_MS > 0)
      {
        kick_active = 1;
        kick_start_tick = HAL_GetTick();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        return;
      }
    }
    if (kick_active)
    {
      if ((HAL_GetTick() - kick_start_tick) < KICK_TIME_MS)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, KICK_DUTY);
        return;
      }
      kick_active = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target_pwm_duty);
  }
  else
  {
    /* Freno activo */
    current_motor_dir = 0;
    kick_active = 0;
    target_pwm_duty = 0;
    HAL_GPIO_WritePin(GPIOA, Escribe_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, Escribe_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  }
}

/**
  * @brief  Supervisa el fin del impulso de arranque (KICK_TIME_MS)
  */
void Motor_ServiceKickStart(void)
{
  if (kick_active && (HAL_GetTick() - kick_start_tick) >= KICK_TIME_MS)
  {
    kick_active = 0;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, target_pwm_duty);
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
  * @brief  Configuracion del filtro CAN para recibir todos los mensajes en FIFO0
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
  * @brief  Procesa todos los mensajes pendientes en el FIFO0 de recepcion CAN
  *         - StdId 0x010: Comandos de botones BTN1..BTN4 (0x01..0x04)
  *         - StdId 0x020: Comando de encendido/apagado de LED_REFLEJO en PB0 (0x01 / 0x00)
  */
void CAN_ProcessRxMessages(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      if (rxHeader.DLC >= 1)
      {
        /* Comandos de los botones de motor/homing y giros por angulo (ID = 0x010) */
        if (rxHeader.StdId == 0x010)
        {
          switch (rxData[0])
          {
            case 0x01: can_btn1_flag = 1; break;
            case 0x02: can_btn2_flag = 1; break;
            case 0x03: can_btn3_flag = 1; break;
            case 0x04: can_btn4_flag = 1; break;
            /* Giros relativos en sentido positivo (+30, +90, +180, +360 grados) */
            case 0x05: can_step_ticks =  (30  * TICKS_PER_GEAR_REV) / 360; break; /* +210 ticks */
            case 0x06: can_step_ticks =  (90  * TICKS_PER_GEAR_REV) / 360; break; /* +630 ticks */
            case 0x07: can_step_ticks =  (180 * TICKS_PER_GEAR_REV) / 360; break; /* +1260 ticks */
            case 0x08: can_step_ticks =  (360 * TICKS_PER_GEAR_REV) / 360; break; /* +2520 ticks */
            /* Giros relativos en sentido negativo (-30, -90, -180, -360 grados) */
            case 0x09: can_step_ticks = -(30  * TICKS_PER_GEAR_REV) / 360; break; /* -210 ticks */
            case 0x0A: can_step_ticks = -(90  * TICKS_PER_GEAR_REV) / 360; break; /* -630 ticks */
            case 0x0B: can_step_ticks = -(180 * TICKS_PER_GEAR_REV) / 360; break; /* -1260 ticks */
            case 0x0C: can_step_ticks = -(360 * TICKS_PER_GEAR_REV) / 360; break; /* -2520 ticks */
            default: break;
          }
        }
        /* Comandos del LED en PB0 (ID = 0x020) */
        else if (rxHeader.StdId == 0x020)
        {
          if (rxData[0] == 0x01)
          {
            HAL_GPIO_WritePin(LED_REFLEJO_GPIO_Port, LED_REFLEJO_Pin, GPIO_PIN_SET);   /* Encender LED PB0 */
          }
          else if (rxData[0] == 0x00)
          {
            HAL_GPIO_WritePin(LED_REFLEJO_GPIO_Port, LED_REFLEJO_Pin, GPIO_PIN_RESET); /* Apagar LED PB0 */
          }
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
  * @brief  Callback de interrupcion al recibir un frame CAN en el FIFO0
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanInstance)
{
  UNUSED(hcanInstance);
  CAN_ProcessRxMessages();
}

/**
  * @brief  Transmite periodicamente la telemetria (posicion y estado) por CAN a BP_A (StdId = 0x030)
  */
void CAN_SendTelemetry(void)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[5];
  uint32_t txMailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
  {
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  }

  txHeader.StdId = 0x030;
  txHeader.ExtId = 0x01;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 5;
  txHeader.TransmitGlobalTime = DISABLE;

  int32_t pos = total_encoder_ticks;
  txData[0] = (uint8_t)(pos & 0xFF);
  txData[1] = (uint8_t)((pos >> 8) & 0xFF);
  txData[2] = (uint8_t)((pos >> 16) & 0xFF);
  txData[3] = (uint8_t)((pos >> 24) & 0xFF);
  txData[4] = (uint8_t)current_state;

  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

/**
  * @brief  Cambia de estado en la FSM de Placa B y actualiza salidas
  */
void Slave_EnterState(SystemState_t next_state)
{
  current_state = next_state;

  switch (next_state)
  {
    case STATE_IDLE:
    case STATE_HOMED:
      Motor_SetSpeed(0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED PC13 apagado en reposo */
      break;

    case STATE_SCANNING:
      leg_start_ticks = total_encoder_ticks;
      stall_ref_ticks = total_encoder_ticks;
      stall_timer_tick = HAL_GetTick();
      scan_phase = SCAN_PHASE_FORWARD;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED PC13 encendido en movimiento */
      Motor_SetSpeed(SWEEP_DUTY);
      break;

    case STATE_HOMING:
    case STATE_POSITIONING:
      stall_ref_ticks = total_encoder_ticks;
      stall_timer_tick = HAL_GetTick();
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED PC13 encendido en movimiento */
      break;

    case STATE_ERROR_LIMIT:
      Motor_SetSpeed(0);
      error_blink_tick = HAL_GetTick();
      break;

    default:
      Motor_SetSpeed(0);
      current_state = STATE_IDLE;
      break;
  }
}

/**
  * @brief  Frena el motor y pasa a STATE_HOMED (si esta dentro de +-8 grados) o STATE_IDLE
  */
void Slave_TransitionToRest(void)
{
  if (abs(total_encoder_ticks - home_position) <= HOMED_TOLERANCE_TICKS)
  {
    Slave_EnterState(STATE_HOMED);
  }
  else
  {
    Slave_EnterState(STATE_IDLE);
  }
}

/**
  * @brief  Fija el origen Home (0 grados / 0 ticks) en la posicion actual y pasa a STATE_HOMED
  */
void Slave_CalibrateHome(void)
{
  Motor_SetSpeed(0);
  total_encoder_ticks = 0;
  home_position = 0;
  target_position = 0;
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  last_timer_cnt = 0;

  Slave_EnterState(STATE_HOMED);

  /* Iniciar blink de 120 ms en PC13 para confirmar calibracion Home */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  home_blink_active = 1;
  home_blink_tick = HAL_GetTick();
}

/**
  * @brief  Inicia un movimiento hacia new_target en STATE_HOMING o STATE_POSITIONING
  */
void Slave_StartMoveToTarget(int32_t new_target, SystemState_t move_state)
{
  target_position = new_target;
  go_home_initial_error = abs(target_position - total_encoder_ticks);
  go_home_start_time = HAL_GetTick();
  Slave_EnterState(move_state);
}

/**
  * @brief  Controlador en lazo cerrado compartido por STATE_HOMING (3) y STATE_POSITIONING (5)
  */
void Slave_RunPositionControl(int32_t step_req, uint8_t btn2, uint8_t btn3)
{
  /* Si se presiona BTN2 (Set Home), calibrar el cero inmediatamente aqui */
  if (btn2)
  {
    Slave_CalibrateHome();
    return;
  }

  /* Si se presiona BTN3 (Ir a Home) durante un posicionamiento, redirigir al origen Home */
  if (btn3 && current_state != STATE_HOMING)
  {
    Slave_StartMoveToTarget(home_position, STATE_HOMING);
  }

  /* Si llega un nuevo paso angular relativo, acumular objetivo y pasar a STATE_POSITIONING */
  if (step_req != 0)
  {
    if (current_state == STATE_POSITIONING)
    {
      target_position += step_req;
    }
    else
    {
      target_position = total_encoder_ticks + step_req;
      current_state = STATE_POSITIONING;
    }
    go_home_initial_error = abs(target_position - total_encoder_ticks);
    go_home_start_time = HAL_GetTick();
    stall_ref_ticks = total_encoder_ticks;
    stall_timer_tick = HAL_GetTick();
  }

  /* Error de posicion respecto al objetivo */
  int32_t error = target_position - total_encoder_ticks;
  int32_t abs_err = abs(error);

  /* Llegada dentro de la banda muerta */
  if (abs_err <= POS_DEADBAND_TICKS)
  {
    Slave_TransitionToRest();
    return;
  }

  /* Si el error aumenta respecto al inicial, invertir sentido de giro */
  if (abs_err > (go_home_initial_error + 30))
  {
    motor_dir_sign = -motor_dir_sign;
    go_home_initial_error = abs_err;
  }

  /* Timeout de seguridad -> pasar a STATE_ERROR_LIMIT */
  if ((HAL_GetTick() - go_home_start_time) > GO_HOME_TIMEOUT_MS)
  {
    Slave_EnterState(STATE_ERROR_LIMIT);
    return;
  }

  /* Velocidad proporcional escalonada */
  int16_t speed = (abs_err > 40) ? POS_MAX_DUTY : POS_MIN_DUTY;

  /* Comandar motor con correccion de signo */
  if (error > 0)
  {
    Motor_SetSpeed(motor_dir_sign * speed);
  }
  else
  {
    Motor_SetSpeed(-motor_dir_sign * speed);
  }
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

  /* Iniciar periferico CAN y activar interrupciones de recepcion */
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
  HAL_GPIO_WritePin(LED_REFLEJO_GPIO_Port, LED_REFLEJO_Pin, GPIO_PIN_RESET);

  /* Resetear contadores de posicion y variables de referencia */
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  last_timer_cnt = 0;
  total_encoder_ticks = 0;
  home_position = 0;
  target_position = 0;
  last_fsm_tick = HAL_GetTick();
  last_telemetry_tick = HAL_GetTick();
  Slave_EnterState(STATE_HOMED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Tareas continuas: Recepcion CAN, Conteo de Encoder y Fin de Kick-Start */
    CAN_ProcessRxMessages();
    Encoder_Update();
    Motor_ServiceKickStart();

    /* Ejecucion periodica de la Maquina de Estados cada 5 ms (200 Hz) */
    if ((HAL_GetTick() - last_fsm_tick) >= 5)
    {
      last_fsm_tick = HAL_GetTick();

      /* Capturar eventos de BTN1..BTN4 y pasos angulares recibidos por CAN desde BP_A_Master_F */
    uint8_t btn1 = can_btn1_flag; can_btn1_flag = 0;
    uint8_t btn2 = can_btn2_flag; can_btn2_flag = 0;
    uint8_t btn3 = can_btn3_flag; can_btn3_flag = 0;
    uint8_t btn4 = can_btn4_flag; can_btn4_flag = 0;
    int32_t step_req = can_step_ticks; can_step_ticks = 0;

    /* PARADA DE EMERGENCIA / STOP (BTN4 tiene prioridad sobre cualquier estado) */
    if (btn4)
    {
      Slave_TransitionToRest();
    }

    /* VERIFICACION DE LIMITE MECANICO Y DETECCION DE MOTOR DESCONECTADO / L298N SIN ENERGIA */
    if (current_state == STATE_SCANNING ||
        current_state == STATE_HOMING   ||
        current_state == STATE_POSITIONING)
    {
      if (abs(total_encoder_ticks - home_position) >= MAX_ENCODER_LIMIT)
      {
        Slave_EnterState(STATE_ERROR_LIMIT);
      }
      else if (current_motor_dir != 0)
      {
        /* Si el puente H esta comandando giro pero el encoder no avanza en 1.2 s -> Falla de motor */
        if (abs(total_encoder_ticks - stall_ref_ticks) >= MOTOR_STALL_MIN_TICKS)
        {
          stall_ref_ticks = total_encoder_ticks;
          stall_timer_tick = HAL_GetTick();
        }
        else if ((HAL_GetTick() - stall_timer_tick) >= MOTOR_STALL_TIMEOUT_MS)
        {
          Slave_EnterState(STATE_ERROR_LIMIT);
        }
      }
      else
      {
        /* En pausa entre tramos de barrido, mantener fresco el temporizador de atasco */
        stall_ref_ticks = total_encoder_ticks;
        stall_timer_tick = HAL_GetTick();
      }
    }

    /* MAQUINA DE ESTADOS DE MOVIMIENTO */
    switch (current_state)
    {
      case STATE_IDLE:
      case STATE_HOMED:
        Motor_SetSpeed(0);
        if (home_blink_active)
        {
          if ((HAL_GetTick() - home_blink_tick) >= 120)
          {
            home_blink_active = 0;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          }
        }
        else
        {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED PC13 apagado en reposo */
        }

        /* Sincronizar estado de reposo segun si el eje esta dentro de +-8 grados (+-56 ticks) de Home */
        if (abs(total_encoder_ticks - home_position) <= HOMED_TOLERANCE_TICKS)
        {
          current_state = STATE_HOMED;
        }
        else
        {
          current_state = STATE_IDLE;
        }

        /* Transiciones de salida desde reposo (STATE_IDLE o STATE_HOMED) */
        if (btn1)
        {
          Slave_EnterState(STATE_SCANNING);
        }
        else if (btn2)
        {
          Slave_CalibrateHome();
        }
        else if (btn3)
        {
          Slave_StartMoveToTarget(home_position, STATE_HOMING);
        }
        else if (step_req != 0)
        {
          Slave_StartMoveToTarget(total_encoder_ticks + step_req, STATE_POSITIONING);
        }
        break;

      case STATE_SCANNING:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED PC13 encendido mientras escanea */

        /* Sub-maquina de estados de barrido por vueltas (PPR) */
        switch (scan_phase)
        {
          case SCAN_PHASE_FORWARD:
            Motor_SetSpeed(SWEEP_DUTY);
            if (abs(total_encoder_ticks - leg_start_ticks) >= SWEEP_TARGET_TICKS)
            {
              Motor_SetSpeed(0);
              scan_pause_timer = HAL_GetTick();
              scan_phase = SCAN_PHASE_PAUSE_FWD;
            }
            break;

          case SCAN_PHASE_PAUSE_FWD:
            Motor_SetSpeed(0);
            if (HAL_GetTick() - scan_pause_timer >= SWEEP_PAUSE_MS)
            {
              leg_start_ticks = total_encoder_ticks;
              Motor_SetSpeed(-SWEEP_DUTY);
              scan_phase = SCAN_PHASE_REVERSE;
            }
            break;

          case SCAN_PHASE_REVERSE:
            Motor_SetSpeed(-SWEEP_DUTY);
            if (abs(total_encoder_ticks - leg_start_ticks) >= SWEEP_TARGET_TICKS)
            {
              Motor_SetSpeed(0);
              scan_pause_timer = HAL_GetTick();
              scan_phase = SCAN_PHASE_PAUSE_REV;
            }
            break;

          case SCAN_PHASE_PAUSE_REV:
            Motor_SetSpeed(0);
            if (HAL_GetTick() - scan_pause_timer >= SWEEP_PAUSE_MS)
            {
              leg_start_ticks = total_encoder_ticks;
              Motor_SetSpeed(SWEEP_DUTY);
              scan_phase = SCAN_PHASE_FORWARD;
            }
            break;
        }

        /* BTN2: Fijar Home en el punto actual y quedar en STATE_HOMED */
        if (btn2)
        {
          Slave_CalibrateHome();
        }
        /* BTN3: Interrumpir barrido y regresar al origen Home */
        else if (btn3)
        {
          Slave_StartMoveToTarget(home_position, STATE_HOMING);
        }
        /* Paso angular relativo: pasar a posicionamiento relativo */
        else if (step_req != 0)
        {
          Slave_StartMoveToTarget(total_encoder_ticks + step_req, STATE_POSITIONING);
        }
        break;

      case STATE_HOMING:
      case STATE_POSITIONING:
        Slave_RunPositionControl(step_req, btn2, btn3);
        break;

      case STATE_ERROR_LIMIT:
        Motor_SetSpeed(0);
        /* Blink de advertencia cada 100 ms */
        if ((HAL_GetTick() - error_blink_tick) >= 100)
        {
          error_blink_tick = HAL_GetTick();
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        /* Salir del estado de falla al presionar BTN2 (Recalibrar Home) o BTN1 */
        if (btn2)
        {
          Slave_CalibrateHome();
        }
        else if (btn1)
        {
          Slave_TransitionToRest();
        }
        break;

      default:
        Slave_TransitionToRest();
        break;
      }
    }

    /* Transmitir telemetria de posicion y estado hacia BP_A (cada 50 ms / 20 Hz) */
    if ((HAL_GetTick() - last_telemetry_tick) >= 50)
    {
      last_telemetry_tick = HAL_GetTick();
      CAN_SendTelemetry();
    }
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
