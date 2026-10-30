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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Relacion de transmision en Placa B:
 * Encoder 30 PPR en modo TI1 (x2) = 60 ticks/vuelta de motor.
 * Tornillo sin fin (1 entrada) + corona de 40 dientes = 2400 ticks por vuelta de engranaje (360 grados). */
#define TICKS_PER_GEAR_REV     2400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Estados de anti-rebote para los 4 botones de control de motor (PB12..PB15) */
uint8_t btn1_last_level = GPIO_PIN_SET;
uint8_t btn1_was_pressed = 0;
uint32_t btn1_debounce_time = 0;

uint8_t btn2_last_level = GPIO_PIN_SET;
uint8_t btn2_was_pressed = 0;
uint32_t btn2_debounce_time = 0;

uint8_t btn3_last_level = GPIO_PIN_SET;
uint8_t btn3_was_pressed = 0;
uint32_t btn3_debounce_time = 0;

uint8_t btn4_last_level = GPIO_PIN_SET;
uint8_t btn4_was_pressed = 0;
uint32_t btn4_debounce_time = 0;

/* Estados de anti-rebote para BTN5 y BTN6 (tanto en PA10/PA11 como en PB10/PB11) */
uint8_t btn5_pa_last_level = GPIO_PIN_SET;
uint8_t btn5_pa_was_pressed = 0;
uint32_t btn5_pa_debounce_time = 0;

uint8_t btn6_pa_last_level = GPIO_PIN_SET;
uint8_t btn6_pa_was_pressed = 0;
uint32_t btn6_pa_debounce_time = 0;

uint8_t btn5_pb_last_level = GPIO_PIN_SET;
uint8_t btn5_pb_was_pressed = 0;
uint32_t btn5_pb_debounce_time = 0;

uint8_t btn6_pb_last_level = GPIO_PIN_SET;
uint8_t btn6_pb_was_pressed = 0;
uint32_t btn6_pb_debounce_time = 0;

/* Estado actual reflejado del LED en PB0 de la placa B */
uint8_t current_led_state = 0;

/* --- MODO DE OPERACION Y CONTROL REMOTO (IDE / LIVE EXPRESSIONS) --- */
typedef enum {
  MODE_LOCAL = 0,    /* Modo Local: Pulsadores fisicos BTN1..4 activos, LED apagado */
  MODE_REMOTE = 1    /* Modo Remoto: Control desde PC (IDE), pulsadores locales bloqueados, LED encendido */
} OperatingMode_t;

volatile OperatingMode_t system_mode = MODE_LOCAL;      /* 0 = Local, 1 = Remoto */
volatile uint8_t remote_cmd = 0;                        /* 1..4: Botones, 5..8: Giro +30/+90/+180/+360, 9..12: Giro -30/-90/-180/-360 */
volatile OperatingMode_t last_system_mode = MODE_LOCAL; /* Detector de cambio desde Live Expressions */

/* --- TELEMETRIA DE POSICION Y ESTADO DESDE PLACA B (FASE B) --- */
volatile int32_t live_position = 0;       /* Posicion real del engranaje en grados (360 grados = 2400 ticks) */
volatile int32_t live_position_ticks = 0; /* Posicion cruda en pulsos de encoder recibidos por CAN */
volatile uint8_t live_motor_state = 0;    /* Estado del motor: 0=IDLE, 1=SCANNING, 2=HOMED, 3=HOMING, 4=ERROR */
volatile uint32_t telemetry_counter = 0;  /* Contador de paquetes de telemetria recibidos */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t CAN_SendMessage(uint32_t stdId, uint8_t dataByte);
uint8_t Button_Edge_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_pin_level, uint8_t *was_pressed, uint32_t *last_time);
void Indicate_CAN_Result(uint8_t ack_ok);
void CAN_Filter_Config(void);
void CAN_ProcessRxMessages(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Envía un comando de 1 byte por el bus CAN y espera confirmación (ACK)
  * @param  stdId: Identificador estándar CAN (0x010 = Motor/Botones 1..4, 0x020 = LED PB0)
  * @param  dataByte: Dato de 1 byte a transmitir
  * @retval 1 si el nodo esclavo respondió ACK, 0 si hubo timeout/error
  */
uint8_t CAN_SendMessage(uint32_t stdId, uint8_t dataByte)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[1];
  uint32_t txMailbox;

  /* Si las casillas de salida quedaron ocupadas por un intento previo fallido, liberarlas */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
  {
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  }

  txHeader.StdId = stdId;
  txHeader.ExtId = 0x01;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 1;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = dataByte;

  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
  {
    return 0;
  }

  /* Esperar hasta 25 ms a que la Blue Pill B confirme recepción (ACK) */
  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan, txMailbox))
  {
    if ((HAL_GetTick() - t0) > 25)
    {
      HAL_CAN_AbortTxRequest(&hcan, txMailbox);
      return 0;
    }
  }

  return 1;
}

/**
  * @brief  Restaura el estado del LED integrado PC13 (y titila rápido si faltó el ACK)
  */
void Indicate_CAN_Result(uint8_t ack_ok)
{
  if (!ack_ok)
  {
    for (int i = 0; i < 4; i++)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(40);
    }
  }

  /* Mantener en PC13 el reflejo fiel del estado del LED en PB0 de la placa B */
  if (current_led_state)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* PC13 ON (activo en bajo) */
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* PC13 OFF */
  }
}

/**
  * @brief  Detector de flanco de bajada con anti-rebote no bloqueante para pulsadores activos en bajo
  */
uint8_t Button_Edge_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_pin_level, uint8_t *was_pressed, uint32_t *last_time)
{
  uint8_t current_pin = HAL_GPIO_ReadPin(port, pin);
  uint8_t edge_detected = 0;

  if (current_pin != *last_pin_level)
  {
    *last_time = HAL_GetTick();
    *last_pin_level = current_pin;
  }

  if ((HAL_GetTick() - *last_time) > 40)
  {
    if (current_pin == GPIO_PIN_RESET && !(*was_pressed))
    {
      edge_detected = 1;
      *was_pressed = 1;
    }
    else if (current_pin == GPIO_PIN_SET)
    {
      *was_pressed = 0;
    }
  }

  return edge_detected;
}

/**
  * @brief  Configuracion del filtro CAN para recibir todos los mensajes (incluida telemetria 0x030) en FIFO0
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
  * @brief  Procesa los mensajes pendientes en FIFO0 de recepcion CAN (Telemetria de Placa B, ID = 0x030)
  */
void CAN_ProcessRxMessages(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      if (rxHeader.StdId == 0x030 && rxHeader.DLC >= 4)
      {
        int32_t raw_ticks = (int32_t)((uint32_t)rxData[0] |
                                      ((uint32_t)rxData[1] << 8) |
                                      ((uint32_t)rxData[2] << 16) |
                                      ((uint32_t)rxData[3] << 24));
        live_position_ticks = raw_ticks;
        live_position = (raw_ticks * 360) / TICKS_PER_GEAR_REV;
        if (rxHeader.DLC >= 5)
        {
          live_motor_state = rxData[4];
        }
        telemetry_counter++;
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* 1. Iniciar el periferico CAN con filtro e interrupcion de recepcion FIFO0 */
  CAN_Filter_Config();
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. Destello inicial de 3 parpadeos en PC13 para confirmar inicio correcto */
  for (int i = 0; i < 3; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* ON */
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* OFF */
    HAL_Delay(100);
  }

  /* Inicialmente apagado reflejando el LED en PB0 apagado */
  current_led_state = 0;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 0. Procesar mensajes CAN pendientes en FIFO0 (respaldo por polling + interrupcion) */
    CAN_ProcessRxMessages();

    /* 1. Sincronizacion si el modo se cambia directamente desde la PC (Live Expressions) */
    if (system_mode != last_system_mode)
    {
      if (system_mode == MODE_REMOTE)
      {
        current_led_state = 1;
        uint8_t ok = CAN_SendMessage(0x020, 0x01);
        Indicate_CAN_Result(ok);
      }
      else
      {
        system_mode = MODE_LOCAL;
        current_led_state = 0;
        uint8_t ok = CAN_SendMessage(0x020, 0x00);
        Indicate_CAN_Result(ok);
      }
      last_system_mode = system_mode;
    }

    /* 2. Pulsadores fisicos de seleccion de modo (BTN5 = Modo Local, BTN6 = Modo Remoto) */
    /* BTN5 (PA10 o PB10): MODO LOCAL -> LED PB0 y PC13 APAGADOS */
    uint8_t b5_pa = Button_Edge_Pressed(BTN5_GPIO_Port, BTN5_Pin, &btn5_pa_last_level, &btn5_pa_was_pressed, &btn5_pa_debounce_time);
    uint8_t b5_pb = Button_Edge_Pressed(GPIOB, GPIO_PIN_10, &btn5_pb_last_level, &btn5_pb_was_pressed, &btn5_pb_debounce_time);
    if (b5_pa || b5_pb)
    {
      system_mode = MODE_LOCAL;
      last_system_mode = MODE_LOCAL;
      current_led_state = 0;
      uint8_t ok = CAN_SendMessage(0x020, 0x00);
      Indicate_CAN_Result(ok);
    }

    /* BTN6 (PA11 o PB11): MODO REMOTO -> LED PB0 y PC13 ENCENDIDOS */
    uint8_t b6_pa = Button_Edge_Pressed(BTN6_GPIO_Port, BTN6_Pin, &btn6_pa_last_level, &btn6_pa_was_pressed, &btn6_pa_debounce_time);
    uint8_t b6_pb = Button_Edge_Pressed(GPIOB, GPIO_PIN_11, &btn6_pb_last_level, &btn6_pb_was_pressed, &btn6_pb_debounce_time);
    if (b6_pa || b6_pb)
    {
      system_mode = MODE_REMOTE;
      last_system_mode = MODE_REMOTE;
      current_led_state = 1;
      uint8_t ok = CAN_SendMessage(0x020, 0x01);
      Indicate_CAN_Result(ok);
    }

    /* 3. Ejecucion de Comandos segun el Modo Activo */
    if (system_mode == MODE_LOCAL)
    {
      /* --- MODO LOCAL: COMANDOS DESDE LOS BOTONES FISICOS (BTN1..BTN4) --- */
      /* BTN1 (PB12): Iniciar barrido oscilante de 2 vueltas */
      if (Button_Edge_Pressed(BTN1_GPIO_Port, BTN1_Pin, &btn1_last_level, &btn1_was_pressed, &btn1_debounce_time))
      {
        uint8_t ok = CAN_SendMessage(0x010, 0x01);
        Indicate_CAN_Result(ok);
      }

      /* BTN2 (PB13): Fijar posicion Home */
      if (Button_Edge_Pressed(BTN2_GPIO_Port, BTN2_Pin, &btn2_last_level, &btn2_was_pressed, &btn2_debounce_time))
      {
        uint8_t ok = CAN_SendMessage(0x010, 0x02);
        Indicate_CAN_Result(ok);
      }

      /* BTN3 (PB14): Regresar a posicion Home */
      if (Button_Edge_Pressed(BTN3_GPIO_Port, BTN3_Pin, &btn3_last_level, &btn3_was_pressed, &btn3_debounce_time))
      {
        uint8_t ok = CAN_SendMessage(0x010, 0x03);
        Indicate_CAN_Result(ok);
      }

      /* BTN4 (PB15): Parada de emergencia / Stop */
      if (Button_Edge_Pressed(BTN4_GPIO_Port, BTN4_Pin, &btn4_last_level, &btn4_was_pressed, &btn4_debounce_time))
      {
        uint8_t ok = CAN_SendMessage(0x010, 0x04);
        Indicate_CAN_Result(ok);
      }

      /* En modo local se descartan comandos remotos */
      if (remote_cmd != 0)
      {
        remote_cmd = 0;
      }
    }
    else
    {
      /* --- MODO REMOTO: COMANDOS DESDE LA PC / HMI ---
       *   1: Barrido (BTN1)        5: Giro +30° (+200 ticks)     9: Giro -30° (-200 ticks)
       *   2: Fijar Home (BTN2)     6: Giro +90° (+600 ticks)    10: Giro -90° (-600 ticks)
       *   3: Ir a Home (BTN3)      7: Giro +180° (+1200 ticks)  11: Giro -180° (-1200 ticks)
       *   4: STOP (BTN4)           8: Giro +360° (+2400 ticks)  12: Giro -360° (-2400 ticks)
       */
      if (remote_cmd >= 1 && remote_cmd <= 12)
      {
        uint8_t cmd_to_send = remote_cmd;
        remote_cmd = 0;
        uint8_t ok = CAN_SendMessage(0x010, cmd_to_send);
        Indicate_CAN_Result(ok);
      }
      else if (remote_cmd != 0)
      {
        remote_cmd = 0;
      }

      /* Seguridad de banco: el boton fisico BTN4 (Stop) siempre frena ante emergencias */
      if (Button_Edge_Pressed(BTN4_GPIO_Port, BTN4_Pin, &btn4_last_level, &btn4_was_pressed, &btn4_debounce_time))
      {
        uint8_t ok = CAN_SendMessage(0x010, 0x04);
        Indicate_CAN_Result(ok);
      }
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
