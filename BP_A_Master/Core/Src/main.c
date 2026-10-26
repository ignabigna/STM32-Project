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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Variables de estado y anti-rebote para los pulsadores */
uint8_t btn5_last_level = GPIO_PIN_SET;
uint8_t btn5_was_pressed = 0;
uint32_t btn5_debounce_time = 0;

uint8_t btn6_last_level = GPIO_PIN_SET;
uint8_t btn6_was_pressed = 0;
uint32_t btn6_debounce_time = 0;

/* Estado actual reflejado del LED */
uint8_t current_led_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t CAN_SendLedCommand(uint8_t state);
uint8_t Button_Edge_Pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t *last_pin_level, uint8_t *was_pressed, uint32_t *last_time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Envía un mensaje por bus CAN para controlar el LED del nodo esclavo
  *         y verifica que el nodo esclavo responda con ACK.
  * @param  state: 1 = Encender LED, 0 = Apagar LED
  * @retval 1 si el mensaje fue confirmado (ACK) en el bus, 0 si hubo error/timeout
  */
uint8_t CAN_SendLedCommand(uint8_t state)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[1];
  uint32_t txMailbox;

  /* Si las casillas de salida quedaron ocupadas por un intento previo fallido, liberarlas */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
  {
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  }

  txHeader.StdId = 0x020;              /* ID estándar acordado para comando de LED */
  txHeader.ExtId = 0x01;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 1;                    /* 1 byte de datos */
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = state;

  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
  {
    return 0;
  }

  /* Esperar hasta 25 ms a que la Blue Pill B confirme recepción (ACK) y se vacíe el mailbox */
  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan, txMailbox))
  {
    if ((HAL_GetTick() - t0) > 25)
    {
      /* No hubo ACK en el bus: abortar para no dejar el transmisor trabado */
      HAL_CAN_AbortTxRequest(&hcan, txMailbox);
      return 0;
    }
  }

  return 1;
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
  /* 1. Iniciar el periférico CAN */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. Destello inicial de 3 parpadeos en PC13 para confirmar inicio correcto */
  for (int i = 0; i < 3; i++)
  {
    HAL_GPIO_WritePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin, GPIO_PIN_RESET); /* ON */
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin, GPIO_PIN_SET);   /* OFF */
    HAL_Delay(100);
  }

  /* El LED integrado de la Blue Pill (PC13) es activo en bajo (0 = encendido, 1 = apagado) */
  /* Inicialmente apagado reflejando el LED en PB0 apagado */
  current_led_state = 0;
  HAL_GPIO_WritePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Detección de pulsación en BTN5 (PB12) -> Encender LED en Blue Pill B */
    if (Button_Edge_Pressed(BTN5_GPIO_Port, BTN5_Pin, &btn5_last_level, &btn5_was_pressed, &btn5_debounce_time))
    {
      current_led_state = 1;
      if (CAN_SendLedCommand(1) == 0)
      {
        /* Parpadeo rápido de aviso si no hubo ACK en el bus CAN */
        for (int i = 0; i < 4; i++)
        {
          HAL_GPIO_TogglePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin);
          HAL_Delay(40);
        }
      }

      /* Reflejar en el LED integrado de la Blue Pill A (PC13 activo en bajo) */
      HAL_GPIO_WritePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin, GPIO_PIN_RESET); /* ON */
    }

    /* Detección de pulsación en BTN6 (PB13) -> Apagar LED en Blue Pill B */
    if (Button_Edge_Pressed(BTN6_GPIO_Port, BTN6_Pin, &btn6_last_level, &btn6_was_pressed, &btn6_debounce_time))
    {
      current_led_state = 0;
      if (CAN_SendLedCommand(0) == 0)
      {
        /* Parpadeo rápido de aviso si no hubo ACK en el bus CAN */
        for (int i = 0; i < 4; i++)
        {
          HAL_GPIO_TogglePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin);
          HAL_Delay(40);
        }
      }

      /* Reflejar en el LED integrado de la Blue Pill A (PC13 activo en bajo) */
      HAL_GPIO_WritePin(LED_INTEGRADO_GPIO_Port, LED_INTEGRADO_Pin, GPIO_PIN_SET); /* OFF */
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
