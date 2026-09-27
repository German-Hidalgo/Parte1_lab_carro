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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---------- Distancias en cm ---------- */
#define DIST_OBJETIVO_CM   9.0f   /* donde debe quedar el carro                   */
#define DIST_FRENADO_CM     5.0f   /* lo que se desliza al cortar: MEDIR y ajustar */
#define TOLERANCIA_CM       1.0f   /* margen para darse por llegado                */
#define HISTERESIS_CM       5.0f   /* si el obstaculo se aleja esto, vuelve a ir   */
#define ZONA_LENTA_CM      30.0f   /* error por debajo del cual trocea y va lento  */
#define DIST_SIN_ECO_CM   400.0f   /* valor usado cuando no hay eco                */

/* ---------- Tiempos de motor en ms (TIM3: 1 tick = 0.1 ms) ----------
 * Lejos: el motor sigue encendido durante los 60 ms de la medicion, asi
 * que va a potencia plena. Cerca: se apaga para medir (tres lecturas) y
 * solo se enciende T_ON_CERCA_MS, con lo que baja la velocidad.       */
#define T_ON_LEJOS_MS         70   /* encendido por vuelta cuando esta lejos       */
#define T_ON_CERCA_MS         35   /* encendido por vuelta cerca del objetivo      */
#define T_ARRANQUE_MS        300   /* primer empujon para vencer la friccion       */

/* ---------- Compensacion de trayectoria ----------
 * Se le restan ms al motor que empuja de mas. Si el carro se desvia a la
 * DERECHA, el motor izquierdo es el rapido: sube COMP_IZQ_MS.          */
#define COMP_IZQ_MS            25
#define COMP_DER_MS            20

/* ---------- Mediciones fallidas ---------- */
#define FALLOS_PARA_PARAR      3   /* lecturas malas seguidas antes de frenar      */

/* ---------- Tiempos del sensor en us (TIM2: 1 tick = 1 us) ---------- */
#define TIMEOUT_SUBIDA_US   5000
#define TIMEOUT_ECO_US     40000
#define CICLO_MEDICION_US  60000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IZQ_ON()     HAL_GPIO_WritePin(MOTOR_IZQ_GPIO_Port, MOTOR_IZQ_Pin, GPIO_PIN_SET)
#define IZQ_OFF()    HAL_GPIO_WritePin(MOTOR_IZQ_GPIO_Port, MOTOR_IZQ_Pin, GPIO_PIN_RESET)
#define DER_ON()     HAL_GPIO_WritePin(MOTOR_DER_GPIO_Port, MOTOR_DER_Pin, GPIO_PIN_SET)
#define DER_OFF()    HAL_GPIO_WritePin(MOTOR_DER_GPIO_Port, MOTOR_DER_Pin, GPIO_PIN_RESET)
#define MOTOR_ON()   do { IZQ_ON();  DER_ON();  } while(0)
#define MOTOR_OFF()  do { IZQ_OFF(); DER_OFF(); } while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static uint8_t llegado = 0;
static uint8_t en_marcha = 0;              /* 1 = el carro ya venia rodando  */
static float   error_prev = 1000.0f;       /* error de la vuelta anterior    */
static float   d_valida = DIST_SIN_ECO_CM; /* ultima lectura buena           */
static uint8_t fallos = 0;                 /* lecturas malas consecutivas    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void  espera_ms(uint16_t ms);
static void  motor_on_compensado(void);
static void  motor_pulso(uint16_t t_ms);
static float medir_cm(void);
static float medir_estable_cm(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Espera bloqueante con TIM3 (maximo 6553 ms) */
static void espera_ms(uint16_t ms)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while (__HAL_TIM_GET_COUNTER(&htim3) < ((uint32_t)ms * 10U));
}

/* Enciende los dos motores, pero al mas rapido lo arranca con retardo.
 * Asi la compensacion tambien actua durante la fase de medicion. */
static void motor_on_compensado(void)
{
  if (COMP_IZQ_MS == COMP_DER_MS)
  {
    MOTOR_ON();
  }
  else if (COMP_IZQ_MS > COMP_DER_MS)
  {
    DER_ON();
    espera_ms((uint16_t)(COMP_IZQ_MS - COMP_DER_MS));
    IZQ_ON();
  }
  else
  {
    IZQ_ON();
    espera_ms((uint16_t)(COMP_DER_MS - COMP_IZQ_MS));
    DER_ON();
  }
}

/* Un tramo de empuje. Al motor mas rapido se le restan sus ms de
 * compensacion, de modo que se apaga antes y el carro no se desvia. */
static void motor_pulso(uint16_t t_ms)
{
  uint16_t ti = (t_ms > COMP_IZQ_MS) ? (uint16_t)(t_ms - COMP_IZQ_MS) : 0;
  uint16_t td = (t_ms > COMP_DER_MS) ? (uint16_t)(t_ms - COMP_DER_MS) : 0;
  uint16_t tmin = (ti < td) ? ti : td;
  uint16_t tmax = (ti > td) ? ti : td;

  if (ti) IZQ_ON();
  if (td) DER_ON();

  espera_ms(tmin);
  if (ti < td)      IZQ_OFF();
  else if (td < ti) DER_OFF();

  espera_ms((uint16_t)(tmax - tmin));
  MOTOR_OFF();
}

/* Una medicion con TIM2 (1 us por tick). Siempre dura 60 ms en total,
 * asi dos llamadas seguidas respetan el tiempo minimo del HC-SR04.
 * No toca los motores: conservan el estado en que entren.
 * Devuelve cm, o -1 si no hubo eco. */
static float medir_cm(void)
{
  float d = -1.0f;
  uint32_t t1, t2;

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  while (__HAL_TIM_GET_COUNTER(&htim2) < 10);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_SUBIDA_US) goto fin;
  }
  t1 = __HAL_TIM_GET_COUNTER(&htim2);

  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
  {
    if ((__HAL_TIM_GET_COUNTER(&htim2) - t1) > TIMEOUT_ECO_US) goto fin;
  }
  t2 = __HAL_TIM_GET_COUNTER(&htim2);

  d = (float)(t2 - t1) / 58.0f;      /* ida y vuelta: 0.0343/2 cm/us = 1/58 */

fin:
  while (__HAL_TIM_GET_COUNTER(&htim2) < CICLO_MEDICION_US);
  return d;
}

/* Tres lecturas, se descartan las fallidas y se devuelve la mediana
 * (con dos validas, la menor, que es la opcion segura).
 * Devuelve -1 si ninguna sirvio. */
static float medir_estable_cm(void)
{
  float v[3], t;
  uint8_t n = 0, i;

  for (i = 0; i < 3; i++)
  {
    float x = medir_cm();
    if (x > 0.0f) v[n++] = x;
  }

  if (n == 0) return -1.0f;
  if (n == 1) return v[0];
  if (n == 2) return (v[0] < v[1]) ? v[0] : v[1];

  if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
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
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  MOTOR_OFF();
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint8_t lejos = (en_marcha && !llegado && (error_prev > ZONA_LENTA_CM));
    float d;

    /* --- Medicion --- */
    if (lejos)
    {
      motor_on_compensado();          /* lejos: mide empujando */
      d = medir_cm();
    }
    else
    {
      MOTOR_OFF();                    /* cerca: frena y mide quieto, 3 lecturas */
      d = medir_estable_cm();
    }

    /* --- Tratamiento de lecturas fallidas: sin dato NO es camino libre --- */
    if (d < 0.0f)
    {
      fallos++;
      if (fallos >= FALLOS_PARA_PARAR) d = 0.0f;   /* sin datos: detenerse */
      else                             d = d_valida;
    }
    else
    {
      fallos = 0;
      d_valida = d;
    }

    float error = d - (DIST_OBJETIVO_CM + DIST_FRENADO_CM);
    error_prev = error;

    /* --- Decision de marcha, con histeresis --- */
    if (llegado)
    {
      if (error > HISTERESIS_CM) llegado = 0;
    }
    else if (error <= TOLERANCIA_CM)
    {
      llegado = 1;
    }

    if (llegado)
    {
      MOTOR_OFF();
      en_marcha = 0;                  /* quieto en el objetivo */
    }
    else
    {
      uint16_t t_on;

      if (!en_marcha)
      {
        t_on = T_ARRANQUE_MS;         /* empujon inicial contra la friccion */
        en_marcha = 1;
      }
      else if (error > ZONA_LENTA_CM)
      {
        t_on = T_ON_LEJOS_MS;
      }
      else
      {
        t_on = T_ON_CERCA_MS;         /* cerca: tramos cortos, va mas lento */
      }

      motor_pulso(t_on);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|MOTOR_IZQ_Pin|MOTOR_DER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG_Pin MOTOR_IZQ_Pin MOTOR_DER_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|MOTOR_IZQ_Pin|MOTOR_DER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
