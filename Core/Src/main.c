/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for Tarea1 */
osThreadId_t Tarea1Handle;
const osThreadAttr_t Tarea1_attributes = {
  .name = "Tarea1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Tarea2 */
osThreadId_t Tarea2Handle;
const osThreadAttr_t Tarea2_attributes = {
  .name = "Tarea2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue */
osMessageQueueId_t QueueHandle;
const osMessageQueueAttr_t Queue_attributes = {
  .name = "Queue"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for BinarySem */
osSemaphoreId_t BinarySemHandle;
const osSemaphoreAttr_t BinarySem_attributes = {
  .name = "BinarySem"
};
/* Definitions for Flag */
osEventFlagsId_t FlagHandle;
const osEventFlagsAttr_t Flag_attributes = {
  .name = "Flag"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void Presion(void *argument);
void MoveServos(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address);
void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off);
void pca9685_Degrees2PWM(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint8_t grados);

void Indice(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Corazon(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Anular(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Menique(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);
void Pulgar(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados);

uint8_t buffRx[6];
//uint8_t presion[5] = {0,0,0,0,0};
//uint8_t presion_M[5] = {0,0,0,0,0};
uint8_t oldMove[5] = {180,180,180,180,180};
uint8_t Move[5] = {0,0,0,0,0};

uint8_t buffError[1] = {20};
//uint8_t count = 0;
//uint8_t testigo = 0;

struct datos
{
	uint8_t Move_pulgar;
	uint8_t Move_indice;
	uint8_t Move_corazon;
	uint8_t Move_anular;
	uint8_t Move_menique;
};

struct presion{
	uint8_t Pres_Pulgar;
	uint8_t Pres_Indice;
	uint8_t Pres_Corazon;
	uint8_t Pres_Anular;
	uint8_t Pres_Menique;

};

#define presionMax  40 //???

//struct datos buff;
struct datos Dedos;
struct presion press;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

	pca9685_init(&hi2c1, 0x80);

	pca9685_Degrees2PWM(&hi2c1, 0x80, 0, 180);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 1, 180);

	pca9685_Degrees2PWM(&hi2c1, 0x80, 2, 180);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 3, 180);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 4, 180);
	//pca9685_Degrees2PWM(&hi2c1, 0x80, 2, 180);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem */
  BinarySemHandle = osSemaphoreNew(1, 1, &BinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue */
  QueueHandle = osMessageQueueNew (16, sizeof(struct datos), &Queue_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(struct presion), &myQueue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Tarea1 */
  Tarea1Handle = osThreadNew(Presion, NULL, &Tarea1_attributes);

  /* creation of Tarea2 */
  Tarea2Handle = osThreadNew(MoveServos, NULL, &Tarea2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Flag */
  FlagHandle = osEventFlagsNew(&Flag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  HAL_UART_Receive_IT(&huart1,buffRx,6);


  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address)
{
	#define PCA9685_MODE1 0x00

	uint8_t initStruct[2];
	uint8_t prescale = 3;
	HAL_I2C_Master_Transmit(hi2c, address, PCA9685_MODE1,1 ,1);
	uint8_t oldmode = 0;

	uint8_t newmode = ((oldmode & 0x7F)|0x10);
	initStruct[0] = PCA9685_MODE1;
	initStruct[1] = newmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = prescale;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = oldmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	osDelay(5);
	initStruct[1] = (oldmode|0xA1);
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
}

void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[5] = {0x06 + 4*num,on,(on >> 8), off, (off >> 8)};
	HAL_I2C_Master_Transmit(&hi2c1, address, outputBuffer, 5, 1);
}

void pca9685_Degrees2PWM(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint8_t grados)
{
	uint16_t off;
	off = (9.45*grados) + 300;

	if(off > 2000){
		off = 2000;
	}
	else if(off < 300){
		off = 300;
	}

	pca9685_pwm(hi2c, address, num, 0, off);

}

void Indice(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 15, grados);
	pca9685_Degrees2PWM(hi2c, address, 11, grados);
	pca9685_Degrees2PWM(hi2c, address, 7, grados);
}

void Corazon(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 14, grados);
	pca9685_Degrees2PWM(hi2c, address, 10, grados);
	pca9685_Degrees2PWM(hi2c, address, 6, grados);
}

void Anular(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 13, grados);
	pca9685_Degrees2PWM(hi2c, address, 9, grados);
	pca9685_Degrees2PWM(hi2c, address, 5, grados);
}

void Menique(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	pca9685_Degrees2PWM(hi2c, address, 12, grados);
	pca9685_Degrees2PWM(hi2c, address, 8, grados);
	pca9685_Degrees2PWM(hi2c, address, 4, grados);
}

void Pulgar(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t grados){
	uint8_t grados_t;
	grados_t = ((180-grados)/1.8)+50;
	pca9685_Degrees2PWM(hi2c, address, 1, grados_t);
	pca9685_Degrees2PWM(hi2c, address, 3, grados);
	pca9685_Degrees2PWM(hi2c, address, 2, grados);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//xEventGroupSetBitsFromISR(FlagHandle, 1, pdFALSE);
	//struct datos Dedos;


	/*if(buffRx[0] == 200){
		Dedos.Move_pulgar = buffRx[1];
		Dedos.Move_indice = buffRx[2];
		Dedos.Move_corazon = buffRx[3];
		Dedos.Move_anular = buffRx[4];
		Dedos.Move_menique = buffRx[5];

	}else if(buffRx[1] == 200){
		Dedos.Move_pulgar = buffRx[2];
		Dedos.Move_indice = buffRx[3];
		Dedos.Move_corazon = buffRx[4];
		Dedos.Move_anular = buffRx[5];
		Dedos.Move_menique = buffRx[0];

	}else if(buffRx[2] == 200){
		Dedos.Move_pulgar = buffRx[3];
		Dedos.Move_indice = buffRx[4];
		Dedos.Move_corazon = buffRx[5];
		Dedos.Move_anular = buffRx[0];
		Dedos.Move_menique = buffRx[1];

	}else if(buffRx[3] == 200){
		Dedos.Move_pulgar = buffRx[4];
		Dedos.Move_indice = buffRx[5];
		Dedos.Move_corazon = buffRx[0];
		Dedos.Move_anular = buffRx[1];
		Dedos.Move_menique = buffRx[2];

	}else if(buffRx[4] == 200){
		Dedos.Move_pulgar = buffRx[5];
		Dedos.Move_indice = buffRx[0];
		Dedos.Move_corazon = buffRx[1];
		Dedos.Move_anular = buffRx[2];
		Dedos.Move_menique = buffRx[3];

	}else if(buffRx[5] == 200){
		Dedos.Move_pulgar = buffRx[0];
		Dedos.Move_indice = buffRx[1];
		Dedos.Move_corazon = buffRx[2];
		Dedos.Move_anular = buffRx[3];
		Dedos.Move_menique = buffRx[4];
	}*/

	for(int i = 0; i<6; i++){
		if(buffRx[i] == 200){
			Dedos.Move_pulgar = buffRx[(i+1)%6];
			Dedos.Move_indice = buffRx[(i+2)%6];
			Dedos.Move_corazon = buffRx[(i+3)%6];
			Dedos.Move_anular = buffRx[(i+4)%6];
			Dedos.Move_menique = buffRx[(i+5)%6];
			break;
		}
	}

	osMessageQueuePut(QueueHandle, &Dedos, 0, 0);

	HAL_UART_Receive_IT(&huart1, buffRx, 6);
	//xEventGroupSetBitsFromISR(FlagHandle, 1, pdFALSE);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_Presion */
/**
* @brief Function implementing the Trarea7 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Presion */
void Presion(void *argument)
{
  /* USER CODE BEGIN 5 */

	HAL_StatusTypeDef status;
	//osStatus_t val;
	//struct presion press;
  /* Infinite loop */
  for(;;)
  {


	   //val = osMutexAcquire(Mutex1Handle, osWaitForever);
	  //if(val == osOK){
	  //xEventGroupWaitBits(FlagHandle,1,pdFALSE,pdFALSE,portMAX_DELAY);
	 // osSemaphoreAcquire(BinarySemHandle, osWaitForever);

		  HAL_ADC_Start(&hadc1);
			  status = HAL_ADC_PollForConversion(&hadc1, 1);
			  if(status == HAL_OK){
				  press.Pres_Pulgar = HAL_ADC_GetValue(&hadc1);
			  }

			  status = HAL_ADC_PollForConversion(&hadc1, 1);
			  if(status == HAL_OK){
				  press.Pres_Indice = HAL_ADC_GetValue(&hadc1);
			  }

			  status = HAL_ADC_PollForConversion(&hadc1, 1);
			  if(status == HAL_OK){
				  press.Pres_Corazon = HAL_ADC_GetValue(&hadc1);
			  }

			  status = HAL_ADC_PollForConversion(&hadc1, 1);
			  if(status == HAL_OK){
				  press.Pres_Anular = HAL_ADC_GetValue(&hadc1);
			  }

			  status = HAL_ADC_PollForConversion(&hadc1, 1);
			  if(status == HAL_OK){
				  press.Pres_Menique = HAL_ADC_GetValue(&hadc1);
			  }

			  HAL_ADC_Stop(&hadc1);

			  osMessageQueuePut(myQueue02Handle, &press, 0, 0);

			  //osSemaphoreRelease(BinarySemHandle);

	/*	  if(osMessageQueueGet(QueueHandle, &buff, NULL, osWaitForever) == osOK){

			  xEventGroupClearBits(FlagHandle, 2);
			  xEventGroupSetBits(FlagHandle, 4);

			  //osMutexRelease(Mutex1Handle);

		  }*/
	  //}


	  osDelay(5);



  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MoveServos */
/**
* @brief Function implementing the Tarea2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveServos */
void MoveServos(void *argument)
{
  /* USER CODE BEGIN MoveServos */
	osStatus_t val1;
	osStatus_t val2;

	struct datos buff;
	struct presion pres;
  /* Infinite loop */
  for(;;)
  {
	 // val = osMutexAcquire(Mutex1Handle, osWaitForever);
	  //if(val == osOK){
	 //osSemaphoreAcquire(BinarySemHandle, osWaitForever);
	  val1 = osMessageQueueGet(QueueHandle, &buff, NULL, 0);
	  val2 = osMessageQueueGet(myQueue02Handle, &pres, NULL, 0);

	 if((val1 && val2) == osOK){

		 /* if(!(buff.Move_pulgar < oldMove[0] && pres.Pres_Pulgar > presionMax)){
			pca9685_Degrees2PWM(&hi2c1, 0x80, 3, buff.Move_pulgar);
			pca9685_Degrees2PWM(&hi2c1, 0x80, 2, buff.Move_pulgar);
		  }
		  else{
			pca9685_Degrees2PWM(&hi2c1, 0x80, 3, oldMove[0]);
			pca9685_Degrees2PWM(&hi2c1, 0x80, 2, oldMove[0]);
		  }

		  oldMove[0] = buff.Move_pulgar;

		//----------------------------------------------------------------------

		  if(pres.Pres_Indice <40 || buff.Move_indice > oldMove[1]){
			  pca9685_Degrees2PWM(&hi2c1, 0x80, 15, buff.Move_indice);
			  pca9685_Degrees2PWM(&hi2c1, 0x80, 11, buff.Move_indice);
			  pca9685_Degrees2PWM(&hi2c1, 0x80, 7, buff.Move_indice);
		  }
		  oldMove[1] = buff.Move_indice;

		//-----------------------------------------------------------------------
		  if(!(buff.Move_corazon < oldMove[2] && pres.Pres_Corazon > presionMax)){
				pca9685_Degrees2PWM(&hi2c1, 0x80, 14, buff.Move_corazon);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 10, buff.Move_corazon);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 6, buff.Move_corazon);
			  }
		  else{
				pca9685_Degrees2PWM(&hi2c1, 0x80, 14, oldMove[2]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 10, oldMove[2]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 6, oldMove[2]);
		  }
		  oldMove[2]= buff.Move_corazon;
		  //-------------------------------------------------------------------

		  if(!(buff.Move_anular < oldMove[3] && pres.Pres_Anular > presionMax)){
				pca9685_Degrees2PWM(&hi2c1, 0x80, 13, buff.Move_anular);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 9, buff.Move_anular);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 5, buff.Move_anular);
		  }
		  else{
				pca9685_Degrees2PWM(&hi2c1, 0x80, 13, oldMove[3]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 9, oldMove[3]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 5, oldMove[3]);
		  }

		  oldMove[3] = buff.Move_anular;
		  //------------------------------------------------------------------

		  if(!(buff.Move_menique < oldMove[4] && pres.Pres_Menique > presionMax)){
				pca9685_Degrees2PWM(&hi2c1, 0x80, 12, buff.Move_menique);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 8, buff.Move_menique);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 4, buff.Move_menique);
		  }
		  else{
				pca9685_Degrees2PWM(&hi2c1, 0x80, 12, oldMove[4]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 8, oldMove[4]);
				pca9685_Degrees2PWM(&hi2c1, 0x80, 4, oldMove[4]);
		  }

		  oldMove[4] = buff.Move_menique;*/

		  //xEventGroupClearBits(FlagHandle, 2);
		  //xEventGroupSetBits(FlagHandle, 2);
		  //HAL_UART_Receive_IT(&huart1, buffRx, 5);
		  //osMutexRelease(Mutex1Handle);
	  //}

	 }

	 //osSemaphoreRelease(BinarySemHandle);

	Pulgar(&hi2c1, 0x80, buff.Move_pulgar);
	Indice(&hi2c1, 0x80, buff.Move_indice);
	Corazon(&hi2c1, 0x80, buff.Move_corazon);
	Anular(&hi2c1, 0x80, buff.Move_anular);
	Menique(&hi2c1, 0x80, buff.Move_menique);


	/*pca9685_Degrees2PWM(&hi2c1, 0x80, 3, buff.Move_pulgar);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 2, buff.Move_pulgar);

	pca9685_Degrees2PWM(&hi2c1, 0x80, 15, buff.Move_indice);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 11, buff.Move_indice);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 7, buff.Move_indice);

	pca9685_Degrees2PWM(&hi2c1, 0x80, 12, buff.Move_menique);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 8, buff.Move_menique);
	pca9685_Degrees2PWM(&hi2c1, 0x80, 4, buff.Move_menique);*/


	  osDelay(5);

  }
  /* USER CODE END MoveServos */
}

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
