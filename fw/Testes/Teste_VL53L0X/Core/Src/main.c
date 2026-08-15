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
#include "vl53l0x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    LIDAR_OK = 0,
    LIDAR_FAULT,
    LIDAR_RECOVERING
} LidarState_t;

typedef struct {
    VL53L0X_DEV dev;
    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;
    uint8_t target_addr;      // endereço final do sensor (0x64, 0x5A, 0x52...)
    LidarState_t state;
    uint8_t fail_count;
    uint32_t recover_tick;
    uint8_t recover_step;
    uint8_t recover_attempts;
    uint8_t in_queue;
} LidarSensor_t;
/* USER CODE END PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

#define LIDAR_FAIL_THRESHOLD   5     // Numero de falhas seguidas antes de disparar recovery
#define LIDAR_RECOVER_DELAY_MS 20

LidarSensor_t *recovery_queue[3];
uint8_t queue_head = 0;
uint8_t queue_tail = 0;
uint8_t queue_count = 0;

LidarSensor_t sensors[3];

//---------- Dados lidos pelos sensores -----------------
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_RangingMeasurementData_t RangingData2;
VL53L0X_RangingMeasurementData_t RangingData3;
//------------------------------------------------------

VL53L0X_Dev_t vl53l0x_c;
VL53L0X_Dev_t vl53l0x_c2;
VL53L0X_Dev_t vl53l0x_c3;
VL53L0X_DEV Dev = &vl53l0x_c;
VL53L0X_DEV Dev2 = &vl53l0x_c2;
VL53L0X_DEV Dev3 = &vl53l0x_c3;

//=================VARIÁVEIS DE DEBUG====================
uint8_t address; 			//Endereço do sensor I2C do sensor
VL53L0X_Error status; 		//status da leitura I2C (0 é ok)
VL53L0X_Error status2;
VL53L0X_Error status3;
HAL_StatusTypeDef ret; 		// Verifica a comunicação I2C
uint8_t test = 0; 			// Verifica se as conexões de hardware estão ok

uint8_t bus_scan[128];


uint16_t distancia1;
uint16_t distancia2;
uint16_t distancia3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LidarInit(VL53L0X_DEV Dev) {
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	VL53L0X_WaitDeviceBooted(Dev);
	VL53L0X_DataInit(Dev);
	VL53L0X_StaticInit(Dev);
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

}

static void I2C_ScanBus(void)
{
    for (uint8_t addr = 1; addr < 128; addr++) {
        bus_scan[addr] = (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 5) == HAL_OK);
    }
}

//FUNÇÃO PARA CHECAR SE O SENSOR AINDA ESTÁ FUNCIONANDO
static uint8_t LidarIsAlive(uint8_t i2c_addr_shifted)
{
	return (HAL_I2C_IsDeviceReady(&hi2c1, i2c_addr_shifted, 1, 5) == HAL_OK);
}

//----------------- FILA DE RECONEXAO ---------------------------
static void QueuePush(LidarSensor_t *s)
{
    if (queue_count >= 3) return; // já na fila, nunca deveria estourar
    recovery_queue[queue_tail] = s;
    queue_tail = (queue_tail + 1) % 3;
    queue_count++;
}

static LidarSensor_t* QueuePeek(void)
{
    if (queue_count == 0) return NULL;
    return recovery_queue[queue_head];
}

static void QueuePop(void)
{
    if (queue_count == 0) return;
    queue_head = (queue_head + 1) % 3;
    queue_count--;
}

static void LidarRecoveryStep(LidarSensor_t *s)
{
	uint32_t now = HAL_GetTick();
	if (now - s->recover_tick < LIDAR_RECOVER_DELAY_MS) {
		return;
	}
	s->recover_tick = now;

	switch (s->recover_step)
	{
	case 0: // desliga o sensor via XSHUT
		HAL_GPIO_WritePin(s->xshut_port, s->xshut_pin, GPIO_PIN_RESET);
		s->recover_step = 1;
		break;

	case 1: // liga de novo
		HAL_GPIO_WritePin(s->xshut_port, s->xshut_pin, GPIO_PIN_SET);
		s->recover_step = 2;
		break;

	case 2: // reatribui endereço e reinicializa driver
		s->dev->I2cDevAddr = 0x52;
		VL53L0X_WaitDeviceBooted(s->dev);
		if (LidarIsAlive(0x52)) {
			VL53L0X_DataInit(s->dev);
			if (s->target_addr != 0x52) {
				VL53L0X_SetDeviceAddress(s->dev, s->target_addr);
				s->dev->I2cDevAddr = s->target_addr;
			}
			VL53L0X_SetDeviceMode(s->dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
			VL53L0X_StaticInit(s->dev);
			LidarInit(s->dev);
			VL53L0X_StartMeasurement(s->dev);

			s->state = LIDAR_OK;
			s->fail_count = 0;
			s->recover_attempts = 0;
		} else {
			s->recover_attempts++;
		}
		s->recover_step = 0;
		break;
	}
}

//detecta se o sensor está conectado
static void LidarUpdate(LidarSensor_t *s)
{
	if (s->state == LIDAR_FAULT && !s->in_queue) {
		HAL_GPIO_WritePin(s->xshut_port, s->xshut_pin, GPIO_PIN_RESET); // isola do barramento até ser a vez dele
		s->in_queue = 1;
		QueuePush(s);
	}
}

static void RecoveryDispatcher(void)
{
    LidarSensor_t *s = QueuePeek();
    if (s == NULL) return;

    if (s->state != LIDAR_RECOVERING) {
        s->state = LIDAR_RECOVERING;
        s->recover_step = 0;
        s->recover_attempts = 0;
        s->recover_tick = HAL_GetTick();
    }

    LidarRecoveryStep(s);

    if (s->state == LIDAR_OK) {
        s->in_queue = 0;
        QueuePop();
    } else if (s->recover_attempts >= 10) {
        HAL_GPIO_WritePin(s->xshut_port, s->xshut_pin, GPIO_PIN_RESET); // isola de novo enquanto espera a próxima vez
        s->state = LIDAR_FAULT;
        s->recover_attempts = 0;
        QueuePop();
        QueuePush(s);
        // in_queue continua 1, o sensor nunca sai de fato da fila
    }
}

// Chame isso depois de cada medição, passando o status retornado
static void LidarReportStatus(LidarSensor_t *s, VL53L0X_Error status)
{
	if (status != VL53L0X_ERROR_NONE) {
		s->fail_count++;
		if (s->fail_count >= LIDAR_FAIL_THRESHOLD) {
			s->state = LIDAR_FAULT;
		}
	} else {
		s->fail_count = 0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  	//--------------Configurando os pinos Xshut------------------

	HAL_GPIO_WritePin(Lidar_xShutdown_GPIO_Port, Lidar_xShutdown_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(20);

	//------------------ sensor3 ---------------------------
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(20);
	Dev3->I2cHandle = &hi2c1;
	Dev3->I2cDevAddr = 0x52;

	VL53L0X_DataInit(Dev3);
	VL53L0X_SetDeviceAddress(Dev3, 0x64);
	Dev3->I2cDevAddr = 0x64;
	//	LidarInit2();

	VL53L0X_SetDeviceMode(Dev3, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	VL53L0X_StaticInit(Dev3);
	LidarInit(Dev3);
	VL53L0X_StartMeasurement(Dev3);


	//------------------ sensor2 ---------------------------
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(20);
	Dev2->I2cHandle = &hi2c1;
	Dev2->I2cDevAddr = 0x52;

	VL53L0X_DataInit(Dev2);
	VL53L0X_SetDeviceAddress(Dev2, 0x5A);
	Dev2->I2cDevAddr = 0x5A;

	VL53L0X_SetDeviceMode(Dev2, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	VL53L0X_StaticInit(Dev2);
	LidarInit(Dev2);
	VL53L0X_StartMeasurement(Dev2);


	//------------------ sensor1 ---------------------------
	HAL_GPIO_WritePin(Lidar_xShutdown_GPIO_Port, Lidar_xShutdown_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;
	VL53L0X_SetDeviceAddress(Dev, 0x30);
	Dev->I2cDevAddr = 0x30;
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	VL53L0X_StaticInit(Dev);
	LidarInit(Dev);
	VL53L0X_StartMeasurement(Dev);

    HAL_Delay(20);

	ret = HAL_I2C_Mem_Read(
	    &hi2c1,
	    0x64,
	    0xC0,
	    I2C_MEMADD_SIZE_8BIT,
	    &test,
	    1,
	    100
	);

	sensors[0] = (LidarSensor_t){ Dev,  Lidar_xShutdown_GPIO_Port, Lidar_xShutdown_Pin, 0x30, LIDAR_OK, 0, 0, 0 };
	sensors[1] = (LidarSensor_t){ Dev2, GPIOB, GPIO_PIN_4, 0x5A, LIDAR_OK, 0, 0, 0 };
	sensors[2] = (LidarSensor_t){ Dev3, GPIOB, GPIO_PIN_5, 0x64, LIDAR_OK, 0, 0, 0 };



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (sensors[0].state == LIDAR_OK) {
		  status = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
		  LidarReportStatus(&sensors[0], status);
	  }
	  LidarUpdate(&sensors[0]);

	  if (sensors[1].state == LIDAR_OK) {
		  status2 = VL53L0X_PerformSingleRangingMeasurement(Dev2, &RangingData2);
		  LidarReportStatus(&sensors[1], status2);
	  }
	  LidarUpdate(&sensors[1]);

	  if (sensors[2].state == LIDAR_OK) {
		  status3 = VL53L0X_PerformSingleRangingMeasurement(Dev3, &RangingData3);
		  LidarReportStatus(&sensors[2], status3);
	  }
	  LidarUpdate(&sensors[2]);

	  RecoveryDispatcher();   // <-- adicionado, processa só o da frente da fila

	  if (status == VL53L0X_ERROR_NONE && RangingData.RangeStatus == 0) {
		  distancia1 = RangingData.RangeMilliMeter;
	  }
	  if (status2 == VL53L0X_ERROR_NONE && RangingData2.RangeStatus == 0) {
		  distancia2 = RangingData2.RangeMilliMeter;
	  }
	  if (status3 == VL53L0X_ERROR_NONE && RangingData3.RangeStatus == 0) {
		  distancia3 = RangingData3.RangeMilliMeter;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  HAL_GPIO_WritePin(GPIOB, Lidar_xShutdown_Pin|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : Lidar_xShutdown_Pin PB4 PB5 */
  GPIO_InitStruct.Pin = Lidar_xShutdown_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
