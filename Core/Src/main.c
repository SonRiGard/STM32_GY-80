
#include "main.h"
#include "GY80.h"
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

double DeltaT = 0.01;

float Temperature = 0;
float Pressure = 0;
float Altitude = 0;

char Temperature1[10];
char Pressure1[10];
char Altitude1[10];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void Calib (GY80_t *DataStruct);

uint8_t test;
GY80_t GY80;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	MX_TIM2_Init();
  MX_I2C1_Init();
	MX_USART2_UART_Init();
	ADXL_Init(&hi2c1);
	L3G4_Init(&hi2c1);
	HMC5883L_Init(&hi2c1);
	BMP180_Start();
	HAL_Delay(200);
	Calib(&GY80);
	HAL_TIM_Base_Start_IT(&htim2);
	
  while (1)
  {			
			
	char str[200]={"abc"};	
  sprintf(str,"Pitch_GYRO:%f:Roll_GYRO:%f:Yaw_GYRO:%f:Pitch_ACC:%f:Roll_ACC:%f:KalmanAngleX:%f:KalmanAngleY:%f\r\n",
					GY80.Pitch_GYRO,GY80.Roll_GYRO,GY80.Yaw_GYRO,GY80.Pitch_ACC,GY80.Roll_ACC,GY80.KalmanAngleX,GY80.KalmanAngleY);
	HAL_UART_Transmit(&huart2,(uint8_t*)str,strlen(str),0x100);
  }
  /* USER CODE END 3 */
}
static void Calib (GY80_t *DataStruct){
	int32_t SumACC_X = 0 ,SumACC_Y=0 ,SumACC_Z=0 , sumGyro_X = 0, sumGyro_Y= 0 , sumGyro_Z = 0;
	for(int16_t i = 0; i < 100 ; i++){
		uint8_t ACC[6];
		HAL_I2C_Mem_Read (&hi2c1, ADXL345_ADDRESS, DATAX0 , 1 ,(uint8_t *)ACC , 6 ,100 );
		DataStruct->Accel_X_RAW = (ACC[1]<<8)|ACC[0];
		DataStruct->Accel_Y_RAW = (ACC[3]<<8)|ACC[2];
		DataStruct->Accel_Z_RAW = (ACC[5]<<8)|ACC[4];
		
		uint8_t GYRO[6];
		static uint8_t Status = 0;
		while (Status != 1){
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, L3G4200D_STATUS , 1 ,&Status , 1 ,100 );
		Status = (Status&0x08)>>3;   
		}
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_X_L , 1 ,&GYRO[0] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_X_H , 1 ,&GYRO[1] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Y_L , 1 ,&GYRO[2] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Y_H , 1 ,&GYRO[3] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Z_L , 1 ,&GYRO[4] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Z_H , 1 ,&GYRO[5] , 1 ,100 );
			
		DataStruct->Gyro_X_RAW = ((GYRO[1]<<8)|GYRO[0]);
		DataStruct->Gyro_Y_RAW = ((GYRO[3]<<8)|GYRO[2]);
		DataStruct->Gyro_Z_RAW = ((GYRO[5]<<8)|GYRO[4]);
		
		SumACC_X +=   DataStruct->Accel_X_RAW;
		SumACC_Y += 	DataStruct->Accel_Y_RAW;
		SumACC_Z += 	DataStruct->Accel_Z_RAW;
		
		sumGyro_X += DataStruct->Gyro_X_RAW;
		sumGyro_Y += DataStruct->Gyro_Y_RAW;
		sumGyro_Z += DataStruct->Gyro_Z_RAW;
	}
	DataStruct->Accel_X_RAW_0 = SumACC_X/100;
	DataStruct->Accel_Y_RAW_0 = SumACC_Y/100;
	DataStruct->Accel_Z_RAW_0 = SumACC_Z/100 - 256;
	
	DataStruct->Gyro_X_RAW_0 = sumGyro_X/100;
	DataStruct->Gyro_Y_RAW_0 = sumGyro_Y/100;
	DataStruct->Gyro_Z_RAW_0 = sumGyro_Z/100;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Angle_GYRO(&hi2c1,&GY80);
	Angle_ACC(&hi2c1,&GY80);
	Kalman_angle_solve(&hi2c1,&GY80);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
	//RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}


static void MX_I2C1_Init(void)
{
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
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
