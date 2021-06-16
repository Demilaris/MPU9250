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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf_SWO.h"
#include "mpu9250.h"
#include "tm_stm32_ahrs_imu.h"
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
MPU9250_t mpu9250;
TM_AHRSIMU_t IMU;
I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef status;
uint8_t pTxData[200];
UART_HandleTypeDef huart4;

uint8_t isDeviceConnected = 0;
HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250);

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */


	sprintf((char*) pTxData, "UART IS WORKS 0000\r\n");
	status = HAL_UART_Transmit(&huart4, pTxData, strlen((char*) pTxData), 200);
//	I2C_Scan();

	MPU9250_Init(&mpu9250, MPU9250_Device_0, ACCEL_SCALE_2G, GYRO_SCALE_250dps, MAG_SCALE_16bit);

	if (whoAmI_Check(&mpu9250) != HAL_ERROR)
			isDeviceConnected = 1;
		else
			isDeviceConnected = 0;

//    sprintf((char *)pTxData, "status = %d\r\n", isDeviceConnected);
//    status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);




    //
    TM_AHRSIMU_Init(&IMU, 1000, 0.5, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  printf("Hello SWO in new project!\r\n");
//		MPU9250_ReadTemperature(&mpu9250);


//		 sprintf((char *)pTxData, "Is data ready ? = %x\r ", MPU9250_DataReady(&mpu9250));
//		 status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);
		 if (MPU9250_DataReady(&mpu9250))
		 {
			MPU9250_ReadAcc(&mpu9250);
			MPU9250_ReadGyro(&mpu9250);
			MPU9250_ReadMag(&mpu9250);

			MPU9250_ReadTemperature(&mpu9250);

			sprintf((char *)pTxData, "x= %f, 	y = %f,	 z = %f	 	temp = %f\r\n", mpu9250.acc[0],mpu9250.acc[1],mpu9250.acc[2],mpu9250.temp);
			status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);
		 }


//		 TM_AHRSIMU_UpdateIMU(&IMU,mpu9250.gyro[0],mpu9250.gyro[1],mpu9250.gyro[2],mpu9250.acc[0],mpu9250.acc[1],mpu9250.acc[2]);
//
//		sprintf((char *)pTxData, "R= %f, 	P = %f,	 Y = %f	 	temp = %f\r\n", IMU.Roll,IMU.Pitch,IMU.Yaw,mpu9250.temp);
//		status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);

//		HAL_Delay(100);
//		sprintf((char *)pTxData,"x = %f,   y= %f,   z = %f ,  temp = %f \r\n",mpu9250.acc[0],mpu9250.acc[1],mpu9250.acc[2],mpu9250.temp);
//		status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//void I2C_Scan() {
//	HAL_StatusTypeDef res;
//	char info [] = "Scanning I2C bus..\r\n";
//	HAL_UART_Transmit(&huart4, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);
//	for (uint16_t i = 0; i < 128; i++) {
//		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, HAL_MAX_DELAY);
//
//		if (res == HAL_OK)
//
//		{
//			char msg[64];
//			    	snprintf(msg, sizeof(msg), "0x%02X", i);
//			    	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//			      	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
////
////					for (uint16_t i = 0; i < strlen(msg); i++)
////					{
////						printf(" %c", msg[i]);
////
////					}
////					printf("\r\n");
//		}
//
//		else HAL_UART_Transmit(&huart4, (uint8_t*)".", 1, HAL_MAX_DELAY);
//
//	}
//	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//}



HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250)
{

	uint8_t data;
	/* MPU9250 Who Am I Register Check */
	if (readByte(&hi2c1, mpu9250->I2C_Addr, WHO_AM_I, &data) != HAL_OK)
	{

		if (data != 0x71)
			return HAL_ERROR;
	}
//	sprintf((char *)pTxData, "status = %x\r\n", data);
//	status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);

	/* AK8963 Who Am I Register Check */
	if (readByte(&hi2c1, mpu9250->I2C_Addr_Mag, WIA, &data) != HAL_OK)
	{
		if (data != 0x48)
			return HAL_ERROR;
	}

	return HAL_OK;
}



//HAL_StatusTypeDef readAcc(MPU9250_t *mpu9250)
//{
//	uint8_t data[6];
//	/* MPU9250 Who Am I Register Check */
//	readMultiBytes(&hi2c1, mpu9250->I2C_Addr, ACCEL_XOUT_H, data, 6);
//
//	mpu9250->acc_raw[0] = ((int16_t)data[0] << 8) | data[1];
//	mpu9250->acc_raw[1] = ((int16_t)data[2] << 8) | data[3];
//	mpu9250->acc_raw[2] = ((int16_t)data[4] << 8) | data[5];
//
//	mpu9250->acc[0] = (float)MPU9250->acc_raw[0] * MPU9250->accMult;
//	mpu9250->acc[1] = (float)MPU9250->acc_raw[1] * MPU9250->accMult;
//	MPU9250->acc[2] = (float)MPU9250->acc_raw[2] * MPU9250->accMult;
//}



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
	while (1) {
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
