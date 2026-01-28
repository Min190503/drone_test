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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

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
uint8_t Rec_Data[14];
int16_t Accel_X_Raw, Accel_Y_Raw, Accel_Z_Raw;
int16_t Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw;
float Ax, Ay, Az, Gx, Gy, Gz;
char usb_msg[128];
float Pitch = 0.0f;
float Roll = 0.0f;
float Yaw = 0.0f;
//chon chan CS
#define CS_PORT GPIOA
#define CS_PIN GPIO_PIN_4
#define DT 0.02f
#define ALPHA 0.98f
#define RAD_TO_DEG 57.2957795f

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//ham doc 1 thanh ghi qua SPI
uint8_t SPI_Read(uint8_t RegAddr){
	uint8_t rx_data = 0;
	uint8_t tx_data = RegAddr | 0x80; // Bit 0x80 báo hiệu là lệnh ĐỌC (Read)

	//keo CS xuong thap de bat dau
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	//gui dia chi can doc
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100);

	//Nhan du lieu ve
	HAL_SPI_Receive(&hspi1, &rx_data, 1, 100);

	//Keo CS len cao de ket thuc
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

	return rx_data;
}

//Ham ghi vaof 1 thanh ghi qua SPI
void SPI_Write(uint8_t RegAddr, uint8_t Value){
	uint8_t data[2];
	data[0] = RegAddr; // bie dau la 0 bao hieu la GHI
	data[1] = Value;

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

//Ham doc nhieu bytes lien tiep - De doc Accel/Gyro
void SPI_Read_Burst(uint8_t RegAddr, uint8_t *pData, uint16_t Size){
	uint8_t tx_data = RegAddr | 0x80;

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100);
	HAL_SPI_Receive(&hspi1, pData, Size, 100);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}
void MPU_Complementary_Filter(void){
	float Accel_Pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * RAD_TO_DEG;
	float Accel_Roll  = atan2(-Ay, Az) * RAD_TO_DEG;

	Pitch = ALPHA * (Pitch + Gy * DT) + (1.0f - ALPHA) * Accel_Pitch;
	Roll  = ALPHA * (Roll  - Gx * DT) + (1.0f - ALPHA) * Accel_Roll;

	Yaw = Yaw + Gz * DT;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


//	HAL_Delay(1000);

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
//  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  SCB_DisableDCache();
    SCB_DisableICache();
//    HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // 1. Reset device
  SPI_Write(0x6B, 0x80);
  HAL_Delay(100);

  // 2. Disable I2C interface (Quan trọng khi dùng SPI)
  // Thanh ghi USER_CTRL (0x6A), bit 4 = 1 (I2C_IF_DIS)
  SPI_Write(0x6A, 0x10);

  // 3. Đánh thức (Wake up) - Clock Source = PLL Gyro Z
  SPI_Write(0x6B, 0x03);

  // 4. Cấu hình Gyro Range (Ví dụ +/- 500dps)
  SPI_Write(0x1B, 0x08);

  // 5. Cấu hình Accel Range (Ví dụ +/- 8G)
  SPI_Write(0x1C, 0x10);

  //Kiem tra chip (WHO_AM_I - 0x75)
  uint8_t who_am_i = SPI_Read(0x75);
  if(who_am_i == 0x70 || who_am_i == 0x71 || who_am_i == 0x73){
	  CDC_Transmit_FS((uint8_t*)"SPI OK! MOU Found!\n", 19);
  } else {
	  CDC_Transmit_FS((uint8_t*)"SPI Fail! Check wiring.!\n", 24);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	SPI_Read_Burst(0x3B, Rec_Data, 14);


	// Ghép dữ liệu (Bit cao << 8 | Bit thấp)
	Accel_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	Gyro_X_Raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	Gyro_Y_Raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	Gyro_Z_Raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	// Chuyển đổi đơn vị (Tùy cấu hình Range ở trên)
	// Với +/- 8G thì chia 4096.0
	Ax = Accel_X_Raw / 4096.0;
	Ay = Accel_Y_Raw / 4096.0;
	Az = Accel_Z_Raw / 4096.0;

	// Với +/- 500dps thì chia 65.5
	Gx = Gyro_X_Raw / 65.5;
	Gy = Gyro_Y_Raw / 65.5;
	Gz = Gyro_Z_Raw / 65.5;

	MPU_Complementary_Filter();

	// Gửi lên máy tính
	sprintf(usb_msg, "%.2f,%.2f,%.2f\r\n", Pitch, Roll, Yaw);

	CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));

	HAL_Delay(20);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
