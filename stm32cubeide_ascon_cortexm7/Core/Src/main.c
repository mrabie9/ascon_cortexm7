/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
//#include "stdio.h"
#include "aead.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned long long bit64;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 320-bit state size
//bit64 state[5] = {0}, t[5] = {0};
//bit64 constants[12] = {
//		0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5,
//		0x96, 0x87, 0x78, 0x69, 0x5a, 0x4b
//};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Required for printf
//int _write(int file, char *ptr, int len){
//	int i = 0;
//	for (i=0;i<len;i++){
//		ITM_SendChar((*ptr++));
//	}
//	return len;
//}
//
//bit64 rotate(bit64 x, int l){
//	bit64 temp;
//	temp = (x>>l) ^ (x<<(64-l));
//	return temp;
//}
//
//void linear(bit64 state[5]){
//	bit64 temp0, temp1;
//
//	temp0 = rotate(state[0], 19);
//	temp1 = rotate(state[0], 28);
//	state[0] ^= temp0 ^ temp1;
//
//	temp0 = rotate(state[0], 61);
//	temp1 = rotate(state[0], 39);
//	state[1] ^= temp0 ^ temp1;
//
//	temp0 = rotate(state[0], 1);
//	temp1 = rotate(state[0], 6);
//	state[2] ^= temp0 ^ temp1;
//
//	temp0 = rotate(state[0], 10);
//	temp1 = rotate(state[0], 17);
//	state[3] ^= temp0 ^ temp1;
//
//	temp0 = rotate(state[0], 7);
//	temp1 = rotate(state[0], 41);
//	state[4] ^= temp0 ^ temp1;
//}
//
//void sbox(bit64 x[5]){
//	// bit slice implementation
//	x[0] ^= x[4]; x[4] ^= x[3]; x[2] ^= x[1];
//	t[0] = x[0]; t[1] = x[1]; t[2] = x[2]; t[3] = x[3]; t[4] = x[4];
//	t[0] =~ t[0]; t[1] =~ t[1]; t[2] =~ t[2]; t[3] =~ t[3]; t[4] =~ t[4];
//	t[0] &= x[1]; t[1] &= x[2]; t[2] &= x[3]; t[3] &= x[4]; t[4] &= x[0];
//	x[0] ^= t[1]; x[1] ^= t[2]; x[2] ^= t[3]; x[3] ^= t[4]; x[4] ^= t[0];
//	x[1] ^= x[0]; x[0] ^= x[4]; x[3] ^= x[2]; x[2] =~ x[2];
//}
//
//void add_constant(bit64 state[5], int i, int a){
//	state[2] ^= constants[12 - a - i];
//}
//
//void p(bit64 state[5], int a){
//	for(int i=0; i<a; i++){
//		add_constant(state, i, a);
//		sbox(state);
//		linear(state);
//	}
//}
//
//void initialisation(bit64 state[5], bit64 key[2]){
//	p(state, 12);
//	state[3] ^= key[0];
//	state[4] ^= key[1];
//}
//
//void print_states(bit64 state[5]){
//	for(int i = 0; i<5; i++) printf("0x%llx\n", state[i]);
//}
//
//void encrypt(bit64 state[5], int length, bit64 plaintext[], bit64 ciphertext[]){
//	for(int i =0; i<length; i++){
//		p(state, 6);
//		ciphertext[i] = plaintext[i] ^ state[0];
//		state[0] = ciphertext[i];
//	}
//}
//
//void finalisation(bit64 state[5], bit64 key[2], bit64 tag[2]){
//	state[0] ^= key[0];
//	state[1] ^= key[1];
//	p(state, 12);
//	tag[0] = state[3] ^ key[0];
//	tag[1] = state[4] ^ key[1];
//}

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /*
  bit64 nonce[2] = {0};
  bit64 key[2] = {0};
  bit64 IV = 0x80400c0600000000;
  bit64 plaintext[] = {0x123456789abcdef, 0x34543523213}, ciphertext[10] = {0};
  bit64 tag[2] = {0};
  state[0] = IV;
  state[1] = key[0];
  state[2] = key[1];
  state[3] = nonce[0];
  state[4] = nonce[1];
  initialisation(state, key);
  print_states(state);
  encrypt(state, sizeof(plaintext)/sizeof(bit64), plaintext, ciphertext);
  finalisation(state, key, tag);
  */

  // Declare and initialise ASCON variables
  unsigned char nonce[16] = {0xf0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
  unsigned char key[16] =  {0xf0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
  unsigned char msg[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
  unsigned char dt[8] = {0};
  uint64_t msglen = sizeof(msg)/sizeof(unsigned char);
  unsigned long long ctlen;
  unsigned char ct[50] = {0};

  // Declare pointers
  unsigned char *c;
  unsigned long long *clen;
  uint64_t *mlen;
  unsigned char * dm;
  unsigned char *k, *npub;
  unsigned char *m;

  // Initialise pointers
  k = key;
  npub = nonce;
  mlen = &msglen;
  clen = &ctlen;
  m = msg;
  dm = dt;
  c = ct;

  // ASCON functions
  crypto_aead_encrypt(c, clen, m, msglen, NULL, 0, NULL, npub, k);
  crypto_aead_decrypt(dm, mlen, NULL, c, ctlen, NULL, 0, npub, k);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
