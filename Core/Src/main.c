/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sound Localization for STM32F446RE
  * I2S2 (Master) on PB10/PB12/PC1
  * I2S3 (Slave)  on PC10/PA4/PC12
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h> // For abs()
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
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */
#define BUFFER_SIZE 256       // Process 256 samples per batch
#define THRESHOLD   1500000   // Noise Gate (Lower this if mics are quiet)
#define MIC_DIST    0.10f     // 10 cm spacing between mics
#define SPEED_SOUND 343.0f    // m/s
#define SAMPLE_RATE 44100.0f

// --- Buffers ---
// Raw DMA data (Interleaved L/R)
volatile uint32_t i2s2_buff[BUFFER_SIZE]; // Carries Mic 1 (L) and Mic 2 (R)
volatile uint32_t i2s3_buff[BUFFER_SIZE]; // Carries Mic 3 (L)

// Processed channels (separated)
int32_t mic1[BUFFER_SIZE/2]; // Left Channel of I2S2
int32_t mic2[BUFFER_SIZE/2]; // Right Channel of I2S2
int32_t mic3[BUFFER_SIZE/2]; // Left Channel of I2S3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
int calc_lag(int32_t *sig1, int32_t *sig2, int len);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  // 2. Start Recording
  // IMPORTANT: Start the SLAVE (I2S3) first, then the MASTER (I2S2).
  // If you start Master first, Slave might miss the first few clock cycles and desync.
  if(HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)i2s3_buff, BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
    }
  if(HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)i2s2_buff, BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  HAL_Delay(50);

  // --- Step 1: Data Separation ---
  uint64_t total_energy = 0;

  for(int i=0; i < BUFFER_SIZE/2; i++) {
    mic1[i] = (int32_t)i2s2_buff[i*2];
    mic2[i] = (int32_t)i2s2_buff[i*2+1];
    mic3[i] = (int32_t)i2s3_buff[i*2];
    total_energy += (uint64_t)abs(mic1[i]);
  }

  // --- Step 2: Noise Gate ---
  if(total_energy > THRESHOLD) {

    // --- Step 3: TDOA Calculation ---
    int lag12 = calc_lag(mic1, mic2, BUFFER_SIZE/2);
    int lag13 = calc_lag(mic1, mic3, BUFFER_SIZE/2);

    float time_delay12 = (float)lag12 / SAMPLE_RATE;
    float time_delay13 = (float)lag13 / SAMPLE_RATE;

    float d12 = time_delay12 * SPEED_SOUND;
    float d13 = time_delay13 * SPEED_SOUND;

    // --- Step 4: Analytical Hyperbolic Intersection ---
    float D = MIC_DIST;
    float sqrt3 = 1.73205f;
    float y1 = D / sqrt3; // Mic 1 Y-coordinate

    // Calculate the linear coefficients for X = alpha*r1 + beta
    float alpha = (d12 - d13) / D;
    float beta  = (d13*d13 - d12*d12) / (2.0f * D);

    // Calculate the linear coefficients for Y = gamma*r1 + delta
    float gamma = (d12 + d13) / (sqrt3 * D);
    float delta = -(d13*d13 + d12*d12) / (2.0f * sqrt3 * D);

    // Formulate the quadratic equation parameters (A*r1^2 + B*r1 + C = 0)
    float A = (alpha * alpha) + (gamma * gamma) - 1.0f;
    float B = 2.0f * (alpha * beta + gamma * (delta - y1));
    float C = (beta * beta) + (delta - y1)*(delta - y1);

    // Solve the quadratic equation for r1
    float discriminant = (B * B) - (4.0f * A * C);

    float angle_deg = 0.0f;
    float source_distance = -1.0f;

    // If discriminant < 0, the delays are physically impossible (usually hardware noise)
    if (discriminant >= 0.0f) {
      float r1 = -1.0f;

      // Handle the Far-Field limit (where A approaches 0)
      if (fabsf(A) < 0.0001f) {
        if (fabsf(B) > 0.0001f) {
          r1 = -C / B; // Linear fallback
        }
      } else {
        // Near-Field: Standard quadratic formula. Take the maximum positive root.
        float root1 = (-B + sqrtf(discriminant)) / (2.0f * A);
        float root2 = (-B - sqrtf(discriminant)) / (2.0f * A);
        r1 = (root1 > root2) ? root1 : root2;
      }

      // --- Step 5: Final Coordinate Mapping ---
      if (r1 >= 0.0f) {
        // Plug r1 back into our algebraic reductions
        float final_X = alpha * r1 + beta;
        float final_Y = gamma * r1 + delta;

        // Calculate absolute distance from the array center
        source_distance = sqrtf(final_X * final_X + final_Y * final_Y);

        // Calculate 360-degree angle
        float angle_rad = atan2f(final_Y, final_X);
        angle_deg = angle_rad * (180.0f / 3.14159f);

        // Normalize to 0-360 range
        if (angle_deg < 0.0f) angle_deg += 360.0f;
      }
    }
    // --- Output ---
    // printf("Angle: %.1f | Distance: %.3f m | X: %.3f, Y: %.3f\n", angle_deg, source_distance, final_X, final_Y);
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 271;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = 44100;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = 44100;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Simple Cross-Correlation to find time offset
int calc_lag(int32_t *sig1, int32_t *sig2, int len) {
  int64_t max_corr = 0;
  int best_lag = 0;

  // Search range: +/- 20 samples covers ~15cm at 44.1kHz
  int max_shift = 20;

  for (int lag = -max_shift; lag <= max_shift; lag++) {
    int64_t sum = 0;
    // Convolve the signals
    for (int i = max_shift; i < len - max_shift; i++) {
      sum += ((int64_t)sig1[i] >> 8) * ((int64_t)sig2[i + lag] >> 8);
      // Note: Bitshift >> 8 prevents int64 overflow if signals are very loud
    }

    if (sum > max_corr) {
      max_corr = sum;
      best_lag = lag;
    }
  }
  return best_lag;
}
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
