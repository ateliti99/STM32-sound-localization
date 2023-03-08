/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SPI buffer size
#define SPI_BUFFER_SIZE 128

// FIR filter taps
#define NUM_TAPS 225

// Decimation factor
#define DECIMATION_FACTOR 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// flag to indicate DMA transfer is complete
uint8_t SPI2_DMA_RX_Complete = 0;

// SPI buffer
uint8_t SPI_Buffer[SPI_BUFFER_SIZE] = {0};

// left and right buffers for each mic
uint8_t left_buffer[SPI_BUFFER_SIZE*4] = {0};
uint8_t right_buffer[SPI_BUFFER_SIZE*4] = {0};

// previous buffers values for FIR filter
int8_t left_buffer_prev[NUM_TAPS] = {0};
int8_t right_buffer_prev[NUM_TAPS] = {0};

// left and right buffers for each mic after filtering
float left_buffer_filtered[SPI_BUFFER_SIZE*4/DECIMATION_FACTOR];
float right_buffer_filtered[SPI_BUFFER_SIZE*4/DECIMATION_FACTOR];

// finger snap flag
uint8_t finger_snap_flag = 0;

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 1000000 Hz

* 0 Hz - 8000 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 0.7299864365934536 dB

* 15625 Hz - 500000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.258113782444845 dB

* NUM_TAPS = 225

*/

// INT8 FIR filter coefficients
/*
static uint8_t fir_coeff[NUM_TAPS] ={
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  3,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  2,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  1,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  1
};
*/

// FLOAT FIR filter coefficients
static float fir_coeff[NUM_TAPS] = {
  0.005142840545276502,
  0.0005776119551103716,
  0.000603452635046104,
  0.0006258476939145514,
  0.0006434920183790002,
  0.0006568929308390448,
  0.0006652564505335804,
  0.0006678248280790667,
  0.00066467267376079,
  0.000655418362176927,
  0.0006394151288354629,
  0.00061641561843551,
  0.0005861893330172126,
  0.00054845108388333,
  0.0005030552758477393,
  0.0004497791706022509,
  0.00038837043045260365,
  0.0003187765940718276,
  0.00024099784089950783,
  0.00015498800550232237,
  0.000060778686176569876,
  -0.000041526854903904153,
  -0.00015177231816138268,
  -0.00026966421681820296,
  -0.0003948238278051311,
  -0.0005268935517652757,
  -0.0006654632374119981,
  -0.0008100272079632247,
  -0.0009600902246855826,
  -0.0011151062071835987,
  -0.0012743503438861145,
  -0.0014370203343255534,
  -0.0016022305922854731,
  -0.0017689590087946387,
  -0.001936352230518814,
  -0.002103685910167141,
  -0.0022699031624359507,
  -0.0024338297997767207,
  -0.0025943247232153126,
  -0.002749916347432099,
  -0.0028996072059222244,
  -0.0030430446211033493,
  -0.003178243182587049,
  -0.0033026617863639706,
  -0.0034178085358503208,
  -0.003520610576806379,
  -0.0036082671781069997,
  -0.003685179926599956,
  -0.003743320048133456,
  -0.0037841808549810047,
  -0.0038080482645113795,
  -0.0038127117750296428,
  -0.003796502219907655,
  -0.0037583913764318564,
  -0.003697341860647573,
  -0.003612766494578202,
  -0.003504156555145221,
  -0.003370606058748487,
  -0.003211262944679431,
  -0.003025498343914041,
  -0.002812738174919532,
  -0.0025726043879186287,
  -0.002304908017112659,
  -0.0020094887743440904,
  -0.0016863076105492604,
  -0.0013354739085991217,
  -0.0009571054035656962,
  -0.0005513740269179282,
  -0.00011862385688208173,
  0.00034066356291987456,
  0.0008258712118125614,
  0.0013361824206220298,
  0.0018706447174870902,
  0.0024281909856145533,
  0.003007509949736543,
  0.0036071382165185273,
  0.00422565035611871,
  0.00486154042060089,
  0.005513228114057391,
  0.006179229273900601,
  0.006857927133896979,
  0.007547593532307397,
  0.008246688404210124,
  0.008953181817299916,
  0.009664187961002695,
  0.010376918340606679,
  0.011088378301638464,
  0.011795523510462931,
  0.012499661516296631,
  0.013208885785526675,
  0.013892862180754598,
  0.014573780076006244,
  0.015240311165693975,
  0.01589026498111119,
  0.016522995668809933,
  0.01713563028038285,
  0.017725547130625265,
  0.018291357659988922,
  0.0188311394030313,
  0.01934264073399854,
  0.019824066263244372,
  0.02027369126297506,
  0.020689868419807533,
  0.021071224499483145,
  0.021416336732246948,
  0.02172378689933182,
  0.02199247250250785,
  0.022221490677728963,
  0.02241003470473071,
  0.022557465848915766,
  0.02266323405683669,
  0.022726863243144692,
  0.022748092902815847,
  0.022726863243144692,
  0.02266323405683669,
  0.022557465848915766,
  0.02241003470473071,
  0.022221490677728963,
  0.02199247250250785,
  0.02172378689933182,
  0.021416336732246948,
  0.021071224499483145,
  0.020689868419807533,
  0.02027369126297506,
  0.019824066263244372,
  0.01934264073399854,
  0.0188311394030313,
  0.018291357659988922,
  0.017725547130625265,
  0.01713563028038285,
  0.016522995668809933,
  0.01589026498111119,
  0.015240311165693975,
  0.014573780076006244,
  0.013892862180754598,
  0.013208885785526675,
  0.012499661516296631,
  0.011795523510462931,
  0.011088378301638464,
  0.010376918340606679,
  0.009664187961002695,
  0.008953181817299916,
  0.008246688404210124,
  0.007547593532307397,
  0.006857927133896979,
  0.006179229273900601,
  0.005513228114057391,
  0.00486154042060089,
  0.00422565035611871,
  0.0036071382165185273,
  0.003007509949736543,
  0.0024281909856145533,
  0.0018706447174870902,
  0.0013361824206220298,
  0.0008258712118125614,
  0.00034066356291987456,
  -0.00011862385688208173,
  -0.0005513740269179282,
  -0.0009571054035656962,
  -0.0013354739085991217,
  -0.0016863076105492604,
  -0.0020094887743440904,
  -0.002304908017112659,
  -0.0025726043879186287,
  -0.002812738174919532,
  -0.003025498343914041,
  -0.003211262944679431,
  -0.003370606058748487,
  -0.003504156555145221,
  -0.003612766494578202,
  -0.003697341860647573,
  -0.0037583913764318564,
  -0.003796502219907655,
  -0.0038127117750296428,
  -0.0038080482645113795,
  -0.0037841808549810047,
  -0.003743320048133456,
  -0.003685179926599956,
  -0.0036082671781069997,
  -0.003520610576806379,
  -0.0034178085358503208,
  -0.0033026617863639706,
  -0.003178243182587049,
  -0.0030430446211033493,
  -0.0028996072059222244,
  -0.002749916347432099,
  -0.0025943247232153126,
  -0.0024338297997767207,
  -0.0022699031624359507,
  -0.002103685910167141,
  -0.001936352230518814,
  -0.0017689590087946387,
  -0.0016022305922854731,
  -0.0014370203343255534,
  -0.0012743503438861145,
  -0.0011151062071835987,
  -0.0009600902246855826,
  -0.0008100272079632247,
  -0.0006654632374119981,
  -0.0005268935517652757,
  -0.0003948238278051311,
  -0.00026966421681820296,
  -0.00015177231816138268,
  -0.000041526854903904153,
  0.000060778686176569876,
  0.00015498800550232237,
  0.00024099784089950783,
  0.0003187765940718276,
  0.00038837043045260365,
  0.0004497791706022509,
  0.0005030552758477393,
  0.00054845108388333,
  0.0005861893330172126,
  0.00061641561843551,
  0.0006394151288354629,
  0.000655418362176927,
  0.00066467267376079,
  0.0006678248280790667,
  0.0006652564505335804,
  0.0006568929308390448,
  0.0006434920183790002,
  0.0006258476939145514,
  0.000603452635046104,
  0.0005776119551103716,
  0.005142840545276502
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void BuffersInit(uint8_t *left, uint8_t *right, int size);
void ConvertToBit(uint8_t *buffer, uint8_t *left, uint8_t *right, int size);
void LowPassFilterWithDecimation(uint8_t *left, uint8_t *right, float *left_filtered, float *right_filtered, int size, int decimation_factor);
void SoundLocalization(float *left, float *right, int size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Rx complete callback in DMA mode
  * @param  None
  * @retval None
  */
void SPI2_DMA_RX_Complete_Callback() {
  // set flag to indicate DMA transfer is complete
  SPI2_DMA_RX_Complete = 1;
}

/**
  * @brief  Initialize buffers
  * @param  left pointer to left buffer
  * @param  right pointer to right buffer
  * @param  size size of buffer
  * @retval None
  */
void BuffersInit(uint8_t *left, uint8_t *right, int size){
  // initialize buffers
  for(uint16_t i = 0; i < size; i++){
    // initialize raw microphones buffers
    left[i] = 0;
    right[i] = 0;
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  // reset the snap flag when the button is pressed
  if(GPIO_Pin == GPIO_PIN_13){
    finger_snap_flag = 0;
  }
}

/**
  * @brief  Convert SPI buffer to bit the original bit stream (PDM signal) and deinterleave data into left and right buffers
  * @param  buffer pointer to SPI buffer
  * @param  left pointer to left buffer
  * @param  right pointer to right buffer
  * @retval None
  */
void ConvertToBit(uint8_t *buffer, uint8_t *left, uint8_t *right, int size){
  uint8_t num, bin;
  uint16_t k;

  // 1. convert to binary
  // 2. deinterleave data into left mic and right mic buffers
  for(uint16_t i = 0; i < size; i++){
    // get number from SPI buffer
    num = buffer[i];

    // convert to binary and deinterleave data
    for(uint8_t j = 0; j < 8; j++){
      // convert to binary
      bin = num & 1;
      num = num >> 1;

      // position of binary data in buffer
      // first bit is at position 0
      k = i*4 + 3 - j/2;

      // deinterleave data and store in left and right buffers
      if(j % 2 == 0){
        left[k] = bin;
      } else {
        right[k] = bin;
      }
    }
  }
}

/**
  * @brief  Low pass filter with decimation, this function can be modified to work with
  *         integers coefficients instead of floats
  * @param  left pointer to left buffer
  * @param  right pointer to right buffer
  * @param  left_filtered pointer to left filtered buffer
  * @param  right_filtered pointer to right filtered buffer
  * @param  size size of left and right buffers
  * @param  decimation_factor decimation factor
  * @retval None
  */
void LowPassFilterWithDecimation(uint8_t *left, uint8_t *right, float *left_filtered, float *right_filtered, int size, int decimation_factor){
  //char str[100];
  //int16_t x1, x2;
  
  // initialize buffers
  for(uint16_t i = 0; i < size/decimation_factor; i++){
    // initialize raw microphones buffers
    left_filtered[i] = 0;
    right_filtered[i] = 0;
  }

  for(uint16_t i = 0; i < size; i ++){

    // shift buffer
    for(uint16_t j = NUM_TAPS - 1; j > 0; j--){
      left_buffer_prev[j] = left_buffer_prev[j - 1];
      right_buffer_prev[j] = right_buffer_prev[j - 1];
    }

    // move input to sample buffer
    left_buffer_prev[0] = left[i];
    right_buffer_prev[0] = right[i];
    
    // calculate FIR filter only for decimated values
    if(i % decimation_factor == 0){
      for(uint16_t j = 0; j < NUM_TAPS; j++){
        left_filtered[i/decimation_factor] += left_buffer_prev[j] * fir_coeff[j];
        right_filtered[i/decimation_factor] += right_buffer_prev[j] * fir_coeff[j];
      }
      
      
      // scale to -127 to 128
      left_filtered[i/decimation_factor] = left_filtered[i/decimation_factor] * 255.f - 128.f;
      right_filtered[i/decimation_factor] = right_filtered[i/decimation_factor] * 255.f - 128.f;

      if(left_filtered[i/decimation_factor] > 127.f){
        left_filtered[i/decimation_factor] = 127.f;
      } else if(left_filtered[i/decimation_factor] < -128.f){
        left_filtered[i/decimation_factor] = -128.f;
      }

      if(right_filtered[i/decimation_factor] > 127.f){
        right_filtered[i/decimation_factor] = 127.f;
      } else if(right_filtered[i/decimation_factor] < -128.f){
        right_filtered[i/decimation_factor] = -128.f;
      }
      
      //x1 = round(left_filtered[i/decimation_factor]);
      //x2 = round(right_filtered[i/decimation_factor]);
      
      //check if finger snap is detected with a threshold
      if(right_buffer_filtered[i/decimation_factor] > 7 && left_buffer_filtered[i/decimation_factor] > 7 && finger_snap_flag == 0){
        HAL_UART_Transmit(&huart2, (uint8_t *)"snap\n\r", 6, 10);
        finger_snap_flag = 1;
      }

      // send data to PC and print them with serialoscilloscope
      //sprintf(str, "%d,%d\r\n", x1, x2);
      //HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);

    }
  }
}

/**
  * @brief  Sound localization algorithm using cross correlation
  * @param  left pointer to left buffer
  * @param  right pointer to right buffer
  * @param  size size of left and right buffers
  * @retval None
  */
void SoundLocalization(float *left, float *right, int size){
  //float max_left = 0, max_right = 0;
  float result[2*size];
  float max = 0;
  uint8_t idx = 0;
  uint16_t idx_left = 0, idx_right = 0;
  uint8_t flag_left = 0, flag_right = 0;
  uint16_t slope_left, slope_right;


  if(finger_snap_flag == 1){
    
    // check if the current value has a magnitude higher than the previous one
    for(uint16_t i = 0; i < size; i++){
      // check left buffer for max value and index of max value
      if(left[i] - left[i - 1] > 5 && flag_left == 0){
        idx_left = i - 1;
        flag_left = 1;
      }
      // check right buffer for max value and index of max value
      if(right[i] - right[i - 1] > 5 && flag_right == 0){
        idx_right = i - 1;
        flag_right = 1;
      }
    }

    // check which index is lower and send the corresponding message
    if(idx_left < idx_right){
      HAL_UART_Transmit(&huart2, (uint8_t *)"left\r\n", 6, 10);
    } else if(idx_left > idx_right){
      HAL_UART_Transmit(&huart2, (uint8_t *)"right\r\n", 7, 10);
    } 
    

    /*
    //Cross Correlation Algorithm
    // initialize result buffer
    for(uint16_t i = 0; i < 2*size - 1; i++){
      result[i] = 0;
    }

    // perform cross correlation
    for(uint16_t i = 0; i < size; i++){
      for(uint16_t j = 0; j < size; j++){
        result[i + j] += left[i] * right[j];
      }
    }

    // find max value
    for(uint16_t i = 0; i < 2*size - 1; i++){
      if(result[i] > max){
        max = result[i];
        idx = i;
      }
    }

    // find phase shift
    if(idx < size){
      HAL_UART_Transmit(&huart2, (uint8_t *)"left\r\n", 6, 10);
    } else if(idx > size){
      HAL_UART_Transmit(&huart2, (uint8_t *)"right\r\n", 7, 10);
    }
    */

    finger_snap_flag = -1;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // start MIC_CLOCKx2 (PB4)
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  // start MIC_CLOCK_NUCLEO (PB5)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);


  HAL_Delay(500);
  // start SPI in DMA mode 
  HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)SPI_Buffer, SPI_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    
    // check if DMA receive is complete
    if(SPI2_DMA_RX_Complete){
      // reset flag
      SPI2_DMA_RX_Complete = 0;

      // convert data from SPI buffer to binary stream 
      // deinterleave data into left and right buffers
      ConvertToBit(SPI_Buffer, left_buffer, right_buffer, SPI_BUFFER_SIZE);

      // FIR filter with decimation
      LowPassFilterWithDecimation(left_buffer, right_buffer, left_buffer_filtered, right_buffer_filtered, SPI_BUFFER_SIZE*4, DECIMATION_FACTOR);

      // sound localization
      //SoundLocalization(left_buffer_filtered, right_buffer_filtered, SPI_BUFFER_SIZE*4/DECIMATION_FACTOR);

      // re-initialize buffers
      BuffersInit(left_buffer, right_buffer, SPI_BUFFER_SIZE*4);

      // start new SPI transfer (no need in circular mode)
      // only used for debugging
      //HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)SPI_Buffer, SPI_BUFFER_SIZE);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 42-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 21;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 42-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
