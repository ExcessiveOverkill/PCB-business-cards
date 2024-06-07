/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ledDrivers.h"
#include "imageData.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POWER_OFF_DELAY 30	// how many seconds of no movement before powering down
#define LED_ENABLE_ACCELERATION 6000

// use measured x accel values as "random" input
// max is 0x7FFE
// min is 1
// a point is counted each time the actual measured value falls between high and low range (inclusive), once enough points are accumulated, the random event is triggered
#define RANDOM_HIGH_RANGE 0x3FFF + RANDOM_CHANCE  	// mid-scale
#define RANDOM_LOW_RANGE 0x3FFF - RANDOM_CHANCE		// the wider this range, the more likely it will match
#define RANDOM_POINTS_REQUIRED RANDOM_POINTS
#define SECRET_IMAGE_DISPLAY_CYCLES SECRET_DISPLAY_CYCLES	// number of cycles to display the secret image once the random threshold is reached

//#define ANGLE_STABILIZATION_MODE 1	// enable stabilization mode, this displays animation frames based on the tilt angle of the card instead of playing the animation. frame display cycles should be set to 1 when using this

#define PI 3.14159265359


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// write addresses
const uint8_t ACCEL_MODE = 0x07;
const uint8_t ACCEL_SAMPLE_RATE = 0x08;
const uint8_t ACCEL_RANGE = 0x20;
const uint8_t ACCEL_INTERRUPT_ENABLE = 0x06;
const uint8_t ACCEL_GPIO_CONTROL = 0x33;
const uint8_t ACCEL_COMMUNICATION_CONTROL = 0x31;
const uint8_t ACCEL_INTERRUPT_STATUS = 0x14;

// read addresses
const uint8_t ACCEL_STATUS = 0x05 + 128;
const uint8_t ACCEL_READ_COUNT = 0x4b + 128;
const uint8_t ACCEL_X_DATA = 0x0d + 128;
const uint8_t ACCEL_RUN_STATUS = 0x13 + 128;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
	FLASH_OBProgramInitTypeDef OB;

	HAL_FLASHEx_OBGetConfig(&OB);
	if((OB.USERConfig & OB_BOR_ENABLE)){
	   HAL_FLASH_Unlock();
	   HAL_FLASH_OB_Unlock();

	   OB.OptionType = OPTIONBYTE_USER;
	   OB.USERType = OB_USER_BOR_EN;
	   OB.USERConfig = OB_BOR_DISABLE;

	   if ( HAL_FLASHEx_OBProgram(&OB) != HAL_OK )
		  {
			  HAL_FLASH_OB_Lock();
			  HAL_FLASH_Lock();
			  return HAL_ERROR;
		  }

		  HAL_FLASH_OB_Launch();

		  /* We should not make it past the Launch, so lock
		   * flash memory and return an error from function
		   */
		  HAL_FLASH_OB_Lock();
		  HAL_FLASH_Lock();
		  return HAL_ERROR;
	}

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  TIM1->CR1 |= 1;

  // lock on power
  HAL_GPIO_WritePin(KEEP_POWER_ON_GPIO_Port, KEEP_POWER_ON_Pin, GPIO_PIN_SET);
  //HAL_Delay(100);
  disableLEDs();
  setLEDs(0x80000001);
  enableLEDs();
  HAL_Delay(100);
  setLEDs(0);

  uint32_t tick_count = 0;
  uint32_t last_active_tick_count = 0;

  const uint32_t imageData[IMAGE_LENGTH] = IMAGE_DATA;
  const uint8_t normal_frame_cycles[IMAGE_LENGTH/FRAME_LENGTH] = FRAME_DISPLAY_CYCLES;

  #ifdef SECRET_IMAGE_DATA
  	  const uint32_t secretImageData[SECRET_IMAGE_LENGTH] = SECRET_IMAGE_DATA;
  	  const uint8_t secret_frame_cycles[SECRET_IMAGE_LENGTH/SECRET_FRAME_LENGTH] = SECRET_FRAME_DISPLAY_CYCLES;
  #endif


  char spi_buf[10];
  char spi_read_buf[10];

  #ifdef ANGLE_STABILIZATION_MODE
  	  int16_t y_accel_raw = 0;
  	  int32_t angle_step = 0;
  	  int32_t x_accel_zero_average = 0;
  	  int32_t y_accel_zero_average = 0;
  #endif

  int16_t x_accel_raw = 0;


  int32_t low_cut_integrator_rate = 10; // max accel magnitude to reduce toward zero each cycle. this is used to remove affects of gravity
  int32_t x_low_cut_integrator = 0;

  int32_t x_accel = 0;
  int32_t x_accel_interpolated = 0;
  int32_t x_accel_previous = 0;
  uint32_t accel_last_update_time = 0;	// us
  uint32_t accel_update_period = 1000; 	// us (1000Hz)
  int32_t max_dynamic_accel = 0x7FFF; // acceleration needed to reach ends of image, this should adjust based on the measured max acceleration


  int32_t x_accel_magnitude_average = 0;

  int32_t image_line = 0;

  int32_t max_cycle_accel = 0;
  int32_t min_cycle_accel = 0;
  uint32_t max_cycle_accel_time = 0;
  uint32_t min_cycle_accel_time = 0;

  uint32_t estimated_next_edge_time = 0;	// when we think we will be at the end of the shake motion again
  uint32_t frame_trigger_time = 0xFFFFFFFF;

  uint32_t random_points = 0;
  int32_t image_length = IMAGE_LENGTH;
  int32_t frame_length = FRAME_LENGTH;
  
  uint32_t current_frame_cycle = 0;
  uint32_t frame_display_cycles = normal_frame_cycles[0];

  uint32_t current_frame = 0;

  uint32_t secret_display_cycles = 0;

  uint32_t accel_read_attemps = 0;

  // send nothing to get SPI clk level set correctly
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 1, 1);

  // configure accelerometer

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_MODE;
  spi_buf[1] = 0b00000000;	// set to standby mode
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  HAL_Delay(1);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_SAMPLE_RATE;
  spi_buf[1] = 0x0E;	// set sample rate (1000 Hz)
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_INTERRUPT_ENABLE;
  spi_buf[1] = 0x80;	// enable sample (ACQ) interrupt
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_GPIO_CONTROL;
  spi_buf[1] = 0xC0;	// set INTN2 pin as push/pull active high
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_COMMUNICATION_CONTROL;
  spi_buf[1] = 0x10;	// swap INTN output pins
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_RANGE;
  spi_buf[1] = 0b00100000;	// set +- 8G's
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

  ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
  spi_buf[0] = ACCEL_MODE;
  spi_buf[1] = 0b00000001;	// set to wake mode
  HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
  ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);


  tick_count = (tick_count & 0xFFFF0000) + TIM1->CNT;
  	if(TIM1->SR & 0b1){
  		tick_count += 0x10000;
  		TIM1->SR &= ~(1);
  	}

  accel_last_update_time = tick_count;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	tick_count = (tick_count & 0xFFFF0000) + TIM1->CNT;
	if(TIM1->SR & 0b1){
		tick_count += 0x10000;
		TIM1->SR &= ~(1);
	}

	accel_read_attemps = 5;
	while(ACCEL_INTERRUPT_GPIO_Port->IDR & (1 << 6) && accel_read_attemps > 0){

		tick_count = (tick_count & 0xFFFF0000) + TIM1->CNT;
		if(TIM1->SR & 0b1){
			tick_count += 0x10000;
			TIM1->SR &= ~(1);
		}

		accel_read_attemps--;
		// read XYZ accel values
		ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
		spi_buf[0] = (uint8_t)ACCEL_X_DATA;
		spi_buf[1] = 0;
		spi_buf[2] = 0;
		spi_buf[4] = 0;
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)spi_buf, (uint8_t *)spi_read_buf, 8, 1);
		ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

		// clear interrupt
		ACCEL_SELECT_GPIO_Port->ODR &= ~(1 << 4);
		spi_buf[0] = ACCEL_INTERRUPT_STATUS;
		spi_buf[1] = 0;
		HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 2, 1);
		ACCEL_SELECT_GPIO_Port->ODR |= (1 << 4);

		accel_update_period = tick_count - accel_last_update_time;
		accel_last_update_time = tick_count;

		x_accel_raw = spi_read_buf[2] | spi_read_buf[3] << 8;
        #ifdef ANGLE_STABILIZATION_MODE
			y_accel_raw = spi_read_buf[4] | spi_read_buf[5] << 8;
		#endif
		//z_accel_raw = spi_read_buf[6] | spi_read_buf[7] << 8;


		if(x_accel_magnitude_average > LED_ENABLE_ACCELERATION){
			if(x_accel_previous < 0 && x_accel >= 0){	// transitioned from negative to positive acceleration
				max_cycle_accel = 0;	// clear previous max recorded accel
				estimated_next_edge_time = (min_cycle_accel_time-max_cycle_accel_time) + min_cycle_accel_time;

				#ifdef ANGLE_STABILIZATION_MODE
					// save angle when there should be negligible affect from shaking
					x_accel_zero_average -= (x_accel_zero_average - x_accel_raw) / 5;
					y_accel_zero_average -= (y_accel_zero_average - y_accel_raw) / 5;
					angle_step = atan2(x_accel_zero_average, y_accel_zero_average) * ((IMAGE_LENGTH/FRAME_LENGTH) / (2*PI)) + ((IMAGE_LENGTH/FRAME_LENGTH) / 2);
				#endif
			}

			if(x_accel_previous > 0 && x_accel <= 0){	// transitioned from positive to negative acceleration
				min_cycle_accel = 0;	// clear previous min recorded accel
				estimated_next_edge_time = (max_cycle_accel_time-min_cycle_accel_time) + max_cycle_accel_time;

				#ifdef ANGLE_STABILIZATION_MODE
					// save angle when there should be negligible affect from shaking
					x_accel_zero_average -= (x_accel_zero_average - x_accel_raw) / 5;
					y_accel_zero_average -= (y_accel_zero_average - y_accel_raw) / 5;
					angle_step = atan2(x_accel_zero_average, y_accel_zero_average) * ((IMAGE_LENGTH/FRAME_LENGTH) / (2*PI)) + ((IMAGE_LENGTH/FRAME_LENGTH) / 2);
				#endif
			}
		}

		if(estimated_next_edge_time > tick_count + 500000) estimated_next_edge_time = tick_count;

		x_accel_previous = x_accel;


		// integrate towards zero to cancel constant acceleration (gravity)
		x_accel = x_accel_raw + x_low_cut_integrator;

		// random event generation
		if((x_accel >= RANDOM_LOW_RANGE && x_accel <= RANDOM_HIGH_RANGE) && secret_display_cycles==0) random_points++;


		// run integrator
		if(x_accel > low_cut_integrator_rate){
			x_low_cut_integrator -= low_cut_integrator_rate;
		}
		else if(x_accel < -low_cut_integrator_rate){
			x_low_cut_integrator += low_cut_integrator_rate;
		}
		else{
			x_low_cut_integrator -= x_accel;
		}

		// estimate average magnitude (used to detect motion)
		x_accel_magnitude_average += (abs(x_accel) - x_accel_magnitude_average) / 200;


		if(x_accel>max_cycle_accel){
			max_cycle_accel = x_accel;
			max_cycle_accel_time = tick_count;
		}

		if(x_accel<min_cycle_accel){
			min_cycle_accel = x_accel;
			min_cycle_accel_time = tick_count;
		}

	}



	// interpolate acceleration steps to get a smooth change
	x_accel_interpolated = x_accel_previous * (int32_t)accel_update_period + (x_accel / (int32_t)(tick_count-accel_last_update_time));
	x_accel_interpolated /= (int32_t)accel_update_period;
	
	
	// increment display frame cycle counter
	if(tick_count >= frame_trigger_time && current_frame_cycle == 0){	// reset counter and increment to next frame index

		current_frame++;
		frame_display_cycles = normal_frame_cycles[current_frame];
		current_frame_cycle = frame_display_cycles;

		if(current_frame * frame_length > image_length-frame_length){
			current_frame = 0;		// loop back to first frame once end of image is reached
			frame_display_cycles = normal_frame_cycles[current_frame];
			current_frame_cycle = frame_display_cycles;

			if(secret_display_cycles != 0){
				secret_display_cycles--;	// decrement secret display cycles since we went through all of the frames

				if(secret_display_cycles == 0){		// switch back to normal image when secret cycles are up
					image_length = IMAGE_LENGTH;
					frame_length = FRAME_LENGTH;
					current_frame = 0;	// pick up where we left off
					frame_display_cycles = normal_frame_cycles[current_frame];
					current_frame_cycle = frame_display_cycles;
				}
			}
		}
		
		#ifdef SECRET_IMAGE_DATA
			if(random_points >= RANDOM_POINTS_REQUIRED && current_frame == 0){	// switch to secret image, random points are not generated while secret_display_cycles != 0
				random_points = 0;
				image_length = SECRET_IMAGE_LENGTH;
				frame_length = SECRET_FRAME_LENGTH;
				secret_display_cycles = SECRET_IMAGE_DISPLAY_CYCLES;
				current_frame = 0;	// reset frame counter so we start at first frame of animation
				frame_display_cycles = secret_frame_cycles[current_frame];
				current_frame_cycle = frame_display_cycles;
			}
		#endif

		#ifdef ANGLE_STABILIZATION_MODE
			// select frame based on tilt angle
			if(secret_display_cycles != 0){
				secret_display_cycles--;	// decrement secret display cycles

				if(secret_display_cycles == 0){		// switch back to normal image when secret cycles are up
					image_length = IMAGE_LENGTH;
					frame_length = FRAME_LENGTH;
					frame_display_cycles = FRAME_DISPLAY_CYCLES;
				}
			}

			current_frame = angle_step + (IMAGE_LENGTH/FRAME_LENGTH) - 1;
			current_frame %= (IMAGE_LENGTH/FRAME_LENGTH);
		#endif
  	    
	}

    if(tick_count >= frame_trigger_time && current_frame_cycle != 0 && frame_trigger_time < estimated_next_edge_time){	// decrement counter
    	frame_trigger_time = estimated_next_edge_time;
    	current_frame_cycle--;
    }

    if(current_frame_cycle==0) frame_trigger_time = estimated_next_edge_time;



  	// find line of image to display based on current acceleration and image length
	image_line =  ((x_accel_interpolated + max_dynamic_accel/2) / (max_dynamic_accel / frame_length)) + current_frame * frame_length;

	// display image if image_line is valid and motion is detected
	if(image_line >= current_frame * frame_length && image_line < (current_frame+1) * frame_length && x_accel_magnitude_average > LED_ENABLE_ACCELERATION){
		#ifdef SECRET_IMAGE_DATA
			if(secret_display_cycles > 0){
				setLEDs(secretImageData[image_line]);
		  	}
		  	else{
		  		setLEDs(imageData[image_line]);
		  	}
		#else
			setLEDs(imageData[image_line]);
		#endif

		last_active_tick_count = tick_count;
	}
	else{
		setLEDs(0);
	}

	if(x_accel_magnitude_average < LED_ENABLE_ACCELERATION){
		// reset animation variables when card is no longer being shaken
		image_length = IMAGE_LENGTH;
		frame_length = FRAME_LENGTH;
		frame_display_cycles = normal_frame_cycles[0];
		current_frame = 0;	// reset frame counter so we start at first frame of animation
	}


	if(tick_count - last_active_tick_count >= 1000000 * POWER_OFF_DELAY){
		// shut down
		setLEDs(0xFFFFFFFF);
		HAL_GPIO_WritePin(KEEP_POWER_ON_GPIO_Port, KEEP_POWER_ON_Pin, GPIO_PIN_RESET);
		NVIC_SystemReset();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACCEL_SELECT_GPIO_Port, ACCEL_SELECT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLANK_GPIO_Port, LED_BLANK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEEP_POWER_ON_Pin|LED_DATA_LATCH_Pin|LED_DATA_CLOCK_Pin|LED_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ACCEL_SELECT_Pin */
  GPIO_InitStruct.Pin = ACCEL_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ACCEL_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLANK_Pin */
  GPIO_InitStruct.Pin = LED_BLANK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLANK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEEP_POWER_ON_Pin */
  GPIO_InitStruct.Pin = KEEP_POWER_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEEP_POWER_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DATA_LATCH_Pin LED_DATA_CLOCK_Pin LED_DATA_Pin */
  GPIO_InitStruct.Pin = LED_DATA_LATCH_Pin|LED_DATA_CLOCK_Pin|LED_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = ACCEL_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACCEL_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB7);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB8);

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
