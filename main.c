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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NOTE_DURATION 250


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char data;
uint32_t IC_Val1 = 0; // Input Capture Value (Rising)
uint32_t IC_Val2 = 0; // Input Capture Value (Falling)
uint32_t Difference = 0; // Duration
int p=0;
int dutycycle=0;
int rf_duty=50;
int rb_duty=50;
int lf_duty=50;
int lb_duty=50;
int speed;
int buzzer_state=0;
int music_mode=0;
char text[16]="                ";
uint8_t Is_First_Captured = 0; // Detect Flag
uint16_t Distance = 0; // Distance after conversion
void HCSR05_Read(void); // Declare Trigger subroutine
void delay (uint16_t time) // Define microsecond Function
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}
void SetSpeed(int r1,int r2, int l1,int l2){
	rf_duty=100-r1;

	lf_duty=100-r2;
	rb_duty=l1;
	lb_duty=l2;


}
void Forward_Speed(int speed){
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,0);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,1);


    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,1);

    //SetSpeed(100-speed,100-speed,speed,speed);
    SetSpeed(speed,speed,speed,speed);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,rf_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,lf_duty);
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,rb_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,lb_duty);

}
void Backward_Speed(int speed){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,0);//
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);//
    


    SetSpeed(speed,speed,speed,speed);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100-rf_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,100-lf_duty);
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100-rb_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100-lb_duty);

}
void Left_Speed(int speed){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,1);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,1);//
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);//
    

    SetSpeed(speed,speed,speed,speed);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,rf_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,100-lf_duty);
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,rb_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100-lb_duty);

}
void Right_Speed(int speed){

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,0);//
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,1);//
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,0);
    
    SetSpeed(speed,speed,speed,speed);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100-rf_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,lf_duty);
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100-rb_duty);//right forward
       __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,lb_duty);

}
void Stop_Speed(int speed,int wait_1){
    HAL_Delay(wait_1);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,0);//
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);//
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,0);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,0);

SetSpeed(speed,speed,speed,speed);
__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,rf_duty);//right forward
   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,lf_duty);
   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100-rb_duty);//right forward
   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100-lb_duty);

}



void Buzz(){

	for(int i=0;i<5;i++){
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,10);
	  HAL_Delay(100);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  HAL_Delay(100);

	}




}
//
#define NOTE_C4  261.63
#define NOTE_CS4 277.18
#define NOTE_D4  293.66
#define NOTE_DS4 311.13
#define NOTE_E4  329.63
#define NOTE_F4  349.23
#define NOTE_FS4 369.99
#define NOTE_G4  392.00
#define NOTE_GS4 415.30
#define NOTE_A4  440.00
#define NOTE_AS4 466.16
#define NOTE_B4  493.88

#define NOTE_C5  523.25
#define NOTE_CS5 554.37
#define NOTE_D5  587.33
#define NOTE_DS5 622.25
#define NOTE_E5  659.25
#define NOTE_F5  698.46
#define NOTE_FS5 739.99
#define NOTE_G5  783.99
#define NOTE_GS5 830.61
#define NOTE_A5  880.00
#define NOTE_AS5 932.33
#define NOTE_B5  987.77

#define NOTE_REST 0 //


void PlayTone(float frequency, uint16_t duration) {
    if (frequency == NOTE_REST) {
      
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    } else {
        uint32_t clock_freq = 72000000; //  72 MHz
        uint32_t prescaler = 719; //   100 kHz
        uint32_t timer_freq = clock_freq / (prescaler + 1); //  Timer freq
        uint32_t arr = (uint32_t)((timer_freq / frequency) - 1); //  ARR

        if (arr > 2000) arr = 2000; //  ARR 

        __HAL_TIM_SET_PRESCALER(&htim3, prescaler); //  PSC
        __HAL_TIM_SET_AUTORELOAD(&htim3, arr); //  ARR

        // 设置占空比为 50%
        uint32_t pulse = (arr + 1) / 2;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    }

    HAL_Delay(duration); // 
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // 
}


void PlayTwinkleTwinkle() {
	static int i=0;
  
    float melody[] = {
        NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
        NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
        NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
        NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
        NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
        NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
    };

    uint16_t durations[] = {
        500, 500, 500, 500, 500, 500, 1000, // 
        500, 500, 500, 500, 500, 500, 1000, // 
        500, 500, 500, 500, 500, 500, 1000, // 
        500, 500, 500, 500, 500, 500, 1000, // 
        500, 500, 500, 500, 500, 500, 1000, // 
        500, 500, 500, 500, 500, 500, 1000  //
    };

    // 播放旋律
    //for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {

        PlayTone(melody[i], durations[i]);
        HAL_Delay(50); // 
        i++;
        if(i==sizeof(melody) / sizeof(melody[0])-1) i=0;
    //}
}


void PlayHappyBirthday() {
	static int i=0;
    float melody[] = {
        NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_F4, NOTE_E4, // Happy birthday to you
        NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_G4, NOTE_F4, // Happy birthday to you
        NOTE_C4, NOTE_C4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_E4, NOTE_D4, // Happy birthday dear [Name]
        NOTE_AS4, NOTE_AS4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_F4 // Happy birthday to you
    };

    uint16_t durations[] = {
        250, 250, 500, 500, 500, 1000, // 
        250, 250, 500, 500, 500, 1000, // 
        250, 250, 500, 500, 500, 500, 1000, // 
        250, 250, 500, 500, 500, 1000 // 
    };

    //for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
        PlayTone(melody[i], durations[i]);
        HAL_Delay(50);
        i++;
        if(i==sizeof(melody) / sizeof(melody[0])-1) i=0;
    //}
}


void PlayOdeToJoy() {
	static int i=0;
    float melody[] = {
        NOTE_E4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
        NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4,
        NOTE_E4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
        NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4
    };

    uint16_t durations[] = {
        250, 250, 250, 250, 250, 250, 250, 250,
        250, 250, 250, 250, 250, 250, 500,
        250, 250, 250, 250, 250, 250, 250, 250,
        250, 250, 250, 250, 250, 250, 500
    };

   // for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
        PlayTone(melody[i], durations[i]);
        HAL_Delay(50);
        i++;
        if(i==sizeof(melody) / sizeof(melody[0])-1) i=0;
    //}
}
void PlayEmergencyTone() {
    float melody[] = {
        NOTE_E5, NOTE_REST, NOTE_E5, NOTE_REST, NOTE_E5, NOTE_REST,
        NOTE_C5, NOTE_G4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4
    };

    uint16_t durations[] = {
        100, 50, 100, 50, 100, 50, 200, 200, 200, 200, 150, 150, 300
    };

    for (int i = 0; i < sizeof(melody) / sizeof(melody[0])/2; i++) {
        PlayTone(melody[i], durations[i]);
        HAL_Delay(50); // 每个音符间隔
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
 speed=70;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 //Forward_Speed(speed);

// Stop_Speed(0,1000);
 Stop_Speed(0,0);
Distance=10;
 //Buzz();
// while (1)
 //PlayTwinkleTwinkle();
 //PlayHappyBirthday() ;
 //PlayOdeToJoy();
  while (1)
  { //PlayTwinkleTwinkleStep() ;
	  switch (music_mode) {
	      case 1:
	    	  PlayTwinkleTwinkle() ;
	          break;

	      case 2:
	    	  PlayHappyBirthday() ;
	          break;

	      case 3:
	    	  PlayOdeToJoy();
	          break;

	      default:

	          break;
	  }
	 // music_mode=0;
	  //buzzer_state++;
	// Buzz();
//    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,rf_duty);//right forward
//    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,lf_duty);
//    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,rb_duty);//right forward
//    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,lb_duty);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 //Start Ultrasonic Sensor to measure
	  HCSR05_Read();
	  if(Distance < 5)
	  {Distance=10;
		  Stop_Speed(0,0);
		  PlayEmergencyTone();
		  Backward_Speed(speed);
		  Stop_Speed(0,200);
	  }
	  else
	  {
		//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	  }
	  HCSR05_Read();

	    data = ' ';
	    p++;
		  HAL_UART_Receive(&huart6, &data, 1, 100);
		  switch(data)
		  {
			  case 'w':
				  //sprintf(text,"   Forward!!");
				  Forward_Speed(speed);
				  p=100;
				 // Stop_Speed(0,1000);
				  break;
			  case 'a':
					//sprintf(text,"    Turn Left!!");
				  Left_Speed(speed);//turn left

				  p=200;


					//Stop_Speed(0,500);
					p=12345;
					break;
			  case 's':
					//sprintf(text,"    Backward!!");
				  Backward_Speed(speed);//turn right
				  p=300;
				// Stop_Speed(0,1000);
					break;
			  case 'd':
					//sprintf(text,"    Turn Right!!");
					Right_Speed(speed);
					  p=400;
					//  HAL_Delay(1000);
					//Stop_Speed(0,500);
						p=12345;
					break;
			  case 'h'://speed high
					if (speed<=95)
					{
					//sprintf(text,"    Speed Up!!");
					speed+=5;
					}


					break;
			  case 'l'://speed low

					if (speed>=5)
					{
					//sprintf(text,"    Speed Down!!");
					speed-=5;
					}
					break;
			  case '0'://sound
			  				music_mode=0;
			  					break;
			  case '1'://sound
				  music_mode=1;

					break;
			  case '2'://sound
				  music_mode=2;
			  					break;
			  case '3'://sound
				  music_mode=3;
			  					break;
			  case 'j'://mode1
					//sprintf(text,"      Low Speed ");
					speed=100;
					break;
			  case 'q'://mode2
					//sprintf(text,"      Mid Speed ");
					speed=50;
					break;
			  case 'k'://mode3
					//sprintf(text,"      High Speed ");
					speed=10;
					break;
			  case 'p':
					//sprintf(text,"      STOP ");
					//HAL_Delay(1000);
				  //music_mode=0;
				  Stop_Speed(0,0);
				  //p=12345;
					//Stop_Speed(0);
					break;
		  }
		 // HAL_Delay(10000);
		//Stop_Speed();



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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HCSR05_Read(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); // PE_8 Output High Level Voltage.

	delay(10); // Delay 10us.
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); // PE_8 Output Low Level Voltage.
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1); // Enable Input Capture.
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  //Check Source of input_capture
	{
		if(Is_First_Captured==0) // Confirmation of rising edges of capture??
			{
			IC_Val1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // Get Capture Value of rising (Count)
			Is_First_Captured = 1; //Change detection flag
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING); //Change detection edge to falling
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); //Toggle PD_15
		}
		else if(Is_First_Captured==1) // Confirmation of falling edges of capture
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // Get Capture Value of falling (Count)
			__HAL_TIM_SET_COUNTER(&htim1,0); //Zeroing the count

			if (IC_Val2>IC_Val1) //Compare count value
			{
				Difference = IC_Val2 - IC_Val1; // Calculate Difference (Duration)
			}
			else if(IC_Val1 > IC_Val2)
			{
				Difference = (65535 - IC_Val1) + IC_Val2; //If IC_Val1 > IC_Val2 ->> Diff = ARR + (IC_Val2 - IC_Val1)
			}
			Distance = Difference * 0.0343 / 2; // Calculate Distance , Please explain why multiply by 0.0343.
			Is_First_Captured = 0; // Change detection flag

		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); //Change detection edge to rising.
		__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC1); // Disable Input Capture.
		}
	}
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
