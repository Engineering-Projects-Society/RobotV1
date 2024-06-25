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
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//----------------------register record

typedef struct __attribute__((packed)) {
    uint8_t shutdown : 1;			//shutdown robot
    uint8_t irleds : 1;				//enable IRleds
    uint8_t sonar : 1;				//enable hc-sr04
    uint8_t servo : 1;				//enable servo output
    uint8_t motors : 1;				//enable motor drivers & PWM out
    uint8_t encoderMeas : 1;		//enable encoder step counting
    uint8_t powerMeas : 1;			//enable power measurements over i2c
    uint8_t neopixels : 1;			//enable neopixel driver
} EnableConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t rate1 : 2;				//group 1 sensor polling rate 20hz/60hz/120hz/240hz
    uint8_t rate2 : 2;				//group 2 sensor polling rate 20hz/60hz/120hz/240hz
    uint8_t interrupt1 : 1;			//interrupt for group 1 (signals data ready)
    uint8_t interrupt2 : 1;			//interrupt for group 2 (signals data ready)
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} PollingConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t pollN : 1;				//which sensor group it belongs (1 or 2)
    uint8_t power : 3;				//power settings (to be implemented)
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} IRledConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t PollN : 1;				//which sensor group it belongs (1 or 2)
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} SonarConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t pollN : 1;				//sensor group it belongs
    uint8_t bit1 : 1;				//low power cutoff
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} PowerConfig_t;

typedef struct __attribute__((packed)) {
    uint8_t mode : 2;				//power, encoder, steps modes
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} MotorsConfig_t;

typedef struct __attribute__((packed)) {
    int8_t data : 8;
} Motor1Data_t;

typedef struct __attribute__((packed)) {
	int8_t data : 8;
} Motor2Data_t;



typedef struct __attribute__((packed)) {
	EnableConfig_t 	ec;
	PollingConfig_t pc;
	IRledConfig_t	ir;
	SonarConfig_t	sc;
	PowerConfig_t	pwc;
	MotorsConfig_t	mc;
	Motor1Data_t	m1;
	Motor2Data_t	m2;
} I2C_RegRecord_t;

//------------------------end of register record

//i2c receive options, used for external device to change reg record
typedef struct {
    uint8_t address;
    uint8_t data;
} I2C_RecieveOptions_t;

typedef struct {
	uint8_t ledN;
	uint8_t scl;
	uint8_t count;

} IRscanVars_t;

typedef struct {
	volatile int count;

} SequencerVars_t;

typedef struct {
	volatile int count;
	volatile int mode;
	volatile int trig;
	volatile int reading;
} SonarVars_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BQ24295_ADDR       (0x6B << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

I2C_RegRecord_t i2cReg ={0};
I2C_RecieveOptions_t i2cRo ={0};

IRscanVars_t irscan={0};
SequencerVars_t sequencerVars={0};
ADC_ChannelConfTypeDef adcConfig={0};
SonarVars_t sonarVars={0};

uint16_t adcDMAbuf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t adcDMAbuf2[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t adcConversions[12]={0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t i2cTransmitBuf[30]={0};

GPIO_PinState lastButtonState = GPIO_PIN_RESET;   // Previous button state, assumed initially released.
uint32_t lastDebounceTime = 0;                    // Last time the button input was toggled.
const uint32_t debounceDelay = 50;                // Debounce time in milliseconds.




//flags
uint8_t firstBoot_flag = 0;
volatile uint8_t i2cCheck_flag = 0;
volatile uint8_t rxCheck = 0;
volatile uint8_t wrCheck = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PollToggleButton(void) {
    GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

    // Check for a button state change
    if (currentButtonState != lastButtonState) {
        // If the current state of the button is LOW, that means it was just pressed
        if (currentButtonState == GPIO_PIN_RESET) {
            uint32_t currentTime = HAL_GetTick();

            if ((currentTime - lastDebounceTime) > debounceDelay) {
                // Toggle the LED
            	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
            	// Reset the debounce timer
            	lastDebounceTime = currentTime;
            }
        }
        lastButtonState = currentButtonState;
    }
}


void BQ24295_SetShippingMode(void) {
    uint8_t data;

    // Disable Watchdog Timer: Set address 0x05[5:4] to 00
    HAL_I2C_Mem_Read(&hi2c1, BQ24295_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    data &= 0xCF;  // Clear bits 5 and 4
    HAL_I2C_Mem_Write(&hi2c1, BQ24295_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    // Disable BATFET: Set address 0x07[5] to 1
    HAL_I2C_Mem_Read(&hi2c1, BQ24295_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    data |= 0x20;  // Set bit 5
    HAL_I2C_Mem_Write(&hi2c1, BQ24295_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){

	uint8_t *regAddr = (uint8_t *)&i2cReg;  // Cast struct to byte pointer
/*
	for(int i = 0; i < 12; i++) {
	    i2cTransmitBuf[i * 2 + 1] = (uint8_t)(adcConversions[i] & 0xFF); // Lower 8 bits
	    i2cTransmitBuf[i * 2] = (uint8_t)((adcConversions[i] >> 8) & 0xFF); // Upper 8 bits
	}
*/
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)
		{
			i2cCheck_flag=1;	//check option fields flag
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *)&i2cRo, 2, I2C_FIRST_AND_LAST_FRAME);	//receive i2c command
		}
		else
		{
			if(wrCheck==0){
				int length=(i2cReg.ec.irleds)*16+(i2cReg.ec.powerMeas)*8+(i2cReg.ec.sonar)*2+(i2cReg.ec.encoderMeas)*2;
				int count=-1; int pattern; int check[4]={0};
				for(int i=0;i<(sizeof(i2cTransmitBuf)/sizeof(i2cTransmitBuf[0]))/2;i++){

					if(count<0){
						if(i2cReg.ec.irleds&&!check[0]){
							count=7;
							pattern=0;
							check[0]=1;
						}else if(i2cReg.ec.sonar&&!check[1]){
							count=0;
							pattern=1;
							check[1]=1;
						}else if(i2cReg.ec.powerMeas&&!check[2]){
							count=3;
							pattern=2;
							check[2]=1;
						}else if(i2cReg.ec.encoderMeas&&!check[3]){
							count=0;
							pattern=3;
							check[3]=1;
						}else{pattern=-1;}
					}

					switch(pattern){
					case 0:
						i2cTransmitBuf[i * 2 + 1] = (uint8_t)(adcConversions[7-count] & 0xFF); // Lower 8 bits
						i2cTransmitBuf[i * 2] = (uint8_t)((adcConversions[7-count] >> 8) & 0xFF); // Upper 8 bits
						break;
					case 1:
						i2cTransmitBuf[i * 2 + 1] = (uint8_t)(sonarVars.reading & 0xFF); // Lower 8 bits
						i2cTransmitBuf[i * 2] = (uint8_t)((sonarVars.reading >> 8) & 0xFF); // Upper 8 bits
						break;
					case 2:
						i2cTransmitBuf[i * 2 + 1] = (uint8_t)(adcConversions[11-count] & 0xFF); // Lower 8 bits
						i2cTransmitBuf[i * 2] = (uint8_t)((adcConversions[11-count] >> 8) & 0xFF); // Upper 8 bits
						break;
					case 3:
						i2cTransmitBuf[i * 2 + 1] = (uint8_t)(adcConversions[11-count] & 0xFF); // Lower 8 bits
						i2cTransmitBuf[i * 2] = (uint8_t)((adcConversions[11-count] >> 8) & 0xFF); // Upper 8 bits
						break;
					default:
						i2cTransmitBuf[i * 2 + 1] = 0;
						i2cTransmitBuf[i * 2] = 0;
					}

					count--;

				}
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTransmitBuf, length, I2C_FIRST_AND_LAST_FRAME);	//send data
			} else {
				HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &(regAddr[i2cRo.data-1]), 1, I2C_FIRST_AND_LAST_FRAME);
				wrCheck=0;
			}
		}


}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	uint8_t *regAddr = (uint8_t *)&i2cReg;  // Cast struct to byte pointer
	if(i2cRo.address==0){
		wrCheck=1;		//special address 0 triggers next read to be data from specified address of register, flag sets next address callback to write single byte data
	} else if(i2cRo.address < sizeof(I2C_RegRecord_t) + 1) {
        regAddr[i2cRo.address-1] = i2cRo.data;  // Write received data to the specified register
    }
    else {
        Error_Handler();  // Invalid address received
    }
	rxCheck=1;
}

void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM6){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

		if(sequencerVars.count%4==0&&i2cReg.ec.irleds==1){HAL_TIM_Base_Start_IT(&htim3);}

		if(sequencerVars.count%80==0&&sonarVars.mode==0&&htim14.State==1&&i2cReg.ec.sonar==1){
			sonarVars.count=1;
			sonarVars.trig=0;
			sonarVars.mode=1;
			//HAL_TIM_Base_Start(&htim14);
			HAL_TIM_Base_Start_IT(&htim14);

		}



		if(i2cReg.m1.data>0){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint8_t)abs(i2cReg.m1.data)<<1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		}else{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint8_t)abs(i2cReg.m1.data)<<1);
		}

		if(i2cReg.m2.data>0){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint8_t)abs(i2cReg.m2.data)<<1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		}else{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint8_t)abs(i2cReg.m2.data)<<1);
		}

		if(sequencerVars.count==240){sequencerVars.count=1;}else{sequencerVars.count++;}
	}

	if (htim->Instance == TIM3){

		if(i2cReg.ec.irleds==0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
		}

		if(irscan.scl==0){

			switch(irscan.ledN){
				case 0:
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
					break;
				case 1:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1);
					break;
				case 2:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
					break;
				case 3:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1);
					break;
				case 4:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,1);
					break;
				case 5:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
					break;
				case 6:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1);
					break;
				case 7:
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
					break;
				default:
					Error_Handler();
			}
			irscan.scl=1;
		}else if(irscan.scl==2){

			switch(irscan.ledN){
				case 0:
					adcConversions[0]=adcDMAbuf[0];
					break;
				case 1:
					adcConversions[1]=adcDMAbuf[1];
					break;
				case 2:
					adcConversions[2]=adcDMAbuf[2];
					break;
				case 3:
					adcConversions[3]=adcDMAbuf[3];
					break;
				case 4:
					adcConversions[4]=adcDMAbuf[4];
					break;
				case 5:
					adcConversions[5]=adcDMAbuf[5];
					break;
				case 6:
					adcConversions[6]=adcDMAbuf[6];
					break;
				case 7:
					adcConversions[7]=adcDMAbuf[7];
					break;
				default:
					Error_Handler();
			}
			irscan.scl=0;
		}else if(irscan.scl==1){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDMAbuf, 12);
			irscan.scl=2;
		}

		if (irscan.scl==0) {
			irscan.ledN++;
			if(irscan.ledN>7){irscan.ledN=0;}
		}


		//hadc1.Instance->CHSELR=irscan.ledN;
		//HAL_ADC_ConfigChannel(hadc, pConfig)
		//adcConfig.Channel=irscan.ledN;
		//HAL_ADC_ConfigChannel(&hadc1,&adcConfig);
		//hadc1.Instance->CHSELR &= ~(0x1F);  // Clear the current channel selection
		//hadc1.Instance->CHSELR |= (irscan.ledN);  // Set the new channel

		if(irscan.count>22){
			irscan.ledN=0;
			irscan.scl=0;
			irscan.count=0;
			HAL_TIM_Base_Stop(htim);
			__HAL_TIM_SET_COUNTER(htim,0);
		}else{irscan.count++;}

	}

	if (htim->Instance == TIM14){
		if(sonarVars.mode==1){
			HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, 1);
			sonarVars.count--;
			if(sonarVars.count<1){
				sonarVars.mode=2;
			}
		}else if(sonarVars.mode==2){
			sonarVars.mode=0;
			HAL_TIM_Base_Stop(htim);
			__HAL_TIM_SET_COUNTER(htim,0);
			HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, 0);
		} else {
			sonarVars.count++;
			if(sonarVars.trig==1){
				HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, 0);
				HAL_TIM_Base_Stop(htim);
				sonarVars.reading=sonarVars.count;
				sonarVars.count=0;
				sonarVars.mode=0;
				sonarVars.trig=0;
			}
		}


	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_15){
		if(firstBoot_flag==1){
			BQ24295_SetShippingMode();
		}
	}
	if(GPIO_Pin==GPIO_PIN_10){
		sonarVars.trig=1;
	}

}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10){
		sonarVars.count=0;
		sonarVars.mode=3;
		if(HAL_TIM_Base_Start_IT(&htim14)==1){HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, 0);};
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  //input button has no interrupt available, polling
  GPIO_PinState input1;

  //DMA for ADC
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDMAbuf, 1);

  //enable i2c
  HAL_I2C_Init(&hi2c2);
  HAL_I2C_EnableListen_IT(&hi2c2);

  //enable motor PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //enable timer interrupts
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);
  //HAL_TIM_Base_Start_IT(&htim14);

  //calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  //adcConfig.Channel = ADC_CHANNEL_0;
  //adcConfig.Rank = ADC_REGULAR_RANK_1;
  //adcConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  input1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

	  if(firstBoot_flag==0){
		  if(HAL_GetTick()>1000){

			  HAL_GPIO_WritePin(EN_ESP_GPIO_Port, EN_ESP_Pin, 1);
			  firstBoot_flag=1;
		  }
	  }

	  if(i2cCheck_flag==1){
			if(i2cReg.ec.irleds==0){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
			}
	  }

	  //HAL_TIM_Base_Start(&htim7);

	  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	  HAL_Delay(1);
/*
	  if(i2cReg.m1.data>0){
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint8_t)abs(i2cReg.m1.data)<<1);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  }else{
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint8_t)abs(i2cReg.m1.data)<<1);
	  }

	  if(i2cReg.m2.data>0){
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint8_t)abs(i2cReg.m2.data)<<1);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  }else{
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint8_t)abs(i2cReg.m2.data)<<1);
	  }
*/
	  PollToggleButton();
	  //HAL_I2C_EnableListen_IT(&hi2c2);
	  //HAL_I2C_EnableListen_IT(&hi2c1);
	  /*HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
		  HAL_ADC_Stop_DMA(&hadc1);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcConversions, 12);

	  }*/

	  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
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
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 2;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00602173;
  hi2c2.Init.OwnAddress1 = 2;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 64000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9600;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2400;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CE_BOOST_Pin|OE_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OTG1_Pin|S_MUX_Pin|NEOPIXEL_DI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STM_INT_OUT2_Pin|STM_INT_OUT1_Pin|LED_Pin|GPIO_PIN_3
                          |EN_ESP_Pin|TRIG_SONAR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nSLEEP_MOTORS_GPIO_Port, nSLEEP_MOTORS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CE_BOOST_Pin OTG1_Pin OE_MUX_Pin S_MUX_Pin */
  GPIO_InitStruct.Pin = CE_BOOST_Pin|OTG1_Pin|OE_MUX_Pin|S_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NEOPIXEL_DI_Pin */
  GPIO_InitStruct.Pin = NEOPIXEL_DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NEOPIXEL_DI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_INT_OUT2_Pin STM_INT_OUT1_Pin LED_Pin PB3
                           EN_ESP_Pin nSLEEP_MOTORS_Pin TRIG_SONAR_Pin */
  GPIO_InitStruct.Pin = STM_INT_OUT2_Pin|STM_INT_OUT1_Pin|LED_Pin|GPIO_PIN_3
                          |EN_ESP_Pin|nSLEEP_MOTORS_Pin|TRIG_SONAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall1_1_Pin Hall1_2_Pin */
  GPIO_InitStruct.Pin = Hall1_1_Pin|Hall1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall2_1_Pin Hall2_2_Pin */
  GPIO_InitStruct.Pin = Hall2_1_Pin|Hall2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ON_PRESS_Pin */
  GPIO_InitStruct.Pin = ON_PRESS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ON_PRESS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_BUTTON_Pin */
  GPIO_InitStruct.Pin = INPUT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_BOOST_Pin INT_ACCEL_Pin */
  GPIO_InitStruct.Pin = INT_BOOST_Pin|INT_ACCEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_SONAR_Pin */
  GPIO_InitStruct.Pin = ECHO_SONAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_SONAR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
