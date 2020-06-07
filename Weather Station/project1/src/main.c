/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <math.h>

#define THRESHOLD_TEMPERATURE 30
#define THRESHOLD_WIND 90
#define THRESHOLD_NOISE 60

#define MAX_ANGLE 4096
#define B 4275 
#define R0 100000

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

int soundValue, temperatureValue, brValue, adcValue,speed;
ADC_ChannelConfTypeDef sConfig = {0};
int buffer[4];

char sonido[3];
char temperature [3];
char luz [7];
char aux[4];

float temp;
float lux;
float db;

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _numlines, _currline;
typedef unsigned char BYTE;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void clear();
void setReg(unsigned char addr, unsigned char dta);
void display();
void command(uint8_t value);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */


    /* INICIALIZAMOS EL PANEL CON TODOS LOS LEDS EN VERDE*/
    HAL_GPIO_WritePin(GPIOA, greenWind_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, redWind_Pin, GPIO_PIN_RESET);

    //Restart temperature alarms
    HAL_GPIO_WritePin(GPIOA, greenTemperature_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, greenTemperature_Pin, GPIO_PIN_RESET);

    //Restart sound alarms
    HAL_GPIO_WritePin(GPIOB, greenSound_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, redSound_Pin, GPIO_PIN_RESET);

    
      BYTE viento[8] = {
      0x11,
      0x0A,
      0x04,
      0x0E,
      0x15,
      0x04,
      0x04,
      0x04
      
    };

      BYTE sonido[8] = {
      0x04,
      0x06,
      0x05,
      0x04,
      0x04,
      0x1C,
      0x1C,
      0x1C
    };

    BYTE bombilla[8] = {
      0x15,
      0x0E,
      0x11,
      0x11,
      0x11,
      0x0A,
      0x0E
    };
    BYTE temperatura[8] = {
    0x18,
    0x18,
    0x07,
    0x04,
    0x04,
    0x04,
    0x07,
    0x00
      };
      BYTE vacio[8] = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
      };

    //Creamos unos caracteres
    createChar(0,vacio);
    createChar(3, viento);
    createChar(4,sonido);
    createChar(5, temperatura);
    createChar(1,bombilla);

    rgb_lcd_begin(16,2,LCD_5x8DOTS);
    clear();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /*Cambiamos de canal del ADC, para ir recogiendo todos los valores*/

    //VIENTO
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start_IT(&hadc1);
    HAL_Delay(1000);
    HAL_ADC_Stop_IT(&hadc1);

    lcd_verde();

    //SONIDO
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    HAL_ADC_Start_IT(&hadc1);
    HAL_Delay(1000);
    HAL_ADC_Stop_IT(&hadc1);

    lcd_verde();
    
    //TEMPERATURA
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);


    HAL_ADC_Start_IT(&hadc1);
    HAL_Delay(1000);
    HAL_ADC_Stop_IT(&hadc1);

    lcd_verde();
    
    //BRILLO
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start_IT(&hadc1);
    HAL_Delay(1000);
    HAL_ADC_Stop_IT(&hadc1);

    lcd_verde();

    HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/*Recogemos los valores del ADC mediante interrupciones*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  int sum;
  if(hadc->Instance == ADC1)
  {    
    switch(sConfig.Channel){
      case 0:
        HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
        adcValue=HAL_ADC_GetValue(&hadc1);
        htim2.Instance->CCR1 = 75 + (adcValue*1425/MAX_ANGLE); 
      break;
      case 1:
        soundValue = HAL_ADC_GetValue(&hadc1);
        if(soundValue>0){
          db=10.0*log(soundValue/300.0)+20.0*log(soundValue/300.0);
          sprintf(sonido, "%d", (int)db);
        }

          sum = 10*log10(10/6) + 20 *log10(adcValue);
          printf("sound; %d dB\r\n", sum);
      break;
      case 4:
        temperatureValue = HAL_ADC_GetValue(&hadc1);
        float R = (4095.0/temperatureValue)-1.0;
        R = R0*R;
        temp = 1.0/(log(R/R0)/B+1/298.15) - 273.15;
        sprintf(temperature,"%d",(int) temp);
                
      break;
      case 8:
        brValue = HAL_ADC_GetValue(&hadc1);
        lux = exp(brValue/265.8); //355.0 //resistencia

        if(lux < 10000.0){
          int i = 2;
          while(i<7){
            luz[i] = ' ';
            i++;
          }
        }
        sprintf(luz,"%d",(int) lux);
      break;
      default:
      break;
    }
  }
}

/*Con este Timer, hacemos que no sea un sistema critico, actualizamos cada 5 segundos*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim11.Instance == htim->Instance){

    //Viento
    setCursor(0,0);
    write(3);
    enviarWrite(aux,sizeof(aux));
    enviarWrite(" km/h", sizeof(" km/h"));

    if(speed > THRESHOLD_WIND){
    HAL_GPIO_WritePin(GPIOA, greenWind_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, redWind_Pin, GPIO_PIN_SET);
    }

    //Temperatura
    setCursor(12,0);
    enviarWrite(temperature,sizeof(temperature));
    write(5);

    if(temp > THRESHOLD_TEMPERATURE){
    HAL_GPIO_WritePin(GPIOA, greenTemperature_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, redTemperature_Pin, GPIO_PIN_SET);
    }
    
    //Sonido
    setCursor(0,1);
    write(4);
    enviarWrite(sonido,sizeof(sonido));
    enviarWrite(" dB", sizeof(" dB"));

    if(db > THRESHOLD_NOISE){
      HAL_GPIO_WritePin(GPIOB, greenSound_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, redSound_Pin, GPIO_PIN_SET);
    }

    //Brillo
    setCursor(7,1);
    write(1);
    enviarWrite(luz,sizeof(luz));
    enviarWrite(" L", sizeof(" L"));
  }
}

/*Obtenemos la velocidad de la onda del viento*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
  int final, range;  

  int start = htim2.Instance->CCR1;
  range = start - final;
  speed = (range / 7.125) - 10;

  if(speed < 10){
    aux[2] = ' ';
  }
   
  sprintf(aux, "%d", speed);
  
  if (htim3.Instance->CCR1==599){
    htim3.Instance->CCR1=2999;
  }else{
    htim3.Instance->CCR1=599; 
  }
  final = htim2.Instance->CCR1;
  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
}

/*Configuramos el boton reset y el sensor de lluvia*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	__disable_irq();
	if(GPIO_Pin == GPIO_PIN_10){
    //Restart wind alarms
    HAL_GPIO_WritePin(GPIOA, greenWind_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, redWind_Pin, GPIO_PIN_RESET);

    //Restart temperature alarms
    HAL_GPIO_WritePin(GPIOA, greenTemperature_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, redTemperature_Pin, GPIO_PIN_RESET);

    //Restart sound alarms
    HAL_GPIO_WritePin(GPIOB, greenSound_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, redSound_Pin, GPIO_PIN_RESET);
    
  }else if(GPIO_Pin == GPIO_PIN_5){
    printf("Precipitation detected\r\n");
    lcd_azul();
    //HAL_Delay(1);
  }
	__enable_irq();
}


int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart2, &*c, 1, 100);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}

void createChar(uint8_t location, uint8_t charmap[]) {

    location &= 0x7; // we only have 8 locations 0-7
    command(LCD_SETCGRAMADDR | (location << 3));

    unsigned char dta[9];
    dta[0] = 0x40;
    for (int i = 0; i < 8; i++) {
        dta[i + 1] = charmap[i];
    }
  enviarWrite(dta, sizeof(dta));
}

void setCursor(uint8_t col, uint8_t row) {

    col = (row == 0 ? col | 0x80 : col | 0xc0);
    unsigned char dta[2] = {0x80, col};

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, dta, sizeof(dta), 100);
}

void rgb_lcd_begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {

    if (lines > 1) {
        _displayfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    // for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lines == 1)) {
        _displayfunction |= LCD_5x10DOTS;
    }

    HAL_Delay(50);

    // Send function set command sequence
    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(5);  // wait more than 4.1ms

    // second try
    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);

    // third go
    command(LCD_FUNCTIONSET | _displayfunction);


    // finally, set # lines, font size, etc.
    command(LCD_FUNCTIONSET | _displayfunction);

    // turn the display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    display();

    // clear it off
    clear();

    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    command(LCD_ENTRYMODESET | _displaymode);


    // backlight init
    setReg(REG_MODE1, 0);
    // set LEDs controllable by both PWM and GRPPWM registers
    setReg(REG_OUTPUT, 0xFF);
    // set MODE2 values
    // 0010 0000 -> 0x20  (DMBLNK to 1, ie blinky mode)
    setReg(REG_MODE2, 0x20);

}

void clear() {
    command(LCD_CLEARDISPLAY);        // clear display, set cursor position to zero
    HAL_Delay(2);          
}

void display() {
    _displaycontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void setReg(unsigned char addr, unsigned char dta) {
  unsigned char data[2] = {addr, dta};
  HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, data, 2, 100);
}

void setRGB(unsigned char r, unsigned char g, unsigned char b) {
    setReg(REG_RED, r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE, b);
}

void command(uint8_t value) {
    unsigned char dta[2] = {0x80, value};
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, dta, sizeof(dta), 100);
    
}

void write(uint8_t value) {

    unsigned char dta[2] = {0x40, value};
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, dta, 2, 100);
}

void enviarWrite(unsigned char* dta, unsigned char len) {      
    for (int i = 0; i < len - 1; i++) {
      write(dta[i]);
    }
}
void lcd_rojo(){
    setRGB(255, 0, 0);
}

void lcd_azul(){
    setRGB(0, 0, 255);
}

void lcd_verde(){
    setRGB(0, 255, 0);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;  
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
 
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1119;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 41999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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

  /*Configure GPIO pin Output Lespeed */
  HAL_GPIO_WritePin(GPIOA, greenTemperature_Pin|greenWind_Pin|redSound_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Lespeed */
  HAL_GPIO_WritePin(GPIOB, redWind_Pin|greenSound_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Lespeed */
  HAL_GPIO_WritePin(redTemperature_GPIO_Port, redTemperature_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : greenTemperature_Pin greenWind_Pin redSound_Pin */
  GPIO_InitStruct.Pin = greenTemperature_Pin|greenWind_Pin|redSound_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : redWind_Pin greenSound_Pin */
  GPIO_InitStruct.Pin = redWind_Pin|greenSound_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : redTemperature_Pin */
  GPIO_InitStruct.Pin = redTemperature_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(redTemperature_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/