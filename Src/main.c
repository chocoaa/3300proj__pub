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
#include "adc.h"
#include "crc.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ds18b20.h"
#include "TB6612FNG.h"
#include "bsp_ili9341_lcd.h"
#include "bsp_xpt2046_lcd.h"
#include "bsp_led.h"
#include "palette.h"
#include "bsp_sdio_sdcard.h"
#include "File_Handling.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//LED
#define LED_PORT                                GPIOB
#define LED_PIN_POWER                           GPIO_PIN_0
#define LED_PIN_TEMP                            GPIO_PIN_1
#define LED_PIN_LEVEL                           GPIO_PIN_5
#define NUM_LEDs                                3

//SENSOR
#define SENSOR_PORT_TEMP                        GPIOE
#define SENSOR_PIN_TEMP                         GPIO_PIN_6
#define SENSOR_PORT_TRIGGER                     GPIOC
#define SENSOR_PIN_TRIGGER                      GPIO_PIN_13
#define SENSOR_PORT_PROX                        GPIOA
#define SENSOR_PIN_PROX                         GPIO_PIN_4
#define SENSOR_PORT_LEVEL                       GPIOE
#define SENSOR_PIN_LEVEL                        GPIO_PIN_5

//Large Power Applis
#define PUMP_PORT                               GPIOB
#define PUMP_PIN                                GPIO_PIN_13
#define MOV_PORT                                GPIOB
#define MOV_PIN_1                               GPIO_PIN_14
#define MOV_PIN_2                               GPIO_PIN_15
#define MOV_PWM_PORT                            GPIOC
#define MOV_PWM_PIN                             GPIO_PIN_7
#define MOV_PWM_TIM                             &htim3
#define MOV_PWM_CH                              TIM_CHANNEL_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef FREERTOS_CONFIG_H
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t mainScreen_Handle = NULL;
static TaskHandle_t debugScreen_Handle = NULL;
static TaskHandle_t palette_Handle = NULL;
static void AppTaskCreate(void);
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t LED_PINs[NUM_LEDs]={LED_PIN_POWER,LED_PIN_TEMP,LED_PIN_LEVEL};
GPIO_PinState  Status_Flags[NUM_LEDs]={GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET};//{POWER/TEMP/LEVEL}
uint8_t demoFlag=0;
TB6612FNG_Motor Mov;
TB6612FNG_Pin MovIN1,MovIN2;
TB6612FNG_Timer MovTIM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void mainScreen(void* pvParameters);
static void debugScreen(void* pvParameters);
static void palette(void* pvParameters);
static void ledUpdate();
static void getvalue(int);
static uint8_t notification(char*,char*,char*,int );
static uint8_t trap(char*);
static uint8_t pourWater();
static uint8_t startCook();
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
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_SDIO_SD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  LCD_SCAN_MODE=3;
	HAL_ADCEx_Calibration_Start(&hadc1);
  ILI9341_Init();
  XPT2046_Init();
	HAL_Delay(500);
	HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
  //setup mov1
  MovIN1.Port=MOV_PORT;
  MovIN1.Pin=MOV_PIN_1;
  MovIN2.Port=MOV_PORT;
  MovIN2.Pin=MOV_PIN_2;
  MovTIM.htim=MOV_PWM_TIM;
  MovTIM.channel=MOV_PWM_CH;
  Mov.I1=&MovIN1;
  Mov.I2=&MovIN2;
  Mov.TIMER=&MovTIM;
  Mov.MOTOR=TB6612FNG_Motor_A;
  setupTB6612FNG(MOV_PWM_TIM,&Mov,&Mov);
  startTB6612FNG();
  #ifdef FREERTOS_CONFIG_H
  BaseType_t xReturn = pdPASS;
  xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,
                        (const char*)"AppTaskCreate",
                        (uint16_t)512,
                        (void*)NULL,
                        (UBaseType_t)1,
                        (TaskHandle_t*)&AppTaskCreate_Handle);
  
  //if done init and working
  if(xReturn!=pdPASS) return -1;
  #endif
  ledUpdate();
  mainScreen(NULL);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
static void getvalue(int task){
	int heig[6]={1,3,6,8,10,12};  //rows of values to be upd
  ILI9341_Clear(14*WIDTH_EN_CHAR, heig[task-1]*HEIGHT_EN_CHAR, 10*WIDTH_EN_CHAR, HEIGHT_EN_CHAR);
	if(task==1){	//adc
		uint32_t value = HAL_ADC_GetValue(&hadc1);
		char valueDec[5];
		char valueHex[5];
		sprintf(valueDec,"%lu",(unsigned long)value);
		sprintf(valueHex,"%lx",value);
		ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,valueDec);
		ILI9341_DispString_EN(20*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,valueHex);
	}else if (task == 2){ //temp sen
    if (!DS18B20_Init ()){
    int Temperature = (int)DS18B20_GetTemp_SkipRom(); //float disp sucks, int disp then. it floors
    char tempStr[8];
    sprintf(tempStr,"%d",Temperature);
    ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,tempStr);
    }else ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,"NO PROBE!");
  }else if (task == 3){ //touch
    static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
    XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
    char xStr[4];
    char yStr[4];
    sprintf(xStr,"%d",cinfo.x);
    sprintf(yStr,"%d",cinfo.y);
    ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,xStr);
		ILI9341_DispString_EN(20*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,yStr);
  }else if (task>=4 && task<=6){ //GPIO sensors
    GPIO_TypeDef *sensorPorts[3]={SENSOR_PORT_TRIGGER,SENSOR_PORT_PROX,SENSOR_PORT_LEVEL};
    uint16_t sensorPins[3]={SENSOR_PIN_TRIGGER,SENSOR_PIN_PROX,SENSOR_PIN_LEVEL};
    uint8_t Pinstate=(HAL_GPIO_ReadPin(sensorPorts[task-4],sensorPins[task-4])==GPIO_PIN_SET)?1:0;
    ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,(Pinstate)?"On":"Off");
  }
}

static void debugScreen(void* Parameter){
  volatile uint32_t lastUpd=0;
  ILI9341_Clear(0,0,ILI9341_MORE_PIXEL,ILI9341_LESS_PIXEL);
  ILI9341_DispString_EN(14*WIDTH_EN_CHAR , 0*HEIGHT_EN_CHAR , "DEC");
	ILI9341_DispString_EN(20*WIDTH_EN_CHAR , 0*HEIGHT_EN_CHAR , "HEX");
	ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 1*HEIGHT_EN_CHAR ,"ADC1IN10_C0: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 3*HEIGHT_EN_CHAR ,"DS18B20_E6: ");
	ILI9341_DispString_EN(14*WIDTH_EN_CHAR , 5*HEIGHT_EN_CHAR , "X");
	ILI9341_DispString_EN(20*WIDTH_EN_CHAR , 5*HEIGHT_EN_CHAR , "Y");  
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 6*HEIGHT_EN_CHAR ,"LAST TOUCH: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 8*HEIGHT_EN_CHAR ,"Trigger_C13: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 10*HEIGHT_EN_CHAR ,"Prox_A4: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 12*HEIGHT_EN_CHAR ,"Level_E5: ");
  ILI9341_DrawRectangle(0,ILI9341_LESS_PIXEL-25,75,25,0);
  ILI9341_DispString_EN(1,ILI9341_LESS_PIXEL-21,"Cal Touch");
  ILI9341_DrawRectangle(75,ILI9341_LESS_PIXEL-25,84,25,0);
  ILI9341_DispString_EN(76,ILI9341_LESS_PIXEL-21,"Demo Mode");
  ILI9341_DrawRectangle(ILI9341_MORE_PIXEL-40,ILI9341_LESS_PIXEL-25,40,25,0);
  ILI9341_DispString_EN(ILI9341_MORE_PIXEL-39,ILI9341_LESS_PIXEL-21,"Exit");
  if(demoFlag)ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-12,4,1);
  while(1){
    ledUpdate();
    if(HAL_GetTick()-lastUpd>500){
			lastUpd=HAL_GetTick();
      getvalue(1);
			getvalue(2);
      getvalue(4);
      getvalue(5);
      getvalue(6);
		} //update per 500ms
    if(XPT2046_TouchDetect() == TOUCH_PRESSED) {
      getvalue(3);
      static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
      XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
      if(cinfo.x>0 &&cinfo.x<(0+75)&&cinfo.y>(ILI9341_LESS_PIXEL-25)&&cinfo.y<ILI9341_LESS_PIXEL){
        uint16_t bgclr=0,txclr=0;
        LCD_GetColors(&txclr,&bgclr);
        XPT2046_Touch_Calibrate(LCD_SCAN_MODE);
        LCD_SetColors(txclr,bgclr);
        debugScreen(NULL); //Cal TouchScreen
      } else if (cinfo.x>75 &&cinfo.x<(75+84)&&cinfo.y>(ILI9341_LESS_PIXEL-25)&&cinfo.y<ILI9341_LESS_PIXEL){
        demoFlag=!demoFlag;
        if(!demoFlag){
          uint16_t bgclr=0,txclr=0;
          LCD_GetColors(&txclr,&bgclr);
          LCD_SetTextColor(bgclr); //overwrite diag content with bgclr
          ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-12,4,1);
          LCD_SetTextColor(txclr);
        }else ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-12,4,1);
      }else if (cinfo.x>(ILI9341_MORE_PIXEL-40)&&cinfo.x<ILI9341_MORE_PIXEL&&cinfo.y>(ILI9341_LESS_PIXEL-25)&&cinfo.y<ILI9341_LESS_PIXEL) mainScreen(NULL);
    } //touch handler with cord disp
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL); //debug exit
  }
}

static void palette(void* Parameter){
  //WARNING: last clr used will carry forward to other screens. excerice caution here.
  Palette_Init(LCD_SCAN_MODE);  //calls palette
  while (1){
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL);//debug exit
    XPT2046_TouchEvenHandler();
  }
}

static void ledUpdate(){
  Status_Flags[1]=(!DS18B20_Init() && DS18B20_GetTemp_SkipRom()>=80.0)?GPIO_PIN_SET:GPIO_PIN_RESET; //temp led
  Status_Flags[2]=(HAL_GPIO_ReadPin(SENSOR_PORT_LEVEL,SENSOR_PIN_LEVEL)==GPIO_PIN_SET)?GPIO_PIN_SET:GPIO_PIN_RESET; //level led
  for (int i=0;i<NUM_LEDs;i++) HAL_GPIO_WritePin(LED_PORT,LED_PINs[i],Status_Flags[i]);
}

static uint8_t notification(char * Message,char* optionL,char* optionR, int time_s){
  HAL_Delay(200); //Avoid multiclick
  uint16_t diagStartY=(time_s<=0)?80:40;
  uint16_t diagHeight=(time_s<=0)?80:160;
  uint16_t optStartY=(time_s<=0)?130:170; //countdown window being longer
  ILI9341_DrawRectangle(60,diagStartY,200,diagHeight,0);
  ILI9341_DispString_EN(160-(strlen(Message)*WIDTH_EN_CHAR/2),diagStartY+20,Message);
  ILI9341_DrawRectangle(90,optStartY,50,20,0);
  ILI9341_DispString_EN(115-(strlen(optionL)*WIDTH_EN_CHAR/2),optStartY+1,optionL);
  ILI9341_DrawRectangle(180,optStartY,50,20,0);
  ILI9341_DispString_EN(205-(strlen(optionR)*WIDTH_EN_CHAR/2),optStartY+1,optionR);
  uint8_t ret=1;
  while(time_s<=0){                                 //noti waittime inf
    ledUpdate();
    if(XPT2046_TouchDetect() == TOUCH_PRESSED){ 
      ret=8;
      static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
      XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
      if(cinfo.x>90 &&cinfo.x<(90+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)) ret=0; //L or Bypass
      if(cinfo.x>180 &&cinfo.x<(180+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)) ret=1; //R or OK
      if (ret==0 || ret==1) break;
    }
  }
  if(time_s>0)  for (int i = (int)time_s;i>=0;i--){ //countdown
    volatile uint32_t starting=HAL_GetTick();
    char timeStr [4];
    sprintf(timeStr,"%d",i);
    ILI9341_Clear(160-(4*2*WIDTH_EN_CHAR),90,(strlen(timeStr)+1)*4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR); //clear countdown zone to avoid leftover pixels
    ILI9341_DisplayStringEx(160-(strlen(timeStr)*4*WIDTH_EN_CHAR/2),90,4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR,(unsigned char *)timeStr,0); //disp countdown
    while (HAL_GetTick()-starting<1000){
      ledUpdate();
      if(time_s>=10&&HAL_GPIO_ReadPin(SENSOR_PORT_TRIGGER,SENSOR_PIN_TRIGGER)==GPIO_PIN_RESET&&(!demoFlag)){
        ret=0;
        break;
      }//if countdown time more than 10s, enable interrupt upon trigger removal
      if(XPT2046_TouchDetect() == TOUCH_PRESSED){
        static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
        XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
        if(cinfo.x>90 &&cinfo.x<(90+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)){
          ret=0;
          break;
        } else if(cinfo.x>180 &&cinfo.x<(180+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)) break;//Interrrupt by option
      }
      HAL_Delay(10);  //short enough to catch interrupts, tricky sol for interrupt
    } //do until 1s
    if(ret==0) break;
  }
  //clean screen
  uint16_t bgclr=0,txclr=0;
  LCD_GetColors(&txclr,&bgclr);
  LCD_SetTextColor(bgclr); //overwrite diag content with bgclr
  ILI9341_DrawRectangle(60,diagStartY,200,diagHeight,0);
  ILI9341_DispString_EN(160-(strlen(Message)*WIDTH_EN_CHAR/2),diagStartY+20,Message);
  ILI9341_DrawRectangle(90,optStartY,50,20,0);
  ILI9341_DispString_EN(115-(strlen(optionL)*WIDTH_EN_CHAR/2),optStartY+1,optionL);
  ILI9341_DrawRectangle(180,optStartY,50,20,0);
  ILI9341_DispString_EN(205-(strlen(optionR)*WIDTH_EN_CHAR/2),optStartY+1,optionR);
  if(diagHeight>80)  ILI9341_DisplayStringEx(160-(strlen("9999")*4*WIDTH_EN_CHAR/2),90,4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR,(unsigned char *)"9999",0);
  LCD_SetTextColor(txclr);//restore text clr
  return ret;
}

static uint8_t trap(char* Message){
  return notification (Message,"Bypass","OK",-1);
}

static void mainScreen(void* Parameter){
  ILI9341_Clear(0,0,ILI9341_MORE_PIXEL,ILI9341_LESS_PIXEL);
  
  if (notification("Hello! Continue Boot?","DevOps","Yes",3)){  //boot loader for devops, 3s timeout
    //normal mode
    Status_Flags[0]=1;
    while(1){
      ILI9341_DispString_EN(50,120,"Waiting for a cup noodle...");
      ledUpdate();
      uint8_t triggerBypass=0;
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL);
      if(demoFlag) {
        if(notification("Run Simulation","No","Yes",-1)) triggerBypass=1;
        else mainScreen(NULL);
      } //only appears when demoflag is set, overriding trigger switch
      if(HAL_GPIO_ReadPin(SENSOR_PORT_TRIGGER,SENSOR_PIN_TRIGGER)==GPIO_PIN_SET||triggerBypass){
        uint16_t bgclr=0,txclr=0;
        LCD_GetColors(&txclr,&bgclr);
        LCD_SetTextColor(bgclr); //overwrite diag content with bgclr
        ILI9341_DispString_EN(50,120,"Waiting for a cup noodle...");
        LCD_SetTextColor(txclr);//restore text clr
        if(startCook()){
          //noodle done! reset inject
          motorState(&Mov,FORWARD);
          setMotorDutyCycle(&Mov,0.99);
          notification("Homing Water Injet...","Abort","Skip",5);
          motorState(&Mov,STOP);
          //push noti
          HAL_UART_Transmit(&huart3,(unsigned char *)"SendTimeUP",10,300);
          //buzzer
          __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,30000);
          //Screen Noti
          notification("Noodle Ready!","Close","OK",5);
          __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
          //wait until noodle is picked up
          while(HAL_GPIO_ReadPin(SENSOR_PORT_TRIGGER,SENSOR_PIN_TRIGGER)==GPIO_PIN_SET)notification("Noodle Ready!","Close","OK",1);
        }else notification("Cooking Aborted!","Cancel","OK",-1);  //aborted
      }
    }
  }else {
    //devops
    if(notification("Dev Options","Debug","DwaR",-1))palette(NULL);
    else debugScreen(NULL);
  }
}



static uint8_t pourWater(){
  //Do linear moving
  motorState(&Mov,REVERSE);
  setMotorDutyCycle(&Mov,0.99);
  notification("Starting cooking...","Abort","Skip",5);
  motorState(&Mov,STOP);
  HAL_GPIO_WritePin(PUMP_PORT,PUMP_PIN,GPIO_PIN_SET);
  uint8_t ret = notification("Pouring water, time remaining:","Abort","Skip",10); //pump for 20s, time TBD
  HAL_GPIO_WritePin(PUMP_PORT,PUMP_PIN,GPIO_PIN_RESET); //turn off pump
  return ret;
}

static uint8_t startCook(){
  //foolproof
  if (HAL_GPIO_ReadPin(SENSOR_PORT_PROX,SENSOR_PIN_PROX)==GPIO_PIN_SET){
    if(trap("This is not a Cup Noodle!")) return 0; //didnt bypass,abort
  }
  //Water Level
  if (HAL_GPIO_ReadPin(SENSOR_PORT_LEVEL,SENSOR_PIN_LEVEL)==GPIO_PIN_RESET){
    if(trap("No Water!")) return 0;//didnt bypass,abort
  }
  //Water Temp
  if (DS18B20_Init() || DS18B20_GetTemp_SkipRom()<70.0){
    if(trap("Water not hot enough!")) return 0;//didnt bypass,abort
  }
  if(pourWater()){
    if(notification("Cooking, Time remaining:","Abort","Skip",180)){
      return 1;
    }
  }
  return 0; //by whatever reason aborted
}
#ifdef FREERTOS_CONFIG_H
static void AppTaskCreate(void)      
{
  BaseType_t xReturn = pdPASS;
  
  taskENTER_CRITICAL();          
  
  
  xReturn = xTaskCreate((TaskFunction_t )mainScreen, 
                        (const char*    )"mainScreen",
                        (uint16_t       )1024,   
                        (void*          )NULL,	
                        (UBaseType_t    )2,	   
                        (TaskHandle_t*  )&mainScreen_Handle);
  if(pdPASS == xReturn)
    printf("mainScreen Create Success!\r\n");
  

  xReturn = xTaskCreate((TaskFunction_t )debugScreen,
                        (const char*    )"debugScreen",
                        (uint16_t       )512,   
                        (void*          )NULL,	
                        (UBaseType_t    )3,	    
                        (TaskHandle_t*  )&debugScreen_Handle);
  if(pdPASS == xReturn)
    printf("debugScreen Create Success!\r\n");
	
	  xReturn = xTaskCreate((TaskFunction_t )palette,
                        (const char*    )"palette",
                        (uint16_t       )512,   
                        (void*          )NULL,	
                        (UBaseType_t    )2,	    
                        (TaskHandle_t*  )&palette_Handle);
  if(pdPASS == xReturn)
    printf("palette Create Success!\r\n");
  
  vTaskDelete(AppTaskCreate_Handle); 
  
  taskEXIT_CRITICAL();            
}
#endif

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
