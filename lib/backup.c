static void getvalue(int task){
	int heig[6]={1,3,6,8,10,12};
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
    int Temperature = (int)DS18B20_GetTemp_SkipRom();
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
  }else if (task>=4 && task<=6){
    uint16_t sensorPins[3]={SENSOR_PIN_TRIGGER,SENSOR_PIN_PROX,SENSOR_PIN_LEVEL};
    uint8_t Pinstate=(HAL_GPIO_ReadPin(SENSOR_PORT_LEVEL,sensorPins[task-4])==GPIO_PIN_SET)?1:0;
    ILI9341_DispString_EN(14*WIDTH_EN_CHAR,heig[task-1]*HEIGHT_EN_CHAR,(Pinstate)?"On":"Off");
  }
}

static void debugScreen(void* Parameter){
  volatile uint32_t lastUpd=0;
  ILI9341_Clear(0,0,ILI9341_MORE_PIXEL,ILI9341_LESS_PIXEL);
  ILI9341_DispString_EN(14*WIDTH_EN_CHAR , 0*HEIGHT_EN_CHAR , "DEC");
	ILI9341_DispString_EN(20*WIDTH_EN_CHAR , 0*HEIGHT_EN_CHAR , "HEX");
	ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 1*HEIGHT_EN_CHAR ,"ADC1IN10_C0: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 3*HEIGHT_EN_CHAR ,"DS18B20_B15: ");
	ILI9341_DispString_EN(14*WIDTH_EN_CHAR , 5*HEIGHT_EN_CHAR , "X");
	ILI9341_DispString_EN(20*WIDTH_EN_CHAR , 5*HEIGHT_EN_CHAR , "Y");  
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 6*HEIGHT_EN_CHAR ,"LAST TOUCH: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 8*HEIGHT_EN_CHAR ,"Trigger_B13: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 10*HEIGHT_EN_CHAR ,"Prox_B12: ");
  ILI9341_DispString_EN(0*WIDTH_EN_CHAR , 12*HEIGHT_EN_CHAR ,"Level_B14: ");
  ILI9341_DrawRectangle(0,ILI9341_LESS_PIXEL-20,75,20,0);
  ILI9341_DispString_EN(1,ILI9341_LESS_PIXEL-19,"Cal Touch");
  ILI9341_DrawRectangle(75,ILI9341_LESS_PIXEL-20,84,20,0);
  ILI9341_DispString_EN(76,ILI9341_LESS_PIXEL-19,"Demo Mode");
  if(demoFlag)ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-10,4,1);
  while(1){
    if(HAL_GetTick()-lastUpd>500){
			lastUpd=HAL_GetTick();
      getvalue(1);
			getvalue(2);
      getvalue(4);
      getvalue(5);
      getvalue(6);
		}
    if(XPT2046_TouchDetect() == TOUCH_PRESSED) {
      getvalue(3);
      static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
      XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
      if(cinfo.x>0 &&cinfo.x<(0+75)&&cinfo.y>(ILI9341_LESS_PIXEL-20)&&cinfo.y<ILI9341_LESS_PIXEL){
        uint16_t bgclr=0,txclr=0;
        LCD_GetColors(&txclr,&bgclr);
        XPT2046_Touch_Calibrate(LCD_SCAN_MODE);
        LCD_SetColors(txclr,bgclr);
        debugScreen(NULL); //Cal TouchScreen
      } else if (cinfo.x>75 &&cinfo.x<(75+84)&&cinfo.y>(ILI9341_LESS_PIXEL-20)&&cinfo.y<ILI9341_LESS_PIXEL){
        demoFlag=!demoFlag;
        if(!demoFlag){
          uint16_t bgclr=0,txclr=0;
          LCD_GetColors(&txclr,&bgclr);
          LCD_SetTextColor(bgclr); //overwrite diag content with bgclr
          ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-10,4,1);
          LCD_SetTextColor(txclr);
        }else ILI9341_DrawCircle(152,ILI9341_LESS_PIXEL-10,4,1);
      }
    }
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL);
  }
}

static void palette(void* Parameter){
  Palette_Init(LCD_SCAN_MODE);
  while (1){
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL);
    XPT2046_TouchEvenHandler();
  }
}

static void ledUpdate(){
  Status_Flags[1]=(!DS18B20_Init() && DS18B20_GetTemp_SkipRom()>=80.0)?GPIO_PIN_SET:GPIO_PIN_RESET;
  Status_Flags[2]=(HAL_GPIO_ReadPin(SENSOR_PORT_LEVEL,SENSOR_PIN_LEVEL)==GPIO_PIN_SET)?GPIO_PIN_SET:GPIO_PIN_RESET;
  for (int i=0;i<NUM_LEDs;i++) HAL_GPIO_WritePin(LED_PORT,LED_PINs[i],Status_Flags[i]);
}

static uint8_t notification(char * Message,char* optionL,char* optionR, int time_s){
  HAL_Delay(200); //Avoid multiclick
  uint16_t diagStartY=(time_s<=0)?80:40;
  uint16_t diagHeight=(time_s<=0)?80:160;
  uint16_t optStartY=(time_s<=0)?130:170;
  ILI9341_DrawRectangle(60,diagStartY,200,diagHeight,0);
  ILI9341_DispString_EN(160-(strlen(Message)*WIDTH_EN_CHAR/2),diagStartY+20,Message);
  ILI9341_DrawRectangle(90,optStartY,50,20,0);
  ILI9341_DispString_EN(115-(strlen(optionL)*WIDTH_EN_CHAR/2),optStartY+1,optionL);
  ILI9341_DrawRectangle(180,optStartY,50,20,0);
  ILI9341_DispString_EN(205-(strlen(optionR)*WIDTH_EN_CHAR/2),optStartY+1,optionR);
  uint8_t ret=1;
  while(time_s<=0){                                 //noti waittime inf
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
    ILI9341_Clear(160-(4*2*WIDTH_EN_CHAR),90,(strlen(timeStr)+1)*4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR);
    ILI9341_DisplayStringEx(160-(strlen(timeStr)*4*WIDTH_EN_CHAR/2),90,4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR,timeStr,0);
    while (HAL_GetTick()-starting<1000){
      if(XPT2046_TouchDetect() == TOUCH_PRESSED){
        static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
        XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
        if(cinfo.x>90 &&cinfo.x<(90+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)){
          ret=0;
          break;
        } else if(cinfo.x>180 &&cinfo.x<(180+50)&&cinfo.y>optStartY&&cinfo.y<(optStartY+20)) break;//Interrrupt by option
      }
      HAL_Delay(10);
    }
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
  if(diagHeight>80)  ILI9341_DisplayStringEx(160-(strlen("9999")*4*WIDTH_EN_CHAR/2),90,4*WIDTH_EN_CHAR,4*HEIGHT_EN_CHAR,"9999",0);
  LCD_SetTextColor(txclr);//restore text clr
  return ret;
}

static uint8_t trap(char* Message){
  return notification (Message,"Bypass","OK",-1);
}

static void mainScreen(void* Parameter){
  ILI9341_Clear(0,0,ILI9341_MORE_PIXEL,ILI9341_LESS_PIXEL);
  
  if (notification("Hello? Continue Boot?","DevOps","Yes",3)){
    //normal mode
    Status_Flags[0]=1;
    while(1){
      ledUpdate();
      uint8_t triggerBypass=0;
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)mainScreen(NULL);
      if(demoFlag) {
        if(notification("Run Simulation","No","Yes",-1)) triggerBypass=1;
        else mainScreen(NULL);
      }
      if(HAL_GPIO_ReadPin(SENSOR_PORT_TRIGGER,SENSOR_PIN_TRIGGER)==GPIO_PIN_RESET||triggerBypass){
        if(startCook()){
        //noodle done!
        //push noti

        //buzzer
        
        //Screen Noti
        notification("Noodle Ready!","Close","OK",5);
        }else notification("Cooking Aborted!","Cancel","OK",-1);
      }
    }
  }else {
     
    if(notification("Dev Options","Debug","DwaR",-1))palette(NULL);
    else debugScreen(NULL);
  }
}



static uint8_t pourWater(){
  HAL_GPIO_WritePin(PUMP_PORT,PUMP_PIN,GPIO_PIN_SET);
  uint8_t ret = notification("Pouring water, time remaining:","Abort","Skip",20);
  HAL_GPIO_WritePin(PUMP_PORT,PUMP_PIN,GPIO_PIN_RESET);
  return ret;
}

static uint8_t startCook(){
  //foolproof
  if (HAL_GPIO_ReadPin(SENSOR_PORT_PROX,SENSOR_PIN_PROX)==GPIO_PIN_SET){
    if(trap("This is not a Cup Noodle!")) return 0; //didnt bypass,abort
  }
  //Water Temp
  if (DS18B20_Init() || DS18B20_GetTemp_SkipRom()<80.0){
    if(trap("Water not hot enough!")) return 0;//didnt bypass,abort
  }
  //Water Level
  if (HAL_GPIO_ReadPin(SENSOR_PORT_LEVEL,SENSOR_PIN_LEVEL)==GPIO_PIN_SET){
    if(trap("No Water!")) return 0;//didnt bypass,abort
  }
  if(pourWater()){
    if(notification("Cooking, Time remaining:","Abort","Skip",180)){
      return 1;
    }
  }
  return 0; //by whatever reason aborted
}