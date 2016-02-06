/*
 * hw_init.c
 *
 *  Created on: Aug 22, 2014
 *      Author: mobintu
 */

#include "hw_init.h"
#define ADC2
#ifdef ADC0
#define ADC_WDG_H 2048
#define ADC_WDG_L 1800
#else
#define ADC_WDG_H 1024
#define ADC_WDG_L 900
#endif


void hw_init(void){


	/*
	 * TIM2 CCR is used for bridge pulse updates
	 * ADC1_COMP Irq is for analog watchdog function. Every event causes the update of CCR, and frequency value
	 */
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	// MOTOR CONTROL GPIO / PWM CONFIG
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);



	// Configure Pins for UP fets
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //should be faster than OD, go up to 3.3V pushing, then up to 5 with external
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	//AU(1);

	// 1 keeps Upper PMOS off
	AU(1);
	BU(1);
	CU(1);

	// Configure Pins for DOWN fets
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	// Configure Pins for Analog In (Hall,V,I) I
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_Init(GPIOA,&GPIO_InitStructure);


	//Init TIMER1 for PWM Out
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period=M_PWM_TickPeriod-1;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_2);



	//Setup 32bit TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period=0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse=48*10000; //10ms
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Disable);
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // well it doesn't have minus ones..!
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM2, ENABLE);

	//---------------------------------------
	//I2C  - PB6/PB7
	//RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_4);
	//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10, GPIO_AF_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);



	// Analog Init
	ADC_InitTypeDef     ADC_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADbufSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);


	ADC_DeInit(ADC1);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ClockModeConfig(ADC1,ADC_ClockMode_AsynClk);
	ADC_ChannelConfig(ADC1, ADC_Channel_2 , ADC_SampleTime_71_5Cycles);
	ADC_GetCalibrationFactor(ADC1);

	/*Configure Half anf Full  transmision irq to keep ADC scanning*/
	DMA_ITConfig(DMA1_Channel1,DMA_IT_HT|DMA_IT_TC,ENABLE);
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
	DMA_ClearITPendingBit(DMA1_IT_GL1);
	//ADC_DMACmd(ADC1, ENABLE);

	ADC_AnalogWatchdogThresholdsConfig(ADC1,ADC_WDG_H,0);//> should wait for real ADmean first
	ADC_AnalogWatchdogSingleChannelConfig(ADC1,ADC_AnalogWatchdog_Channel_2);
	ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
	ADC_ITConfig(ADC1,ADC_IT_AWD,ENABLE);
	ADC_AnalogWatchdogCmd(ADC1,ENABLE);





	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=1;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
	ADC_StartOfConversion(ADC1);
	//ADmean=2048;





	//	#define DBG
#ifdef DBG
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,ENABLE);
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period=4096;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse=10;
	TIM_OC1Init(TIM17,&TIM_OCInitStructure);

	TIM_Cmd(TIM17, ENABLE);
	TIM_CtrlPWMOutputs(TIM17, ENABLE);
	//GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_2);
#endif
	/*

	// ----------------I2C-----------------------------------
	// For discovery I2C1, PB6/SCL, PB7/SDA spoils MOTOR4!
	// PA9/SCL, PA10/SDA
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10, GPIO_AF_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_Own_Addr;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_10bit;
	I2C_InitStructure.I2C_Timing =  0xB042C3C7;
	//0xPRESC4b | RES4b | SCLDEL4b | SDADEL4b || SCLH8b | SCLL8b
	//	0xB 0 4 2 C3 C7 10kHz @48MHz
	//	0xB 0 4 2 0F 13 100kHz @48MHz
	//	0x5 0 3 3 03 09 400kHz @48MHz
	//	0x5 0 1 0 01 03 1000kHz @48MHz

	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_StretchClockCmd(I2C1,DISABLE);
	I2C_GeneralCallCmd(I2C1,ENABLE);
	I2C_ITConfig(I2C1,I2C_IT_ADDRI,ENABLE);
	I2C_ITConfig(I2C1,I2C_IT_RXI,ENABLE);
	//I2C_CalculatePEC(I2C1,ENABLE);
	//I2C_PECRequestCmd(I2C1,ENABLE);

	// Enable CRC AHB clock interface
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	CRC_DeInit();
	CRC_ReverseOutputDataCmd(DISABLE);

	I2C_Cmd(I2C1, ENABLE);
	 */



}

void ADC1_COMP_IRQHandler(void){
	volatile static uint16_t adc,adc_thd=ADC_WDG_H;
	volatile static uint32_t t=0,dt,t1,CCRtmp,locked=0;

	t1=toc;
	int i=2;

	if(ADC_GetITStatus(ADC1, ADC_IT_AWD) != RESET){
		ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);


		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		adc=ADC1->DR;
		adc=0;
		while(i--){
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
			adc+=ADC1->DR;
		}
		adc=adc/2;

		if(adc>adc_thd){
			adc_thd=ADC_WDG_L;
			ADC_AnalogWatchdogThresholdsConfig(ADC1,4095,ADC_WDG_H+50);
			ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
		}
		else{
			dt=t1-t;
			t=t1;
			CCRtmp=(250*CCRtmp+6*(dt)/6)/256; //6 steps per Electrical cycle,
			//CCRtmp=(dt)/6; //6 steps per Electrical cycle,
			Led1(1);
			Led1(0);

			if(6*CCRtmp>(RPM2CNT(RPMr-5))){
				locked=0;
				CCR1r=CCRtmp;//-100;
				//Pstate=1;//1180Hz@3V
			 	TIM2->CCR1=toc+10; //1080Hz@3V , more stable
				Pstate=6;TIM2->CCR1=toc+20; //1220Hz@3V, 1700Hz@4.2
				Led1(0);
			}
			else{
				locked=1;
				Led1(1);
				//Pstate=6;
				//TIM2->CCR1=toc+20;
				CCR1r=RPM2CNT(RPMr)/6;
				//CCR1r+=200;
			}
			//	ADC_AnalogWatchdogCmd(ADC1,DISABLE);
			ADC_AnalogWatchdogThresholdsConfig(ADC1,ADC_WDG_H,0);
			ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
			//	ADC_AnalogWatchdogCmd(ADC1,ENABLE);
			//	TIM2->CCR1=toc+100;
			//	Pstate=4;
		}
		//Led2(0);
	}
}

void DMA1_Channel1_IRQHandler(void){
	uint32_t ADsum=0;
	uint32_t i;
	if(DMA_GetITStatus(DMA1_IT_HT1)){
		DMA_ClearITPendingBit(DMA1_IT_HT1);
		i=ADbufSize/2-1;
		ADsum=0;
		while(i--) //takes ~82us for 500smp
			ADsum+=ADbuff[i];
		//ADmean=(1000*ADmean+24*ADsum/(ADbufSize/2))/1024;
	}
	if(DMA_GetITStatus(DMA1_IT_TC1)){
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		i=ADbufSize/2;
		ADsum=0;
		while(i<ADbufSize)
			ADsum+=ADbuff[i++];
		//ADmean=(1000*ADmean+24*ADsum/(ADbufSize/2))/1024;
	}
}

void TIM2_IRQHandler(void){

	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET){
		TIM2->SR = (uint16_t)~TIM_IT_CC1; //CLEAR FLAG
		TIM2->CCR1+=CCR1r;
		//TIM2->CCR1=toc+CCR1r;
		switch (Pstate){

		// -----------------PMOS LOGIC-------------------
		//!!!  AU(0) will turn the UP PMOS ON.. obviously
		// -----------------------------------------------
		case 1:
			Led2(1);
			CU(1);
			BU(1);
			AD(0);
			CD(0);

			AU(0);
			BD(PWMval);
			Pstate++;
			delay_us(1);
			Led2(0);

			break;
		case 2:
			Led2(1);
			AD(0);
			BD(0);
			BU(1);
			CU(1);

			AU(0);
			CD(PWMval);
			Pstate++;
			delay_us(1);
			Led2(0);

			break;
		case 3:
			Led2(1);
			AD(0);
			BD(0);
			AU(1);
			CU(1);

			BU(0);
			CD(PWMval);
			Pstate++;
			delay_us(1);
			Led2(0);
			//delay_us(2); //stay away from noise...
			//ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
			//ADC_ITConfig(ADC1,ADC_IT_AWD,ENABLE);
			break;
		case 4:
			Led2(1);
			BD(0);
			CD(0);
			AU(1);
			CU(1);

			BU(0);
			AD(PWMval);
			Pstate++;
			delay_us(1);
			Led2(0);
			break;
		case 5:
			Led2(1);
			BD(0);
			CD(0);
			AU(1);
			BU(1);

			CU(0);
			AD(PWMval);
			Pstate++;
			delay_us(1);
			Led2(0);
			break;
		case 6:
			Led2(1);
			AD(0);
			CD(0);
			AU(1);
			BU(1);

			CU(0);
			BD(PWMval);
			Pstate=1;
			delay_us(1);
			Led2(0);
			//	delay_us(20); //stay away from noise...
			//	ADC_ClearITPendingBit(ADC1,ADC_IT_AWD);
			//	ADC_AnalogWatchdogCmd(ADC1,ENABLE);
			break;
		}
		//DCnt=CCR1_INC-RPM2CNT(CNT2RPM((float)CCR1_INC)*1.001)+1;
		//DCnt=(float)(CCR1_INC/1.0e5)*(float)(CCR1_INC/1.0e5);

	}
}


void delay_us(uint32_t delay){
	volatile uint32_t start=toc;
	delay=48*delay;
	while((toc-start)<delay);
}

