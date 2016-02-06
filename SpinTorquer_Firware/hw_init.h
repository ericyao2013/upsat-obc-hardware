/*
 * hw_init.h
 *
 *  Created on: Aug 22, 2014
 *      Author: mobintu
 */

#ifndef HW_INIT_H_
#define HW_INIT_H_

#include "stm32f0xx.h"
#include "math.h"

//PA10/T1C3
#define AU(x) 	    ((x)==0)?(GPIOA->BRR=GPIO_Pin_10):(GPIOA->BSRR=GPIO_Pin_10)
//PA9/T1C2
#define BU(x) 	    ((x)==0)?(GPIOA->BRR=GPIO_Pin_9):(GPIOA->BSRR=GPIO_Pin_9)
//PA8/T1C1
#define CU(x) 	    ((x)==0)?(GPIOA->BRR=GPIO_Pin_8):(GPIOA->BSRR=GPIO_Pin_8)

//PB1/T1C3N
#define AD(x)		TIM1->CCR3=(x)
//PB0/T1C2N
#define BD(x)		TIM1->CCR2=(x)
//PA7/T1C1N
#define CD(x)		TIM1->CCR1=(x)


#define toc		TIM2->CNT

#define CNT2RPM(x) 			48000000*15/(x) //CNT in 1/48 us, 4 pairs of poles
#define RPM2CNT(x)			CNT2RPM(x)
#define dt_min RPM2CNT(35000)	//
#define dt_max RPM2CNT(1) 	//



volatile int32_t avance;
volatile uint32_t PWMval;
volatile uint32_t RPMr;
volatile  uint32_t CCR1r;
volatile uint32_t Pstate;

#define			ADbufSize 1000
volatile uint16_t ADbuff[ADbufSize];
//volatile uint32_t ADmean;

#define Led1(x) ((x)==0)?(GPIOB->BRR=GPIO_Pin_6):(GPIOB->BSRR=GPIO_Pin_6)
#define Led2(x) ((x)==0)?(GPIOB->BRR=GPIO_Pin_7):(GPIOB->BSRR=GPIO_Pin_7)



#define M_PWM_TickPeriod 1000 //used -1 for period

//#ifndef I2C_Address
#define I2C_Address 3
//#endif


// USE TYPEDEF ENUM ->array!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//ENABLE Motor 1, PA7 - T17C1
#define M1_PWM_PORT 		GPIOA
#define M1_PWM_PIN			GPIO_Pin_7
#define M1_PWM_CLOCK		RCC_AHBPeriph_GPIOA
#define M1_TIMER			TIM17
#define M1_TIMER_CLOCK(x)	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,x);
#define M1_PWM_UPD(x)		TIM_SetCompare1(M1_TIMER,(x>M_PWM_TickPeriod)?(M_PWM_TickPeriod):(x) )
#define	M1_PWM_Init			TIM_OC1Init
#define M1_PWM_AFset		GPIO_PinAFConfig(M1_PWM_PORT, GPIO_PinSource7, GPIO_AF_5);
//Phase Motor 1, PB0
#define M1_PH_PORT 			GPIOB
#define M1_PH_PIN			GPIO_Pin_0
#define M1_PH_CLOCK			RCC_AHBPeriph_GPIOB
#define M1_PH_SET(x) 	    (x==0)?(M1_PH_PORT->BRR=M1_PH_PIN):(M1_PH_PORT->BSRR=M1_PH_PIN)
//ENCODER PIN, M1, PB1
#define M1_ENC_PORT 		GPIOB
#define M1_ENC_PIN			GPIO_Pin_1
#define M1_ENC_CLOCK		RCC_AHBPeriph_GPIOB
#define M1_ENC_EXTI_PORT 	EXTI_PortSourceGPIOB
#define M1_ENC_EXTI_PIN 	EXTI_PinSource1
#define M1_ENC_EXTI_LINE 	EXTI_Line1
#define M1_ENC_IRQ 			EXTI0_1_IRQn

//#define M1_ENC_SET(x) 	    (x==0)?(M1_PH_PORT->BRR=M1_PH_PIN):(M1_PH_PORT->BSRR=M1_PH_PIN);


//ENABLE Motor 2, PA3 - T2C4
#define M2_PWM_PORT 		GPIOA
#define M2_PWM_PIN			GPIO_Pin_3
#define M2_PWM_CLOCK		RCC_AHBPeriph_GPIOA
#define M2_TIMER			TIM2
#define M2_TIMER_CLOCK(x)	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,x);
#define M2_PWM_UPD(x)		TIM_SetCompare4(M2_TIMER,(x>M_PWM_TickPeriod)?(M_PWM_TickPeriod):(x))
#define	M2_PWM_Init			TIM_OC4Init
#define M2_PWM_AFset		GPIO_PinAFConfig(M2_PWM_PORT, GPIO_PinSource3, GPIO_AF_2);
//Phase Motor 2, PA4
#define M2_PH_PORT 			GPIOA
#define M2_PH_PIN			GPIO_Pin_4
#define M2_PH_CLOCK			RCC_AHBPeriph_GPIOA
#define M2_PH_SET(x) 	    (x==0)?(M2_PH_PORT->BRR=M2_PH_PIN):(M2_PH_PORT->BSRR=M2_PH_PIN)
//ENCODER PIN, M2, PA2
#define M2_ENC_PORT 		GPIOA
#define M2_ENC_PIN			GPIO_Pin_2
#define M2_ENC_CLOCK		RCC_AHBPeriph_GPIOA
#define M2_ENC_EXTI_PORT 	EXTI_PortSourceGPIOA
#define M2_ENC_EXTI_PIN 	EXTI_PinSource2
#define M2_ENC_EXTI_LINE 	EXTI_Line2
#define M2_ENC_IRQ 			EXTI2_3_IRQn



//ENABLE Motor 3, PB3 - T2C2
#define M3_PWM_PORT 		GPIOB
#define M3_PWM_PIN			GPIO_Pin_3
#define M3_PWM_CLOCK		RCC_AHBPeriph_GPIOB
#define M3_TIMER			TIM2
#define M3_TIMER_CLOCK(x)	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,x);
#define M3_PWM_UPD(x)		TIM_SetCompare2(M3_TIMER,(x>M_PWM_TickPeriod)?(M_PWM_TickPeriod):(x))
#define	M3_PWM_Init			TIM_OC2Init
#define M3_PWM_AFset		GPIO_PinAFConfig(M3_PWM_PORT, GPIO_PinSource3, GPIO_AF_2);
//Phase Motor 3, PB4
#define M3_PH_PORT 			GPIOB
#define M3_PH_PIN			GPIO_Pin_4
#define M3_PH_CLOCK			RCC_AHBPeriph_GPIOB
#define M3_PH_SET(x) 	    (x==0)?(M3_PH_PORT->BRR=M3_PH_PIN):(M3_PH_PORT->BSRR=M3_PH_PIN)
//ENCODER PIN, M3, PA8
#define M3_ENC_PORT 		GPIOA
#define M3_ENC_PIN			GPIO_Pin_8
#define M3_ENC_CLOCK		RCC_AHBPeriph_GPIOA
#define M3_ENC_EXTI_PORT 	EXTI_PortSourceGPIOA
#define M3_ENC_EXTI_PIN 	EXTI_PinSource8
#define M3_ENC_EXTI_LINE 	EXTI_Line8
#define M3_ENC_IRQ 			EXTI4_15_IRQn


//ENABLE Motor 4, PB5 - T3C2
#define M4_PWM_PORT 		GPIOB
#define M4_PWM_PIN			GPIO_Pin_5
#define M4_PWM_CLOCK		RCC_AHBPeriph_GPIOB
#define M4_TIMER			TIM3
#define M4_TIMER_CLOCK(x)	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,x);
#define M4_PWM_UPD(x)		TIM_SetCompare2(M4_TIMER,(x>M_PWM_TickPeriod)?(M_PWM_TickPeriod):(x))
#define	M4_PWM_Init			TIM_OC2Init
#define M4_PWM_AFset		GPIO_PinAFConfig(M4_PWM_PORT, GPIO_PinSource5, GPIO_AF_1);
//Phase Motor 4, PB6
#define M4_PH_PORT 			GPIOB
#define M4_PH_PIN			GPIO_Pin_6
#define M4_PH_CLOCK			RCC_AHBPeriph_GPIOB
#define M4_PH_SET(x) 	    (x==0)?(M4_PH_PORT->BRR=M4_PH_PIN):(M4_PH_PORT->BSRR=M4_PH_PIN)
//ENCODER PIN, M1, PB7
#define M4_ENC_PORT 		GPIOB
#define M4_ENC_PIN			GPIO_Pin_7
#define M4_ENC_CLOCK		RCC_AHBPeriph_GPIOB
#define M4_ENC_EXTI_PORT 	EXTI_PortSourceGPIOB
#define M4_ENC_EXTI_PIN 	EXTI_PinSource7
#define M4_ENC_EXTI_LINE 	EXTI_Line7
#define M4_ENC_IRQ 			EXTI4_15_IRQn

// UPD FUN
#define SATUR(x,Lim) (x>Lim)?(Lim):((x<-Lim)?(-Lim):x)
#define SATUR2(x,Low,High) (x>High)?(High):((x<Low)?(Low):x)

#define M1_UPD(x) (x<0)?(M1_PH_SET(0),M1_PWM_UPD(-x)):(M1_PH_SET(1),M1_PWM_UPD(x))
#define M2_UPD(x) (x<0)?(M2_PH_SET(0),M2_PWM_UPD(-x)):(M2_PH_SET(1),M2_PWM_UPD(x))
#define M3_UPD(x) (x<0)?(M3_PH_SET(0),M3_PWM_UPD(-x)):(M3_PH_SET(1),M3_PWM_UPD(x))
#define M4_UPD(x) (x<0)?(M4_PH_SET(0),M4_PWM_UPD(-x)):(M4_PH_SET(1),M4_PWM_UPD(x))


#define Pck_Flag_NewAndReady 	1<<0
#define Pck_Flag_Time 			1<<1
#define Pck_Flag_PWM 			1<<2
#define Pck_Flag_Speed 			1<<3
#define Pck_Flag_Cnt 			1<<4
#define Pck_Flag_StartPWM		1<<5
#define Pck_Flag_StartSpeed		1<<6
#define Pck_Flag_StartCnt		1<<7

//
#define Pck_Payload 		2*4
#define I2C_Idle_Mode 0
#define I2C_GC_Mode 1


#define I2C_Own_Addr I2C_Address //10bit
#define I2C_Own_Pck Pck_Payload*(I2C_Own_Addr-2)+2 //10bit

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef struct {
	uint16_t 	time[4]; //Operation time in ms
	int16_t 	pwm[4];
	int16_t 	speed[4];
	int16_t 	cnt[4];
	uint16_t 	flag;  // keep last for aligning..
}I2C_pck;
I2C_pck global_pck;

struct M_ENC_pck{
	volatile uint32_t Cnt[4];
	volatile uint32_t dt[4];
	volatile uint32_t t0[4];
}M_ENC;



void delay_us(uint32_t delay);
uint32_t getTime_us(void);
void processEXTI(void);
#endif /* HW_INIT_H_ */
