/**
 ******************************************************************************
 * @file    Project/STM32F0xx_StdPeriph_Templates/main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    31-July-2013
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
//#define USE_STDPERIPH_DRIVER=1  //Just for Eclipse Indexer, also defined in makefile for build
// Upd. define it under Path and Symbols

#include "hw_init.h"
#include <stdio.h>

/** @addtogroup STM32F0xx_StdPeriph_Templates
 * @{
 */




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
	 */
	//-------------------------------
	Pstate=1;

	PWMval=1000;
	avance=0;
	RPMr=20000;
	CCR1r=RPM2CNT(RPMr);

	hw_init();
	//SysTick_Config(SystemCoreClock/1e3); //ms timer
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	/* Infinite loop */
	uint32_t start,adc,adc_sum=2048;
	volatile uint32_t tt;
	start=toc;
	while (1)
	{
	}
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void){// move the main loop to main to avoid preemption..
}


void I2C1_IRQHandler(void){

	static int16_t cnt=0;
	static int16_t I2C_state=I2C_Idle_Mode;
	__IO uint8_t data;
	static uint8_t *ptr;
	static uint8_t pck_len;
	static uint8_t CRCdat[4],CRCind;
	uint32_t a;


	if(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR)){

		if(I2C_GetAddressMatched(I2C1)==0x00){ //General Call
			if(I2C_GetTransferDirection(I2C1)==I2C_Direction_Transmitter){// means Master transmits..
				cnt=0;
				pck_len=0xFF;
				I2C_state=I2C_GC_Mode;
				CRC_ResetDR();
				CRCind=0;

			}
		}
		I2C_ClearITPendingBit(I2C1,I2C_IT_ADDR);
	}
	if(I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE)){
		data=I2C_ReceiveData(I2C1); //must read always to reset RX flag
		if (cnt<pck_len-1){
			CRCdat[CRCind++]=data;
			if(CRCind==4){
				CRCind=0;
				CRC_CalcCRC(*((uint32_t *)CRCdat));
			}
		}

		if(I2C_state==I2C_GC_Mode && !(global_pck.flag & Pck_Flag_NewAndReady)){ //Pck_Flag_NewAndReady must be zerod by mainloop/init
			if(cnt==0){
				global_pck.flag=data&(~Pck_Flag_NewAndReady);//Pck_Flag_NewAndReady force RESERVED..
				switch (global_pck.flag){
				case Pck_Flag_Time:
					ptr=(uint8_t *)&global_pck.time;
					break;
				case Pck_Flag_PWM:
					ptr=(uint8_t *)&global_pck.pwm;
					break;
				case Pck_Flag_Cnt:
					ptr=(uint8_t *)&global_pck.cnt;
					break;
				case Pck_Flag_Speed:
					ptr=(uint8_t *)&global_pck.speed;
					break;
				default:
					ptr=NULL;
					break;
				}
			}else{
				if(cnt==1){
					pck_len=data;
				}
				else{
					if(cnt>=I2C_Own_Pck && cnt<(I2C_Own_Pck+Pck_Payload)){
						if(ptr!=NULL)
							*ptr++=data;
					}
					else{
						if(cnt==pck_len-1){
							if(CRCind>0){//zero padding if any left
								while(CRCind<4)
									CRCdat[CRCind++]=0;
								CRC_CalcCRC(*((uint32_t *)CRCdat));
							}
							if((uint8_t)CRC->DR==data){
								global_pck.flag|=Pck_Flag_NewAndReady;
							}
						}
					}
				}
			}
		}
		cnt++;
	}



}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
