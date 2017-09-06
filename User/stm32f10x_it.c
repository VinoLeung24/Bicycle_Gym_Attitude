/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "AHRS_Attitude.h"
#include "AttitudeRedress.h"
#include "usart.h"

extern MPU mMPU_L;
extern MPU mMPU_R;
extern MPU mMPU_B;

uint8_t sys_count;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	sys_count++;
}

extern BICYCLE mBicycle;

void USART1_IRQHandler(void)
{
	static uint8_t model_flag;
	uint8_t data_rx;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		data_rx = USART_ReceiveData(USART1);

		if(model_flag)
		{
			model_flag = 0;
			switch(data_rx)
			{
				case '0':													//返回主界面 初始化  关闭led指示灯  关闭tim3 (心率)															
					mBicycle.outer_status = STATE_START_MODE;
//					printf("0");
				break;
				
				case '1':													//自由模式 打开tim3(心率)	//计时清零									
					mBicycle.outer_status = STATE_FREEDOM_MODE;
//					printf("1");
				break;
				
				case '2':													//自由模式  (平路模式) 打开tim3(心率)	
				    mBicycle.outer_status = STATE_SPORT_MODE_LEVEL;
//					printf("2");
				break;
				
				case '3':													//自由模式 (爬坡模式) 打开tim3(心率)			
					mBicycle.outer_status = STATE_SPORT_MODE_CLIMB;
//					printf("3");
				break;
				
				case '4':													//自由模式 (热身模式) 关闭tim3(心率)
					mBicycle.outer_status = STATE_SPORT_MODE_WARM1;
//					printf("4");
				break;
				
				case '5':													//自由模式 (热身模式) 关闭tim3(心率)
					mBicycle.outer_status = STATE_SPORT_MODE_WARM2;
//					printf("5");
				break;
				
				case '6':													//自由模式 (热身模式) 关闭tim3(心率)
					mBicycle.outer_status = STATE_SPORT_MODE_WARM3;										//计时清零				
//					printf("6");
				break;
				
				case '7':
                    mBicycle.outer_status = STATE_USER_REMENBER_DATA;
//					printf("7");
				break;
				
				case '8':													//动作扩展  请求背心发送 pitch roll yaw
					mBicycle.outer_status = STATE_USER_CHECK;
//					printf("8");
				break;
				
				case '9':													//校准yaw
					mBicycle.outer_status = STATE_CALIBRATION;
//					printf("9");
				break;
				
				default:
				break;
			}
		}
		
		if(data_rx == 'm')
			model_flag = 1;
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
