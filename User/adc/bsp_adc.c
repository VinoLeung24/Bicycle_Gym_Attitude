#include "bsp_adc.h"

/*12位AD ，由于心率信号采集只需要10位精度，所以AD采样值右移2位
 * 数字量化范围0-1024
 */

/**
  * @brief  ADC GPIO 初始化
  * @param  无
  * @retval 无
  */
static void ADC2_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 打开 ADC IO端口时钟
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE );
	
	// 配置 ADC IO 引脚模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	// 初始化 ADC IO
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
}

/**
  * @brief  配置ADC工作模式
  * @param  无
  * @retval 无
  */
static void ADC2_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// 打开ADC时钟
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_ADC2, ENABLE );
	
	// 配置ADC时钟Ｎ狿CLK2的8分频，即9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	// ADC 模式配置
	// 只使用一个ADC，属于单模式
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	// 禁止扫描模式，多通道才要，单通道不需要
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// 单次转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;//选择TIM3作为外部触发源

	// 转换结果右对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// 转换通道1个
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
		
	// 初始化ADC
	ADC_Init(ADC2, &ADC_InitStructure);
	

	
	//采用外部触发
	ADC_ExternalTrigConvCmd(ADC2,ENABLE);
	
	// 配置 ADC 通道转换顺序为1，第一个转换，采样时间为239.5个时钟周期   ADC转换时间21us
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	
	// ADC 转换结束产生中断，在中断服务程序中读取转换值
	//ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	
	// 开启ADC ，并开始转换
	ADC_Cmd(ADC2, ENABLE);
	
	// 初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC2);
	// 等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC2));
	
	// ADC开始校准
	ADC_StartCalibration(ADC2);
	// 等待校准完成
	while(ADC_GetCalibrationStatus(ADC2));
	
	// 由于没有采用外部触发，所以使用软件触发ADC转换 
	//ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}
/*
static void ADC_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // 配置中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
*/

/**
  * @brief  ADC初始化
  * @param  无
  * @retval 无
  */
void ADC2_Init(void)
{
	ADC2_GPIO_Config();
	ADC2_Mode_Config();
	//ADC_NVIC_Config();
}
/*********************************************END OF FILE**********************/
