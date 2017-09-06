#include "bsp_adc.h"

/*12λAD �����������źŲɼ�ֻ��Ҫ10λ���ȣ�����AD����ֵ����2λ
 * ����������Χ0-1024
 */

/**
  * @brief  ADC GPIO ��ʼ��
  * @param  ��
  * @retval ��
  */
static void ADC2_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// �� ADC IO�˿�ʱ��
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE );
	
	// ���� ADC IO ����ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	// ��ʼ�� ADC IO
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
}

/**
  * @brief  ����ADC����ģʽ
  * @param  ��
  * @retval ��
  */
static void ADC2_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	

	// ��ADCʱ��
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_ADC2, ENABLE );
	
	// ����ADCʱ�ӣΪPCLK2��8��Ƶ����9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	
	// ADC ģʽ����
	// ֻʹ��һ��ADC�����ڵ�ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	// ��ֹɨ��ģʽ����ͨ����Ҫ����ͨ������Ҫ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// ����ת��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;//ѡ��TIM3��Ϊ�ⲿ����Դ

	// ת������Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// ת��ͨ��1��
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
		
	// ��ʼ��ADC
	ADC_Init(ADC2, &ADC_InitStructure);
	

	
	//�����ⲿ����
	ADC_ExternalTrigConvCmd(ADC2,ENABLE);
	
	// ���� ADC ͨ��ת��˳��Ϊ1����һ��ת��������ʱ��Ϊ239.5��ʱ������   ADCת��ʱ��21us
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	
	// ADC ת�����������жϣ����жϷ�������ж�ȡת��ֵ
	//ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	
	// ����ADC ������ʼת��
	ADC_Cmd(ADC2, ENABLE);
	
	// ��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADC2);
	// �ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADC2));
	
	// ADC��ʼУ׼
	ADC_StartCalibration(ADC2);
	// �ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADC2));
	
	// ����û�в����ⲿ����������ʹ���������ADCת�� 
	//ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}
/*
static void ADC_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // �����ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
*/

/**
  * @brief  ADC��ʼ��
  * @param  ��
  * @retval ��
  */
void ADC2_Init(void)
{
	ADC2_GPIO_Config();
	ADC2_Mode_Config();
	//ADC_NVIC_Config();
}
/*********************************************END OF FILE**********************/
