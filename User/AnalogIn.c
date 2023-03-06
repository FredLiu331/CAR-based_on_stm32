#include "AnalogIn.h"
__IO uint16_t uhADCConvertedValue[ADC_Channel_Num*ADC_Channel_SampleCount];
void ADC_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
    
  /* Enable peripheral clocks *************************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);

  /* DMA2_Stream0 channel0 configuration **************************************/
  DMA_DeInit(DMA2_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_CHANNELx;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADCx_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC_Channel_Num*ADC_Channel_SampleCount;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA_STREAMx, &DMA_InitStructure);
  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA_STREAMx, ENABLE);
	
	/* 引脚配置，PC0/PC1/PC2/PC3/PC4/PC5/PA4/PA5/PB0/PB1*/
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* 引脚配置，PC0/PC1/PC2/PC3/PC4/PC5/PA4/PA5/PB0/PB1*/	
	
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = ADC_Channel_Num;
  ADC_Init(ADCx, &ADC_InitStructure);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADCx, ENABLE);
  
  /* ADC1 转换通道，注意和总的通道数要对应 ******************************/
	ADC_RegularChannelConfig(ADCx, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);  //第0个对应PC1
	ADC_RegularChannelConfig(ADCx, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);	//PC2
	ADC_RegularChannelConfig(ADCx, ADC_Channel_13, 3, ADC_SampleTime_15Cycles);	//PC3
	ADC_RegularChannelConfig(ADCx, ADC_Channel_4, 4, ADC_SampleTime_15Cycles);	//PA4
	ADC_RegularChannelConfig(ADCx, ADC_Channel_5, 5, ADC_SampleTime_15Cycles);	//PA5
	ADC_RegularChannelConfig(ADCx, ADC_Channel_14, 6, ADC_SampleTime_15Cycles);	//PC4
	ADC_RegularChannelConfig(ADCx, ADC_Channel_15, 7, ADC_SampleTime_15Cycles);	//PC5
	ADC_RegularChannelConfig(ADCx, ADC_Channel_8, 8, ADC_SampleTime_15Cycles);	//PB0
	ADC_RegularChannelConfig(ADCx, ADC_Channel_9, 9, ADC_SampleTime_15Cycles);	//PB1
  ADC_RegularChannelConfig(ADCx, ADC_Channel_10, 10, ADC_SampleTime_15Cycles);	//PC0，电池电压采样

  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);

  /* Enable ADC1 **************************************************************/
  ADC_Cmd(ADCx, ENABLE);
}

uint16_t ADC_GetPowerVoltage(void)
{
	uint8_t i;
	uint32_t tmp;
	tmp = 0;
	for(i=0;i<ADC_Channel_SampleCount;i++)
	{
		tmp += uhADCConvertedValue[i*ADC_Channel_Num+9];  //第9个对应PC0，是电池电压
	}
	
	tmp = tmp*3300/(0xFFF*ADC_Channel_SampleCount);  //取平均值，并转换成电压（毫伏）
	tmp = tmp*11;  //乘以分压系数。
	return tmp;
}
