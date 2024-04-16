#include "My_ADC.h"
int16 diangan[4];
void AdcInit()
{
    adc_init(ADC_P00, ADC_SYSclk_DIV_2);	//初始化ADC,P1.0通道 ，ADC时钟频率：SYSclk/2
    adc_init(ADC_P01, ADC_SYSclk_DIV_2);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2
    adc_init(ADC_P05, ADC_SYSclk_DIV_2);	//初始化ADC,P1.2通道 ，ADC时钟频率：SYSclk/2
    adc_init(ADC_P06, ADC_SYSclk_DIV_2);	//初始化ADC,P1.2通道 ，ADC时钟频率：SYSclk/2
}
void AdcRead()
{
    diangan[0] =  adc_once(ADC_P00, ADC_12BIT);
    diangan[1] =  adc_once(ADC_P01, ADC_12BIT);
    diangan[2] =  adc_once(ADC_P05, ADC_12BIT);
    diangan[3] =  adc_once(ADC_P06, ADC_12BIT);
    
}
