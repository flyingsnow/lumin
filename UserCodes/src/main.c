/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V3.0.0
* Date               : 04/06/2009
* Description        : Custom HID demo main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_lib.h"
#include "hw_config.h"
#include <math.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SENSE_THRESHOLD 150
#define TRACK_THRESHOLD 230
#define DEBOUNCE_CYCLES 30
#define START_TRACKING_CYCLES 180
#define END_TRACKING_CYCLES 210
#define HAND_MINIMUM_CHANGE 15


/* Private macro -------------------------------------------------------------*/
#define ABS(X)  ((X) > 0 ? (X) : -(X))    
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

uint16_t adc;
uint32_t sum;

ADC_InitTypeDef ADC_InitStructure;

uint16_t adc_input = 0;
uint16_t pwm_output = 0;
uint16_t stored_bright = 0xFFF;
uint16_t target_bright = 0;
uint16_t hand_tracked_bright = 0;

bool lamp_lighted = FALSE;
bool hand_tracking = FALSE;

uint8_t hand_cycles = 0;
uint8_t debounce_cycles = 0;

uint16_t sample[5];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);

/* Private functions ---------------------------------------------------------*/
void loop(void)
{
   uint8_t i = 0;
   adc_input = 0;
    while(i++ < 5) {
        /* Start ADC1 Software Conversion */ 
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET); //检查制定ADC标志位置1与否 ADC_FLAG_EOC 转换结束标志位 
      sample[i] = ADC_GetConversionValue(ADC1);
      adc_input +=  sample[i];
    }
    adc_input = adc_input >> 6;

  if(debounce_cycles)
    debounce_cycles -= 1;
  else {
    if(hand_tracking) {
      if(adc_input > SENSE_THRESHOLD) {        
        if(adc_input > TRACK_THRESHOLD + 0xFF)
          hand_tracked_bright = 0;
        else if(adc_input < TRACK_THRESHOLD)
          hand_tracked_bright = 0xfFFf;
        else
          hand_tracked_bright = 0xFFFF - (adc_input - TRACK_THRESHOLD);

        if(ABS(target_bright - hand_tracked_bright) > HAND_MINIMUM_CHANGE || !lamp_lighted) {
          target_bright = (target_bright + (hand_tracked_bright > 8 ? hand_tracked_bright : 8)) >> 1;
          lamp_lighted = TRUE;
        }
        hand_cycles = 0;
      }
      else {
        target_bright = pwm_output;
        stored_bright = pwm_output;
        hand_cycles += 1;
        if(hand_cycles == END_TRACKING_CYCLES) {
          hand_tracking = FALSE;
          hand_cycles = 0;
        }
      }      
    }
    else {
      if(adc_input > SENSE_THRESHOLD) {
        hand_cycles += 1;
        if(hand_cycles == START_TRACKING_CYCLES) {
          hand_tracking = TRUE;
          hand_cycles = 0;
        }
      }
      else {
        if(hand_cycles) {
          if(lamp_lighted ==  TRUE)
            lamp_lighted = FALSE;
          else 
            lamp_lighted = TRUE;
          target_bright = lamp_lighted ? stored_bright : 0;
          debounce_cycles = DEBOUNCE_CYCLES;
        }
        hand_cycles = 0;
      }
    }
  }

  if(pwm_output != target_bright) {
    if(pwm_output > target_bright && pwm_output > 0) --pwm_output;
    if(pwm_output < target_bright && pwm_output < 0xFFFF) ++pwm_output;
    TIM_OCInitStructure.TIM_Pulse = pwm_output ;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    Delay((0xFFFF - ABS(target_bright - pwm_output)) >> 5);
  }
 
    
}
/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  Set_System();

  
      /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

  /* Enable ADC1 DMA */
//  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  
  /* TIM1 Configuration ---------------------------------------------------
   Generate 7 PWM signals with 4 different duty cycles:
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   and Connectivity line devices and to 24 MHz for Low-Density Value line and
   Medium-Density Value line devices
   
   The objective is to generate 7 PWM signal at 17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   The channel 1 and channel 1N duty cycle is set to 50%
   The channel 2 and channel 2N duty cycle is set to 37.5%
   The channel 3 and channel 3N duty cycle is set to 25%
   The channel 4 duty cycle is set to 12.5%
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
  ----------------------------------------------------------------------- */
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  TimerPeriod = (SystemFrequency / 17570 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
  Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
  /* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N */
  Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);
  /* Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4 */
  Channel4Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod- 1)) / 1000);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;// Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

#if 0
  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
#endif
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);  

  while (1){
    loop();
  }
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
  for(; nCount!= 0;nCount--);
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while(1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
