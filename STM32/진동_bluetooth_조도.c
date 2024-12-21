#include "stm32f10x.h"
#include <stdio.h>

volatile uint8_t press_count = 0;
char msg[] = "PLAY\r\n";

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);
void GPIO_Config(void);
void TIM3_PWM_Config(void);
void ADC_Configure(void);
uint16_t ADC_Read(void);
void Delay(uint32_t ms);

void RCC_Configure(void)
{
    /* USART1, USART2 TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);    
    /* USART1, USART2 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* ADC1 and GPIOA clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* USART1 pin setting */
    // TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);    
    // RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);    
    /* USART2 pin setting */
    // TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);    
    // RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ADC pin setting */
    // PA0 as analog input // for light sensor
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART1_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART2_InitStructure);    

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // USART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART1);
        USART_SendData(USART1, word);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART2);
        USART_SendData(USART1, word);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Enable GPIOC clock

    // Configure PC4 as input (button)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with pull-up
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// for vibrate motor setting
void TIM3_PWM_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // Enable TIM3 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Enable GPIOA clock

    // Configure PA6 as TIM3 CH1 (PWM output)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM3 configuration
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;          // Period (1ms resolution)
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;       // Prescaler (10kHz timer clock)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // TIM3 PWM1 Mode configuration: Channel1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // Start with 0 duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE); // Enable TIM3
}

void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* ADC1 초기화 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 채널 0 (PA0) 설정 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

    /* ADC1 활성화 */
    ADC_Cmd(ADC1, ENABLE);

    /* ADC1을 초기화하고, 변환을 시작 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));  // 리셋이 완료될 때까지 대기

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));  // 보정이 완료될 때까지 대기
}

uint16_t ADC_Read(void)
{
    /* ADC 변환 시작 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 변환이 완료될 때까지 대기 */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    /* 변환된 값을 반환 */
    return ADC_GetConversionValue(ADC1);
}

void Delay(uint32_t ms)
{
    ms *= 1000;
    while (ms--)
    {
        __NOP(); // Do nothing, just wait
    }
}

int main(void) {
    uint16_t adc_value;
    SystemInit();

    RCC_Configure();
    GPIO_Configure();
    USART1_Init();      // PC
    USART2_Init();      // Bluetooth
    NVIC_Configure();
    GPIO_Config();
    TIM3_PWM_Config();
    ADC_Configure();

    while (1) {
        // Check if PC4 is pressed
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_RESET){
            Delay(20); // Debouncing delay
            press_count ++;
           
            printf("count : %d\n", press_count);
            if(press_count == 3){
              // Send "PLAY" string via USART2
              for (int i = 0; i < sizeof(msg) - 1; i++) {
                  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
                  USART_SendData(USART2, msg[i]);
              }            
              TIM_SetCompare1(TIM3, 300 * press_count); // Activate motor (50% duty cycle)
              Delay(50000);              // 5 seconds
              TIM_SetCompare1(TIM3, 0);   // Stop motor
                      
              press_count = 0; // Reset press count
            }
         else if (press_count != 3){  // If it's not the 3rd press
            TIM_SetCompare1(TIM3, 300 * press_count); // Activate motor (30% duty cycle)
            Delay(20000);              // 2 seconds
            TIM_SetCompare1(TIM3, 0);   // Stop motor
          }
          Delay(500);   
        } 
        // Read ADC value
        adc_value = ADC_Read(); // 조도센서 값
        //// Map ADC value to PWM duty cycle (0-100%)
        //uint16_t duty_cycle = (adc_value * 100) / 4095;
        //TIM3->CCR1 = (duty_cycle * 1000) / 100; // Update PWM duty cycle

        Delay(10); // Small delay
    }
}
