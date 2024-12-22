#include "stm32f10x.h"
#include <stdio.h>

volatile uint8_t press_count = 0;
volatile uint16_t lightSensorValue = 0;
float threshold = 0.5;
char msg[] = "PLAY\r\n";

/* 수신 데이터 버퍼 */
#define RX_BUFFER_SIZE 50
volatile char rxBuffer[RX_BUFFER_SIZE]; // 수신된 문자열 저장
volatile uint8_t rxIndex = 0;           // 수신 문자열 인덱스
volatile uint8_t rxReady = 0;


/* function prototype */
void controlVibrateMotor(void);
void controlPIRSensor(void);
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void updateThreshold(uint16_t value);
void NVIC_Configure(void);
void TIM2_Init(uint16_t time_ms);
void TIM3_PWM_Config(void);
void ADC_Configure(void);
void Delay(uint32_t ms);


int main(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();         // PC와 통신
    USART2_Init();         // Bluetooth와 통신
    NVIC_Configure();      // 인터럽트 설정
    TIM2_Init(5000);       // PIR 센서를 위한 타이머 (5초 주기)
    TIM3_PWM_Config();     // 진동 모터 설정
    ADC_Configure();       // 조도 센서 설정

    while (1) {
        controlVibrateMotor(); // 버튼 입력에 따라 진동 모터 제어
        controlPIRSensor();    // PIR 센서로 인체 감지 및 LED 제어
        Delay(10);             // 짧은 대기
    }
}

// 졸음 감지할 때마다 진동 세기 up
void controlVibrateMotor() {
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
}

// PIR Sensor to control LED
//TODO: 값 유지되도록 하여 일정 시간마다 측정하도록 변경 -> 타이머 사용
//TODO: 사람이 있을 때 만 할 수 있도록..?
void controlPIRSensor() {
    uint8_t pirState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
    if(pirState == Bit_SET){
        GPIO_SetBits(GPIOA, GPIO_Pin_7);
        
        TIM_SetCounter(TIM2, 0); // 타이머 초기화
        TIM_Cmd(TIM2, ENABLE); // 타이머 시작
         
    } // LED : PA7 ON
    
}

// clock 인가 함수
void RCC_Configure(void) {
    /* USART1, USART2 TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    /* USART1, USART2 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* ADC1 and GPIOA clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
}

// pin 설정 함수
/**
 * Button     - PC4
 * ADC        - PA0
 * PIR Sensor - PC9
 * LED        - PA7
 * MOTOR      - PA6
 * TX         - PA2
 * RX         - PA3
*/
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ADC (조도 센서, PA0) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PIR Sensor (PC9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 버튼1 (PC4) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with pull-up
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* LED (PA7) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 (ESP32-CAM 통신) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 (Bluetooth 통신) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// USART1 설정 함수
void USART1_Init(void) {
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

// USART2 설정 함수
void USART2_Init(void) {
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

// Interrupt Handler for USART1(ESP32)
void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART1);
        printf("Received from ESP32: %c\n", word);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
   } else {
       //  printf("No Received from ESP32\n");
    }
}


// Interrupt Handler for USART2(BlueTooth)
void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART2);
        // USART_SendData(USART1, word);
        printf("Received from Bluetooth: %c\n", word);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    } // else {
        // printf("No Received from Bluetooth\n");
    // }
}

// Interrupt Handler for 조도센서 to change a 임계값
void ADC1_2_IRQHandler(void){
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        lightSensorValue = ADC_GetConversionValue(ADC1);
        updateThreshold(lightSensorValue);
        printf("ADC Value: %d, Threshold: %.1f\n", lightSensorValue, threshold);
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

// update a 임계값 based on 조도센서 값
void updateThreshold(uint16_t value) {
    if (3700 <= value && value <= 3900) {
        threshold = 0.5; // 기본 상태
    } else if (4000 <= value && value <= 4080) {
        threshold = 0.3; // 어두운 상태
    } else if (2000 <= value && value <= 3000) {
        threshold = 0.7; // 밝은 상태
    } else {
        threshold = 0.5; // 기본값으로 되돌림
    }
}

void TIM2_IRQHandler(void){
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
            GPIO_ResetBits(GPIOA, GPIO_Pin_7); // LED 끄기
            printf("No Motion Detected! LED OFF\n");

            TIM_Cmd(TIM2, DISABLE);            // 타이머 비활성화
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 인터럽트 플래그 클리어
    }
}

// ADC, USART1, USART2, TIM2 인터럽트 우선순위 설정 및 활성화
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // ADC1 : Light Sensor
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART1 : ESP32
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 : BlueTooth
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // TIM2 : PIR Sensor
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// PIR 센서용 타이머 설정
void TIM2_Init(uint16_t time_ms) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_Cmd(TIM2, ENABLE); // 타이머 TIM2 활성화
    
    // 타이머 주기 설정 (time_ms 단위)
    TIM_TimeBaseStructure.TIM_Period = time_ms * 1000 - 1; // 1ms 주기
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;          // 72MHz -> 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 타이머 인터럽트 활성화
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

// 진동 모터 설정
void TIM3_PWM_Config(void) {
    // PA6 핀을 PWM 출력으로 설정
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

// 조도센서를 위한 ADC(Analog To Digital) 함수
void ADC_Configure(void){
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
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// User Define Delay Function
void Delay(uint32_t ms) {
    ms *= 1000;
    while (ms--)
    {
        __NOP(); // Do nothing, just wait
    }
}