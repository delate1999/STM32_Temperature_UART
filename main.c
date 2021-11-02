/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "stm32f0xx_nucleo.h"

#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))

int16_t finite_impulse_response(int8_t);
void send_temperature();
int16_t buffer_read();
void buffer_write();

int8_t number[3];
int16_t buffer[32];
int8_t head = 0;
int8_t tail = 0;
int16_t temperature;

int8_t message[] = "\n  .   degC\r\n";

volatile int8_t g_rxData;
volatile int32_t g_rxDataSize = 0;

volatile int8_t *g_txData = (int8_t *) 0;
volatile int32_t g_txDataSize = 0;
volatile int32_t g_txDataTransmitted = 0;

volatile int8_t T_flag = 0;
volatile int8_t N_flag = 0;
volatile int8_t input_accepted_flag = 0;
volatile int8_t change_flag = 0;

int8_t T = 40;
int8_t N = 1;
int8_t temp_N;
int8_t i = 0;
int16_t time = 0;
int32_t output = 0;

void USART2_IRQHandler(void) {
	if (0 != (USART_ISR_RXNE & USART2->ISR)) {
		g_rxData = USART2->RDR;
		g_rxDataSize = 1;
	}

	if (0 != (USART_ISR_TXE & USART2->ISR)) {
		if ((((int8_t *) 0) != g_txData) && (0 != g_txDataSize) && (g_txDataTransmitted < g_txDataSize)) {
			USART2->TDR = (uint32_t) *(g_txData + g_txDataTransmitted);

			g_txDataTransmitted++;
			if (g_txDataSize == g_txDataTransmitted) {
				USART2->CR1 &= ~USART_CR1_TXEIE;
			}
		}
	}
}

void ADC1_COMP_IRQHandler(void){
	if(ADC1->ISR && ADC_ISR_EOC){
		head = head + 1;
		if(head == 32)
			head = 0;
		buffer[head] = ADC1->DR;
	}
}

void TIM3_IRQHandler(void){
	if(0 != (TIM_SR_UIF & TIM3->SR))
	{
		TIM3->SR &= ~TIM_SR_UIF;
		send_temperature();
	}
}

int16_t finite_impulse_response(int8_t number_of_points){
	output = 0;
	for(int k = 0; k < number_of_points; k++){
		output += buffer_read();
	}
	output = output/number_of_points;
	return output;
}

void send_temperature(){
	__disable_irq();
	temperature = finite_impulse_response(N);
	message[1] = ((float)((temperature) - *TEMP30_CAL_ADDR)* (float)(110-30)/(float)(*TEMP110_CAL_ADDR-*TEMP30_CAL_ADDR)+ 30)/10 + 48;
	message[2] = ((float)((temperature) - *TEMP30_CAL_ADDR)* (float)(110-30)/(float)(*TEMP110_CAL_ADDR-*TEMP30_CAL_ADDR)+ 30)-10*(message[1]-48) + 48;
	message[4] = (((float)((temperature) - *TEMP30_CAL_ADDR)* (float)(110-30)/(float)(*TEMP110_CAL_ADDR-*TEMP30_CAL_ADDR)+ 30)-10*(message[1]-48)-(message[2]-48))*10 + 48;
	message[5] = (((float)((temperature) - *TEMP30_CAL_ADDR)* (float)(110-30)/(float)(*TEMP110_CAL_ADDR-*TEMP30_CAL_ADDR)+ 30)-10*(message[1]-48)-(message[2]-48)-0.1*(message[4]-48))*100 + 48;
	g_txData = message;
	g_txDataSize = 13;
	g_txDataTransmitted = 0;
	USART2->CR1 |= USART_CR1_TXEIE;
	__enable_irq();
}

void buffer_write(){
	uint8_t head_temp = head + 1;
	if(head_temp == 32)
		head_temp = 0;
	buffer[head_temp] = ADC1->DR;
	head = head_temp;
}

int16_t buffer_read(){
	tail++;
	if(tail == 32)
		tail = 0;
	return buffer[tail];
}

int main(void)
{
	// ------------- RCC -------------
	RCC->CR |= RCC_CR_HSION; // enable HSI
	while (0 == (RCC->CR  & RCC_CR_HSIRDY)) { // wait until RDY bit is set
		// empty
	}
	RCC->CFGR |= RCC_CFGR_SW_HSI; // set SYSCLK to HSI - 8MHz

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable GPIOA clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 clock


	// ------------- TIM3 -------------
	TIM3->ARR = T * 100-1;	//	T * 100ms is interval of sending the temperature
	TIM3->PSC = 8000-1;
	TIM3->DIER |= TIM_DIER_UIE;


	// ------------- USART2 -------------
	USART2->BRR = 69 - 1; // 8 000 000 / 115 200 = 69.44
	USART2->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;


	// ------------- GPIOA (pin 2 - TX & 3 - RX) -------------
	// USART_RX & TX --> AF1
	GPIOA->AFR[0] &= ~0x0000FF00;
	GPIOA->AFR[0] |=  0x00001100;

	GPIOA->MODER &= ~(GPIO_MODER_MODER3_1 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER2_0);
	GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1);

	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3_1 | GPIO_OSPEEDER_OSPEEDR3_0 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR2_0);

	// ------------- NVIC -------------
	NVIC_EnableIRQ(USART2_IRQn); // nvic interrupt enable (USART2 interrupt)
	NVIC_EnableIRQ(TIM3_IRQn); //	enable interrupt from TIM3


	// ------------- ADC -------------
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;	//	ADC1 is working on dedicated 14MHz clock
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0)
	{

	}

	if ((ADC1->CR & ADC_CR_ADEN) != 0)	// Disable ADC1 if enabled
	{
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		//empty
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;	//	Calibrate ADC1
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)
	{
		//empty
	}

	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)	// 	If ready ...
	{
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	ADC1->CR |= ADC_CR_ADEN;	//	... then enable
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
	{

	}

	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;

	ADC1->CHSELR = ADC_CHSELR_CHSEL16;	//	temperature sensor is connected to channel 16

	//With code constructed like this, with continuous ADC1, there is no possibility of setting 100ms period of sampling
	//We have chosen total conversion time of 239.5 + 12.5
	//We couldn't work out how to make it work even on APB clock

	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
	ADC->CCR |= ADC_CCR_TSEN;

	ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_SCANDIR;
	ADC1->IER |= ADC_IER_EOCIE;
	ADC->CCR |= ADC_CCR_VREFEN;
	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	NVIC_SetPriority(ADC1_COMP_IRQn,0);

	ADC1->CR |= ADC_CR_ADSTART;
	TIM3->CR1 |= TIM_CR1_CEN;

	number[0] = '=';
	number[1] = '=';
	number[2] = '=';

	while(1)
	{

 		if(input_accepted_flag == 1){
			time++;
		}

		if(time >= 300){	//	wait for all data from USART2 to come
			change_flag = 1;
			input_accepted_flag = 0;
			time = 0;
		}

		if(0 != g_rxDataSize)
		{
			g_rxDataSize = 0;
			if(input_accepted_flag == 1){
				if((g_rxData >= 48) && (g_rxData <= 57)){
					number[i] = g_rxData;
					i++;
				}
			}
			if(((T_flag == 1) || (N_flag == 1)) && (g_rxData == ' ')){
				input_accepted_flag = 1;
			}

			if(g_rxData == 'T'){
				T_flag = 1;
			}

			if(g_rxData == 'N'){
				N_flag = 1;
			}
		}

		if(change_flag == 1 && T_flag == 1){
			if(number[2] == '=' && number[1] == '='){
				T = number[0] - 48;
			}else if(number[2] == '=' && number[1] != '='){
				T = (number[0] - 48)*10 + number[1]-48;
			}else{
				T = (number[0] - 48)*100 + (number[1] - 48)*10 + number[2]-48;
			}
			if(T >= 1 && T <= 100 ){
				TIM3->CR1 = 0;
				for(int k = 0; k < 10; k++){
					//empty
				}
				TIM3->ARR = T * 100-1;
				for(int k = 0; k < 10; k++){
					//empty
				}
				TIM3->CR1 |= TIM_CR1_CEN;
			}
			if(T == 0){
				TIM3->CR1 = 0;
				send_temperature();
			}
			change_flag = 0;
			T_flag = 0;
			i = 0;
			while(i < 3){
				number[i] = '=';
				i++;
			}
			i = 0;
		}

		if(change_flag == 1 && N_flag == 1){
			if(number[2] == '=' && number[1] == '='){
				temp_N = number[0] - 48;
			}else if(number[2] == '=' && number[1] != '='){
				temp_N = (number[0] - 48)*10 + number[1]-48;
			}else{
				temp_N = (number[0] - 48)*100 + (number[1] - 48)*10 + number[2]-48;
			}
			if(temp_N >= 1 && temp_N <= 32){
				N = temp_N;
			}
			change_flag = 0;
			N_flag = 0;
			i = 0;
			while(i < 3){
				number[i] = '=';
				i++;
			}
			i = 0;
		}
	}
}

