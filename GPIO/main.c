#include "stm32f10x.h"
#include "GPIO_STM32F10x.h"            
#include "stm32f10x_rcc.h"
#include "stdio.h"
#include <string.h>
#include <ctype.h>
#define LED_PIN 13
#define BAUD_RATE 9600

//varible
char rxBuffer[100];
int rxIndex = 0;

//func
void USART_Configuration(void);
void USART_SendChar(char ch);
void USART_SendString(char *str);

void SysTick_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

void USART1_IRQHandler(void);

void trim(char *str);

//handle
void USART_Configuration(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;   // A9-TX
    GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // A10-RX
    GPIOA->CRH |= GPIO_CRH_CNF10_0;

    USART1->BRR = SystemCoreClock / BAUD_RATE; 
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
}

void USART_SendChar(char ch) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (ch & 0xFF);
}

void USART_SendString(char *str) {
    while (*str) {
        USART_SendChar(*str++);
    }
}

void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000000) - 1; 
    SysTick->VAL = 0;                                
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void delay_us(uint32_t us) {
    SysTick->LOAD = us * (SystemCoreClock / 1000000) - 1;
    SysTick->VAL = 0;                                     
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;              
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) { 
        char data = USART1->DR; 
        if (rxIndex < 100 - 1) {
            rxBuffer[rxIndex++] = data; 
            if (data == '\n') { 
                rxBuffer[rxIndex] = '\0'; 
                rxIndex = 0; 
            }
        }
    }
}

void trim(char *str) {
    char *dst = str;
    while (*str) {
        if (!isspace((unsigned char)*str)) {
            *dst++ = *str; 
        }
        str++;
    }
    *dst = '\0'; 
}

int main(void) {
    SysTick_Init();
    USART_Configuration();
	  GPIO_PinConfigure(GPIOC,13,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT2MHZ);
	  GPIO_PinWrite(GPIOC,13,1);
    while (1) {
        //gui
        //char buffer[20];
        //static int counter = 0;
        //counter++;
        //sprintf(buffer, "Counter: %d\r\n", counter);
        //USART_SendString(buffer);
        
			  //nhan
        if (rxIndex > 0) {
					  trim(rxBuffer);
					  if(strcmp(rxBuffer, "on")==0){
						   GPIO_PinWrite(GPIOC,13,0);
							 memset(rxBuffer, 0, sizeof(rxBuffer));
					     rxIndex = 0;
						}
						if(strcmp(rxBuffer,"off")==0){
						   GPIO_PinWrite(GPIOC,13,1);
							 memset(rxBuffer, 0, sizeof(rxBuffer));
					     rxIndex = 0;  
						}
						if(strcmp(rxBuffer, "on")!=0 && strcmp(rxBuffer, "off") !=0){
						   memset(rxBuffer, 0, sizeof(rxBuffer));
					     rxIndex = 0;  
						}
				}
	      delay_ms(50);
    }
}



