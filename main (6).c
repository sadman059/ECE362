/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Nov 26, 2022
  * @brief   ECE 362 Lab 11 student template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

// Uncomment only one of the following to test each step
//#define STEP41
//#define STEP42
//#define STEP43
#define STEP44

void init_usart5();

void init_usart5() {
    // TODO
	    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;

		//configure pin PC12 to be routed to USART5_TX.
		GPIOC->MODER &= ~(GPIO_MODER_MODER12);
		GPIOC->MODER |= GPIO_MODER_MODER12_1;
		GPIOC->AFR[1] &= ~(GPIO_AFRH_AFR12);//af2
		GPIOC->AFR[1] |= (0b1<<17);

		//configure pin PD2 to be routed to USART5_RX.
		GPIOD->MODER &= ~(GPIO_MODER_MODER2);
		GPIOD->MODER |= GPIO_MODER_MODER2_1;
		GPIOD->AFR[0] &= ~(GPIO_AFRL_AFR2);//af2
		GPIOD->AFR[0] |= 0b1<<9;

		RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
		//ue off
		USART5->CR1 &= ~(0b1<<0);
		//word size of 8 bits.
		USART5->CR1 &=~(0b1<<12);
		USART5->CR1 &=~(0b1<<28);

		//one stop bit
		USART5->CR2 &= ~(0b1<<12);
		USART5->CR2 &= ~(0b1<<13);
		//no parity
		USART5->CR1 &= ~(0b1<<10);
		//16x oversampling.
		USART5->CR1 &= ~(0b1<<15);

		//baud rate of 115200;?
		USART5->BRR &= ~0xffff;
		USART5->BRR = 0x1a1;
		//Enable the transmitter and the receiver by setting the TE and RE bits
		USART5->CR1 |= 0b1<<3;
		USART5->CR1 |= 0b1<<2;

		//Enable the USART
		USART5->CR1 |= (0b1<<0);

		//wait for the TE and RE bits to be acknowledged by checking that TEACK and REACK?
		while (!(USART5->ISR & USART_ISR_TEACK) && !(USART5->ISR & USART_ISR_REACK));//HOW IN BIT SHIFTING?->(USART5->ISR)&(USART5->ISR |= 1<<21)
		//while ((USART5->ISR & USART_ISR_REACK)==0);
}

#ifdef STEP41
int main(void){
    init_usart5();
    for(;;) {
        while (!(USART5->ISR & USART_ISR_RXNE)) { }
        char c = USART5->RDR;
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = c;
    }
}
#endif

#ifdef STEP42
#include <stdio.h>

// TODO Resolve the echo and carriage-return problem

int __io_putchar(int c) {
	while(!(USART5->ISR & USART_ISR_TXE));
    if (c== '\n')
    {

    	USART5->TDR = '\r';
    	while(!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
	while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE));
    char c = USART5->RDR;
    if (c== '\r')
    {
    	c = '\n';
    }
    __io_putchar(c);
    return c;
}

int main() {
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP43
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
int __io_putchar(int c) {
    // TODO Copy from your STEP42
	while(!(USART5->ISR & USART_ISR_TXE));
		    if (c== '\n')	USART5->TDR = '\r';
	while(!(USART5->ISR & USART_ISR_TXE));
	USART5->TDR = c;
	return c;
}

int __io_getchar(void) {
    // TODO
	 return line_buffer_getchar();
	}

int main() {
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP44

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    // TODO
	    USART5->CR1 |= USART_CR1_RXNEIE;
		USART5->CR3 |= USART_CR3_DMAR;
		NVIC->ISER[0]|= 1<<USART3_8_IRQn;

		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
		DMA2->RMPCR |= DMA_RMPCR2_CH2_USART5_RX;
		DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off

		// The DMA channel 2 configuration goes here
		DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);
		DMA2_Channel2->CMAR = (uint32_t)&(serfifo);
		DMA2_Channel2->CNDTR = FIFOSIZE;
		DMA2_Channel2->CCR &= ~(DMA_CCR_DIR);
		//MSIZE and the PSIZE should be set for 8 bits
		DMA2_Channel2->CCR &= ~(DMA_CCR_MSIZE);
		DMA2_Channel2->CCR &= ~(DMA_CCR_PSIZE);
		DMA2_Channel2->CCR |= DMA_CCR_MINC;//MINC should be set to increment the CMAR ??
		//PINC should not be set so that CPAR always points at the USART5->RDR.
		DMA2_Channel2->CCR &=~(DMA_CCR_PINC);
		DMA2_Channel2->CCR |= DMA_CCR_CIRC;
		DMA2_Channel2->CCR &= ~(DMA_CCR_MEM2MEM);
		//Priority Level to highest.
		DMA2_Channel2->CCR &= ~(DMA_CCR_PL);
		DMA2_Channel2->CCR |= (DMA_CCR_PL_0 | DMA_CCR_PL_1);

		DMA2_Channel2->CCR |= DMA_CCR_EN;
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    // TODO
	while(fifo_newline(&input_fifo) == 0) {
		        asm volatile ("wfi");
		    }
		    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
    // TODO Copy from step 42
	while(!(USART5->ISR & USART_ISR_TXE));
		    if (c == '\n') USART5->TDR = '\r';
	while(!(USART5->ISR & USART_ISR_TXE));
	USART5->TDR = c;
	return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
	return interrupt_getchar();
	}
// TODO Copy the content for the USART5 ISR here
// TODO Remember to look up for the proper name of the ISR function
void USART3_4_5_6_7_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

int main() {
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0); // These turn off buffering; more efficient, but makes it hard to explain why first 1023 characters not dispalyed
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: "); // Types name but shouldn't echo the characters; USE CTRL-J to finish
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n"); // After, will type TWO instead of ONE
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif
