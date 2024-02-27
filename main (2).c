/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Oct 24, 2022
  * @brief   ECE 362 Lab 7 template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

// Global data structure
char* login          = "xyz"; // Replace with your login.
char disp[9]         = "Hello...";
uint8_t col          = 0;
uint8_t mode         = 'A';
uint8_t thrust       = 0;
int16_t fuel         = 800;
int16_t alt          = 4500;
int16_t velo         = 0;

// Make them visible to autotest.o
extern char* login;
// Keymap is in `font.S` to match up what autotester expected
extern char keymap;
extern char disp[9];
extern uint8_t col;
extern uint8_t mode;
extern uint8_t thrust;
extern int16_t fuel;
extern int16_t alt;
extern int16_t velo;

char* keymap_arr = &keymap;

// Font array in assembly file
// as I am too lazy to convert it into C array
extern uint8_t font[];

// The functions we should implement
void enable_ports();
void setup_tim6();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
void handle_key(char key);
void setup_tim7();
void write_display();
void update_variables();
void setup_tim14();

// Auotest functions
extern void check_wiring();
extern void autotest();
extern void fill_alpha();

int main(void) {
    //check_wiring();
    //autotest();
    //fill_alpha();
    enable_ports();
    setup_tim6();
    setup_tim7();
    setup_tim14();

    for(;;) {
        asm("wfi");
    }
}

/**
 * @brief Enable the ports and configure pins as described
 *        in lab handout
 * 
 */
void enable_ports(){
	    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
		RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;

		GPIOB -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
		GPIOB -> MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0);

		GPIOC -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8);
		GPIOC -> MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0);

		GPIOC ->  PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
		GPIOC -> PUPDR |= (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1);
}

//-------------------------------
// Timer 6 ISR goes here
//-------------------------------
// TODO
void TIM6_DAC_IRQHandler()
{
	TIM6->SR &=(~TIM_SR_UIF);

	if (GPIOC->ODR & (1<<8))
	{
		GPIOC->BRR=1<<8;
	}
	else
	{
		GPIOC->BSRR=1<< 8;
	}
}

/**
 * @brief Set up timer 6 as described in handout
 * 
 */
void setup_tim6() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		TIM6->PSC = 48000 - 1;
		TIM6->ARR = 500 - 1;
		TIM6 ->DIER = TIM_DIER_UIE;
		TIM6-> CR1 = TIM_CR1_CEN;
		NVIC -> ISER[0] = 1 << TIM6_DAC_IRQn;

}

/**
 * @brief Show a character `c` on column `n`
 *        of the segment LED display
 * 
 * @param n 
 * @param c 
 */
void show_char(int n, char c) {
	GPIOB->ODR = ((n & 7) << 8) | font[c];
}

/**
 * @brief Drive the column pins of the keypad
 *        First clear the keypad column output
 *        Then drive the column represented by `c`
 * 
 * @param c 
 */
void drive_column(int c) {
	GPIOC->ODR &= ~(0b11110000);

	GPIOC->ODR |= (0b1 << ((c & 3) + 4));
}

/**
 * @brief Read the rows value of the keypad
 * 
 * @return int 
 */
int read_rows() {
	return (GPIOC->IDR & 0xF);
}

/**
 * @brief Convert the pressed key to character
 *        Use the rows value and the current `col`
 *        being scanning to compute an offset into
 *        the character map array
 * 
 * @param rows 
 * @return char 
 */
char rows_to_key(int rows) {
	int ng = (col & 0x3)*4;
	while(rows !=0)
	{
			if (rows & 1)
			{
				break;
			}
			ng++;
			rows = rows >>1;
	}

	char c = keymap_arr[ng];

	return c;
}

/**
 * @brief Handle key pressed in the game
 * 
 * @param key 
 */
void handle_key(char key) {
	if (key=='A'|| key=='B'||key=='D')
	{
			mode = key;
	}
	else if(key >='0' && key <= '9')
	{
			thrust = key - '0';
	}
}

//-------------------------------
// Timer 7 ISR goes here
//-------------------------------
// TODO
void TIM7_IRQHandler()
{
	TIM7->SR &= ~(TIM_SR_UIF);

	int r = read_rows();

	if(r != 0)
	{
		char k = rows_to_key(r);

		handle_key(k);
	}
	char cr = disp[col];

	show_char(col,cr);
	col = (col+1) & 7;
	drive_column(col);
}
/**
 * @brief Setup timer 7 as described in lab handout
 * 
 */
void setup_tim7() {
	    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
		TIM7->PSC = 48 - 1;
		TIM7-> ARR = 1000 - 1;
		TIM7 ->DIER = TIM_DIER_UIE;
		TIM7-> CR1 = TIM_CR1_CEN;
		NVIC->ISER[0] = 1 << TIM7_IRQn;
}

/**
 * @brief Write the display based on game's mode
 * 
 */
void write_display() {
	if (mode == 'C')
	{
			snprintf(disp, 9, "Crashed");
	}
	else if (mode == 'L'){
		    snprintf(disp, 9, "Landed ");
	}
	else if (mode == 'A'){
		    snprintf(disp, 9, "ALt%5d", alt);
	}
	else if (mode == 'B')
	{
		    snprintf(disp, 9, "FUEL %3d", fuel);
	}
	else if (mode == 'D'){
		    snprintf(disp, 9, "Spd %4d", velo);
	}
}

/**
 * @brief Game logic
 * 
 */
void update_variables() {
	fuel -= thrust;
	if (fuel <= 0)
	{
		     thrust=0;
		     fuel= 0;
	}
	alt += velo;
	if (alt <= 0)
	{
		if (-velo < 10)
	    {
		         mode = 'L';
	     }
		 else{
		    	 mode = 'C';
		 }
		        return;
     }

	velo += thrust - 5;
}

//-------------------------------
// Timer 14 ISR goes here
//-------------------------------
// TODO
void TIM14_IRQHandler()
{
	TIM14->SR &= ~(TIM_SR_UIF);

	update_variables();
	write_display();
}

/**
 * @brief Setup timer 14 as described in lab
 *        handout
 * 
 */
void setup_tim14() {
	    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		TIM14->PSC = 48000 - 1;
		TIM14->ARR = 500 - 1;
		TIM14->DIER = TIM_DIER_UIE;
		TIM14->CR1 = TIM_CR1_CEN;
		NVIC->ISER[0] = 1 << TIM14_IRQn;

}
