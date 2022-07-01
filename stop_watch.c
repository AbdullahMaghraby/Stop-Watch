/*
 * stop_watch.c
 *
 *  Created on: 14/9þ/2021
 *  Author: Abdullah Mohammed Al-Maghraby
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char g_tick=0;
unsigned char sec1=0,sec2=0,min1=0,min2=0,hr1=0,hr2=0;

ISR(INT2_vect)
{
	TCCR1B |= (1<<CS10) | (1<<CS12); // return clk so timer counts so resumed stop watch
}

ISR(INT1_vect)
{
	TCCR1B &= ~(1<<CS10) & ~(1<<CS11) & ~(1<<CS12); // no clk so no timer counts so pausing stop watch
}

ISR(INT0_vect)
{
	TCNT1 = 0;		/* reset timer1  */
	g_tick=0,sec1=0,sec2=0,min1=0,min2=0,hr1=0,hr2=0;// reset stop watch
}

ISR (TIMER1_COMPA_vect)
{
	g_tick++;
	if (g_tick<=9){sec1=g_tick;}
	else {sec1=10;g_tick=0;}
	if(sec1==10){sec2++;sec1=0;}
	if(sec2==6){min1++;sec2=0;}
	if(min1==10){min2++;min1=0;}
	if(min2==6){hr1++;min2=0;}
	if(hr1==10){hr2++;hr1=0;}
	if(hr2==10){g_tick=0,sec1=0,sec2=0,min1=0,min2=0,hr1=0,hr2=0;}//reset the stop watch after over flow
}

void Timer1_CTC_init (void){
	TCNT1 = 0;		/* Set timer1 initial count to zero */
	OCR1A = 1000;    /* Set the Compare value to 1000ms==1second */
	TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */
	/*
	 *  Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=0 non PWM
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)

	 * Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1A = (1<<FOC1A);
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);
	SREG |= (1<<7);//I_bit activation (global interrupt)
}

void INT0_Init(void)
{
	SREG  &= ~(1<<7);                   // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD2));               // Configure INT0/PD2 as input pin
	PORTD|=(1<<PD2);                    //internal pull up
	GICR  |= (1<<INT0);                 // Enable external interrupt pin INT0
	MCUCR |= (1<<ISC01);                // Trigger INT0 with the falling edge ISC00=zero
	SREG  |= (1<<7);                    // Enable interrupts by setting I-bit
}

void INT1_Init(void)
{
	SREG  &= ~(1<<7);      // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD3));  // Configure INT1/PD3 as input pin
	PORTD |= (1<<PD3);     // Enable the internal pull up resistor at PD3 pin
	GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
	MCUCR |= (1<<ISC10) | (1<<ISC11); // Trigger INT1 with the raising edge
	SREG  |= (1<<7);       // Enable interrupts by setting I-bit
}

void INT2_Init(void)
{
	SREG   &= ~(1<<7);       // Disable interrupts by clearing I-bit
	DDRB   &= (~(1<<PB2));   // Configure INT2/PB2 as input pin
	PORTB  |= (1<<PB2);     //internal pull up
	GICR   |= (1<<INT2);	 // Enable external interrupt pin INT2
	MCUCSR &= ~(1<<ISC2);     // Trigger INT2 with the falling edge
	SREG   |= (1<<7);        // Enable interrupts by setting I-bit
}

int main (void){
	DDRC |=0x0f;//first 4 pins at portC is output
	PORTC &=0xf0;//zero initial value
	DDRA |=0x1f;//first 5 pins at portA is output
	PORTA &=0xe0;//zero initial value

	INT0_Init();
	INT1_Init();
	INT2_Init();
	Timer1_CTC_init ();

	while (1){
		PORTA =(1<<PA0);
		PORTC = (PORTC & 0xf0) | (sec1 & 0x0f);
		_delay_ms(3);
		PORTA =(1<<PA1);
		PORTC = (PORTC & 0xf0) | (sec2 & 0x0f);
		_delay_ms(3);
		PORTA =(1<<PA2);
		PORTC = (PORTC & 0xf0) | (min1 & 0x0f);
		_delay_ms(3);
		PORTA =(1<<PA3);
		PORTC = (PORTC & 0xf0) | (min2 & 0x0f);
		_delay_ms(3);
		PORTA =(1<<PA4);
		PORTC = (PORTC & 0xf0) | (hr1 & 0x0f);
		_delay_ms(3);
		PORTA =(1<<PA5);
		PORTC = (PORTC & 0xf0) | (hr2 & 0x0f);
		_delay_ms(3);
	}

}
