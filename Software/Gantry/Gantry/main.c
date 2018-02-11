/*
 * Atmega16AttemptMotorStep.c
 *
 * Created: 22.12.2016 21:37:05
 * Author : lukasz.jamroz
 */ 
#include <avr/io.h>
#include <stdlib.h>
#include "stepper.h"
#include "hd44780.h"

#define ORDER_SIZE 18
volatile char command[ORDER_SIZE];

ISR(USART_RXC_vect)
{
	uint8_t p;
	for(p = 1; p < ORDER_SIZE; p++)
	{
		command[p-1] = command[p];
	}
	command[ORDER_SIZE-1] = UDR;
	
	if(command[ORDER_SIZE-1] == '/')
	{
		enum StepperDirection dir = UNDEFINED;
		if(command[1] == '<') dir = COUNTERCLOCKWISE;
			else
				if(command[1] == '>')	dir = CLOCKWISE;
				
		switch(command[0])
		{
			case 'm':;
				stepper_push_back_order(MOVE,atoi(command+2),dir,atoi(command+8),atoi(command+12),command[ORDER_SIZE-2]);
			break;
			case 'l':;
				stepper_push_back_order(LEFTRIGHT,atoi(command+2),dir,atoi(command+8),atoi(command+12),command[ORDER_SIZE-2]);
			break;
			case 'u':;
				stepper_push_back_order(UPDOWN,atoi(command+2),dir,atoi(command+8),atoi(command+12),command[ORDER_SIZE-2]);
			break;
			case 't':;
				stepper_push_back_order(TURN,atoi(command+2),dir,atoi(command+8),atoi(command+12),command[ORDER_SIZE-2]);
			break;
			case 's':;
				if(command[1] == 'S') 
				{
					servo_set_servo(PUSH,atoi(command+2));
				}else
					servo_push_back_order(PUSH,atoi(command+2),atoi(command+8),atoi(command+12),command[ORDER_SIZE-2]);
			break;
			default:
			break;
		}

	}
	else 
	/*if(command[ORDER_SIZE-1] == ';')
	{
		for(uint8_t i = 0; i < STEPPER_NUM_OF_MOTORS; i++) stepper_pop_front_order(i);
	}else*/
	if(command[ORDER_SIZE-1] == 'X')
	{
		TCCR0 &= ~(1<<CS00);
		TCCR1B&= ~( (1<<CS11)|(1<<CS10) );
	}else
	if(command[ORDER_SIZE-1] == 'Y')
	{
		TCCR0 |= (1<<CS00);
		TCCR1B|= (1<<CS11)|(1<<CS10);
	}
		
}

int main(void)
{
	PORTB = 0x00;
	DDRB = 0xff;
	
	PORTA = 0x00;
	DDRA = 0xff;
	
	PORTC = 0x00;
	DDRC = 0xff;
	
	PORTD = 0x00;
	DDRD = 0x02;
	
	UBRRH = 0; //data bits: 8 //baud:  9600 
	UBRRL = 103; //stop bits:  1 //parity:  No
	UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  
	UCSRB |= (1<<RXEN)|(1<<RXCIE)|(1<<TXEN)|(1<<TXCIE);
	
	stepper_add_motor(TURN,&PORTB,LSB);
	stepper_add_motor(LEFTRIGHT,&PORTB,MSB);	
	stepper_add_motor(MOVE,&PORTC,MSB);
	stepper_add_motor(UPDOWN,&PORTC,LSB);
	stepper_set_speed(UPDOWN,200);
	servo_add_servo(PUSH,&PORTD,PD5,0);
	
	stepper_fastest_and_init_motors();	
	stepper_init();
	servo_init();
	
    while(1) {}
}

