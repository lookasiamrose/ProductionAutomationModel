/*
 * Atmega16AttemptMotorStep.c
 *
 * Created: 22.12.2016 21:37:05
 * Author : lukasz.jamroz
 */ 
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "orders.h" 

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
	
	if(command[ORDER_SIZE-1] == '.')
	{
		enum StepperDirection dir = UNDEFINED;
		if(command[1] == '<') dir = COUNTERCLOCKWISE;
		else
		if(command[1] == '>')	dir = CLOCKWISE;
		
		switch(command[0])
		{
			case 'l':
				stepper_push_back_order(FORKLIFT_LEAN,atoi(command+2),dir,atoi(command+8),atoi(command+12),orders_get_barrier_from_char(command[ORDER_SIZE-2]));
			break;
			case 'u':
				stepper_push_back_order(FORKLIFT_UPDOWN,atoi(command+2),dir,atoi(command+8),atoi(command+12),orders_get_barrier_from_char(command[ORDER_SIZE-2]));
			break;
			case 'd':
				stepper_push_back_order(FORKLIFT_DRIVE,atoi(command+2),dir,atoi(command+8),atoi(command+12),orders_get_barrier_from_char(command[ORDER_SIZE-2]));
			break;
			case 's':
				stepper_push_back_order(FORKLIFT_STEERING,atoi(command+2),dir,atoi(command+8),atoi(command+12),orders_get_barrier_from_char(command[ORDER_SIZE-2]));
			break;
			case 'b':
				orders_sensors_push_back_order(atoi(command+2),atoi(command+8),atoi(command+12),orders_get_barrier_from_char(command[ORDER_SIZE-2]));
			break;
			default:
			break;
		}
	}
	else
	if(command[ORDER_SIZE-1] == 'X')
	{
		cli();
	}else
	if(command[ORDER_SIZE-1] == 'Y')
	{
		sei();
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
	DDRD = 0xfe;
	
	UBRRH = 0; //data bits: 8 //baud:  9600 
	UBRRL = 103; //stop bits:  1 //parity:  No
	UCSRC |= (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  
	UCSRB |= (1<<TXEN)|(1<<RXEN)|(1<<RXCIE)|(1<<TXCIE);
	
	stepper_add_motor(FORKLIFT_LEAN,&PORTA,MSB);
	stepper_add_motor(FORKLIFT_DRIVE,&PORTC,LSB);
	stepper_add_motor(FORKLIFT_STEERING,&PORTD,MSB);
	stepper_add_motor(FORKLIFT_UPDOWN,&PORTC,MSB);
	
	stepper_set_speed(FORKLIFT_UPDOWN,200);
	stepper_set_speed(FORKLIFT_DRIVE,130);
	
	stepper_fastest_and_init_motors();	
	stepper_init();
	
	sensors_init();
	       
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	TIMSK |= (1<<OCIE1B);
	
	//orders_sensors_push_back_order(0b01110010,0,3,orders_get_barrier_from_char('s'));
	
	sei();
	
    while(1) {}
}

