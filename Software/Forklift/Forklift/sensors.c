/*
 * sensors.c
 *
 * Created: 13.07.2017 18:31:33
 *  Author: lukasz.jamroz
 */ 
#include "sensors.h"

volatile uint8_t  sensors_sensor_index;

ISR(ADC_vect)
{
	sensors_read_adc();
	ADCSRA |= (1<<ADSC);
}

void sensors_read_adc()
{
	switch(sensors_sensor_index)
	{
		case 0:
		sensors_sensors[0] = ADC;
		ADMUX &= 0xF8;
		
		if(sensors_sensors[0] > 512) sensors_decision &= ~(1<<0);
		else sensors_decision |= (1<<0);
		
		ADMUX  |= (1<<MUX0);
		break;
		
		case 2:
		sensors_sensors[1] = ADC;
		ADMUX &= 0xF8;
		
		if(sensors_sensors[1] > 512) sensors_decision &= ~(1<<1);
		else sensors_decision |= (1<<1);
		
		ADMUX  |= (1<<MUX1);
		break;
		
		case 4:
		sensors_sensors[2] = ADC;
		ADMUX &= 0xF8;
		
		if(sensors_sensors[2] > 512) sensors_decision &= ~(1<<2);
		else sensors_decision |= (1<<2);
		break;
		
		default:
		break;
	}
	if(sensors_sensor_index > 4)
	{
		sensors_sensor_index = 0;
	}
	else
	sensors_sensor_index++;
}

void sensors_init()
{
	/*sensors_sensor_index = 0;
	sensors_decision = 0;
	sensors_sensors[0] = 0;
	sensors_sensors[1] = 0;
	sensors_sensors[2] = 0;*/
	
	buttons_init_io();
	
	SENSORS_IN_PORT  &= ~( (1<<SENSORS_IN_A)|(1<<SENSORS_IN_B)|(1<<SENSORS_IN_C) ); // Sensors port without pullups
	SENSORS_IN_DDR   &= ~( (1<<SENSORS_IN_A)|(1<<SENSORS_IN_B)|(1<<SENSORS_IN_C) );  // Sensors port as input
	
	ADMUX  |= (1<<REFS0); // AREF with external capacitor
	ADCSRA |= (1<<ADEN)|(1<<ADIE); // ADC enabled, ADC interrupt enabled
	
	// EDIT BELOW IF OTHER THAN 16MHZ RESONATOR IS BEING USED
	ADCSRA |= (1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // 125 kHz for 16 MHz resonator, read the ADCSRA description for details
	/////////////////////////////////////////////////////////
	
	ADCSRA |= (1<<ADSC);
}

void buttons_init_io(void)
{
	debounce_cnt = 0;
	//Setup Buttons
	KEY_DDR &= ~((1<<KEY0) | (1<<KEY1));  //Set pins as input
	KEY_PORT |= (1<<KEY0) | (1<<KEY1);    //enable pull-up resistors
}

uint8_t get_key_press( uint8_t key_mask )
{
	cli();            // read and clear atomic !
	key_mask &= key_press;    // read key(s)
	key_press ^= key_mask;    // clear key(s)
	sei();            // enable interrupts
	return key_mask;
}