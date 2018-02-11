#define F_CPU 2000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "hd44780.h"
#include "usart_driver.h"

#define ORDER_SINGLE_SIZE 18

typedef struct Orders_Order
{
	uint8_t ordersString[ORDER_SINGLE_SIZE];
} Orders_Order;


#define ORDERS_BUFFOR_SIZE 1024
#define ORDERS_LIST_SIZE 64

volatile uint8_t ordersBuffor[ORDERS_BUFFOR_SIZE];
volatile uint8_t globalI;
volatile Orders_Order forkliftOrders[ORDERS_LIST_SIZE];
volatile uint8_t globalOrders;
volatile Orders_Order gantryOrders[ORDERS_LIST_SIZE];
volatile uint8_t globalOrdersGantry;
volatile uint8_t globalType;
volatile uint8_t globalFlag;
volatile char * message;

//#################################### UART
#define USART_SLAVES_PORT USARTD0 /* Define select Usart */
USART_data_t USART_SLAVES; /* USART data struct */

#define USART_MOTHER_DEVICE_PORT USARTE0 /* Define select Usart */
USART_data_t USART_MOTHER_DEVICE; /* USART data struct */

#define USART_GANTRY_DEVICE_PORT USARTF0 /* Define select Usart */
USART_data_t USART_GANTRY_DEVICE; /* USART data struct */

void uartInit( void )
{
	//PORTD.DIRSET = PIN3_bm; /* PD3 (TXD0) as output. */
	//PORTD.DIRCLR = PIN2_bm; /* PD2 (RXD0) as input. */
	
	PORTD_REMAP |= 0x16; //See page 152 in datasheet, remaps the USART0
	
	PORTD_OUTSET = PIN7_bm; //Let's make PC7 as TX
	PORTD_DIRSET = PIN7_bm; //TX pin as output
	
	PORTD_OUTCLR = PIN6_bm;
	PORTD_DIRCLR = PIN6_bm; //PC6 as RX
	
	PORTF_OUTSET = PIN3_bm; //Let's make PC7 as TX
	PORTF_DIRSET = PIN3_bm; //TX pin as output
	
	//PORTF_OUTCLR = PIN2_bm;
	//PORTF_DIRCLR = PIN2_bm; //PC6 as RX
	
	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_SLAVES,&USART_SLAVES_PORT,USART_DREINTLVL_LO_gc);
	USART_InterruptDriver_Initialize(&USART_GANTRY_DEVICE,&USART_GANTRY_DEVICE_PORT,USART_DREINTLVL_LO_gc);
	USART_InterruptDriver_Initialize(&USART_MOTHER_DEVICE,&USART_MOTHER_DEVICE_PORT,USART_DREINTLVL_LO_gc);
	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_SLAVES.usart,USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc,false);
	USART_Format_Set(USART_GANTRY_DEVICE.usart,USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc,false);
	USART_Format_Set(USART_MOTHER_DEVICE.usart,USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc,false);
	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_SLAVES.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(USART_GANTRY_DEVICE.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(USART_MOTHER_DEVICE.usart, USART_RXCINTLVL_LO_gc);
	
	/* calculate Baudrate = (1/(16*(((I/O clock frequency)/Baudrate)-1) = 12 */
	/* BSEL[11:0]-Wert bei 32MHz Takt und BSCALE[3:0]==0: */
	/* 207 : 9600 */
	/* 103 : 19200 */
	/* 51 : 38400 */
	/* 34 : 57600 */
	/* 16 : 115200 */
	USART_Baudrate_Set(&USART_SLAVES_PORT, 12, 0);
	USART_Baudrate_Set(&USART_MOTHER_DEVICE_PORT, 12, 0);
	USART_Baudrate_Set(&USART_GANTRY_DEVICE_PORT, 12, 0);
	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_SLAVES.usart);
	USART_Tx_Enable(USART_SLAVES.usart);
	
	USART_Rx_Enable(USART_MOTHER_DEVICE.usart);
	USART_Tx_Enable(USART_MOTHER_DEVICE.usart);
	
	//USART_Rx_Enable(USART_GANTRY_DEVICE.usart);
	USART_Tx_Enable(USART_GANTRY_DEVICE.usart);
	
	PORTE_REMAP |= 0x16; //See page 152 in datasheet, remaps the USART0
	
	PORTE_OUTSET = PIN7_bm; //Let's make PC7 as TX
	PORTE_DIRSET = PIN7_bm; //TX pin as output
	
	PORTE_OUTCLR = PIN6_bm;
	PORTE_DIRCLR = PIN6_bm; //PC6 as RX
	
	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
}

void addCharToOrdersBuffor(char temporary)
{
	ordersBuffor[globalI] = temporary;
	if( globalI == (ORDERS_BUFFOR_SIZE - 1) )
	{
		globalI = 0;
	}else
		globalI++;
}

void showOrdersBufferToLCD()
{
	LcdClear();
	uint16_t i;
	for(i = 0; i< ORDERS_BUFFOR_SIZE; i++)
	{
		LcdDec(ordersBuffor[i]);
		if(i == 4){
			 Lcd2;
		}
	}	
}

void AddOrderToForklift()
{
	uint8_t ii;
	for(ii =0;ii<ORDER_SINGLE_SIZE;ii++)
	{
		forkliftOrders[globalOrders].ordersString[ORDER_SINGLE_SIZE-1-ii] = ordersBuffor[globalI-1-ii];		
	}
	
	if( globalOrders == (ORDERS_LIST_SIZE-1) )
	{
		globalOrders = 0;
	}else
		globalOrders++;
}
void AddOrderToGantry()
{
	uint8_t ii;
	for(ii =0;ii<ORDER_SINGLE_SIZE;ii++)
	{
		gantryOrders[globalOrdersGantry].ordersString[ORDER_SINGLE_SIZE-1-ii] = ordersBuffor[globalI-1-ii];
	}
	
	if( globalOrdersGantry == (ORDERS_LIST_SIZE-1) )
	{
		globalOrdersGantry = 0;
	}else
		globalOrdersGantry++;
}

void showOrderFromStruct(volatile Orders_Order* order, uint8_t sizeToShow)
{
	uint8_t i;
	for(i = 0; i< sizeToShow; i++)
	{
		//LcdDec(order->ordersString[i]);
		LcdWrite(order->ordersString);
	}
}
void sendOrderToSlaves(volatile Orders_Order* order)
{
	uint8_t i;
	for(i = 0; i< ORDER_SINGLE_SIZE; i++)
	{
		usart_putc(&USART_GANTRY_DEVICE,order->ordersString[i]);
	}
}

void sendOrderToGantry(volatile Orders_Order* order)
{
	uint8_t i;
	for(i = 0; i< ORDER_SINGLE_SIZE; i++)
	{
		usart_putc(&USART_GANTRY_DEVICE,order->ordersString[i]);
	}
}

ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&USART_MOTHER_DEVICE);

	char temporary = usart_getc(&USART_MOTHER_DEVICE);
	addCharToOrdersBuffor(temporary);
	
	if((temporary == '.') || (temporary == '/'))
	{
		if(globalType == 0)
		{
			 AddOrderToForklift();
		}else if(globalType == 1)
		{
			 AddOrderToGantry();
		}
	}
	
	if(temporary == 'Y')
	{			
		globalFlag = 1;
	}
	if(temporary == '_')
	{
		globalType = 1;
	}
}

ISR(USARTD0_DRE_vect)
{
	USART_DataRegEmpty(&USART_SLAVES);
}

ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_MOTHER_DEVICE);
}

ISR(USARTF0_DRE_vect)
{
	USART_DataRegEmpty(&USART_GANTRY_DEVICE);
}

int main(void) 
{
	cli();
	
	message = " Orders loaded!";
	uartInit();
	
	LcdInit();               
	LcdClear();
	
	Lcd(" Ready for orders list!");
	
	globalI = 0;
	globalOrders = 0;
	globalType = 0;
	globalOrdersGantry = 0;
	globalFlag = 0;
	PORTF_DIRSET = PIN4_bm;
	PORTF.OUTCLR = PIN4_bm;
	
	sei();
	while(1) {
		if(globalFlag == 1)
		{
			LcdClear();
			Lcd(" Orders autotriggered!");
			uint16_t i;
			usart_putc(&USART_GANTRY_DEVICE,'X');
			for(i = 0; i< globalOrders; i++)
			{
				sendOrderToSlaves(&forkliftOrders[i]);
			}
			usart_putc(&USART_GANTRY_DEVICE,'Y');
		
			PORTF.OUTSET = PIN4_bm;
			usart_putc(&USART_GANTRY_DEVICE,'X');
			for(i = 0; i< globalOrdersGantry; i++)
			{
				sendOrderToGantry(&gantryOrders[i]);
			}
			usart_putc(&USART_GANTRY_DEVICE,'Y');
			PORTF.OUTCLR = PIN4_bm;
			
			Lcd2;
			Lcd(" Orders loaded!");
			
			//LcdClear();
			//showOrderFromStruct(&gantryOrders[0], 1);
		
			globalType = 0;
			globalOrdersGantry = 0;
			globalOrders = 0;
			globalI = 0;
			globalFlag = 0;
		}
	}
}