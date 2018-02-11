#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "avr_compiler.h"

/** @brief  UART Baudrate Expression
 *  @param  xtalcpu  system clock in Mhz, e.g. 4000000L for 4Mhz          
 *  @param  baudrate baudrate in bps, e.g. 1200, 2400, 9600     
 */
#define USART_BAUD_SELECT(baudRate,xtalCpu)					((xtalCpu)/((baudRate)*16l)-1)
#define USART_BAUD_SELECT_DOUBLE_SPEED(baudRate,xtalCpu)	(((xtalCpu)/((baudRate)*8l)-1)|0x8000)

/* USART buffer defines. */
#define USART_RX_BUFFER_SIZE 32								// brief  Receive buffer size: 2,4,8,16,32,64,128 or 256 bytes. 
#define USART_TX_BUFFER_SIZE 256								// Transmit buffer size: 2,4,8,16,32,64,128 or 256 bytes 
#define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )		// Receive buffer mask. 
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )		// Transmit buffer mask. 


#if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif

/* test if the size of the circular buffers fits into SRAM */
#if ( (USART_RX_BUFFER_SIZE+USART_TX_BUFFER_SIZE) >= (RAMEND-0x60 ) )
	#error "size of USART_RX_BUFFER_SIZE + USART_TX_BUFFER_SIZE larger than size of SRAM"
#endif

/* \brief USART transmit and receive ring buffer. */
typedef struct USART_Buffer
{
	volatile char RX[USART_RX_BUFFER_SIZE];						// Receive buffer
	volatile char TX[USART_TX_BUFFER_SIZE];						// Transmit buffer
	volatile uint16_t RX_Head;									// Receive buffer head
	volatile uint16_t RX_Tail;									// Receive buffer tail
	volatile uint16_t TX_Head;									// Transmit buffer head
	volatile uint16_t TX_Tail;									// Transmit buffer tail
} USART_Buffer_t;


/* 
** high byte error return code of uart_getc()					// todo error integration
*/
#define USART_FRAME_ERROR      0x0800							// Framing Error by UART       
#define USART_OVERRUN_ERROR    0x0400            				// Overrun condition by UART   
#define USART_BUFFER_OVERFLOW  0x0200            				// receive ringbuffer overflow 
#define USART_NO_DATA          0x0100            				// no receive data available   


/*! \brief Struct used when interrupt driven driver is used.
*
*  Struct containing pointer to a usart, a buffer and a location to store Data
*  register interrupt level temporary.
*/
typedef struct Usart_and_buffer
{	
	USART_t * usart;            								// Pointer to USART module to use. 	
	USART_DREINTLVL_t dreIntLevel;            					// Data register empty interrupt level. 	
	USART_Buffer_t buffer;				            			// Data buffer. 
} USART_data_t;


/* Macros. */

/*! \brief Macro that sets the USART frame format.
 *
 *  Sets the frame format, Frame Size, parity mode and number of stop bits.
 *
 *  \param _usart        Pointer to the USART module
 *  \param _charSize     The character size. Use USART_CHSIZE_t type.
 *  \param _parityMode   The parity Mode. Use USART_PMODE_t type.
 *  \param _twoStopBits  Enable two stop bit mode. Use bool type.
 */
#define USART_Format_Set(_usart, _charSize, _parityMode, _twoStopBits)		(_usart)->CTRLC = (uint8_t) _charSize | _parityMode | (_twoStopBits ? USART_SBMODE_bm : 0)


/*! \brief Set USART baud rate.
 *
 *  Sets the USART's baud rate register.
 *
 *  UBRR_Value   : Value written to UBRR
 *  ScaleFactor  : Time Base Generator Scale Factor
 *
 *  Equation for calculation of BSEL value in asynchronous normal speed mode:
 *  	If ScaleFactor >= 0
 *  		BSEL = ((I/O clock frequency)/(2^(ScaleFactor)*16*Baudrate))-1
 *  	If ScaleFactor < 0
 *  		BSEL = (1/(2^(ScaleFactor)*16))*(((I/O clock frequency)/Baudrate)-1)
 *
 *	\note See XMEGA manual for equations for calculation of BSEL value in other
 *        modes.
 *
 *  \param _usart          Pointer to the USART module.
 *  \param _bselValue      Value to write to BSEL part of Baud control register.
 *                         Use uint16_t type.
 *  \param _bScaleFactor   USART baud rate scale factor.
 *                         Use uint8_t type
 */
#define USART_Baudrate_Set(_usart, _bselValue, _bScaleFactor)		(_usart)->BAUDCTRLA =(uint8_t)_bselValue; (_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)

#define USART_Rx_Enable(_usart)			((_usart)->CTRLB |= USART_RXEN_bm)		// Enable USART receiver.
#define USART_Rx_Disable(_usart)		((_usart)->CTRLB &= ~USART_RXEN_bm)		// Disable USART receiver.
#define USART_Tx_Enable(_usart)			((_usart)->CTRLB |= USART_TXEN_bm)		// Enable USART transmitter.
#define USART_Tx_Disable(_usart)		((_usart)->CTRLB &= ~USART_TXEN_bm)		// Disable USART transmitter.
#define USART_CLK2X_Enable(_usart)		((_usart)->CTRLB |= USART_CLK2X_bm)		// Enable Double Speed Modus

/*! \brief Set USART RXD interrupt level.
 *
 *  Sets the interrupt level on RX Complete interrupt.
 *
 *  \param _usart        Pointer to the USART module.
 *  \param _rxdIntLevel  Interrupt level of the RXD interrupt.
 *                       Use USART_RXCINTLVL_t type.
 */
#define USART_RxdInterruptLevel_Set(_usart, _rxdIntLevel)   ((_usart)->CTRLA = ((_usart)->CTRLA & ~USART_RXCINTLVL_gm) | _rxdIntLevel)


/*! \brief Set USART TXD interrupt level.
 *
 *  Sets the interrupt level on TX Complete interrupt.
 *
 *  \param _usart        Pointer to the USART module.
 *  \param _txdIntLevel  Interrupt level of the TXD interrupt.
 *                       Use USART_TXCINTLVL_t type.
 */
#define USART_TxdInterruptLevel_Set(_usart, _txdIntLevel)   (_usart)->CTRLA = ((_usart)->CTRLA & ~USART_TXCINTLVL_gm) | _txdIntLevel


/*! \brief Set USART DRE interrupt level.
 *
 *  Sets the interrupt level on Data Register interrupt.
 *
 *  \param _usart        Pointer to the USART module.
 *  \param _dreIntLevel  Interrupt level of the DRE interrupt.
 *                       Use USART_DREINTLVL_t type.
 */
#define USART_DreInterruptLevel_Set(_usart, _dreIntLevel)   (_usart)->CTRLA = ((_usart)->CTRLA & ~USART_DREINTLVL_gm) | _dreIntLevel


/*! \brief Set the mode the USART run in.
 *
 * Set the mode the USART run in. The default mode is asynchronous mode.
 *
 *  \param  _usart       Pointer to the USART module register section.
 *  \param  _usartMode   Selects the USART mode. Use  USART_CMODE_t type.
 *
 *  USART modes:
 *  - 0x0        : Asynchronous mode.
 *  - 0x1        : Synchronous mode.
 *  - 0x2        : IrDA mode.
 *  - 0x3        : Master SPI mode.
 */
#define USART_SetMode(_usart, _usartMode)  ((_usart)->CTRLC = ((_usart)->CTRLC & (~USART_CMODE_gm)) | _usartMode)


/*! \brief Check if data register empty flag is set.
 *
 *  \param _usart      The USART module.
 */
#define USART_IsTXDataRegisterEmpty(_usart) (((_usart)->STATUS & USART_DREIF_bm) != 0)


/*! \brief Put data (5-8 bit character).
 *
 *  Use the macro USART_IsTXDataRegisterEmpty before using this function to
 *  put data to the TX register.
 *
 *  \param _usart      The USART module.
 *  \param _data       The data to send.
 */
#define USART_PutChar(_usart, _data) ((_usart)->DATA = _data)


/*! \brief Checks if the RX complete interrupt flag is set.
 *
 *   Checks if the RX complete interrupt flag is set.
 *
 *  \param _usart     The USART module.
 */
#define USART_IsRXComplete(_usart) (((_usart)->STATUS & USART_RXCIF_bm) != 0)


/*! \brief Get received data (5-8 bit character).
 *
 *  This macro reads out the RX register.
 *  Use the macro USART_RX_Complete to check if anything is received.
 *
 *  \param _usart     The USART module.
 *
 *  \retval           Received data.
 */
#define USART_GetChar(_usart)  ((_usart)->DATA)


/**
 *  @brief   Put string to ringbuffer for transmitting via UART
 *
 *  The string is buffered by the uart library in a circular buffer
 *  and one character at a time is transmitted to the UART using interrupts.
 *  Blocks if it can not write the whole string into the circular buffer.
 * 
 *  @param   s string to be transmitted
 *  @return  none
 */
#define usart_puts_P(__s)       usart_puts_p(PSTR(__s))	// Macro to automatically put a string constant into program memory




void	usart_init(USART_data_t * usart_data, USART_t * usart, unsigned int baudrate);

bool	usart_putc(USART_data_t * usart_data, char data);
void	usart_puts(USART_data_t * usart_data, const char *str );
void	usart_puts_p(USART_data_t * usart_data, const char *str );

unsigned char usart_getc(USART_data_t * usart_data);


void USART_InterruptDriver_Initialize(USART_data_t * usart_data, USART_t * usart, USART_DREINTLVL_t dreIntLevel );
void USART_InterruptDriver_DreInterruptLevel_Set(USART_data_t * usart_data, USART_DREINTLVL_t dreIntLevel);

bool USART_TXBuffer_FreeSpace(USART_data_t * usart_data);
bool USART_RXBufferData_Available(USART_data_t * usart_data);
bool USART_RXComplete(USART_data_t * usart_data);
void USART_DataRegEmpty(USART_data_t * usart_data);

/* Functions for polled driver. */
void USART_NineBits_PutChar(USART_t * usart, uint16_t data);
uint16_t USART_NineBits_GetChar(USART_t * usart);

#endif
