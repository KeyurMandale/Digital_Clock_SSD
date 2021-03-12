#ifndef USART_Header
#define USART_Header

#include<avr/io.h>

void USART_Init(unsigned int baud_rate);
void USART_Transmit( unsigned char data );
unsigned char USART_Receive(void);

void USART_Init(unsigned int baud_rate)
{
	/* Set baud rate */
	UBRRH = (unsigned char)(baud_rate>>8);
	UBRRL = (unsigned char)baud_rate;
	/* Enable receiver and transmitter */
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	/* Set frame format: 8data, 2stop bit */
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
	//UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	return UDR;
}

#endif