
#include <avr/io.h>
#define F_CPU 11059200UL
#include <util/delay.h>
#include <util/twi.h>
#include "twi_header.h"
#include "USART_Header.h"
#include <avr/interrupt.h>

/*************************74HC595********************************/
#define DS_Pin       0
#define SHCP         2
#define STCP         3
#define SIPO         PORTB
#define SIPO_Direct  DDRB

/**************************SSD***********************************/
#define H			 6		// PINB6 
#define h			 7		// PINB7. Had to use another port as PIND0 and PIND1 are used for UART comm with HC-05 
#define M			 3		 
#define m			 4		 
#define S	         5
#define s            6
#define am_pm        7   // This SSD is a CA type


void init_shift(void);
void shiftout(uint8_t num);
void RTC_Update_Time(void);
void Clear_Time_Buffer(void);
void Send_Time(void);
void Send_error(void);

volatile unsigned char Data_Received[10];
volatile uint8_t clock_digits[7];
volatile uint8_t bluetooth_flag = 0;
volatile uint8_t UART_Index = 0;

uint8_t numbers[12]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x08,0x0C}	;
	
uint8_t errors[3]={0x79,0x50,0x5C};
	
int main(void)
{ 	
	sei();
	twi_init();
    init_shift();
	USART_Init(71);
	
    
    while (1) 
    {
		Send_Time();
		if ((bluetooth_flag == 1) && (UART_Index == 10))
		{
			bluetooth_flag = 0;
			UART_Index     = 0;
			RTC_Update_Time();
		}
		if (Data_Received[UART_Index-1]=='c'||Data_Received[UART_Index-1]=='C')
		{
			bluetooth_flag = 0;
			UART_Index     = 0;
			Clear_Time_Buffer();
		}
			
    }

}

void init_shift(void)
{
	SIPO_Direct |= 1 << DS_Pin | 1 << SHCP | 1 << STCP | 1 << H | 1 << h;
	SIPO &= ~((1 << DS_Pin) | (1 << SHCP) | (1 << STCP) | (1 << H) | (1 << h));
	DDRD |= ( 1 << M | 1 << m | 1 << S | 1 << s | 1 << am_pm );
	PORTD &= ~(1 << M | 1 << m | 1 << S | 1 << s | 1 << am_pm);
	
}

void shiftout(uint8_t num)
{
	SIPO &= ~(1 << STCP);
	uint8_t counter = 8;
	while(counter > 0)
	{
		SIPO &= ~(1 << SHCP);
		if(((num & 0b10000000) >> 7))  SIPO |= 1 << DS_Pin;
		else                           SIPO &= ~(1 << DS_Pin);
		SIPO |= 1 << SHCP;
		num = num << 1;
		counter --;
	}
    SIPO |= 1 << STCP;	

}

ISR(USART_RXC_vect)
{
	bluetooth_flag = 1;
	Data_Received[UART_Index] = UDR;
	UART_Index ++;	
	if (UART_Index > 10)
	{
		UCSRA &= ~(1 << RXCIE);
		UART_Index = 0;
		bluetooth_flag =0;
		Clear_Time_Buffer();
		UCSRA |= 1 << RXCIE;
	}
}

void RTC_Update_Time(void)
{
	uint8_t temp_num = 0;
	uint8_t err_flag = 0;
	for (int j = 3; j < 10; j++)
	{
		switch(Data_Received[j])
		{
			case 0:
			case '0':
				temp_num = 0;
				break;
			case 1:
			case '1':
				temp_num = 1;
				break;
			case 2:
			case '2':
				temp_num = 2;
				break;
			case 3:
			case '3':
				temp_num = 3;
				break;
			case 4:
			case '4':
				temp_num = 4;
				break;
			case 5:
			case '5':
				temp_num = 5;
				break;
			case 6:
			case '6':
				temp_num = 6;
				break;
			case 7:
			case '7':
				temp_num = 7;
				break;
			
			case 8:
			case '8':
				temp_num = 8;
				break;
			case 9:
			case '9':
				temp_num = 9;
				break;
			case 'a':
			    temp_num = 0;
				break;
			case 'p':
				temp_num = 1;
				break;	
			default:
			    err_flag = 1;
				Send_error();
				break;
		}
	if(err_flag == 0)
	 {
		 	switch(j)
		 {
			case 3:
			    if (temp_num > 1)
			    {
					err_flag = 1;
					Send_error();
					break;
			    }
				else
				{
					clock_digits[0] = temp_num;
					break;	
				}
			case 4:
				clock_digits[1] = temp_num;
				break;
			case 5:
				if (temp_num > 5)
				{
					err_flag = 1;
					Send_error();
					break;
				}
				else
				{
					clock_digits[2] = temp_num;
					break;					
				}

			case 6:
				clock_digits[3] = temp_num;
				break;
			case 7:
				if (temp_num > 5)
				{
					err_flag = 1;
					Send_error();
					break;
				}
				else
				{
					clock_digits[4] = temp_num;
					break;					
				}
			case 8:
				clock_digits[5] = temp_num;
				break;
			case 9:
				if (temp_num == 1 || temp_num == 0)
				{
					if (temp_num == 0)
					{
						clock_digits[6] = numbers[10];
						break;
					}
					else
					{
						clock_digits[6] = numbers[11];
						break;
					}
				}
				else
				{
					err_flag = 1;
					Send_error();
					break;
				}
			}
		}	
	 }
	if(err_flag == 0) 
	 {
		temp_num = 0;
		temp_num |= clock_digits[5];
		temp_num |= (clock_digits[4] << 4);
		twi_Write_Register(0b00000000,temp_num);
	
		temp_num = 0;
		temp_num |= clock_digits[3];
		temp_num |= (clock_digits[2] << 4);
		twi_Write_Register(0b00000001,temp_num);
	
		temp_num = 0b01000000;
		if (clock_digits[6]==0x08)
		{
			temp_num |= 0x00;
		}
		if (clock_digits[6]==0x0C)
		{
			temp_num |= 0x20;
		}
		temp_num |= clock_digits[1];
		temp_num |= (clock_digits[0] << 4);
		twi_Write_Register(0b00000010,temp_num);
	 }
}

void Send_Time(void)
{
		uint8_t temp_sec = 0;
		uint8_t temp_hr  = 0;
		uint8_t temp_min = 0;
		uint8_t units_digit = 0;
		uint8_t tens_digit = 0;
		uint8_t am_pm_bit =0;
		
	    temp_hr = twi_ReadRegister(0b00000010);
		am_pm_bit   = ( temp_hr & 0b00100000);
		
	    units_digit = temp_hr & 0b00001111;
	    tens_digit  = temp_hr & 0b00010000;
	    tens_digit = (tens_digit >> 4);

	    clock_digits[0]=tens_digit;
	    clock_digits[1]=units_digit;
	    
	    temp_min = twi_ReadRegister(0b00000001);
	    units_digit = temp_min & 0b00001111;
	    tens_digit  = temp_min & 0b01110000;
	    tens_digit = (tens_digit >> 4);
	    
	    clock_digits[2]=tens_digit;
	    clock_digits[3]=units_digit;
	    
	    temp_sec = twi_ReadRegister(0b00000000);
	    units_digit = 0;
	    tens_digit  = 0;
	    units_digit = temp_sec & 0b00001111;
	    tens_digit = temp_sec & 0b01110000;
	    tens_digit = (tens_digit >> 4);
	    
	    clock_digits[4]=tens_digit;
	    clock_digits[5]=units_digit;
	    
		if (am_pm_bit > 0 )
		{
			clock_digits[6]=numbers[11];
		}
		if (am_pm_bit == 0)
		{
			clock_digits[6]=numbers[10];
		}
		
	    shiftout(numbers[clock_digits[0]]);
	    PORTD &= 0b00000011;
	    //PORTD = 1 << Unit_Digit;
	    SIPO |= (1 << H);
	    _delay_ms(1);
	    SIPO &= ~(1 << H);
	    
	    shiftout(numbers[clock_digits[1]]);
	    //PORTD = 1 << Tens_Digit;
	    PORTD &= 0b00000011;
	    SIPO |= (1 << h);
	    _delay_ms(1);
	    SIPO &= ~(1 << h);
	    
	    shiftout(numbers[clock_digits[2]]);
	    PORTD = 1 << M;
	    _delay_ms(1);
	    
	    shiftout(numbers[clock_digits[3]]);
	    PORTD = 1 << m;
	    _delay_ms(1);
	    
	    shiftout(numbers[clock_digits[4]]);
	    PORTD = 1 << S;
	    _delay_ms(1);

	    
	    shiftout(numbers[clock_digits[5]]);
	    PORTD = 1 << s;
	    _delay_ms(1);
		
		shiftout(clock_digits[6]);
		PORTD = 1 << am_pm;
		_delay_ms(1);
}

void Clear_Time_Buffer(void)
{
	for (int i=0; i < 10; i++)
	{
		Data_Received[i]=' ';
	}
}

void Send_error(void)
{
	uint8_t count = 0;
	while(count < 200)
	{
		    shiftout(errors[0]);
		    PORTD &= 0b00000011;
		    //PORTD = 1 << Unit_Digit;
		    SIPO |= (1 << H);
		    _delay_ms(1);
		    SIPO &= ~(1 << H);
		    
		    shiftout(errors[1]);
		    //PORTD = 1 << Tens_Digit;
		    PORTD &= 0b00000011;
		    SIPO |= (1 << h);
		    _delay_ms(1);
		    SIPO &= ~(1 << h);
		    
		    shiftout(errors[1]);
		    PORTD = 1 << M;
		    _delay_ms(1);
		    
		    shiftout(errors[2]);
		    PORTD = 1 << m;
		    _delay_ms(1);
		    
		    shiftout(errors[1]);
		    PORTD = 1 << S;
		    _delay_ms(1);

		    
		    shiftout(0x00);
		    PORTD = 1 << s;
		    _delay_ms(1);
			
			shiftout(0xFF);
			PORTD = 1 << am_pm;
			_delay_ms(1);
			count ++;
	}
}