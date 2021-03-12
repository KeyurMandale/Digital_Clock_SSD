#ifndef twi_header
#define twi_header

#define F_CPU 11059200UL
#include <util/delay.h>
#include <avr/io.h>
#include <util/twi.h>


void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_rstart(void);
void twi_SLA_W(uint8_t Slave_Write_Addr);
void twi_SLA_R(uint8_t Slave_Read_Addr);
void twi_SendData(uint8_t Data_Byte);
uint8_t twi_ReadNAck(void);
uint8_t twi_ReadRegister(uint8_t Register_Adrress);
void twi_Write_Register(uint8_t Register_Address, uint8_t Register_Data);
void Encode_time(void);

uint8_t ACK_Err = 0;
uint8_t T_Secs = 22;
uint8_t R_Secs = 0;



void twi_init(void)
{
	TWBR = 2;               // Setting SCL to 50 KHz 
	TWCR |= 1 << TWEN;
}

void twi_start(void)
{
	
	TWCR = (1 << TWINT | 1 << TWSTA | 1 << TWEN);
	while(!(TWCR & (1 << TWINT))); 
	while(((TWSR & 0xF8) != 0x08) && ((TWSR & 0xF8) != 0x10)); 
}

void twi_stop(void)
{
	TWCR = (1 << TWEN | 1 << TWSTO | 1 << TWINT);
}

void twi_rstart(void)
{
	TWCR = (1 << TWINT | 1 << TWSTA | 1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
	while((TWSR & 0xF8) != 0x10);
}

void twi_SLA_W(uint8_t Slave_Write_Addr)
{   
	
	TWDR = Slave_Write_Addr;
	TWCR = (1 << TWEN | 1 << TWINT);
	while(!(TWCR & (1 << TWINT)));
	while(((TWSR & 0xF8) != 0x18) && ((TWSR & 0xF8) != 0x20));
	if ((TWSR & 0xF8) == 0x20) ACK_Err = 1;
}

void twi_SLA_R(uint8_t Slave_Read_Addr)
{
	TWDR = Slave_Read_Addr;
	TWCR =(1 << TWEN | 1 << TWINT);
	while(!(TWCR & (1 << TWINT)));
	while(((TWSR & 0xF8) != 0x40) && ((TWSR & 0xF8) != 0x48));
	if((TWSR & 0xF8) == 0x48) ACK_Err = 1;
}

void twi_SendData(uint8_t Data_Byte)
{
	DDRD |=(1 << PIND6 | 1 << PIND5);
	TWDR = Data_Byte;
	TWCR = (1 << TWEN | 1 << TWINT);
	while(!(TWCR & (1 << TWINT)));
	while(((TWSR & 0xF8) != 0x28) && ((TWSR & 0xF8) != 0x30));
	if((TWSR & 0xF8) == 0x30) ACK_Err = 1;
	/*if ((TWSR & 0xF8) == 0x30)
	{
		PORTD |= 1 << PIND6;
	}
	else
	{
		PORTD |= 1 << PIND5;
	}*/
}

uint8_t twi_ReadNAck(void)
{
	TWCR = (1 << TWINT | 1 << TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}
uint8_t twi_ReadRegister(uint8_t Register_Adrress)
{
	twi_start();
	twi_SLA_W(0xD0);
	twi_SendData(Register_Adrress);
	twi_rstart();
	twi_SLA_R(0xD1);
	
    uint8_t temp_value = twi_ReadNAck();
	twi_stop();
	
	if (ACK_Err)
	{
		return 0XFF;
	}
	else return temp_value;
}
void twi_Write_Register(uint8_t Register_Address, uint8_t Register_Data)
{
	twi_start();
	//_delay_ms(100);
	twi_SLA_W(0xD0);
	//_delay_ms(100);
	twi_SendData(Register_Address);
	//_delay_ms(100);
	twi_SendData(Register_Data);
	//_delay_ms(100);
	twi_stop();
	//_delay_ms(100);
}

void Encode_time(void)
{
	uint8_t t_secs = T_Secs;
	
	T_Secs = 0 ;
	T_Secs = t_secs % 10;
	T_Secs |= ((t_secs / 10) << 4);
}

void Decode_time(void)
{
	uint8_t t_secs = R_Secs;
	
	R_Secs = 0;
	R_Secs = t_secs & 0b00001111;
	t_secs &= 0b01110000;
	R_Secs +=((t_secs >> 4) * 10);
	
}
#endif