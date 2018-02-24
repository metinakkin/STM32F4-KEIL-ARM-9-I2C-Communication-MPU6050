#include "stm32f4xx.h"                  // Device header
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
 
#define SLAVE_ADDRESS 0x68 // the slave address (example)00111000 01110000 70

char str[50];
uint32_t i;
uint16_t a,b,c,a1,b1,c1;
uint8_t m,m1;
int16_t d,d1;
void USART_Puts(USART_TypeDef* USARTx,volatile char *s) // char karakter sayisi kadar döndürüyor
{
 while(*s)
 {
  while(!(USARTx ->SR & 0x00000040));
	 USART_SendData(USARTx,*s);
	 *s++;
 }	 
}	
long map(long x,long in_min,long in_max,long out_min,long out_max)
{
return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
void init_I2C1(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7 
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)); // 0==0 >>1
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

int main(void){
	
	init_I2C1(); // initialize I2C peripheral
	
	uint8_t received_data[3];
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); // transmitter tx A2 GPIOya bagli usartta
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 ;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; //Alternatif fonksiyonlar(input,output,adc disinda baska sey oldugu)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);//USARTtan bilgiyi GPIOya atacaz,CPUya tanitmamiz lazim
	
	USART_InitStructure.USART_BaudRate=115200; // Saniye içinde hat üzerinden kaç tane bit gönderilmesi gerektigi
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
	
	while(1){
		
  	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x6B); // write one byte to the slave // PWR_MGMT_1 register /* MPU-6050 çalistirildi */
		I2C_write(I2C1, 0x00); // write one byte to the slave // set to zero (wakes up the MPU-6050)
		I2C_stop(I2C1); // stop the transmission
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x1C); // write one byte to the slave //
		I2C_write(I2C1, 0x00); // write one byte to the slave // set to zero (wakes up the MPU-6050)
		I2C_stop(I2C1); // stop the transmission
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x3B); // write one byte to the slave
		I2C_stop(I2C1); // stop the transmission
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		a = (I2C_read_nack(I2C1)); // read one byte and request another byte
		 m=(a) & (0x80);
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x3C); // write one byte to the slave
		I2C_stop(I2C1); // stop the transmission
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		b = I2C_read_nack(I2C1); // read one byte and request another byte
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x3E); // write one byte to the slave
		I2C_stop(I2C1); // stop the transmission
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		b1 = I2C_read_nack(I2C1); // read one byte and request another byte
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x3D); // write one byte to the slave
		I2C_stop(I2C1); // stop the transmission
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		a1 = (I2C_read_nack(I2C1)); // read one byte and request another byte
		 m1=(a1) & (0x80);
		 
	
		
    //c=((a<<8) | b);
		//c1=((a1<<8) | b1);
		/*if(m==0x00)
		{
			sprintf(str,"X=%d ",c);
			USART_Puts(USART2,str);
		}
		if(m==0x80)
		{
			c=(~c);
			c=c+0x01;
		 sprintf(str,"X=%d ",-c);
			USART_Puts(USART2,str);
		}
		if(m1==0x00)
		{
		
			sprintf(str,"Y=%d, \n",c1);
			USART_Puts(USART2,str);
		}
		if(m1==0x80)
		{
			c1=(~c1);
			c1=c1+0x01;
			
		 sprintf(str,"Y=%d, \n",-c1);
			USART_Puts(USART2,str);
		}*/
//		sprintf(str,"X=%d \n",t);
//		//sprintf(str,"%d, %d \n",received_data[0],received_data[1] );
//		USART_Puts(USART2,str);
		d=((a<<8) | b);
		d1=((a1<<8) | b1);
		  
	  d=map(d,-17000,17000,0,10);
		d1=map(d1,-17000,17000,0,10);
		 sprintf(str,"X=%d, ",d);
			USART_Puts(USART2,str);
			sprintf(str,"Y=%d, \n",d1);
			USART_Puts(USART2,str);
		if(d>=7 && 2<d1 && d1<7 ) //Kirmizi
		{
		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		GPIO_SetBits(GPIOD,GPIO_Pin_13 | GPIO_Pin_14);
		}	
			if(d<=2 && 2<d1 && d1<7 ) //Yesil
		{
	  GPIO_ResetBits(GPIOD,GPIO_Pin_13);
		GPIO_SetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14);
		}
		if(d1>=7 && 2<d && d<7 ) //Mavi
		{
		GPIO_ResetBits(GPIOD,GPIO_Pin_14);
		GPIO_SetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13);
		}	
			if(d1<=2 && 2<d && d<7 ) //Mor
		{
	  GPIO_ResetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_14 );
		GPIO_SetBits(GPIOD,GPIO_Pin_13);
		}
			if(2<d && d<7 && 2<d1 && d1<7 ) //Beyaz
		{
	  
		GPIO_ResetBits(GPIOD,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
		}
		
		
		
	  i=600000;
		while(i)
			i--;
		
	
		
		
	}
}