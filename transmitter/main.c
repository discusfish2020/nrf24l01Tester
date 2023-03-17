#include "rf24l01.h"
#include "delay.h"
/*
RF24L01 stm8s demo
MASTER
This code sends two 32 bit ints to a slave, and gets the result of the basic
mathematic operations involving them.

This is a proof of concept and not a copy and use library.
Adapt this code to your application.

Erwan Martin
Fall 2013
ARENIB Delta
http://delta.arenib.org/

License: MIT

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PC3 -> CE
  PC4 -> CSN
  PC7 -> MISO
  PC6 -> MOSI
  PC5 -> SCK

*/

uint8_t mutex ; 
 
 


int main( void )
{
		 
	uint8_t tx_addr[5] = {0x04, 0xAD, 0x45, 0x98, 0x51};
  uint8_t rx_addr[5] = {0x44, 0x88, 0xBA, 0xBE, 0x42};
	
		
	/*High speed internal clock prescaler: 1 
   CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  /* Enable SPI clock 
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI , ENABLE);
	*/
	 CLK_DeInit();
	  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
		 
	
 
 GPIO_Init(GPIOB,GPIO_PIN_4,GPIO_MODE_IN_FL_IT); 
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
	
	enableInterrupts();
	
	RF24L01_init();
	
	
 while(NRF24L01_Check());
 
	GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);//LED
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//1
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(50);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//2
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(50);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//3
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(50);				
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//4
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(50);
  GPIO_WriteLow(GPIOB,GPIO_PIN_5);//5
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(50);

	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//6
	Delay_ms(100);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
  Delay_ms( 500);
 
 
 
 
  RF24L01_setup(tx_addr, rx_addr, 34);
  
	 

  while(1) {
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//1
	Delay_ms( 20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(200);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//2
	Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(200);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//3
			Delay_ms(20);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(400);
     
     mutex = 0;
			 
      RF24L01_set_mode_TX();
      
        RF24L01_write_payload("A", 1);
       
	  while(!mutex);
      if (mutex != 1) {
        //The transmission failed
      }
			
  }
}

INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3) {
    
}

@svlreg INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4) {
  uint8_t sent_info;
	 
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    mutex = sent_info;
    
    
		
		GPIO_WriteLow(GPIOB,GPIO_PIN_5);//1
		Delay_ms(200);
	 	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
		Delay_ms(30);
		GPIO_WriteLow(GPIOB,GPIO_PIN_5); //2
		Delay_ms(230);
		GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
		Delay_ms(30);
		GPIO_WriteLow(GPIOB,GPIO_PIN_5); //3
		Delay_ms(230);
		GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
		Delay_ms(30);
		GPIO_WriteLow(GPIOB,GPIO_PIN_5); //4
		Delay_ms(240);
		GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
		Delay_ms(230);
RF24L01_clear_interrupts();
		
    return;
		
  }
  if(RF24L01_is_data_available()) {
    //Packet was received
    mutex = 1;
    RF24L01_clear_interrupts();
    return;
  }
  
  RF24L01_clear_interrupts();
}
