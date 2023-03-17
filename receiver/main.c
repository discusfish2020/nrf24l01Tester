/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
 /*
RF24L01 stm8s demo
MASTER
This code receives two 32 bit ints from a master, and sends the result of the basic
mathematic operations involving them.
This is a proof of concept and not a copy and use library.
Adapt this code to your application.
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
 
 
 
 
//#include "stm8s.h"
#include "rf24l01.h"
#include "delay.h"
uint8_t mutex,flag_send,count=0;
uint8_t recv_data[1];
uint8_t data_sent[1]; 
  
int main( void ) {
 
  uint8_t  rx_addr[5]={0x04, 0xAD, 0x45, 0x98, 0x51}; 
  uint8_t tx_addr[5] = {0x44, 0x88, 0xBA, 0xBE, 0x42};
 
  CLK_DeInit();
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
	 	
	
	
	GPIO_Init(  GPIOB,GPIO_PIN_4,GPIO_MODE_IN_FL_IT);
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
		
	
  enableInterrupts();
	
	GPIO_Init (GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); 
	
  RF24L01_init(); 
	while(NRF24L01_Check());
	
	
	RF24L01_setup(tx_addr, rx_addr, 34);

   
	
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);
	Delay_ms( 200);
	   
  		 
			       

while (1) {     
      
			
			 RF24L01_set_mode_RX();
				GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
			Delay_ms( 100);
			 
			mutex=0;
		 
    
      while(!mutex);
      if (mutex == 145) {
					RF24L01_read_payload(recv_data, 1);
					if(recv_data[0]!='A') 
					{
							disableInterrupts();	
							//halt();
				  }   
			 
					mutex=0;
	    //  halt();
				 
				
		}
       
  }

}




@svlreg INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4) {
  uint8_t sent_info;
 // mutex=1;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    flag_send=1;
    mutex = sent_info;
	
    RF24L01_clear_interrupts();
    return;
  }

  if(RF24L01_is_data_available()) {
    //Packet was received
    //GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
    mutex = 145;
    count++;
			GPIO_WriteLow(GPIOB,GPIO_PIN_5);//1
	Delay_ms(60);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(40);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//2
	Delay_ms(60);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(40);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//3
	Delay_ms(60);	
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(40);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//4
	Delay_ms(60);
	GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
	Delay_ms(40);
	GPIO_WriteLow(GPIOB,GPIO_PIN_5);//5
	Delay_ms(60);
		
		
    RF24L01_clear_interrupts();
    return;
  }
  
  RF24L01_clear_interrupts();
}





INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3) {
/*	uint8_t sent_info;
 // mutex=1;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    flag_send=1;
    mutex = sent_info;
    RF24L01_clear_interrupts();
    return;
  }

  if(RF24L01_is_data_available()) {
    //Packet was received
    //GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
    mutex = 145;
    count++;
    RF24L01_clear_interrupts();
    return;
  }
  
  RF24L01_clear_interrupts();
	
	
	
	
  /*uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    mutex = sent_info;
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
*/}
