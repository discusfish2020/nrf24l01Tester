#include "stm8s.h"
 static void delay(__IO uint32_t nCount){
 /* 
 
 The delay function implemented in this driver is not a precise one, however it allows the insertion of 1ms delay when Fcpu is 16Mhz if the passed parameter is 0x4000. Any change in system clock frequency will impact this delay duration.
 Decrement nCount value */
   while (nCount != 0)
   {
     nCount--;
   }
 }
 void delay_us(__IO uint32_t us)
{
	
    uint32_t  i;
		
		for(i=0;i<us;i++){
    /* Decrement nCount value */
				 delay(0x4 );

		}
}
/*
 void delay_ms(__IO uint32_t  ms)
{ 
    		
		uint32_t  i;
		
		for(i=0;i<ms;i++){
    
				 delay(0x4000);

		}
}
*/

  void Delay_us_asm(  uint32_t n)  //改为延时1US
{
    for(;n>0;n--) 
    { 
        _asm("nop");  //在STM8里面，16M晶振，_nop_() 延时了 333ns
        _asm("nop");  
        _asm("nop");  
        _asm("nop");  
    }
}  void Delay_ms(uint32_t time)
 {
		unsigned int i;
		while(time--)
		for(i=300;i>0;i--)
		Delay_us_asm(1);
 }