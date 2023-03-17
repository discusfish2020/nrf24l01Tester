// it allows the insertion of 1ms delay when Fcpu is 16Mhz if 
   //  the passed parameter is 0x4000.
 
 static void delay(__IO uint32_t nCount);
  void delay_us(__IO uint32_t us); 	           
  void Delay_ms( uint32_t time); 
  void Delay_us_asm( uint32_t n); 
 
