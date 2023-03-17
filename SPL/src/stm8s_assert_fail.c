#include ¡°stm8s_conf.h¡± 
#ifdef USE_FULL_ASSERT 

void assert_failed(u8 file, u32 line)
{ 
/* Infinite loop */
while (1) { }
} 
#endif