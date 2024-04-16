#include "tof.h"
void TofInit()
{
    while(dl1a_init())
	{
		delay_ms(500);
        P52 = !P52;
	}
}
void TofShowDistance()
{
    if(dl1a_finsh_flag)
	{                        
		dl1a_finsh_flag = 0;
	}
    
}
