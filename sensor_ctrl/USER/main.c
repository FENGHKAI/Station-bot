#include "sys.h"
#include "delay.h"
#include "hardware.h"

void key_stop()
{
	LED_Toggle();
	Buzzer_Toggle();
}

int main()
{
	delay_init(168);
	LED_Init();
	Buzzer_Init();
	Key_Init();

	Key_SetCallback(* key_stop);
	
	while(1)
	{
		
	}
	
}
