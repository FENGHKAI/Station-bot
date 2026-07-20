#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "timer.h"


int main()
{
	encoder_init();
	MotorControl_Init();
	TIM6_init();

	while(1)
	{

	}
	
}
