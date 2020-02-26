#include "SubRoutines.h"
#include "PlatformType.h"

void BIP_0(void)
{
	PWM_BUZZ = 50;
	osDelay(10);
	PWM_BUZZ = 0;
}


void BIP_1(void)
{
	PWM_BUZZ = 50;
	osDelay(20);
	PWM_BUZZ = 0;
	osDelay(100);
	PWM_BUZZ = 100;
	osDelay(20);
	PWM_BUZZ = 0;
}


void BIP_2(void)
{
	PWM_BUZZ = 50;
	osDelay(5);
	PWM_BUZZ = 0;
	osDelay(10);
	PWM_BUZZ = 100;
	osDelay(5);
	PWM_BUZZ = 0;
}

void BIP_3(void)
{
	PWM_BUZZ = 50;
	osDelay(5);
	PWM_BUZZ = 0;
	osDelay(10);
	PWM_BUZZ = 100;
	osDelay(5);
	PWM_BUZZ = 0;
}
