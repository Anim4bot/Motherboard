#include "SubRoutines.h"
#include "PlatformType.h"
#include "Robot.h"

void BIP_0(void)
{
	PWM_BUZZ = 5;
	osDelay(30);
	PWM_BUZZ = 0;

	osDelay(50);

	PWM_BUZZ = 5;
	osDelay(30);
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
}

void BIP_3(void)
{
	PWM_BUZZ = 50;
	osDelay(50);
	PWM_BUZZ = 0;
}


void BoardShutdownProcedure(void)
{
	uint8_t i;

	BIP_3();
	osDelay(1000);
	set_PSU_5V(OFF);
	osDelay(500);
	set_PSU_DRS0101(OFF);

	for(i=Robot.OLED.OLED_Contrast ; i>1 ; i--)
	{
		Flex_OLED_setContrast(i);
		osDelay(2);
	}

	osDelay(1500);
	set_MAIN_SWITCH(OFF);
	osDelay(500);
	set_PSU_3V3(OFF);

}


