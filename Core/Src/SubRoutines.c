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

void BIP_4(void)
{
	PWM_BUZZ = 50;
	osDelay(200);
	PWM_BUZZ = 0;
}

void BIP_5(void)
{
	PWM_BUZZ = 50;
	osDelay(25);
	PWM_BUZZ = 0;
}




void BoardShutdownProcedure(void)
{
	BIP_3();
	Flex_OLED_StartupAnimation(10);
	Gaits_PackPosition(150);
	Ears_SetPosition(EarL_Down, EarR_Down, slow);
	Head_SetPosition(TiltNeutral, PanNeutral, slow);
	Eyes_GoingToSleep(slow);
	set_EYES_RST(ON);

	osDelay(1000);

	set_PSU_DRS0101(OFF);
	osDelay(100);
	set_PSU_5V(OFF);
	osDelay(100);
	set_MAIN_SWITCH(OFF);
	osDelay(100);
	set_PSU_3V3(OFF);
}


