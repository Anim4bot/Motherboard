#include <Robot.h>
#include "DRS0101.h"


void Gaits_StandPosition(uint8_t speed)
{
	DRS0101_setTorque(BROADCAST_ID, TORQUE_ON);
	DRS0101_Clear(BROADCAST_ID);

	DRS0101_setAngle(ID_COXA_LF, 0, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_COXA_RF, 0, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_COXA_RM, 0, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_COXA_LM, 0, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_COXA_RR, 0, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_COXA_LR, 0, speed, DRS0101_PLED);

	DRS0101_setAngle(ID_FEMUR_LF, -20, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_FEMUR_RF, -20, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_FEMUR_RM, -20, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_FEMUR_LM, -20, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_FEMUR_RR, -20, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_FEMUR_LR, -20, speed, DRS0101_PLED);


}
