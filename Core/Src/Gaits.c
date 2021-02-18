#include <Robot.h>
#include "DRS0101.h"


void Gaits_PackPosition(uint8_t speed)
{
	int16_t angleCoxa = 60;
	int16_t angleFemur = 100;
	int16_t angleTibia = 80;

	DRS0101_setTorque(BROADCAST_ID, TORQUE_ON);
	DRS0101_Clear(BROADCAST_ID);

	DRS0101_setAngle(ID_LF_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_FEMUR, +angleFemur, speed, DRS0101_PLED);

	DRS0101_setAngle(ID_LF_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_TIBIA, +angleTibia, speed, DRS0101_PLED);

	osDelay(750);

	DRS0101_setAngle(ID_LF_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_COXA, +angleCoxa, speed, DRS0101_PLED);
}

void Gaits_DefaultPosition(uint8_t speed)
{
	int16_t angleCoxa = 0;
	int16_t angleFemur = 70;
	int16_t angleTibia = 70;

	DRS0101_setTorque(BROADCAST_ID, TORQUE_ON);
	DRS0101_Clear(BROADCAST_ID);

	DRS0101_setAngle(ID_LF_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_COXA, +angleCoxa, speed, DRS0101_PLED);

	osDelay(500);

	DRS0101_setAngle(ID_LF_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_FEMUR, +angleFemur, speed, DRS0101_PLED);

	DRS0101_setAngle(ID_LF_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_TIBIA, +angleTibia, speed, DRS0101_PLED);

}

void Servos_ZeroPosition(uint8_t speed)
{
	int16_t angleCoxa = 0;
	int16_t angleFemur = 0;
	int16_t angleTibia = 0;

	DRS0101_setTorque(BROADCAST_ID, TORQUE_ON);
	DRS0101_Clear(BROADCAST_ID);

	DRS0101_setAngle(ID_LF_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_COXA, -angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_COXA, +angleCoxa, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_COXA, +angleCoxa, speed, DRS0101_PLED);

	osDelay(500);

	DRS0101_setAngle(ID_LF_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, -angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_FEMUR, +angleFemur, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_FEMUR, +angleFemur, speed, DRS0101_PLED);

	DRS0101_setAngle(ID_LF_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, -angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_TIBIA, +angleTibia, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_TIBIA, +angleTibia, speed, DRS0101_PLED);

}



void IK_StandbyPos(uint8_t speed)
{

	Robot.LegPos.Init_X = 0;
	Robot.LegPos.Init_Y = 100;
	Robot.LegPos.Init_Z = -25;


	IK_SingleLegDrive(0, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);
	IK_SingleLegDrive(1, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);
	IK_SingleLegDrive(2, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);
	IK_SingleLegDrive(3, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);
	IK_SingleLegDrive(4, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);
	IK_SingleLegDrive(5, Robot.LegPos.Init_X, Robot.LegPos.Init_Y, Robot.LegPos.Init_Z, speed);

}

