#include <Robot.h>


Leg_st Leg[6];


void WriteLegIK(uint8_t speed)
{
	// ADD SYNC MOVE ???
	// Read the calculated joint angles for each leg, then write the angle for each legs.

	// RIGHT FRONT LEG
	DRS0101_setAngle(ID_RF_COXA,  Leg[RIGHT_FRONT].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_FEMUR, Leg[RIGHT_FRONT].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_TIBIA, Leg[RIGHT_FRONT].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// RIGHT MIDDLE LEG
	DRS0101_setAngle(ID_RM_COXA,  Leg[RIGHT_MIDDLE].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_FEMUR, Leg[RIGHT_MIDDLE].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_TIBIA, Leg[RIGHT_MIDDLE].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// RIGHT REAR LEG
	DRS0101_setAngle(ID_RR_COXA,  Leg[RIGHT_REAR].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_FEMUR, Leg[RIGHT_REAR].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_TIBIA, Leg[RIGHT_REAR].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// LEFT FRONT LEG
	DRS0101_setAngle(ID_LF_COXA,  Leg[LEFT_FRONT].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_FEMUR, Leg[LEFT_FRONT].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_TIBIA, Leg[LEFT_FRONT].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// LEFT MIDDLE LEG
	DRS0101_setAngle(ID_LM_COXA,  Leg[LEFT_MIDDLE].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, Leg[LEFT_MIDDLE].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, Leg[LEFT_MIDDLE].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// LEFT REAR LEG
	DRS0101_setAngle(ID_LR_COXA,  Leg[LEFT_REAR].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, Leg[LEFT_REAR].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, Leg[LEFT_REAR].Joint.TibiaJointAngle, speed, DRS0101_PLED);

}


void LegIK (int legNr, float PosX, float PosY, float PosZ)
{
	volatile float LegLentgh;
	volatile float HF;
	volatile float A1, A1Deg;
	volatile float A2, A2Deg;
	volatile float B1, B1Deg;
	volatile float CoxaAngle, FemurAngle, TibiaAngle;

	LegLentgh = sqrtf( powf(PosX,2) + powf(PosY,2) );
	HF = sqrtf( powf((LegLentgh-COXA_LENGTH),2) + powf(PosZ,2) );
	A1 = atan2f( (LegLentgh-COXA_LENGTH), PosZ );
	A2 = acosf( (powf(TIBIA_LENGTH,2) - powf(FEMUR_LENGTH,2) - powf(HF,2) ) / (-2*FEMUR_LENGTH*HF) );
	B1 = acosf( (powf(HF,2) - powf(TIBIA_LENGTH,2) - powf(FEMUR_LENGTH,2) ) / (-2*FEMUR_LENGTH*TIBIA_LENGTH) );

	A1Deg = RadtoDeg(A1);
	A2Deg = RadtoDeg(A2);
	B1Deg = RadtoDeg(B1);

	CoxaAngle  = RadtoDeg(atan2f(PosX, PosY));
	FemurAngle = 90.0 - (A1Deg + A2Deg);
	TibiaAngle = 90.0 - B1Deg;


	if(legNr == 0 || legNr == 1 || legNr == 2)
	{
		//RIGHT Side
		Leg[legNr].Joint.CoxaJointAngle  = +CoxaAngle;
		Leg[legNr].Joint.FemurJointAngle = -(FemurAngle + 10);
		Leg[legNr].Joint.TibiaJointAngle = -(-TibiaAngle - 10);
	}
	if(legNr == 3 || legNr == 4 || legNr == 5)
	{
		//LEFT Side
		Leg[legNr].Joint.CoxaJointAngle  = -CoxaAngle;
		Leg[legNr].Joint.FemurJointAngle = (FemurAngle + 10);
		Leg[legNr].Joint.TibiaJointAngle = (-TibiaAngle - 10);
	}



	/*
	//Right Side
	for( legNum=0; legNum<3; legNum++ )
	{
		Leg[legNum].Joint.CoxaJointAngle  = +CoxaAngle;
		Leg[legNum].Joint.FemurJointAngle = -(FemurAngle + 10);
		Leg[legNum].Joint.TibiaJointAngle = -(-TibiaAngle - 10);
	}
	//Left Side
	for( legNum=3; legNum<6; legNum++ )
	{

	}
*/

}

