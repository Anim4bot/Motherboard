#include <Robot.h>


Leg_st Leg[6];


void WriteLegPos(uint8_t speed)
{
	// ADD SYNC MOVE ???

	// Read the calculated joint angles for each leg, then write the angle for each legs.
/*
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
*/
	// LEFT FRONT LEG
	DRS0101_setAngle(ID_LF_COXA,  Leg[LEFT_FRONT].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_FEMUR, Leg[LEFT_FRONT].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_TIBIA, Leg[LEFT_FRONT].Joint.TibiaJointAngle, speed, DRS0101_PLED);
/*
	// LEFT MIDDLE LEG
	DRS0101_setAngle(ID_LM_COXA,  Leg[LEFT_MIDDLE].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, Leg[LEFT_MIDDLE].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, Leg[LEFT_MIDDLE].Joint.TibiaJointAngle, speed, DRS0101_PLED);

	// LEFT REAR LEG
	DRS0101_setAngle(ID_LR_COXA,  Leg[LEFT_REAR].Joint.CoxaJointAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, Leg[LEFT_REAR].Joint.FemurJointAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, Leg[LEFT_REAR].Joint.TibiaJointAngle, speed, DRS0101_PLED);
	*/
}


//======================================================================================
// Leg Inverse Kinematics
//======================================================================================
// Calculates the angles of the coxa, femur and tibia for the given position of the feet
// ikFeetPosX            - Input position of the Feet X
// ikFeetPosY            - Input position of the Feet Y
// ikFeetPosZ            - Input Position of the Feet Z
// legIKLedNr            - Input Leg Index for initCoxaAngle
// femurAngle            - Output Angle of Femur in degrees
// tibiaAngle            - Output Angle of Tibia in degrees
// coxaAngle             - Output Angle of Coxa in degrees
//======================================================================================

const float initCoxaAngle = 0;

void LegIK (int legNr, float PosX, float PosY, float PosZ)
{
	float LegLentgh;
	float HF;
	float A1, A1Deg;
	float A2, A2Deg;
	float B1, B1Deg;
	float CoxaAngle, FemurAngle, TibiaAngle;

	LegLentgh = sqrtf( pow(PosX,2) + pow(PosY,2) );
	HF = sqrtf( pow((LegLentgh-COXA_LENGTH),2) + pow(PosZ,2) );
	A1 = atan2f( (LegLentgh-COXA_LENGTH), PosZ );
	A2 = acos( (pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2) - pow(HF,2) ) / (-2*FEMUR_LENGTH*TIBIA_LENGTH) );
	B1 = acos( (pow(HF,2) - pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2) ) / (-2*FEMUR_LENGTH*TIBIA_LENGTH) );

	A1Deg = RadtoDeg(A1);
	A2Deg = RadtoDeg(A2);
	B1Deg = RadtoDeg(B1);

	CoxaAngle  = atan2(PosX, PosY);
	FemurAngle = 90 - (A1Deg + A2Deg);
	TibiaAngle = 90 - B1Deg;


	Leg[legNr].Joint.CoxaJointAngle  = CoxaAngle;
	Leg[legNr].Joint.FemurJointAngle = FemurAngle;
	Leg[legNr].Joint.TibiaJointAngle = TibiaAngle;

	DRS0101_setAngle(ID_LF_COXA,  Leg[LEFT_FRONT].Joint.CoxaJointAngle,  70, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_FEMUR, Leg[LEFT_FRONT].Joint.FemurJointAngle, 70, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_TIBIA, Leg[LEFT_FRONT].Joint.TibiaJointAngle, 70, DRS0101_PLED);

    //WriteLegPos(70);

/*
	//Right Side
	for( legNum=0; legNum<3; legNum++ )
	{
		Leg[legNum].Joint.CoxaJointAngle  = Leg[legNum].Joint.CoxaJointAngle;
		Leg[legNum].Joint.FemurJointAngle = Leg[legNum].Joint.FemurJointAngle;
		Leg[legNum].Joint.TibiaJointAngle = Leg[legNum].Joint.TibiaJointAngle;
	}
	//Left Side
	for( legNum=3; legNum<6; legNum++ )
	{
		Leg[legNum].Joint.CoxaJointAngle  = Leg[legNum].Joint.CoxaJointAngle;
		Leg[legNum].Joint.FemurJointAngle = -(Leg[legNum].Joint.FemurJointAngle);
		Leg[legNum].Joint.TibiaJointAngle = -(Leg[legNum].Joint.TibiaJointAngle);
	}
*/
}

