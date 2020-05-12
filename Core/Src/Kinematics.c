#include <Robot.h>


Leg_st Leg[6];


void WriteLegIK(uint8_t speed)
{
	// ADD SYNC MOVE ???
	// Read the calculated joint angles for each leg, then write the angle for each legs.

	// RIGHT FRONT LEG
	DRS0101_setAngle(ID_RF_COXA,  Leg[RIGHT_FRONT].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_FEMUR, Leg[RIGHT_FRONT].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RF_TIBIA, Leg[RIGHT_FRONT].Joint.TibiaAngle, speed, DRS0101_PLED);

	// RIGHT MIDDLE LEG
	DRS0101_setAngle(ID_RM_COXA,  Leg[RIGHT_MIDDLE].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_FEMUR, Leg[RIGHT_MIDDLE].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RM_TIBIA, Leg[RIGHT_MIDDLE].Joint.TibiaAngle, speed, DRS0101_PLED);

	// RIGHT REAR LEG
	DRS0101_setAngle(ID_RR_COXA,  Leg[RIGHT_REAR].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_FEMUR, Leg[RIGHT_REAR].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_RR_TIBIA, Leg[RIGHT_REAR].Joint.TibiaAngle, speed, DRS0101_PLED);

	// LEFT FRONT LEG
	DRS0101_setAngle(ID_LF_COXA,  Leg[LEFT_FRONT].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_FEMUR, Leg[LEFT_FRONT].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LF_TIBIA, Leg[LEFT_FRONT].Joint.TibiaAngle, speed, DRS0101_PLED);

	// LEFT MIDDLE LEG
	DRS0101_setAngle(ID_LM_COXA,  Leg[LEFT_MIDDLE].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_FEMUR, Leg[LEFT_MIDDLE].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LM_TIBIA, Leg[LEFT_MIDDLE].Joint.TibiaAngle, speed, DRS0101_PLED);

	// LEFT REAR LEG
	DRS0101_setAngle(ID_LR_COXA,  Leg[LEFT_REAR].Joint.CoxaAngle,  speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_FEMUR, Leg[LEFT_REAR].Joint.FemurAngle, speed, DRS0101_PLED);
	DRS0101_setAngle(ID_LR_TIBIA, Leg[LEFT_REAR].Joint.TibiaAngle, speed, DRS0101_PLED);

}


void LegIK_Init(void)
{
	/* INITIAL FOOT POSITIONS */
	const uint8_t initLegStretch = 30;
	const uint8_t rideHeightOffset = 50;

    Leg[RIGHT_FRONT].InitFootPos.x = RadtoDeg(sinf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_FRONT].InitFootPos.y = RadtoDeg(cosf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_FRONT].InitFootPos.z = rideHeightOffset;
    Leg[RIGHT_FRONT].LegBasePos.x = X_COXA;
    Leg[RIGHT_FRONT].LegBasePos.y = Y_COXA_FRONT_REAR;
    Leg[RIGHT_FRONT].LegBasePos.z = 0;

    Leg[RIGHT_MIDDLE].InitFootPos.x = 0;
    Leg[RIGHT_MIDDLE].InitFootPos.y = (COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_MIDDLE].InitFootPos.z = rideHeightOffset;
    Leg[RIGHT_MIDDLE].LegBasePos.x = 0;
    Leg[RIGHT_MIDDLE].LegBasePos.y = Y_COXA_MIDDLE;
    Leg[RIGHT_MIDDLE].LegBasePos.z = 0;

    Leg[RIGHT_REAR].InitFootPos.x = RadtoDeg(sinf(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_REAR].InitFootPos.y = RadtoDeg(cosf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_REAR].InitFootPos.z = rideHeightOffset;
    Leg[RIGHT_REAR].LegBasePos.x = -X_COXA;
    Leg[RIGHT_REAR].LegBasePos.y = Y_COXA_FRONT_REAR;
    Leg[RIGHT_REAR].LegBasePos.z = 0;

    Leg[LEFT_FRONT].InitFootPos.x = RadtoDeg(sinf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_FRONT].InitFootPos.y = -RadtoDeg(cosf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_FRONT].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_FRONT].LegBasePos.x = X_COXA;
    Leg[LEFT_FRONT].LegBasePos.y = -Y_COXA_FRONT_REAR;
    Leg[LEFT_FRONT].LegBasePos.z = 0;

    Leg[LEFT_MIDDLE].InitFootPos.x = 0;
    Leg[LEFT_MIDDLE].InitFootPos.y = -(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_MIDDLE].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_MIDDLE].LegBasePos.x = 0;
    Leg[LEFT_MIDDLE].LegBasePos.y = -Y_COXA_MIDDLE;
    Leg[LEFT_MIDDLE].LegBasePos.z = 0;

    Leg[LEFT_REAR].InitFootPos.x = RadtoDeg(sinf(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_REAR].InitFootPos.y = -RadtoDeg(cosf(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_REAR].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_REAR].LegBasePos.x = -X_COXA;
    Leg[LEFT_REAR].LegBasePos.y = -Y_COXA_FRONT_REAR;
    Leg[LEFT_REAR].LegBasePos.z = 0;

}


void BodyIK (float Tx, float Ty, float Tz, float Rx, float Ry, float Rz)
{
	//Pitch = Rotation of Y axis
	//Roll  = Rotation of X axis
	//Yaw   = Rotation of Z axis

	uint8_t legNr;
	float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
	float bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];
	Coord_st globalInitFootPos;

    for( legNr=0; legNr<6; legNr++)
    {
    	// Distance from center of body to foot position
    	globalInitFootPos.x = Leg[legNr].InitFootPos.x + Leg[legNr].LegBasePos.x;
		globalInitFootPos.y = Leg[legNr].InitFootPos.y + Leg[legNr].LegBasePos.y;
		globalInitFootPos.z = Leg[legNr].InitFootPos.z + Leg[legNr].LegBasePos.z;

		// Foot position offsets necessary to acheive given body rotation
        bodyRotOffsetX[legNum] = ( globalInitFootPos.y*cosRotY*sinRotZ + globalInitFootPos.y*cosRotZ*sinRotX*sinRotY + globalInitFootPos.x*cosRotZ*cosRotY - globalInitFootPos.x*sinRotZ*sinRotX*sinRotY - globalInitFootPos.z*cosRotX*sinRotY) - globalInitFootPos.x;
        bodyRotOffsetY[legNum] =   globalInitFootPos.y*cosRotX*cosRotZ - globalInitFootPos.x*cosRotX*sinRotZ         + globalInitFootPos.z*sinRotX         - globalInitFootPos.y;
        bodyRotOffsetZ[legNum] = ( globalInitFootPos.y*sinRotZ*sinRotY - globalInitFootPos.y*cosRotZ*cosRotY*sinRotX + globalInitFootPos.x*cosRotZ*sinRotY + globalInitFootPos.x*cosRotY*sinRotZ*sinRotX + globalInitFootPos.z*cosRotX*cosRotY) - globalInitFootPos.z;

    }










/*
	float LF_x2, LF_y2, LF_R, LF_B;
	float LM_x2, LM_y2, LM_R, LM_B;
	float LR_x2, LR_y2, LR_R, LR_B;


	// Feet position regarding Body center O0
	LF_x2 = Leg[LEFT_FRONT].Foot.PosX = PosX + LEG_FRONT_X_OFFSET;
	LF_y2 = Leg[LEFT_FRONT].Foot.PosX = PosX + LEG_FRONT_Y_OFFSET;
	LF_R  = sqrtf( powf(LF_x2,2) + powf(LF_y2,2) );
	LF_B  = RadtoDeg(atan2f(LF_x2, LF_y2));
*/

}


void LegIK(uint8_t legNr, float PosX, float PosY, float PosZ)
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
		Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
		Leg[legNr].Joint.FemurAngle = -(FemurAngle + 10);
		Leg[legNr].Joint.TibiaAngle = -(-TibiaAngle - 10);
	}
	if(legNr == 3 || legNr == 4 || legNr == 5)
	{
		//LEFT Side
		Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
		Leg[legNr].Joint.FemurAngle = (FemurAngle + 10);
		Leg[legNr].Joint.TibiaAngle = (-TibiaAngle - 10);
	}
}



