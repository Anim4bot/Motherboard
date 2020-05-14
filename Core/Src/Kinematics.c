#include "Robot.h"


Leg_st Leg[6];


void IK_Run(void)
{
	uint8_t speed = 100;

	IK_Body();
	IK_BodyLegs();
	IK_DriveServos(speed);
}


void IK_LegInit(void)
{
	/* INITIAL FOOT POSITIONS */
	uint8_t legNr;
	const int16_t initLegStretch = 00;
	const int16_t rideHeightOffset = 50;

    Leg[RIGHT_FRONT].InitFootPos.x = round(sinf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_FRONT].InitFootPos.y = round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
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

    Leg[RIGHT_REAR].InitFootPos.x = round(sinf(DegtoRad(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_REAR].InitFootPos.y = round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_REAR].InitFootPos.z = rideHeightOffset;
    Leg[RIGHT_REAR].LegBasePos.x = -X_COXA;
    Leg[RIGHT_REAR].LegBasePos.y = Y_COXA_FRONT_REAR;
    Leg[RIGHT_REAR].LegBasePos.z = 0;

    Leg[LEFT_REAR].InitFootPos.x = round(sinf(DegtoRad(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_REAR].InitFootPos.y = -round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_REAR].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_REAR].LegBasePos.x = -X_COXA;
    Leg[LEFT_REAR].LegBasePos.y = -Y_COXA_FRONT_REAR;
    Leg[LEFT_REAR].LegBasePos.z = 0;

    Leg[LEFT_MIDDLE].InitFootPos.x = 0;
    Leg[LEFT_MIDDLE].InitFootPos.y = -(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_MIDDLE].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_MIDDLE].LegBasePos.x = 0;
    Leg[LEFT_MIDDLE].LegBasePos.y = -Y_COXA_MIDDLE;
    Leg[LEFT_MIDDLE].LegBasePos.z = 0;

    Leg[LEFT_FRONT].InitFootPos.x = round(sinf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_FRONT].InitFootPos.y = -round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_FRONT].InitFootPos.z = rideHeightOffset;
    Leg[LEFT_FRONT].LegBasePos.x = X_COXA;
    Leg[LEFT_FRONT].LegBasePos.y = -Y_COXA_FRONT_REAR;
    Leg[LEFT_FRONT].LegBasePos.z = 0;

    for( legNr=0; legNr<6; legNr++)
    {
    	Leg[legNr].FootPos.x = 0;
    	Leg[legNr].FootPos.y = 0;
    	Leg[legNr].FootPos.z = 0;
    }

}


void IK_Body(void)
{
	//Pitch = Rotation of Y axis
	//Roll  = Rotation of X axis
	//Yaw   = Rotation of Z axis

	uint8_t legNr;
	float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
	float bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];
	Coord_st globalInitFootPos;

	sinRotX = sin(DegtoRad(Robot.BodyCmd.RotX));
	cosRotX = cos(DegtoRad(Robot.BodyCmd.RotX));
	sinRotY = sin(DegtoRad(Robot.BodyCmd.RotY));
	cosRotY = cos(DegtoRad(Robot.BodyCmd.RotY));
	sinRotZ = sin(DegtoRad(Robot.BodyCmd.RotZ));
	cosRotZ = cos(DegtoRad(Robot.BodyCmd.RotZ));

    for( legNr=0; legNr<6; legNr++)
    {
    	// Distance from center of body to foot position
    	globalInitFootPos.x = Leg[legNr].InitFootPos.x + Leg[legNr].LegBasePos.x;
		globalInitFootPos.y = Leg[legNr].InitFootPos.y + Leg[legNr].LegBasePos.y;
		globalInitFootPos.z = Leg[legNr].InitFootPos.z + Leg[legNr].LegBasePos.z;

		// Foot position offsets necessary to acheive given body rotation
        bodyRotOffsetX[legNr] = ( globalInitFootPos.y*cosRotY*sinRotZ + globalInitFootPos.y*cosRotZ*sinRotX*sinRotY + globalInitFootPos.x*cosRotZ*cosRotY - globalInitFootPos.x*sinRotZ*sinRotX*sinRotY - globalInitFootPos.z*cosRotX*sinRotY) - globalInitFootPos.x;
        bodyRotOffsetY[legNr] =   globalInitFootPos.y*cosRotX*cosRotZ - globalInitFootPos.x*cosRotX*sinRotZ         + globalInitFootPos.z*sinRotX         - globalInitFootPos.y;
        bodyRotOffsetZ[legNr] = ( globalInitFootPos.y*sinRotZ*sinRotY - globalInitFootPos.y*cosRotZ*cosRotY*sinRotX + globalInitFootPos.x*cosRotZ*sinRotY + globalInitFootPos.x*cosRotY*sinRotZ*sinRotX + globalInitFootPos.z*cosRotX*cosRotY) - globalInitFootPos.z;

        // Calculated foot positions to achieve translation/rotation input. Not coxa mounting angle corrected
        bodyRotOffsetX[legNr] = Leg[legNr].InitFootPos.x + bodyRotOffsetX[legNr] - Robot.BodyCmd.TransX + Leg[legNr].FootPos.x;
        bodyRotOffsetY[legNr] = Leg[legNr].InitFootPos.y + bodyRotOffsetY[legNr] - Robot.BodyCmd.TransY + Leg[legNr].FootPos.y;
        bodyRotOffsetZ[legNr] = Leg[legNr].InitFootPos.z + bodyRotOffsetZ[legNr] - Robot.BodyCmd.TransZ + Leg[legNr].FootPos.z;

        // Rotates X,Y about coxa to compensate for coxa mounting angles.
        Leg[0].CalcFootPos.x = bodyRotOffsetY[0]*cos(DegtoRad(90-COXA_ANGLE))   - bodyRotOffsetX[0]*sin(DegtoRad(90-COXA_ANGLE));
        Leg[0].CalcFootPos.y = bodyRotOffsetY[0]*sin(DegtoRad(90-COXA_ANGLE))   + bodyRotOffsetX[0]*cos(DegtoRad(90-COXA_ANGLE));
        Leg[0].CalcFootPos.z = bodyRotOffsetZ[0];
        Leg[1].CalcFootPos.x = bodyRotOffsetY[1]*cos(DegtoRad(90)) - bodyRotOffsetX[1]*sin(DegtoRad(90));
        Leg[1].CalcFootPos.y = bodyRotOffsetY[1]*sin(DegtoRad(90)) + bodyRotOffsetX[1]*cos(DegtoRad(90));
        Leg[1].CalcFootPos.z = bodyRotOffsetZ[1];
        Leg[2].CalcFootPos.x = bodyRotOffsetY[2]*cos(DegtoRad(90+COXA_ANGLE)) - bodyRotOffsetX[2]*sin(DegtoRad(90+COXA_ANGLE));
        Leg[2].CalcFootPos.y = bodyRotOffsetY[2]*sin(DegtoRad(90+COXA_ANGLE)) + bodyRotOffsetX[2]*cos(DegtoRad(90+COXA_ANGLE));
        Leg[2].CalcFootPos.z = bodyRotOffsetZ[2];
        Leg[3].CalcFootPos.x = bodyRotOffsetY[3]*cos(DegtoRad(-90-COXA_ANGLE)) - bodyRotOffsetX[3]*sin(DegtoRad(-90-COXA_ANGLE));
        Leg[3].CalcFootPos.y = bodyRotOffsetY[3]*sin(DegtoRad(-90-COXA_ANGLE)) + bodyRotOffsetX[3]*cos(DegtoRad(-90-COXA_ANGLE));
        Leg[3].CalcFootPos.z = bodyRotOffsetZ[3];
        Leg[4].CalcFootPos.x = bodyRotOffsetY[4]*cos(DegtoRad(-90)) - bodyRotOffsetX[4]*sin(DegtoRad(-90));
        Leg[4].CalcFootPos.y = bodyRotOffsetY[4]*sin(DegtoRad(-90)) + bodyRotOffsetX[4]*cos(DegtoRad(-90));
        Leg[4].CalcFootPos.z = bodyRotOffsetZ[4];
        Leg[5].CalcFootPos.x = bodyRotOffsetY[5]*cos(DegtoRad(-90+COXA_ANGLE)) - bodyRotOffsetX[5]*sin(DegtoRad(-90+COXA_ANGLE));
        Leg[5].CalcFootPos.y = bodyRotOffsetY[5]*sin(DegtoRad(-90+COXA_ANGLE)) + bodyRotOffsetX[5]*cos(DegtoRad(-90+COXA_ANGLE));
        Leg[5].CalcFootPos.z = bodyRotOffsetZ[5];
    }
}

void IK_BodyLegs()
{
	uint8_t legNr;
	volatile float LegLentgh;
	volatile float HF;
	volatile float A1, A1Deg;
	volatile float A2, A2Deg;
	volatile float B1, B1Deg;
	volatile float CoxaAngle, FemurAngle, TibiaAngle;


	for(legNr=0; legNr<6; legNr++ )
	{
		if(Leg[legNr].CalcFootPos.x > 50) {Leg[legNr].CalcFootPos.x = 50; }
		if(Leg[legNr].CalcFootPos.x < -50)  {Leg[legNr].CalcFootPos.x = -50; }
		if(Leg[legNr].CalcFootPos.y > 140) {Leg[legNr].CalcFootPos.y = 140; }
		if(Leg[legNr].CalcFootPos.y < 10)  {Leg[legNr].CalcFootPos.y = 10; }
		if(Leg[legNr].CalcFootPos.z > 140) {Leg[legNr].CalcFootPos.z = 140; }
		if(Leg[legNr].CalcFootPos.z < 10)  {Leg[legNr].CalcFootPos.z = 10; }

		LegLentgh = sqrtf( powf(Leg[legNr].CalcFootPos.x,2) + powf(Leg[legNr].CalcFootPos.y,2) );
		HF = sqrtf( powf((LegLentgh-COXA_LENGTH),2) + powf(Leg[legNr].CalcFootPos.z,2) );
		A1 = atan2f( (LegLentgh-COXA_LENGTH), Leg[legNr].CalcFootPos.z );
		A2 = acosf( (powf(TIBIA_LENGTH,2) - powf(FEMUR_LENGTH,2) - powf(HF,2) ) / (-2*FEMUR_LENGTH*HF) );
		B1 = acosf( (powf(HF,2) - powf(TIBIA_LENGTH,2) - powf(FEMUR_LENGTH,2) ) / (-2*FEMUR_LENGTH*TIBIA_LENGTH) );

		A1Deg = RadtoDeg(A1);
		A2Deg = RadtoDeg(A2);
		B1Deg = RadtoDeg(B1);

		CoxaAngle  = RadtoDeg(atan2f(Leg[legNr].CalcFootPos.x, Leg[legNr].CalcFootPos.y));
		FemurAngle = 90.0 - (A1Deg + A2Deg);
		TibiaAngle = 90.0 - B1Deg;


		if(legNr == 0 || legNr == 1 || legNr == 2)
		{
			//RIGHT Side
			Leg[legNr].Joint.CoxaAngle  = CoxaAngle;
			Leg[legNr].Joint.FemurAngle = -(FemurAngle + 10);
			Leg[legNr].Joint.TibiaAngle = -(-TibiaAngle - 10);
		}
		if(legNr == 3 || legNr == 4 || legNr == 5)
		{
			//LEFT Side
			Leg[legNr].Joint.CoxaAngle  = CoxaAngle;
			Leg[legNr].Joint.FemurAngle = (FemurAngle + 10);
			Leg[legNr].Joint.TibiaAngle = (-TibiaAngle - 10);
		}
	}
}



void IK_Leg(uint8_t legNr, float PosX, float PosY, float PosZ)
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



void IK_DriveServos(uint8_t speed)
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
