#include "Robot.h"


Leg_st Leg[6];


void IK_Run(void)
{
	uint8_t speed = 150;

	IK_Body_Complex();
	//IK_Body_Light();
	//IK_BodyLegs();
	IK_BodyLegs2();
	IK_DriveServos(speed);
}


void IK_LegInit(void)
{
	/* INITIAL FOOT POSITIONS */
	uint8_t legNr;
	const int16_t initLegStretch = 200;
	const int16_t HeightOffset = 0;

    Leg[RIGHT_FRONT].LegBasePos.x = X_COXA_FRONT;
    Leg[RIGHT_FRONT].LegBasePos.y = Y_COXA_FRONT;
    Leg[RIGHT_FRONT].LegBasePos.z = 0;

    Leg[RIGHT_MIDDLE].LegBasePos.x = X_COXA_MIDDLE;
    Leg[RIGHT_MIDDLE].LegBasePos.y = Y_COXA_MIDDLE;
    Leg[RIGHT_MIDDLE].LegBasePos.z = 0;

    Leg[RIGHT_REAR].LegBasePos.x = -X_COXA_REAR;
    Leg[RIGHT_REAR].LegBasePos.y = Y_COXA_REAR;
    Leg[RIGHT_REAR].LegBasePos.z = 0;

    Leg[LEFT_REAR].LegBasePos.x = -X_COXA_REAR;
    Leg[LEFT_REAR].LegBasePos.y = -Y_COXA_REAR;
    Leg[LEFT_REAR].LegBasePos.z = 0;

    Leg[LEFT_MIDDLE].LegBasePos.x = X_COXA_MIDDLE;
    Leg[LEFT_MIDDLE].LegBasePos.y = -Y_COXA_MIDDLE;
    Leg[LEFT_MIDDLE].LegBasePos.z = 0;

    Leg[LEFT_FRONT].LegBasePos.x = X_COXA_FRONT;
    Leg[LEFT_FRONT].LegBasePos.y = -Y_COXA_FRONT;
    Leg[LEFT_FRONT].LegBasePos.z = 0;

/*
    Leg[RIGHT_FRONT].InitFootPos.x = round(sinf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_FRONT].InitFootPos.y = round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_FRONT].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[RIGHT_MIDDLE].InitFootPos.x = 0;
    Leg[RIGHT_MIDDLE].InitFootPos.y = (COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[RIGHT_MIDDLE].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[RIGHT_REAR].InitFootPos.x = round(sinf(DegtoRad(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_REAR].InitFootPos.y = round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[RIGHT_REAR].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_REAR].InitFootPos.x = round(sinf(DegtoRad(-COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_REAR].InitFootPos.y = -round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_REAR].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_MIDDLE].InitFootPos.x = 0;
    Leg[LEFT_MIDDLE].InitFootPos.y = -(COXA_LENGTH + FEMUR_LENGTH + initLegStretch);
    Leg[LEFT_MIDDLE].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_FRONT].InitFootPos.x = round(sinf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_FRONT].InitFootPos.y = -round(cosf(DegtoRad(COXA_ANGLE))*(COXA_LENGTH + FEMUR_LENGTH + initLegStretch));
    Leg[LEFT_FRONT].InitFootPos.z = TIBIA_LENGTH + HeightOffset;
*/

    Leg[RIGHT_FRONT].InitFootPos.x = 40;
    Leg[RIGHT_FRONT].InitFootPos.y = 90;
    Leg[RIGHT_FRONT].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[RIGHT_MIDDLE].InitFootPos.x = 0;
    Leg[RIGHT_MIDDLE].InitFootPos.y = 90;
    Leg[RIGHT_MIDDLE].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[RIGHT_REAR].InitFootPos.x = -40;
    Leg[RIGHT_REAR].InitFootPos.y = 90;
    Leg[RIGHT_REAR].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_REAR].InitFootPos.x = -40;
    Leg[LEFT_REAR].InitFootPos.y = -90;
    Leg[LEFT_REAR].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_MIDDLE].InitFootPos.x = 0;
    Leg[LEFT_MIDDLE].InitFootPos.y = -90;
    Leg[LEFT_MIDDLE].InitFootPos.z = TIBIA_LENGTH + HeightOffset;


    Leg[LEFT_FRONT].InitFootPos.x = 40;
    Leg[LEFT_FRONT].InitFootPos.y = -90;
    Leg[LEFT_FRONT].InitFootPos.z = TIBIA_LENGTH + HeightOffset;

    for( legNr=0; legNr<6; legNr++)
    {
    	Leg[legNr].FootPos.x = 0;
    	Leg[legNr].FootPos.y = 0;
    	Leg[legNr].FootPos.z = 0;
    }

}


void IK_Body_Complex(void)
{
	//Tilt = Rotation of Y axis
	//Roll = Rotation of X axis
	//Pan  = Rotation of Z axis

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

		// Foot position offsets necessary to achieve given body rotation
        bodyRotOffsetX[legNr] = ( globalInitFootPos.y*cosRotY*sinRotZ + globalInitFootPos.y*cosRotZ*sinRotX*sinRotY + globalInitFootPos.x*cosRotZ*cosRotY - globalInitFootPos.x*sinRotZ*sinRotX*sinRotY - globalInitFootPos.z*cosRotX*sinRotY) - globalInitFootPos.x;
        bodyRotOffsetY[legNr] =   globalInitFootPos.y*cosRotX*cosRotZ - globalInitFootPos.x*cosRotX*sinRotZ         + globalInitFootPos.z*sinRotX         - globalInitFootPos.y;
        bodyRotOffsetZ[legNr] = ( globalInitFootPos.y*sinRotZ*sinRotY - globalInitFootPos.y*cosRotZ*cosRotY*sinRotX + globalInitFootPos.x*cosRotZ*sinRotY + globalInitFootPos.x*cosRotY*sinRotZ*sinRotX + globalInitFootPos.z*cosRotX*cosRotY) - globalInitFootPos.z;

        //bodyRotOffsetX[legNr] =  globalInitFootPos.x*cosRotY*cosRotZ + globalInitFootPos.y*sinRotX*sinRotY*cosRotZ + globalInitFootPos.y*cosRotX*sinRotZ - globalInitFootPos.z*cosRotX*sinRotY*cosRotZ + globalInitFootPos.z*sinRotX*sinRotZ - globalInitFootPos.x;
        //bodyRotOffsetY[legNr] =  -globalInitFootPos.x*cosRotY*sinRotZ - globalInitFootPos.y*sinRotX*sinRotY*sinRotZ + globalInitFootPos.y*cosRotX*cosRotZ + globalInitFootPos.z*cosRotX*sinRotY*sinRotZ + globalInitFootPos.z*sinRotX*cosRotZ - globalInitFootPos.y;
        //bodyRotOffsetZ[legNr] =  globalInitFootPos.x*sinRotY         - globalInitFootPos.y*sinRotX*cosRotY;

        // Calculated foot positions to achieve translation/rotation input. Not coxa mounting angle corrected
        bodyRotOffsetX[legNr] = Leg[legNr].InitFootPos.x + bodyRotOffsetX[legNr] - Robot.BodyCmd.TransX;// + Leg[legNr].FootPos.x;
        bodyRotOffsetY[legNr] = Leg[legNr].InitFootPos.y + bodyRotOffsetY[legNr] - Robot.BodyCmd.TransY;// + Leg[legNr].FootPos.y;
        bodyRotOffsetZ[legNr] = Leg[legNr].InitFootPos.z + bodyRotOffsetZ[legNr] - Robot.BodyCmd.TransZ;// + Leg[legNr].FootPos.z;
    }
        // Rotates X,Y about coxa to compensate for coxa mounting angles.
        Leg[0].InputFootPos.x = bodyRotOffsetY[0]*cos(DegtoRad(70)) - bodyRotOffsetX[0]*sin(DegtoRad(70));
        Leg[0].InputFootPos.y = bodyRotOffsetY[0]*sin(DegtoRad(70)) + bodyRotOffsetX[0]*cos(DegtoRad(70));
        Leg[0].InputFootPos.z = bodyRotOffsetZ[0];
        Leg[1].InputFootPos.x = bodyRotOffsetY[1]*cos(DegtoRad(90)) - bodyRotOffsetX[1]*sin(DegtoRad(90));
        Leg[1].InputFootPos.y = bodyRotOffsetY[1]*sin(DegtoRad(90)) + bodyRotOffsetX[1]*cos(DegtoRad(90));
        Leg[1].InputFootPos.z = bodyRotOffsetZ[1];
        Leg[2].InputFootPos.x = bodyRotOffsetY[2]*cos(DegtoRad(110)) - bodyRotOffsetX[2]*sin(DegtoRad(110));
        Leg[2].InputFootPos.y = bodyRotOffsetY[2]*sin(DegtoRad(110)) + bodyRotOffsetX[2]*cos(DegtoRad(110));
        Leg[2].InputFootPos.z = bodyRotOffsetZ[2];
        Leg[3].InputFootPos.x = bodyRotOffsetY[3]*cos(DegtoRad(-110)) - bodyRotOffsetX[3]*sin(DegtoRad(-110));
        Leg[3].InputFootPos.y = bodyRotOffsetY[3]*sin(DegtoRad(-110)) + bodyRotOffsetX[3]*cos(DegtoRad(-110));
        Leg[3].InputFootPos.z = bodyRotOffsetZ[3];
        Leg[4].InputFootPos.x = bodyRotOffsetY[4]*cos(DegtoRad(-90)) - bodyRotOffsetX[4]*sin(DegtoRad(-90));
        Leg[4].InputFootPos.y = bodyRotOffsetY[4]*sin(DegtoRad(-90)) + bodyRotOffsetX[4]*cos(DegtoRad(-90));
        Leg[4].InputFootPos.z = bodyRotOffsetZ[4];
        Leg[5].InputFootPos.x = bodyRotOffsetY[5]*cos(DegtoRad(-70)) - bodyRotOffsetX[5]*sin(DegtoRad(-70));
        Leg[5].InputFootPos.y = bodyRotOffsetY[5]*sin(DegtoRad(-70)) + bodyRotOffsetX[5]*cos(DegtoRad(-70));
        Leg[5].InputFootPos.z = bodyRotOffsetZ[5];
}


void IK_Body_Light(void)
{
	//Tilt = Rotation of Y axis
	//Roll = Rotation of X axis
	//Pan  = Rotation of Z axis

	uint8_t legNr;
	Coord_st globalInitFootPos;

    for( legNr=0; legNr<6; legNr++)
    {
    	// Distance from center of body to foot position
    	//globalInitFootPos.x = Leg[legNr].InitFootPos.x + Leg[legNr].LegBasePos.x;
		//globalInitFootPos.y = Leg[legNr].InitFootPos.y + Leg[legNr].LegBasePos.y;
		//globalInitFootPos.z = Leg[legNr].InitFootPos.z + Leg[legNr].LegBasePos.z;


		Leg[legNr].InputFootPos.x = globalInitFootPos.x + Robot.BodyCmd.TransX;
		Leg[legNr].InputFootPos.y = globalInitFootPos.x + Robot.BodyCmd.TransY;
		Leg[legNr].InputFootPos.z = globalInitFootPos.x + Robot.BodyCmd.TransZ;
    }

/*
		// Foot position offsets necessary to achieve given body rotation
        bodyRotOffsetX[legNr] = ( globalInitFootPos.y*cosRotY*sinRotZ + globalInitFootPos.y*cosRotZ*sinRotX*sinRotY + globalInitFootPos.x*cosRotZ*cosRotY - globalInitFootPos.x*sinRotZ*sinRotX*sinRotY - globalInitFootPos.z*cosRotX*sinRotY) - globalInitFootPos.x;
        bodyRotOffsetY[legNr] =   globalInitFootPos.y*cosRotX*cosRotZ - globalInitFootPos.x*cosRotX*sinRotZ         + globalInitFootPos.z*sinRotX         - globalInitFootPos.y;
        bodyRotOffsetZ[legNr] = ( globalInitFootPos.y*sinRotZ*sinRotY - globalInitFootPos.y*cosRotZ*cosRotY*sinRotX + globalInitFootPos.x*cosRotZ*sinRotY + globalInitFootPos.x*cosRotY*sinRotZ*sinRotX + globalInitFootPos.z*cosRotX*cosRotY) - globalInitFootPos.z;

        // Calculated foot positions to achieve translation/rotation input. Not coxa mounting angle corrected
        bodyRotOffsetX[legNr] = Leg[legNr].InitFootPos.x + bodyRotOffsetX[legNr] - Robot.BodyCmd.TransX + Leg[legNr].FootPos.x;
        bodyRotOffsetY[legNr] = Leg[legNr].InitFootPos.y + bodyRotOffsetY[legNr] - Robot.BodyCmd.TransY + Leg[legNr].FootPos.y;
        bodyRotOffsetZ[legNr] = Leg[legNr].InitFootPos.z + bodyRotOffsetZ[legNr] - Robot.BodyCmd.TransZ + Leg[legNr].FootPos.z;

        // Rotates X,Y about coxa to compensate for coxa mounting angles.
        Leg[0].InputFootPos.x = bodyRotOffsetY[0]*cos(DegtoRad(90-COXA_ANGLE))   - bodyRotOffsetX[0]*sin(DegtoRad(90-COXA_ANGLE));
        Leg[0].InputFootPos.y = bodyRotOffsetY[0]*sin(DegtoRad(90-COXA_ANGLE))   + bodyRotOffsetX[0]*cos(DegtoRad(90-COXA_ANGLE));
        Leg[0].InputFootPos.z = bodyRotOffsetZ[0];
        Leg[1].InputFootPos.x = bodyRotOffsetY[1]*cos(DegtoRad(90)) - bodyRotOffsetX[1]*sin(DegtoRad(90));
        Leg[1].InputFootPos.y = bodyRotOffsetY[1]*sin(DegtoRad(90)) + bodyRotOffsetX[1]*cos(DegtoRad(90));
        Leg[1].InputFootPos.z = bodyRotOffsetZ[1];
        Leg[2].InputFootPos.x = bodyRotOffsetY[2]*cos(DegtoRad(90+COXA_ANGLE)) - bodyRotOffsetX[2]*sin(DegtoRad(90+COXA_ANGLE));
        Leg[2].InputFootPos.y = bodyRotOffsetY[2]*sin(DegtoRad(90+COXA_ANGLE)) + bodyRotOffsetX[2]*cos(DegtoRad(90+COXA_ANGLE));
        Leg[2].InputFootPos.z = bodyRotOffsetZ[2];
        Leg[3].InputFootPos.x = bodyRotOffsetY[3]*cos(DegtoRad(-90-COXA_ANGLE)) - bodyRotOffsetX[3]*sin(DegtoRad(-90-COXA_ANGLE));
        Leg[3].InputFootPos.y = bodyRotOffsetY[3]*sin(DegtoRad(-90-COXA_ANGLE)) + bodyRotOffsetX[3]*cos(DegtoRad(-90-COXA_ANGLE));
        Leg[3].InputFootPos.z = bodyRotOffsetZ[3];
        Leg[4].InputFootPos.x = bodyRotOffsetY[4]*cos(DegtoRad(-90)) - bodyRotOffsetX[4]*sin(DegtoRad(-90));
        Leg[4].InputFootPos.y = bodyRotOffsetY[4]*sin(DegtoRad(-90)) + bodyRotOffsetX[4]*cos(DegtoRad(-90));
        Leg[4].InputFootPos.z = bodyRotOffsetZ[4];
        Leg[5].InputFootPos.x = bodyRotOffsetY[5]*cos(DegtoRad(-90+COXA_ANGLE)) - bodyRotOffsetX[5]*sin(DegtoRad(-90+COXA_ANGLE));
        Leg[5].InputFootPos.y = bodyRotOffsetY[5]*sin(DegtoRad(-90+COXA_ANGLE)) + bodyRotOffsetX[5]*cos(DegtoRad(-90+COXA_ANGLE));
        Leg[5].InputFootPos.z = bodyRotOffsetZ[5];
    */
}


void IK_BodyLegs(void)
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
		if(Leg[legNr].InputFootPos.x > 50) {Leg[legNr].InputFootPos.x = 50; }
		if(Leg[legNr].InputFootPos.x < -50)  {Leg[legNr].InputFootPos.x = -50; }
		if(Leg[legNr].InputFootPos.y > 140) {Leg[legNr].InputFootPos.y = 140; }
		if(Leg[legNr].InputFootPos.y < 10)  {Leg[legNr].InputFootPos.y = 10; }
		if(Leg[legNr].InputFootPos.z > 140) {Leg[legNr].InputFootPos.z = 140; }
		if(Leg[legNr].InputFootPos.z < 10)  {Leg[legNr].InputFootPos.z = 10; }

		LegLentgh = sqrt( pow(Leg[legNr].InputFootPos.x,2) + pow(Leg[legNr].InputFootPos.y,2) );
		HF = sqrt( pow((LegLentgh-COXA_LENGTH),2) + pow(Leg[legNr].InputFootPos.z,2) );
		A1 = atan2( (LegLentgh-COXA_LENGTH), Leg[legNr].InputFootPos.z );
		A2 = acos( (pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2) - pow(HF,2) ) / (-2*FEMUR_LENGTH*HF) );
		B1 = acos( (pow(HF,2) - pow(TIBIA_LENGTH,2) - pow(FEMUR_LENGTH,2) ) / (-2*FEMUR_LENGTH*TIBIA_LENGTH) );

		A1Deg = RadtoDeg(A1);
		A2Deg = RadtoDeg(A2);
		B1Deg = RadtoDeg(B1);

		CoxaAngle  = RadtoDeg(atan2(Leg[legNr].InputFootPos.x, Leg[legNr].InputFootPos.y));
		FemurAngle = 90.0 - (A1Deg + A2Deg);
		TibiaAngle = 90.0 - B1Deg;


		if(legNr == 0 || legNr == 1 || legNr == 2)
		{
			//RIGHT Side
			Leg[legNr].Joint.CoxaAngle  = CoxaAngle;
			Leg[legNr].Joint.FemurAngle = -(FemurAngle - 10);
			Leg[legNr].Joint.TibiaAngle = -(-TibiaAngle + 20);
		}
		if(legNr == 3 || legNr == 4 || legNr == 5)
		{
			//LEFT Side
			Leg[legNr].Joint.CoxaAngle  = CoxaAngle;
			Leg[legNr].Joint.FemurAngle = (FemurAngle - 10);
			Leg[legNr].Joint.TibiaAngle = (-TibiaAngle + 20);
		}
	}
}


void IK_BodyLegs2(void)
{
	uint8_t legNr;
	volatile float L0, L1, L2, L3;
	volatile float Theta_f, Theta_t, Theta_c, Ups_f, Phi_f, Phi_t, var_f;
	volatile float CoxaAngle, FemurAngle, TibiaAngle;


	for(legNr=0; legNr<6; legNr++ )
	{
		if(Leg[legNr].InputFootPos.x > 50) {Leg[legNr].InputFootPos.x = 50; }
		if(Leg[legNr].InputFootPos.x < -50)  {Leg[legNr].InputFootPos.x = -50; }
		if(Leg[legNr].InputFootPos.y > 140) {Leg[legNr].InputFootPos.y = 140; }
		if(Leg[legNr].InputFootPos.y < 10)  {Leg[legNr].InputFootPos.y = 10; }
		if(Leg[legNr].InputFootPos.z > 140) {Leg[legNr].InputFootPos.z = 140; }
		if(Leg[legNr].InputFootPos.z < 10)  {Leg[legNr].InputFootPos.z = 10; }



		L0 = sqrt( pow(Leg[legNr].InputFootPos.x,2) + pow(Leg[legNr].InputFootPos.y,2) ) - COXA_LENGTH;
		L1 = FEMUR_LENGTH;
		L2 = TIBIA_LENGTH;
		L3 = sqrt( pow((L0),2) + pow(Leg[legNr].InputFootPos.z,2) );
		Phi_t = acos( (pow(L1,2)+pow(L2,2)-pow(L3,2)) / (2*L1*L2) );
		Phi_f = acos( (pow(L1,2)+pow(L3,2)-pow(L2,2)) / (2*L1*L3) );
		Ups_f = atan(Leg[legNr].InputFootPos.z/L0);

		Phi_t = RadtoDeg(Phi_t);
		Phi_f = RadtoDeg(Phi_f);
		Ups_f = RadtoDeg(Ups_f);

		var_f = (Phi_f - Ups_f);
		Theta_f = Phi_f + Ups_f - 10 + var_f - 90;
		Theta_t = Phi_t - 14.35 - 90;// 360-(90+80+26,01) o 360; //360-(82.10+260+90+90)=-62.1
		Theta_c	= RadtoDeg(atan(Leg[legNr].InputFootPos.y/Leg[legNr].InputFootPos.x)) - 90;

		CoxaAngle  = Theta_c;
		FemurAngle = Theta_f;
		TibiaAngle = -Theta_t;

		switch(legNr)
		{
			case 0:
				Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
				Leg[legNr].Joint.FemurAngle = FemurAngle-20;
				Leg[legNr].Joint.TibiaAngle = TibiaAngle;
			break;
			case 1:
				Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
				Leg[legNr].Joint.FemurAngle = FemurAngle-20;
				Leg[legNr].Joint.TibiaAngle = TibiaAngle;
			break;
			case 2:
				Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
				Leg[legNr].Joint.FemurAngle = FemurAngle-20;
				Leg[legNr].Joint.TibiaAngle = TibiaAngle;
			break;
			case 3:
				Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
				Leg[legNr].Joint.FemurAngle = -FemurAngle+20;
				Leg[legNr].Joint.TibiaAngle = -TibiaAngle;
			break;
			case 4:
				Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
				Leg[legNr].Joint.FemurAngle = -FemurAngle+20;
				Leg[legNr].Joint.TibiaAngle = -TibiaAngle;
			break;
			case 5:
				Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
				Leg[legNr].Joint.FemurAngle = -FemurAngle+20;
				Leg[legNr].Joint.TibiaAngle = -TibiaAngle ;
			break;
			default:
			break;
		}
	}
}

void IK_DriveServos(uint8_t speed)
{
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


void IK_SingleLegDrive(uint8_t legNr, float PosX, float PosY, float PosZ, uint8_t speed)
{
	volatile float L0, L1, L2, L3;
	volatile float Theta_T, Theta_F, Theta_C, Ups_F, Phi_F, Phi_T, var_F;
	volatile float CoxaAngle, FemurAngle, TibiaAngle;

	L0 = sqrt( pow(PosX,2) + pow(PosY,2) ) - COXA_LENGTH;
	L1 = FEMUR_LENGTH;
	L2 = TIBIA_LENGTH;
	L3 = sqrt( pow(L0,2) + pow(PosZ,2) );
	Phi_T = acos( (pow(L1,2)+pow(L2,2)-pow(L3,2)) / (2.0*L1*L2) );
	Phi_F = acos( (pow(L1,2)+pow(L3,2)-pow(L2,2)) / (2.0*L1*L3) );
	Ups_F = atan2(PosZ, L0);

	Phi_T = RadtoDeg(Phi_T);
	Phi_F = RadtoDeg(Phi_F);
	Ups_F = RadtoDeg(Ups_F);

	CoxaAngle  = RadtoDeg(atan2(PosX, PosY));
	FemurAngle = (Phi_F - ABS(Ups_F)) - 10.0;
	TibiaAngle = 90.0 - Phi_T + 4.8  + 10.0;

	switch(legNr)
	{
		case 0:
			Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
			Leg[legNr].Joint.FemurAngle = FemurAngle+10.0;
			Leg[legNr].Joint.TibiaAngle = TibiaAngle;
		break;
		case 1:
			Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
			Leg[legNr].Joint.FemurAngle = FemurAngle+10.0;
			Leg[legNr].Joint.TibiaAngle = TibiaAngle;
		break;
		case 2:
			Leg[legNr].Joint.CoxaAngle  = -CoxaAngle;
			Leg[legNr].Joint.FemurAngle = FemurAngle+10.0;
			Leg[legNr].Joint.TibiaAngle = TibiaAngle;
		break;
		case 3:
			Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
			Leg[legNr].Joint.FemurAngle = -FemurAngle-10.0;
			Leg[legNr].Joint.TibiaAngle = -TibiaAngle;
		break;
		case 4:
			Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
			Leg[legNr].Joint.FemurAngle = -FemurAngle-10.0;
			Leg[legNr].Joint.TibiaAngle = -TibiaAngle;
		break;
		case 5:
			Leg[legNr].Joint.CoxaAngle  = +CoxaAngle;
			Leg[legNr].Joint.FemurAngle = -FemurAngle-10.0;
			Leg[legNr].Joint.TibiaAngle = -TibiaAngle;
		break;
		default:
		break;
	}

	IK_DriveServos(speed);
	asm("NOP");
}
