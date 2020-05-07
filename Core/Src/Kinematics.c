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

void LegIK (int legIKNr, float ikFeetPosX, float ikFeetPosY, float ikFeetPosZ)
{
    float ikSW;                       // Length between Femur and Tibia
    float ikRadiansFemurTibiaGround;  // Angle of the line Femur and Tibia with respect to the ground in radians
    float ikRadiansFemurTibia;        // Angle of the line Femur and Tibia with respect to the femur in radians
    float ikFeetPosXZ;                // Distance between the Coxa and Ground Contact

    // Distance between the Coxa and Ground Contact
    ikFeetPosXZ = sqrt ( pow (ikFeetPosX, 2 ) + pow (ikFeetPosZ, 2 ) );

    // ikSW - Length between Femur axis and Tibia
    ikSW = sqrt ( pow ( ( ikFeetPosXZ - COXA_LENGTH ) , 2 ) + pow ( ikFeetPosY , 2 ) );

    // ikRadiansFemurTibiaGround - Angle between Femur and Tibia line and the ground in radians
    ikRadiansFemurTibiaGround = atan2 ( ikFeetPosXZ - COXA_LENGTH, ikFeetPosY );

    // ikRadiansFemurTibia - Angle of the line Femur and Tibia with respect to the Femur in radians
    ikRadiansFemurTibia = acos ( ( ( pow ( FEMUR_LENGTH, 2 ) - pow ( TIBIA_LENGTH, 2 ) ) + pow ( ikSW, 2 ) ) / ( 2 * FEMUR_LENGTH * ikSW ) );

    // ikCoxaAngle in degrees
    Leg[legIKNr].Joint.CoxaJointAngle = atan2 ( ikFeetPosZ, ikFeetPosX ) * 180 / PI + initCoxaAngle;

    // ikFemurAngle in degrees
    Leg[legIKNr].Joint.FemurJointAngle = -( ikRadiansFemurTibiaGround + ikRadiansFemurTibia ) * 180 / PI  + 90;

    // ikTibiaAngle in degrees
    Leg[legIKNr].Joint.TibiaJointAngle = -( 90 - ( ( ( acos ( ( pow (FEMUR_LENGTH, 2 ) + pow ( TIBIA_LENGTH, 2 ) - pow ( ikSW, 2 ) ) / ( 2 * FEMUR_LENGTH * TIBIA_LENGTH ) ) ) * 180 ) / PI ) );


    WriteLegPos(70);

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

