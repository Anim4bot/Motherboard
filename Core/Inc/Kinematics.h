#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"


/******************************************************************************
 *
 *
 * FRONT VIEW
 *      			^        ==0\           /0==
 *     /\___/\      |       |    0==[___]==0    |
 *    /       \     -Z      |                   |
 *
 *
 *
 * TOP VIEW
 *
 *  5  \       / 0      ^
 *      \_____/         |
 *      | |_| |         |
 * 4 ___|     |___ 1    X
 *      |     |
 *      |_____|
 *     /      \        Y--->
 *  3 /        \ 2
 *
 *****************************************************************************/


#define RIGHT_FRONT   0
#define RIGHT_MIDDLE  1
#define RIGHT_REAR    2
#define LEFT_REAR     3
#define LEFT_MIDDLE   4
#define LEFT_FRONT    5

#define COXA_LENGTH		46.20
#define FEMUR_LENGTH	73.67
#define TIBIA_LENGTH	119.38


#define X_COXA_FRONT	95.61
#define X_COXA_MIDDLE	0
#define Y_COXA_FRONT	69.87

#define X_COXA_REAR		95.61
#define Y_COXA_MIDDLE	79.38
#define Y_COXA_REAR		69.87

#define COXA_ANGLE		70



typedef struct
{
	float x;
	float y;
	float z;
}
Coord_st;

typedef struct
{
	float CoxaAngle;
	float FemurAngle;
	float TibiaAngle;
}
Joints_st;

typedef struct
{
	Coord_st FootPos;
	Coord_st InitFootPos;
	Coord_st LegBasePos;
	Coord_st InputFootPos;
	Joints_st Joint;
}
Leg_st;




void IK_Run(void);
void IK_LegInit(void);
void IK_Body_Complex(void);
void IK_Body_Light(void);
void IK_BodyLegs(void);
void IK_BodyLegs2(void);
void IK_Leg (uint8_t legNr, float PosX, float PosY, float PosZ);
void IK_DriveServos(uint8_t speed);
void IK_SingleLegDrive(uint8_t legNr, float PosX, float PosY, float PosZ, uint8_t speed);

void IK_SingleLegDrive2(uint8_t legNr, float PosX, float PosY, float PosZ, uint8_t speed);



#endif /* INC_KINEMATICS_H_ */
