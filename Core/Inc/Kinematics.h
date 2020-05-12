#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"


/******************************************************************************
 *
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     -Z      |               |
 *
 *
 *
 * TOP VIEW
 *    \       /      ^
 *     \_____/       |
 *     |     |
 *  ___|     |___    X
 *     |     |
 *     |_____|
 *     /     \       Y--->
 *    /       \
 *
 *****************************************************************************/


#define RIGHT_FRONT   0
#define RIGHT_MIDDLE  1
#define RIGHT_REAR    2
#define LEFT_REAR     3
#define LEFT_MIDDLE   4
#define LEFT_FRONT    5

#define COXA_LENGTH		46.20
#define FEMUR_LENGTH	73.68
#define TIBIA_LENGTH	119.15

#define X_COXA				95.61
#define Y_COXA_MIDDLE		79.38
#define Y_COXA_FRONT_REAR	69.86
#define COXA_ANGLE			20



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
	Coord_st InitFootPos;
	Coord_st LegBasePos;
	Coord_st CalcFootPos;
	Joints_st Joint;
}
Leg_st;

typedef struct
{
	  int   Xspeed;
	  int   Yspeed;
	  int   Rspeed;
	  float bodyTransX;
	  float bodyTransY;
	  float bodyTransZ;
	  float bodyRotX;
	  float bodyRotY;
	  float bodyRotZ;
}
BodyCmd_st;




void LegIK (uint8_t legNr, float PosX, float PosY, float PosZ);
void WriteLegIK(uint8_t speed);


#endif /* INC_KINEMATICS_H_ */
