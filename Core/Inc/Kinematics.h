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
#define FEMUR_LENGTH	73.67
#define TIBIA_LENGTH	119.34

#define LEG_FRONT_X		95.61
#define LEG_FRONT_Y		69.86
#define LEG_MIDDLE_X	0
#define LEG_MIDDLE_Y	79.38
#define LEG_REAR_X		-95.61
#define LEG_REAR_Y		69.86


typedef struct
{
	float PosX;
	float PosY;
	float PosZ;
}
FeetPos_st;

typedef struct
{
	float CoxaJointAngle;
	float FemurJointAngle;
	float TibiaJointAngle;
}
Joints_st;

typedef struct
{
	FeetPos_st Foot;
	Joints_st Joint;
}
Leg_st;


typedef struct
{
	float PosX;
	float PosY;
	float PosZ;
	float RotX;
	float RotY;
	float RotZ;
}
Body_st;







#endif /* INC_KINEMATICS_H_ */
