#ifndef LSM9DS1_H_
#define LSM9DS1_H_


#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern I2C_HandleTypeDef  hi2c2;

#define LSM9DS1_I2C_PORT	hi2c2
#define LSM9DS1_AG_ADDR 	0xD6
#define LSM9DS1_M_ADDR 		0x3C


/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33


#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058



// accel_scale defines all possible FSR's of the accelerometer:
typedef enum
{
	A_SCALE_2G,  // 00:  2g
	A_SCALE_16G, // 01:  16g
	A_SCALE_4G,  // 10:  4g
	A_SCALE_8G   // 11:  8g
}
accel_scale;

// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum
{
	G_SCALE_245DPS,  // 00:  245 degrees per second
	G_SCALE_500DPS,  // 01:  500 dps
	G_SCALE_2000DPS, // 11:  2000 dps
}
gyro_scale;

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
typedef enum
{
	G_ODR_PD,  // Power down (0)
	G_ODR_149, // 14.9 Hz (1)
	G_ODR_595, // 59.5 Hz (2)
	G_ODR_119, // 119 Hz (3)
	G_ODR_238, // 238 Hz (4)
	G_ODR_476, // 476 Hz (5)
	G_ODR_952  // 952 Hz (6)
}
gyro_odr;

// accel_oder defines all possible output data rates of the accelerometer:
typedef enum
{
	XL_POWER_DOWN, 	// Power-down mode (0x0)
	XL_ODR_10,	 	// 10 Hz (0x1)
	XL_ODR_50,	 	// 50 Hz (0x02)
	XL_ODR_119,		// 119 Hz (0x3)
	XL_ODR_238,		// 238 Hz (0x4)
	XL_ODR_476,		// 476 Hz (0x5)
	XL_ODR_952	 	// 952 Hz (0x6)
}
accel_odr;

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
typedef enum
{
	A_ABW_408, // 408 Hz (0x0)
	A_ABW_211, // 211 Hz (0x1)
	A_ABW_105, // 105 Hz (0x2)
	A_ABW_50,  //  50 Hz (0x3)
}
accel_abw;


typedef enum
{
	XG_INT1 = INT1_CTRL,
	XG_INT2 = INT2_CTRL
}
interrupt_select;

typedef enum
{
	INT_DRDY_XL = (1 << 0),		// Accelerometer data ready (INT1 & INT2)
	INT_DRDY_G = (1 << 1),	 	// Gyroscope data ready (INT1 & INT2)
	INT1_BOOT = (1 << 2),	  	// Boot status (INT1)
	INT2_DRDY_TEMP = (1 << 2), 	// Temp data ready (INT2)
	INT_FTH = (1 << 3),		   	// FIFO threshold interrupt (INT1 & INT2)
	INT_OVR = (1 << 4),		   	// Overrun interrupt (INT1 & INT2)
	INT_FSS5 = (1 << 5),	   	// FSS5 interrupt (INT1 & INT2)
	INT_IG_XL = (1 << 6),	  	// Accel interrupt generator (INT1)
	INT1_IG_G = (1 << 7),	  	// Gyro interrupt enable (INT1)
	INT2_INACT = (1 << 7) 		// Inactivity interrupt output (INT2)
}
interrupt_generators;

typedef enum
{
	XLIE_XL = (1 << 0),
	XHIE_XL = (1 << 1),
	YLIE_XL = (1 << 2),
	YHIE_XL = (1 << 3),
	ZLIE_XL = (1 << 4),
	ZHIE_XL = (1 << 5),
	GEN_6D = (1 << 6)
}
accel_interrupt_generator;

typedef enum
{
	XLIE_G = (1 << 0),
	XHIE_G = (1 << 1),
	YLIE_G = (1 << 2),
	YHIE_G = (1 << 3),
	ZLIE_G = (1 << 4),
	ZHIE_G = (1 << 5)
}
gyro_interrupt_generator;


typedef enum
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
}
h_lactive;

typedef enum
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
}
pp_od;

typedef enum
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 6
}
fifoMode_type;


typedef struct
{
	uint8_t enabled;
	uint16_t scale;
	uint8_t sampleRate;
	uint8_t bandwidth;
	uint8_t lowPowerEnable;
	uint8_t HPFEnable;
	uint8_t HPFCutoff;
	uint8_t flipX;
	uint8_t flipY;
	uint8_t flipZ;
	uint8_t orientation;
	uint8_t enableX;
	uint8_t enableY;
	uint8_t enableZ;
	uint8_t latchInterrupt;
}
gyro;

typedef struct
{
	uint8_t enabled;
	uint8_t scale;
	uint8_t sampleRate;
	uint8_t enableX;
	uint8_t enableY;
	uint8_t enableZ;
	int8_t  bandwidth;
	uint8_t highResEnable;
	uint8_t highResBandwidth;
}
accel;


typedef struct
{
	gyro gyro;
	accel accel;
}
IMUSettings;



HAL_StatusTypeDef LSM9DS1_Write8(uint8_t Reg, uint8_t data);
uint8_t LSM9DS1_Read8(uint8_t Reg);
uint16_t LSM9DS1_Read16(uint8_t Reg);
void LSM9DS1_Init(void);
HAL_StatusTypeDef LSM9DS1_ReadAcc(float *xAcc, float *yAcc, float *zAcc);
HAL_StatusTypeDef LSM9DS1_readAngle(float *rollF, float *pitchF);
HAL_StatusTypeDef LSM9DS1_ReadGyro(float *xRot, float *yRot, float *zRot);


#endif /* LSM9DS1_H_ */
