#ifndef DRS0101_H_
#define DRS0101_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE

extern UART_HandleTypeDef huart3;

#define ROM_MODEL_NO1                       0
#define ROM_MODEL_NO2                       1
#define ROM_VERSION1                        2
#define ROM_VERSION2                        3
#define ROM_BAUD_RATE                       4
#define ROM_RESERVED5                       5
#define ROM_ID                              6
#define ROM_ACK_POLICY                      7
#define ROM_ALARM_LED_POLICY                8
#define ROM_TORQUE_POLICY                   9
#define ROM_RESERVED10                      10
#define ROM_MAX_TEMPERATURE                 11
#define ROM_MIN_VOLTAGE                     12
#define ROM_MAX_VOLTAGE                     13
#define ROM_ACCELERATION_RATIO              14
#define ROM_MAX_ACCELERATION_TIME           15
#define ROM_DEAD_ZONE                       16
#define ROM_SATURATOR_OFFSET                17
#define ROM_SATURATOR_SLOPE                 18  // 2Byte
#define ROM_PWM_OFFSET                      20
#define ROM_MIN_PWM                         21
#define ROM_MAX_PWM                         22  // 2Byte
#define ROM_OVERLOAD_PWM_THRESHOLD          24  // 2Byte
#define ROM_MIN_POSITION                    26  // 2Byte
#define ROM_MAX_POSITION                    28  // 2Byte
#define ROM_POSITION_KP                     30  // 2Byte
#define ROM_POSITION_KD                     32  // 2Byte
#define ROM_POSITION_KI                     34  // 2Byte
#define ROM_POSITION_FEEDFORWARD_1ST_GAIN   36  // 2Byte
#define ROM_POSITION FEEDFORWARD_2ND_GAIN   38  // 2Byte
#define ROM_RESERVED40                      40  // 2Byte
#define ROM_RESERVED42                      42  // 2Byte
#define ROM_LED_BLINK_PERIOD                44
#define ROM_ADC_FAULT_CHECK_PERIOD          45
#define ROM_PACKET_GARBAGE_CHECK_PERIOD     46
#define ROM_STOP_DETECTION_PERIOD           47
#define ROM_OVERLOAD_DETECTION_PERIOD       48
#define ROM_STOP_THRESHOLD                  49
#define ROM_INPOSITION_MARGIN               50
#define ROM_RESERVED51                      51
#define ROM_RESERVED52                      52
#define ROM_CALIBRATION_DIFFERENCE          53

//------------------------------------------------------------------------------
// Herkulex RAM Register
#define RAM_ID                              0
#define RAM_ACK_POLICY                      1
#define RAM_ALARM_LED_POLICY                2
#define RAM_TORQUE_POLICY                   3
#define RAM_RESERVED4                       4
#define RAM_MAX_TEMPERATURE                 5
#define RAM_MIN_VOLTAGE                     6
#define RAM_MAX_VOLTAGE                     7
#define RAM_ACCELERATION_RATIO              8
#define RAM_MAX_ACCELERATION                9
#define RAM_DEAD_ZONE                       10
#define RAM_SATURATOR_OFFSET                11
#define RAM_SATURATOR_SLOPE                 12 // 2Byte
#define RAM_PWM_OFFSET                      14
#define RAM_MIN_PWM                         15
#define RAM_MAX_PWM                         16 // 2Byte
#define RAM_OVERLOAD_PWM_THRESHOLD          18 // 2Byte
#define RAM_MIN_POSITION                    20 // 2Byte
#define RAM_MAX_POSITION                    22 // 2Byte
#define RAM_POSITION_KP                     24 // 2Byte
#define RAM_POSITION_KD                     26 // 2Byte
#define RAM_POSITION_KI                     28 // 2Byte
#define RAM_POSITION_FEEDFORWARD_1ST_GAIN   30 // 2Byte
#define RAM_POSITION_FEEDFORWARD 2ND GAIN   32 // 2Byte
#define RAM_RESERVED34                      34 // 2Byte
#define RAM_RESERVED36                      36 // 2Byte
#define RAM_LED_BLINK_PERIOD                38
#define RAM_ADC_FAULT_DETECTION_PERIOD      39
#define RAM_PACKET_GARBAGE_DETECTION_PERIOD 40
#define RAM_STOP_DETECTION_PERIOD           41
#define RAM_OVERLOAD_DETECTION_PERIOD       42
#define RAM_STOP_THRESHOLD                  43
#define RAM_INPOSITION_MARGIN               44
#define RAM_RESERVED45                      45
#define RAM_RESERVED46                      46
#define RAM_CALIBRATION_DIFFERENCE          47
#define RAM_STATUS_ERROR                    48
#define RAM_STATUS_DETAIL                   49
#define RAM_RESERVED50                      50
#define RAM_RESERVED51                      51
#define RAM_TORQUE_CONTROL                  52
#define RAM_LED_CONTROL                     53
#define RAM_VOLTAGE                         54
#define RAM_TEMPERATURE                     55
#define RAM_CURRENT_CONTROL_MODE            56
#define RAM_TICK                            57
#define RAM_CALIBRATED_POSITION             58 // 2Byte
#define RAM_ABSOLUTE_POSITION               60 // 2Byte
#define RAM_DIFFERENTIAL_POSITION           62 // 2Byte
#define RAM_PWM                             64 // 2Byte
#define RAM_RESERVED66                      66 // 2Byte
#define RAM_ABSOLUTE_GOAL_POSITION          68 // 2Byte
#define RAM_ABS_DESIRED_TRAJECTORY_POS      70 // 2Byte
#define RAM_DESIRED_VELOCITY                72 // 2Byte

//------------------------------------------------------------------------------
// Request Packet [To Servo Module]
#define CMD_ROM_WRITE  0x01    // Write Length number of values to EEP Register Address
#define CMD_ROM_READ   0x02    // Request Length number of values from EEP Register Address
#define CMD_RAM_WRITE  0x03    // Write Length number of values to RAM Register Address
#define CMD_RAM_READ   0x04    // Request Lenght number of values from RAM Register Address
#define CMD_I_JOG      0x05    // Able to send JOG command to maximum 43 servos (operate timing of individual Servo)
#define CMD_S_JOG      0x06    // Able to send JOG command to maximum 53 servos (operate simultaneously at same time)
#define CMD_STAT       0x07    // Status Error, Status Detail request
#define CMD_ROLLBACK   0x08    // Change all EEP Regsters to Factory Default value
#define CMD_REBOOT     0x09    // Request Reboot

//------------------------------------------------------------------------------
// ACK Packet [To Controller(ACK)]
#define CMD_ACK_MASK   		0x40 // ACK Packet CMD is Request Packet CMD + 0x40
#define CMD_EEP_WRITE_ACK   (CMD_EEP_WRITE|CMD_ACK_MASK)
#define CMD_EEP_READ_ACK    (CMD_EEP_READ|CMD_ACK_MASK)
#define CMD_RAM_WRITE_ACK   (CMD_RAM_WRITE|CMD_ACK_MASK)
#define CMD_RAM_READ_ACK    (CMD_RAM_READ|CMD_ACK_MASK)
#define CMD_I_JOG_ACK       (CMD_I_JOG|CMD_ACK_MASK)
#define CMD_S_JOG_ACK       (CMD_S_JOG|CMD_ACK_MASK)
#define CMD_STAT_ACK        (CMD_STAT|CMD_ACK_MASK)
#define CMD_ROLLBACK_ACK    (CMD_ROLLBACK|CMD_ACK_MASK)
#define CMD_REBOOT_ACK      (CMD_REBOOT|CMD_ACK_MASK)

//------------------------------------------------------------------------------
// Status Error
#define STATUS_OK                       0x00;
#define ERROR_EXCEED_INPUT_VOLTAGE      0x01;
#define ERROR_EXCEED_POT_LIMIT          0x02;
#define ERROR_EXCEED_TEMPERATURE_LIMIT  0x04;
#define ERROR_INVALID_PACKET            0x08;
#define ERROR_OVERLOAD                  0x10;
#define ERROR_DRIVER_FAULT              0x20;
#define ERROR_EEP_REG_DISTORT           0x40;

typedef enum
{
	ServoStat_STATUS_OK 					  = 0x00,
	ServoStat_ERROR_EXCEED_INPUT_VOLTAGE      = 0x01,
	ServoStat_ERROR_EXCEED_POT_LIMIT          = 0x02,
	ServoStat_ERROR_EXCEED_TEMPERATURE_LIMIT  = 0x04,
	ServoStat_ERROR_INVALID_PACKET            = 0x08,
	ServoStat_ERROR_OVERLOAD                  = 0x10,
	ServoStat_ERROR_DRIVER_FAULT              = 0x20,
	ServoStat_ERROR_EEP_REG_DISTORT           = 0x40
}
ServoStatus;

//------------------------------------------------------------------------------
// Status Detail
#define MOVING_FLAG                     0x01;
#define INPOSITION_FLAG                 0x02;
#define CHECKSUM_ERROR                  0x04; // Invalid packet`s detailed information
#define UNKNOWN_COMMAND                 0x08; // Invalid packet`s detailed information
#define EXCEED_REG_RANGE                0x10; // Invalid packet`s detailed information
#define GARBAGE_DETECTED                0x20; // Invalid packet`s detailed information
#define MOTOR_ON_FLAG                   0x40;

//------------------------------------------------------------------------------
// Header
#define HEADER                          0xFF

// Size
#define MIN_PACKET_SIZE                 7
#define MIN_ACK_PACKET_SIZE             9
#define WRITE_PACKET_SIZE               13
#define MAX_PACKET_SIZE                 223
#define MAX_DATA_SIZE                   (MAX_PACKET_SIZE-MIN_PACKET_SIZE)

// ID
#define MAX_PID                         0xFD
#define DEFAULT_ID                      0xFD
#define MAX_ID                          0xFD
#define BROADCAST_ID                    0xFE

// Checksum
#define CHKSUM_MASK                     0xFE

// Torque CMD
#define TORQUE_FREE                     0x00
#define BREAK_ON                        0x40
#define TORQUE_ON                       0x60

// Register Size
#define BYTE1                           1
#define BYTE2                           2

// Jog Set CMD
#define STOP                            0x01
#define POS_MODE                        0x00
#define TURN_MODE                       0x02

//LEDS
#define DRS0101_OFFLED	0x00
#define DRS0101_GLED   	0x04	//Green
#define DRS0101_BLED    0x08	//Blue
#define DRS0101_RLED	0x10	//Red
#define DRS0101_YLED  	0x14	//Yellow
#define DRS0101_CLED    0x0C	//Cyan
#define DRS0101_PLED  	0x18	//Purple
#define DRS0101_WLED   	0x1C	//White
#define DRS0101_Ignore  0xFF




//SERVOS ID
#define ID_LF_COXA  0x06
#define ID_LF_FEMUR 0x12
#define ID_LF_TIBIA 0x11

#define ID_LM_COXA  0x05
#define ID_LM_FEMUR 0x0E
#define ID_LM_TIBIA 0x0D

#define ID_LR_COXA  0x04
#define ID_LR_FEMUR 0x08
#define ID_LR_TIBIA 0x07

#define ID_RF_COXA  0x01
#define ID_RF_FEMUR 0x0A
#define ID_RF_TIBIA 0x09

#define ID_RM_COXA  0x02
#define ID_RM_FEMUR 0x0C
#define ID_RM_TIBIA 0x0B

#define ID_RR_COXA  0x03
#define ID_RR_FEMUR 0x10
#define ID_RR_TIBIA 0x0F





void DRS0101_Reboot(uint8_t id);
void DRS0101_Clear(uint8_t id);
void DRS0101_setTorque(uint8_t id, uint8_t cmdTorque);
void DRS0101_setPosition(uint8_t id, uint16_t position_ui16, uint8_t playtime_ui8, uint8_t setLED);
void DRS0101_setAngle(uint8_t id, float angle, uint8_t playtime_ui8, uint8_t setLED);


#endif /* DRS0101_INC_DRS0101_H_ */
