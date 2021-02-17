#include "DRS0101.h"


HAL_StatusTypeDef DRS0101_Write(uint8_t lenght, uint8_t* data)
{
	HAL_StatusTypeDef status;

	status = HAL_UART_Transmit(&huart3, data, lenght, 50);

	return status;
}


HAL_StatusTypeDef DRS0101_Read(uint8_t lenght, uint8_t* data)
{
	HAL_StatusTypeDef status;

	status = HAL_UART_Receive(&huart3, data, lenght, 50);

	return status;
}


void DRS0101_Reboot(uint8_t id)
{
	uint8_t Payload[7];

	Payload[0] = HEADER;              // Packet Header (0xFF)
	Payload[1] = HEADER;              // Packet Header (0xFF)
	Payload[2] = MIN_PACKET_SIZE + 4; // Packet Size
	Payload[3] = id;                  // Servo ID
	Payload[4] = CMD_REBOOT;          // Command Reboot
	Payload[5] = 0;                   // Checksum1
	Payload[6] = 0;                   // Checksum2

	Payload[5] = (Payload[2]^Payload[3]^Payload[4]) & 0xFE;
	Payload[6] = (~Payload[5])&0xFE;

    DRS0101_Write(7, Payload);
}


void DRS0101_Clear(uint8_t id)
{
	uint8_t Payload[11];

	Payload[0] = HEADER;              // Packet Header (0xFF)
	Payload[1] = HEADER;              // Packet Header (0xFF)
	Payload[2] = MIN_PACKET_SIZE + 4; // Packet Size
	Payload[3] = id;                  // Servo ID
	Payload[4] = CMD_RAM_WRITE;       // Command Ram Write (0x03)
	Payload[5] = 0;                   // Checksum1
	Payload[6] = 0;                   // Checksum2
	Payload[7] = RAM_STATUS_ERROR;    // Address 48
	Payload[8] = BYTE2;               // Length
	Payload[9] = 0;                   // Clear RAM_STATUS_ERROR
	Payload[10]= 0;                   // Clear RAM_STATUS_DETAIL

	Payload[5] = (Payload[2]^Payload[3]^Payload[4]^Payload[7]^Payload[8]^Payload[9]^Payload[10]) & 0xFE;
	Payload[6] = (~Payload[5])&0xFE;

    DRS0101_Write(11, Payload);
}


void DRS0101_setTorque(uint8_t id, uint8_t cmdTorque)
{
    uint8_t Payload[10];

    Payload[0] = HEADER;              // Packet Header (0xFF)
    Payload[1] = HEADER;              // Packet Header (0xFF)
    Payload[2] = MIN_PACKET_SIZE + 3; // Packet Size
    Payload[3] = id;                  // Servo ID
    Payload[4] = CMD_RAM_WRITE;       // Command Ram Write (0x03)
    Payload[5] = 0;                   // Checksum1
    Payload[6] = 0;                   // Checksum2
    Payload[7] = RAM_TORQUE_CONTROL;  // Address 52
    Payload[8] = BYTE1;               // Length
    Payload[9] = cmdTorque;           // Torque ON

    Payload[5] = (Payload[2]^Payload[3]^Payload[4]^Payload[7]^Payload[8]^Payload[9]) & 0xFE;
    Payload[6] = (~Payload[5])&0xFE;

    DRS0101_Write(10, Payload);
}


void DRS0101_setPosition(uint8_t id, uint16_t position_ui16, uint8_t playtime_ui8, uint8_t setLED)
{
	uint8_t Payload[12];

	if(position_ui16 > 1024) { position_ui16 = 1024; }
	if(position_ui16 < 0)    { position_ui16 = 0; }
	if(playtime_ui8 > 255)   { playtime_ui8 = 255; }

	Payload[0]  = HEADER;                         // Packet Header (0xFF)
	Payload[1]  = HEADER;                         // Packet Header (0xFF)
	Payload[2]  = MIN_PACKET_SIZE + 5;            // Packet Size
	Payload[3]  = id;                     		  // pID is total number of servos in the network (0 ~ 253)
	Payload[4]  = CMD_I_JOG;                      // Able to send JOG command to maximum 43 servos (operate timing of individual Servo)
	Payload[5]  = 0;                              // Checksum1
	Payload[6]  = 0;                              // Checksum2
	Payload[7]  = position_ui16 & 0x00FF;         // Position (LSB, Least Significant Bit)
	Payload[8]  =(position_ui16 & 0xFF00) >> 8;   // position (MSB, Most Significanct Bit)
	Payload[9]  = setLED;              			  // Pos Mode and LED on/off
	Payload[10] = id;                   		  //  ID
	Payload[11] = playtime_ui8;

	Payload[5] = (Payload[2]^Payload[3]^Payload[4]^Payload[7]^Payload[8]^Payload[9]^Payload[10]^Payload[11]) & 0xFE;
	Payload[6] = (~Payload[5])&0xFE;

	DRS0101_Write(12, Payload);
}


void DRS0101_setAngle(uint8_t id, float angle, uint8_t playtime_ui8, uint8_t setLED)
{
	uint8_t Payload[12];
	uint16_t position_ui16;
	volatile float angleTemp;

	//if (angle > 120.0 || angle < -120.0) { return; }
	//if(playtime_ui8 > 255)   { playtime_ui8 = 255; }

	if (angle > +75) { angle = +75; }
	if (angle < -75) { angle = -75; }

	angleTemp = angle;
	position_ui16 = (uint16_t)((angle/0.325) + 512);
	position_ui16 = position_ui16;

	Payload[0]  = HEADER;                         // Packet Header (0xFF)
	Payload[1]  = HEADER;                         // Packet Header (0xFF)
	Payload[2]  = MIN_PACKET_SIZE + 5;            // Packet Size
	Payload[3]  = id;                     		  // pID is total number of servos in the network (0 ~ 253)
	Payload[4]  = CMD_I_JOG;                      // Able to send JOG command to maximum 43 servos (operate timing of individual Servo)
	Payload[5]  = 0;                              // Checksum1
	Payload[6]  = 0;                              // Checksum2
	Payload[7]  = position_ui16 & 0x00FF;         // Position LSB
	Payload[8]  =(position_ui16 & 0xFF00) >> 8;   // position MSB
	Payload[9]  = setLED;              			  // Pos Mode and LED on/off
	Payload[10] = id;                   		  //  ID
	Payload[11] = playtime_ui8;

	Payload[5] = (Payload[2]^Payload[3]^Payload[4]^Payload[7]^Payload[8]^Payload[9]^Payload[10]^Payload[11]) & 0xFE;
	Payload[6] = (~Payload[5])&0xFE;

	DRS0101_Write(12, Payload);
}


