#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#define _USE_MATH_DEFINES
#include <cmath>

// standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>

// dynamixel sdk include
#include "dynamixel_sdk/dynamixel_sdk.h"    

// addresses of variables in the register
#define ADDR_XL320_CONTROL_MODE 11
#define ADDR_XL320_TORQUE_ENABLE 24
#define ADDR_XL320_GOAL_POSITION 30
#define ADDR_XL320_GOAL_VELOCITY 32
#define ADDR_XL320_PRESENT_POSITION 37
#define ADDR_XL320_PRESENT_VELOCITY 39
#define ADDR_XL320_HARDWARE_ERROR_STATUS 50

// rotation direction
#define ROT_DIRECTION_Q1 1
#define ROT_DIRECTION_Q2 -1
#define ROT_DIRECTION_QPEN 1
// nb of joints
#define NB_JOINTS 3


class DynamixelHandler
{
	
public:
	DynamixelHandler();
	~DynamixelHandler();

public:
	bool openPort();
	void closePort();
	bool setBaudRate(int);
	void setDeviceName(std::string);
	void setProtocolVersion(float);
	bool enableTorque(bool);
	bool setControlMode(int iControlMode);
	
	bool readCurrentJointPosition(std::vector<uint16_t>& vCurrentJointPosition);
	bool readCurrentJointPosition(std::vector<float>& vCurrentJointPosition);
	bool sendTargetJointPosition(std::vector<uint16_t>& vTargetJointPosition);
	bool sendTargetJointPosition(std::vector<float>& vTargetJointPosition);
	bool sendTargetJointVelocity(std::vector<uint16_t>& vTargetJointVelocity);
	bool sendTargetJointVelocity(std::vector<float>& vTargetJointVelocity);
	int convertAngleToJointCmd(float fJointAngle);
	float convertJointCmdToAngle(int iJointCmd);
	int convertJointVelocityToJointCmd(float fJointVelocity);

private:
	std::string m_sDeviceName;
	float m_fProtocolVersion;
	int m_i32BaudRate;
	
	dynamixel::PortHandler* m_pPortHandler;
	dynamixel::PacketHandler* m_pPacketHandler;
	
	bool m_bIsDeviceNameSet;
	bool m_bIsProtocolVersionSet;
	bool m_bIsPortOpened;
	bool m_bIsBaudRateSet;
	
	int m_i32DxlCommunicationResult;             // Communication result
	uint8_t m_ui8DxlError;                          // Dynamixel error
	std::vector<uint16_t> m_vDxlCurrentPosition;              // Present position

	float m_fMinJointCmd = 0;
	float m_fMaxJointCmd = 1023;
	float m_fMinJointAngle = -150.0f/180.0f*M_PI;
	float m_fMaxJointAngle = 150.0f/180.0f*M_PI;

};