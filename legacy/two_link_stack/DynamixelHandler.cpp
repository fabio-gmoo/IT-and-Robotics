#include "DynamixelHandler.h"
#include <algorithm>
#include <cmath>
#include <iostream>



DynamixelHandler::DynamixelHandler():

	m_sDeviceName(""), m_fProtocolVersion(0.0), m_i32BaudRate(0),

	m_pPacketHandler(nullptr), m_pPortHandler(nullptr),

	m_bIsDeviceNameSet(false), m_bIsProtocolVersionSet(false), m_bIsPortOpened(false), m_bIsBaudRateSet(false),

	m_ui8DxlError(0), m_i32DxlCommunicationResult(COMM_TX_FAIL)

{

	

}



DynamixelHandler::~DynamixelHandler()

{

	

}



int DynamixelHandler::convertJointVelocityToJointCmd(float fJointVelocity)

{ 

	if (fJointVelocity == 0.0f)

		return 0;

	

	float a = 0.0f;

	float b = 0.0f;

	if (fJointVelocity>0)

	{

		float l_fMaxJointCmd = 1023;

		float l_fMinJointCmd = 0;

		float l_fMaxJointVelocity = 114 * 60.0f * 2 * M_PI;

		float l_fMinJointAngle = 0.0f;

		// y = ax + b

		a = (l_fMaxJointCmd-l_fMinJointCmd) / (l_fMaxJointVelocity - l_fMinJointAngle);

		b = l_fMinJointCmd - a * l_fMinJointAngle;

	}

	

	if (fJointVelocity<0)

	{

		float l_fMaxJointCmd = 2047;

		float l_fMinJointCmd = 1024;

		float l_fMaxJointVelocity = 0.0f;

		float l_fMinJointAngle = -114 * 60.0f * 2 * M_PI;

		// y = ax + b

		a = (l_fMaxJointCmd-l_fMinJointCmd) / (l_fMaxJointVelocity - l_fMinJointAngle);

		b = l_fMinJointCmd - a * l_fMinJointAngle;

	}

	

	float jointCmd = a * fJointVelocity + b;

	return (int)jointCmd;

}



int DynamixelHandler::convertAngleToJointCmd(float fJointAngle)

{ 

	// y = ax + b

	float a = (m_fMaxJointCmd-m_fMinJointCmd) / (m_fMaxJointAngle - m_fMinJointAngle);

	float b = m_fMinJointCmd - a * m_fMinJointAngle;

	float jointCmd = a * fJointAngle + b;

	return (int)jointCmd;

}



float DynamixelHandler::convertJointCmdToAngle(int iJointCmd)

{ 

	// y = ax + b

	float a = (m_fMaxJointAngle - m_fMinJointAngle) / (m_fMaxJointCmd-m_fMinJointCmd);

	float b = m_fMinJointAngle - a * m_fMinJointCmd;

	float jointAngle = a * iJointCmd + b;

	return jointAngle;

}



bool DynamixelHandler::openPort()

{

	if (m_pPortHandler == nullptr)

	{

		std::cout << "[ERROR](DynamixelHandler::openPort) m_pPortHandler is null!" << std::endl;

		m_bIsPortOpened = false;

		return m_bIsPortOpened;

	}

	

	if (!m_bIsDeviceNameSet)

	{

		std::cout << "[ERROR](DynamixelHandler::openPort) m_sDeviceName is not set!" << std::endl;

		m_bIsPortOpened = false;

		return m_bIsPortOpened;

	}

	

	if (m_bIsPortOpened)

	{

		std::cout << "[WARNING](DynamixelHandler::openPort) port is already opened!" << std::endl;

		return m_bIsPortOpened;

	}

	

	if (m_pPortHandler->openPort())

	{

		std::cout << "[INFO](DynamixelHandler::openPort) Succeeded to open the port!" << std::endl;

		m_bIsPortOpened = true;

	}

	else

	{

		std::cout << "[ERROR](DynamixelHandler::openPort) Failed to open the port!" << std::endl;

		m_bIsPortOpened = false;

	}

	return m_bIsPortOpened;

}



void DynamixelHandler::closePort()

{

	if (m_pPortHandler == nullptr)

	{

		std::cout << "[ERROR](DynamixelHandler::closePort) m_pPortHandler is null!" << std::endl;

		m_bIsPortOpened = false;

		return;

	}

	

	if (!m_bIsPortOpened)

	{

		std::cout << "[WARNING](DynamixelHandler::openPort) port is already closed!" << std::endl;

		return;

	}

	

	m_pPortHandler->closePort();

	

	std::cout << "[INFO](DynamixelHandler::closePort) Succeeded to close the port!" << std::endl;

	m_bIsPortOpened = false;

}



bool DynamixelHandler::setBaudRate(int i32BaudRate)

{

	m_i32BaudRate = i32BaudRate;

		

	if (nullptr != m_pPortHandler)

	{

		if (m_pPortHandler->setBaudRate(m_i32BaudRate))

		{

			std::cout << "[INFO](DynamixelHandler::setBaudRate) Succeeded to change the baudrate!" << std::endl;

			m_bIsBaudRateSet = true;

		}

		else

		{

			std::cout << "[ERROR](DynamixelHandler::setBaudRate) Failed to change the baudrate!" << std::endl;

			m_bIsBaudRateSet = false;

		}

	}

	else

	{

		std::cout << "[ERROR](DynamixelHandler::setBaudRate) m_pPortHandler is null!" << std::endl;

		m_bIsBaudRateSet = false;

	}

	return m_bIsBaudRateSet;

}



void DynamixelHandler::setDeviceName(std::string sDeviceName)

{

	m_sDeviceName = sDeviceName;

	m_bIsDeviceNameSet = true;

	

	if (nullptr != m_pPortHandler)

	{

		delete m_pPortHandler;

		m_pPortHandler = nullptr;

	}

	

	// Initialize PortHandler instance

	m_pPortHandler = dynamixel::PortHandler::getPortHandler(m_sDeviceName.c_str());

}



void DynamixelHandler::setProtocolVersion(float fProtocolVersion)

{

	m_fProtocolVersion = fProtocolVersion;

	m_bIsProtocolVersionSet = true;

	

	if (nullptr != m_pPacketHandler)

	{

		delete m_pPacketHandler;

		m_pPacketHandler = nullptr;

	}

	

	m_pPacketHandler = dynamixel::PacketHandler::getPacketHandler(m_fProtocolVersion);

}



bool DynamixelHandler::readCurrentJointPosition(std::vector<float>& vCurrentJointPosition)
{
    std::vector<uint16_t> rawPositions;
    if (!this->readCurrentJointPosition(rawPositions) || rawPositions.size() != NB_JOINTS || NB_JOINTS != 3) {
        std::cerr << "[DXL][ERROR] Cannot read all three motor positions.\n";
        return false;
    }
    vCurrentJointPosition.clear();
    vCurrentJointPosition.push_back(ROT_DIRECTION_Q1 * convertJointCmdToAngle(rawPositions[0]));
    vCurrentJointPosition.push_back(ROT_DIRECTION_QPEN * convertJointCmdToAngle(rawPositions[1]));
    vCurrentJointPosition.push_back(ROT_DIRECTION_Q2 * convertJointCmdToAngle(rawPositions[2]));
    return true;
}




bool DynamixelHandler::readCurrentJointPosition(std::vector<uint16_t>& vCurrentJointPosition)
{
    if (!m_bIsPortOpened || !m_bIsBaudRateSet || !m_pPortHandler || !m_pPacketHandler) {
        std::cerr << "[DXL][ERROR] Cannot read positions: serial connection not ready.\n";
        return false;
    }
    vCurrentJointPosition.clear();
    bool allSuccess = true;
    for (unsigned int joint = 0; joint < NB_JOINTS; ++joint) {
        const unsigned int id = joint + 1;
        uint8_t error = 0;
        uint16_t presentPosition = 0;
        const int result = m_pPacketHandler->read2ByteTxRx(
            m_pPortHandler, id, ADDR_XL320_PRESENT_POSITION, &presentPosition, &error);
        if (result != COMM_SUCCESS) {
            std::cerr << "[DXL][ERROR] Read ID " << id << ": "
                      << m_pPacketHandler->getTxRxResult(result) << '\n';
            allSuccess = false;
        } else if (error != 0) {
            std::cerr << "[DXL][ERROR] Read ID " << id << ": "
                      << m_pPacketHandler->getRxPacketError(error) << '\n';
            allSuccess = false;
        } else {
            vCurrentJointPosition.push_back(presentPosition);
            std::cout << "[DXL][OK] Read ID " << id
                      << " position=" << presentPosition << "\n";
        }
    }
    return allSuccess;
}




bool DynamixelHandler::sendTargetJointPosition(std::vector<float>& vTargetJointPosition)
{
    if (vTargetJointPosition.size() != NB_JOINTS || NB_JOINTS != 3) {
        std::cerr << "[DXL][ERROR] Expected three joint angles (q1, pen, q2).\n";
        return false;
    }

    const std::vector<float> motorAngles = {
        ROT_DIRECTION_Q1 * vTargetJointPosition[0],
        ROT_DIRECTION_QPEN * vTargetJointPosition[1],
        ROT_DIRECTION_Q2 * vTargetJointPosition[2]
    };

    std::vector<uint16_t> motorCommands;
    for (size_t i = 0; i < motorAngles.size(); ++i) {
        float angle = motorAngles[i];
        float minAngle = std::min(m_fMinJointAngle, m_fMaxJointAngle);
        float maxAngle = std::max(m_fMinJointAngle, m_fMaxJointAngle);
        if (!std::isfinite(angle) || angle < minAngle || angle > maxAngle) {
            std::cerr << "[DXL][ERROR] Motor ID " << (i+1)
                      << ": target angle " << angle
                      << " rad is outside [" << minAngle << ", " << maxAngle << "] rad.\n";
            return false;
        }
        int command = convertAngleToJointCmd(angle);
        int minCmd = static_cast<int>(std::min(m_fMinJointCmd, m_fMaxJointCmd));
        int maxCmd = static_cast<int>(std::max(m_fMinJointCmd, m_fMaxJointCmd));
        if (command < minCmd || command > maxCmd) {
            std::cerr << "[DXL][ERROR] Motor ID " << (i+1)
                      << ": converted goal " << command
                      << " is outside [" << minCmd << ", " << maxCmd << "].\n";
            return false;
        }
        motorCommands.push_back(static_cast<uint16_t>(command));
    }

    return this->sendTargetJointPosition(motorCommands);
}




bool DynamixelHandler::sendTargetJointPosition(std::vector<uint16_t>& vTargetJointPosition)
{
    if (vTargetJointPosition.size() != NB_JOINTS) {
        std::cerr << "[DXL][ERROR] Wrong number of goal positions.\n";
        return false;
    }
    if (!m_bIsPortOpened || !m_bIsBaudRateSet || !m_pPortHandler || !m_pPacketHandler) {
        std::cerr << "[DXL][ERROR] Port, baud rate or packet handler is not ready.\n";
        return false;
    }

    bool allSuccess = true;
    for (unsigned int joint = 0; joint < NB_JOINTS; ++joint) {
        const unsigned int id = joint + 1;
        uint8_t error = 0;
        const uint16_t goal = vTargetJointPosition[joint];
        const int result = m_pPacketHandler->write2ByteTxRx(
            m_pPortHandler, id, ADDR_XL320_GOAL_POSITION, goal, &error);
        if (result != COMM_SUCCESS) {
            std::cerr << "[DXL][ERROR] ID " << id << " goal=" << goal
                      << ": " << m_pPacketHandler->getTxRxResult(result) << '\n';
            allSuccess = false;
        } else if (error != 0) {
            std::cerr << "[DXL][ERROR] ID " << id << " goal=" << goal
                      << ": " << m_pPacketHandler->getRxPacketError(error) << '\n';
            allSuccess = false;
        } else {
            std::cout << "[DXL][OK] ID " << id << " goal=" << goal << " accepted.\n";
        }
    }
    return allSuccess;
}




bool DynamixelHandler::sendTargetJointVelocity(std::vector<float>& vTargetJointVelocity)

{

	// Checks if the input vector has the right size

	if (vTargetJointVelocity.size() != NB_JOINTS)

	{

		std::cout << "[ERROR] (sendTargetJointVelocity) Input vector has not the right size!" << std::endl;

		return false;

	}

	

	// Creates a vector of motor commands

	std::vector<uint16_t> l_vTargetJointVelocity;

	// q1

	l_vTargetJointVelocity.push_back(convertJointVelocityToJointCmd(ROT_DIRECTION_Q1*vTargetJointVelocity[0]));

	// qpen

	l_vTargetJointVelocity.push_back(convertJointVelocityToJointCmd(ROT_DIRECTION_QPEN*vTargetJointVelocity[1]));

	// q2

	l_vTargetJointVelocity.push_back(convertJointVelocityToJointCmd(ROT_DIRECTION_Q2*vTargetJointVelocity[2]));

	

	std::cout << "l_vTargetJointVelocity= " << l_vTargetJointVelocity[0] << ", " <<  l_vTargetJointVelocity[1] << ", " << l_vTargetJointVelocity[2]<< std::endl;

	

	// call the dxl sendTargetJointPosition

	bool bIsSendSuccessfull = this->sendTargetJointVelocity(l_vTargetJointVelocity);

	

	return bIsSendSuccessfull;

}



bool DynamixelHandler::sendTargetJointVelocity(std::vector<uint16_t>& vTargetJointVelocity)

{

	bool bIsSendSuccessfull = false;



	// checks if the vector size is correct

	if (vTargetJointVelocity.size() != NB_JOINTS)

	{

		std::cout << "[ERROR] (DynamixelHandler::sendTargetJointVelocity): Size of command vector is not correct: " << vTargetJointVelocity.size() << " instead of " << NB_JOINTS << "!" << std::endl;

		bIsSendSuccessfull = false;

	}



	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)

	{

		int dxl_comm_result = COMM_TX_FAIL;             // Communication result

		uint8_t dxl_error = 0;

		uint16_t dxl_present_position = 0;

		dxl_comm_result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_GOAL_VELOCITY, vTargetJointVelocity[l_joint], &dxl_error);

		if (dxl_comm_result != COMM_SUCCESS)

		{

			//std::cout << "[ERROR] (DynamixelHandler::sendTargetJointPosition): " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;

			bIsSendSuccessfull = false;

		}

		else if (dxl_error != 0)

		{

			//std::cout << "[ERROR] (DynamixelHandler::sendTargetJointPosition): " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;

			bIsSendSuccessfull = false;

		}

		else

		{

			bIsSendSuccessfull = true;

		}

	}

	return bIsSendSuccessfull;



}



bool DynamixelHandler::enableTorque(bool bEnableTorque)
{
    if (!m_bIsPortOpened || !m_bIsBaudRateSet || !m_pPortHandler || !m_pPacketHandler) {
        std::cerr << "[DXL][ERROR] Cannot change torque: serial connection not ready.\n";
        return false;
    }

    bool allSuccess = true;
    for (unsigned int joint = 0; joint < NB_JOINTS; ++joint) {
        const unsigned int id = joint + 1;
        uint8_t error = 0;
        const int result = m_pPacketHandler->write1ByteTxRx(
            m_pPortHandler, id, ADDR_XL320_TORQUE_ENABLE,
            bEnableTorque ? 1 : 0, &error);
        if (result != COMM_SUCCESS) {
            std::cerr << "[DXL][ERROR] ID " << id << " torque="
                      << (bEnableTorque ? "ON" : "OFF") << ": "
                      << m_pPacketHandler->getTxRxResult(result) << '\n';
            allSuccess = false;
        } else if (error != 0) {
            std::cerr << "[DXL][ERROR] ID " << id << " torque="
                      << (bEnableTorque ? "ON" : "OFF") << ": "
                      << m_pPacketHandler->getRxPacketError(error) << '\n';
            allSuccess = false;
        } else {
            std::cout << "[DXL][OK] ID " << id << " torque="
                      << (bEnableTorque ? "ON" : "OFF") << " accepted.\n";
        }
    }
    return allSuccess;
}




bool DynamixelHandler::setControlMode(int iControlMode)

{

	bool bIsSendSuccessfull = false;

	

	for (unsigned int l_joint = 0; l_joint < NB_JOINTS; l_joint++)

	{

		int dxl_comm_result = COMM_TX_FAIL;             // Communication result

		uint8_t dxl_error = 0;

		

		dxl_comm_result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, l_joint + 1, ADDR_XL320_CONTROL_MODE, iControlMode, &dxl_error);

		if (dxl_comm_result != COMM_SUCCESS) 

		{

			//std::cout << "[ERROR] (DynamixelHandler::enableTorque): " << m_pPacketHandler->getTxRxResult(dxl_comm_result) << std::endl;

			bIsSendSuccessfull = false;

		}

		else if (dxl_error != 0) 

		{

			//std::cout << "[ERROR] (DynamixelHandler::enableTorque): " << m_pPacketHandler->getRxPacketError(dxl_error) << std::endl;

			bIsSendSuccessfull = false;

		}

		else 

		{

			bIsSendSuccessfull = true;

		}

	}

	return bIsSendSuccessfull;


}

