/*
 * sequence.hpp
 *
 *  Created on: 2019/07/29
 *      Author: tench
 */

#ifndef SEQUENCE_HPP_
#define SEQUENCE_HPP_

#include <RS40xCB.hpp>
#include "stm32f3xx_hal.h"
#include "MotorDriver.hpp"


class SequenceTshit{
public:
	enum class CanSendModeTshit{
		kTshitWaiting     = 0x00,
		kTshitemEmergency = 0x10,
		kTshitRoading     = 0x01,
		kTshitPutting     = 0x02,
		kTshitFiring      = 0x03
	};

	RS40xCB* servo_;
	SequenceTshit(RS40xCB* servo);
	~SequenceTshit();
	void ServoMotorStandBy();
	void LoadServoMotor();
	void ServoMotorFiring();
	void ServoMotorEnd();
	void PushCylinder(const IOPin object);
	void PullCylinder(const IOPin object);
	CanSendModeTshit SequenceSelectionTshit(const uint8_t send_data);
private:
	static constexpr uint8_t kServoID_1 = 0x01;
	static constexpr uint8_t kServoID_2 = 0x02;
	static constexpr uint8_t kServoID_3 = 0x03;
	static constexpr uint8_t kServoID_4 = 0x04;
	static constexpr uint8_t kServoID_6 = 0x06;
	static constexpr uint8_t kServoSendp90_HighByte = 0x84;
	static constexpr uint8_t kServoSendp90_LowByte = 0x03;
	static constexpr uint8_t kServoSendm90_HighByte = 0x7C;
	static constexpr uint8_t kServoSendm90_LowByte = 0xFC;
	static constexpr uint8_t kIndicatedPosition = 0x1E;
};

class SequenceElevator{
public:
	enum class CanSendModeElevator{
		kElevatorWaiting      = 0x00,
		kElevatorEmergency    = 0x10,
		kElevatorGoseUp1000mm = 0x01,
		kElevatorGoseUp1500mm = 0x02,
		kElevatorMoveDown     = 0x03
	};

	MotorDriver* const elevatorMotor_;
	SequenceElevator(MotorDriver* elevatorMotor);
	~SequenceElevator();
	float PidControl(float cumulative_rotation_number,float target_height);
	void InitialPosition(bool &initial_position_interrupt);
	CanSendModeElevator SequenceSelectionElevator(uint8_t send_data);
private:
	static constexpr float kp = 0.3;
	static constexpr float ki = 0.2;
	static constexpr float kd = 0.1;
	float previous_constant_data=0;
	float constant_data=0;
	float P=0;
	float I=0;
	float D=0;
};


#endif /* SEQUENCE_HPP_ */
