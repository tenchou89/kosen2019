/*
 * sequence.cpp
 *
 *  Created on: 2019/07/29
 *      Author: tench
 */
#include "sequence.hpp"

SequenceTshit::SequenceTshit(RS40xCB* servo) : servo_(servo){

}

SequenceTshit::~SequenceTshit(){

}

void SequenceTshit::ServoMotorStandBy(){
//	servo_->set_longpacketServoId(kServoID_3);
//	servo_->set_longpacketServoId(kServoID_4);
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_HighByte});
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_LowByte});
	servo_->SendDataOfLongpacket(servo_->get_longpacketServoId(),kIndicatedPosition,servo_->get_longpacketServoData());
//	HAL_Delay(1);
//	servo_->clear_longpacketServoId();
//	servo_->clear_longpacketServoData();

	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_HighByte});
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_LowByte});
	 std::vector<uint8_t> set_data1;
	 set_data1.push_back(0x84);
	 set_data1.push_back(0x03);
	 servo_->SendDataOfShortpacket(kServoID_6,kIndicatedPosition,set_data1);
}


void SequenceTshit::LoadServoMotor(){
	/*
	servo_->set_longpacketServoId(kServoID_3);
	servo_->set_longpacketServoId(kServoID_4);
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendm90_HighByte});
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendm90_LowByte});
	servo_->SendDataOfLongpacket(servo_->get_longpacketServoId(),kIndicatedPosition,servo_->get_longpacketServoData());
	HAL_Delay(1);
	servo_->clear_longpacketServoId();
	servo_->clear_longpacketServoData();
	*/
	 std::vector<uint8_t> set_data2;
	 set_data2.push_back(0x7c);
	 set_data2.push_back(0xfc);
	 servo_->SendDataOfShortpacket(0x06,0x1E,set_data2);
}

void SequenceTshit::ServoMotorFiring(){
	/*
	servo_->set_longpacketServoId(kServoID_1);
	servo_->set_longpacketServoId(kServoID_2);
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_HighByte});
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_LowByte});
	servo_->SendDataOfLongpacket(servo_->get_longpacketServoId(),kIndicatedPosition,servo_->get_longpacketServoData());
    HAL_Delay(1);
	servo_->clear_longpacketServoId();
	servo_->clear_longpacketServoData();
	*/
	std::vector<uint8_t> set_data3;
    set_data3.push_back(0x84);
	set_data3.push_back(0x03);
	servo_->SendDataOfShortpacket(0x06,0x1E,set_data3);
}

void SequenceTshit::ServoMotorEnd(){
	/*
	servo_->set_longpacketServoId(kServoID_1);
	servo_->set_longpacketServoId(kServoID_2);
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_HighByte});
	servo_->set_longpacketServoData(std::vector<uint8_t>{kServoSendp90_LowByte});
	servo_->SendDataOfLongpacket(servo_->get_longpacketServoId(),kIndicatedPosition,servo_->get_longpacketServoData());
	HAL_Delay(1);
	servo_->clear_longpacketServoId();
	servo_->clear_longpacketServoData();
	*/
	std::vector<uint8_t> set_data4;
	set_data4.push_back(0x7c);
	set_data4.push_back(0xfc);
	servo_->SendDataOfShortpacket(0x06,0x1E,set_data4);
}


void SequenceTshit::PushCylinder(IOPin inCharge){
	inCharge.write(true);
}

void SequenceTshit::PullCylinder(IOPin inCharge){
	inCharge.write(false);
}

SequenceTshit::CanSendModeTshit SequenceTshit::SequenceSelectionTshit(uint8_t send_data){
	CanSendModeTshit receiveSequenseTshit = CanSendModeTshit::kTshitWaiting;

	switch(send_data){
	case 0x00:
		receiveSequenseTshit = CanSendModeTshit::kTshitWaiting;
		break;
	case 0x01:
		receiveSequenseTshit = CanSendModeTshit::kTshitRoading;
		break;
	case 0x02:
		receiveSequenseTshit = CanSendModeTshit::kTshitPutting;
		break;
	case 0x03:
		receiveSequenseTshit = CanSendModeTshit::kTshitFiring;
		break;
	default:
		break;
	}
	return receiveSequenseTshit;
}

SequenceElevator::SequenceElevator(MotorDriver* elevatorMotor) : elevatorMotor_(elevatorMotor){

}

SequenceElevator::~SequenceElevator(){

}


float SequenceElevator::PidControl(float cumulative_rotation_number,float target_height){
	previous_constant_data = constant_data;
	constant_data = (target_height-cumulative_rotation_number);
	P  = kp * static_cast<float>(constant_data);
	I += ki * static_cast<float>(constant_data);
	D  = kd * (constant_data - previous_constant_data);
	return P+I+D;
}

void SequenceElevator::InitialPosition(bool &initial_position_interrupt){
	  elevatorMotor_->setDuty(0x15,-0.5);
	  while(!initial_position_interrupt);
	  elevatorMotor_->setDuty(0x15,0);
	  TIM2->CNT =0;
//	  total_rotation_number = 0;
}

SequenceElevator::CanSendModeElevator SequenceElevator::SequenceSelectionElevator(uint8_t send_data){
	CanSendModeElevator receiveSequenseElevator = CanSendModeElevator::kElevatorWaiting;
	switch(send_data){
	case 0x00:
		receiveSequenseElevator = CanSendModeElevator::kElevatorWaiting;
		break;
	case 0x01:
		receiveSequenseElevator = CanSendModeElevator::kElevatorGoseUp1000mm;
		break;
	case 0x02:
		receiveSequenseElevator = CanSendModeElevator::kElevatorGoseUp1500mm;
		break;
	case 0x03:
		receiveSequenseElevator = CanSendModeElevator::kElevatorMoveDown;
		break;
	default:
		break;
	}
	return receiveSequenseElevator;
}























