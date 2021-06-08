/*
 * Futaba.hpp
 *
 *  Created on: 2019/07/28
 *      Author: tench
 */

#ifndef RS40XCB_HPP_
#define RS40XCB_HPP_

#include <vector>
#include <cstring>
#include "stm32f3xx_hal.h"
#include"HAL_global.hpp"

class RS40xCB{
public:
	RS40xCB(const IOPin io);
	 ~RS40xCB();
	void send_data(uint8_t id, uint8_t adr, char data);
	void SendDataOfShortpacket(const uint8_t id, const uint8_t address, const std::vector<uint8_t>& data);
	void SendDataOfLongpacket(const std::vector<uint8_t> id,const uint8_t address, const std::vector<std::vector<uint8_t>> data);
	void TorqueOn(std::vector<uint8_t>sendector_short_torque_on);
	void ServoMotorInitialPosition(std::vector<uint8_t>sendVector_short_initial_position);
	void Init(UART_HandleTypeDef* uarthandle);
	void write_rom(uint8_t id);
	void change_id(uint8_t old_id, uint8_t new_id);
	inline std::vector<uint8_t> get_shortpacketServoData(){return shortpacketServoData_;}
	inline std::vector<uint8_t> get_longpacketServoId(){return longpacketServoId_;}
	inline std::vector<std::vector<uint8_t>> get_longpacketServoData(){return longpacketServoData_;}
    inline void set_shortpacketServoData(uint8_t set_id){shortpacketServoData_.push_back(set_id);}
    inline void set_longpacketServoId(uint8_t set_id){longpacketServoId_.push_back(set_id);}
    inline void set_longpacketServoData(std::vector<uint8_t> set_data){longpacketServoData_.push_back(set_data);}
    inline void clear_longpacketServoId(){longpacketServoId_.clear();}
    inline void clear_longpacketServoData(){longpacketServoData_.clear();}
private:
//  static constexpr uint8_t kHeader_HighByte = 0xFA;
//	static constexpr uint8_t kHeader_LowByte  = 0xAF;
	static constexpr uint8_t kHeader_ALLSERVO = 0xFF;
	static constexpr uint8_t kAdr_TorqueOn = 0x24;
    const uint8_t kHeader_HighByte = 0xFA;
	const uint8_t kHeader_LowByte = 0xAF;
//	static constexpr uint8_t kData_TorqueOn = 0x01;
	const uint8_t kData_TorqueOn = 0x01;
	static constexpr uint8_t kID = 0x04;
	static constexpr uint8_t kWRITE_FLASH_ROM  = 0x40;
	std::vector<uint8_t> shortpacketServoData_;
	std::vector<uint8_t> longpacketServoId_;
	std::vector<std::vector<uint8_t>> longpacketServoData_;
	UART_HandleTypeDef* uart_;
	IOPin servoWriteRead_;
};


#endif /* RS40XCB_HPP_ */
