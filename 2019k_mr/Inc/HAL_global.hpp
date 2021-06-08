/*
 * HAL_global.hpp
 *
 *  Created on: 2019/07/28
 *      Author: tench
 */

#ifndef HAL_GLOBAL_HPP_
#define HAL_GLOBAL_HPP_

/*
 * HAL���C�u������c++�Ŏg�p���邽�߂�extern
 */
extern "C"{
#ifdef STM32F303xx
#include "stm32f3xx_hal.h"
#endif
}

class IOPin{
public:
	GPIO_TypeDef* const	port_;
	const uint16_t		pin_;


	/*
	 * �R���X�g���N�^
	 * ������:�|�[�g�A�h���X, ������:�s��bit
	 */
	IOPin(GPIO_TypeDef* io_port, uint16_t io_pin):port_(io_port), pin_(io_pin){};

	inline void write(bool state) const{port_->BSRR = pin_ << (state ? 0 : 16);}//HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	~IOPin(){};
};

#endif /* HAL_GLOBAL_HPP_ */
