#include "Arduino.h"
#include "myPWMServo.h"
#include "errors.h"

/*
  myPWMServo.cpp - Hardware Servo Timer Library
  http://arduiniana.org/libraries/myPWMServo/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to myPWMServo by Mikal Hart
  ported to other chips by Paul Stoffregen
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define NO_PULSE (0)

myPWMServo::myPWMServo(PinName pinArg, TIM_TypeDef *InstanceArg, uint32_t channelArg)
{
	// if (pinArg < 0 || pinArg >= NUM_DIGITAL_PINS) {
	// 	returnCode = ERR_FAILED; return;
	// }
	// if (!digitalPinHasPWM(pinArg)) {
	// 	returnCode = ERR_FAILED; return;
	// }
	_pin = pinArg;
	_HW_TIM = new HardwareTimer(InstanceArg);
	_channel = channelArg;
  _HW_TIM->setMode(_channel, TIMER_OUTPUT_COMPARE_PWM1, _pin);
	_pulseWidth = NO_PULSE;
  _HW_TIM->pause();
}

uint8_t myPWMServo::attach(uint8_t pinArg, uint16_t freq, uint16_t min, uint16_t max)
{
	_pulseWidthMin = min;
	_pulseWidthMax = max;
	digitalWriteFast(_pin, LOW);
  _HW_TIM->setOverflow(freq, HERTZ_FORMAT);
	_HW_TIM->resume();
	return ERR_OK;
}

void myPWMServo::write(uint16_t pulseWidthUSec)
{
	if (pulseWidthUSec < _pulseWidthMin) pulseWidthUSec = _pulseWidthMin;
	if (pulseWidthUSec > _pulseWidthMax) pulseWidthUSec = _pulseWidthMax;
	_HW_TIM->setCaptureCompare(_channel, pulseWidthUSec, MICROSEC_COMPARE_FORMAT);
}