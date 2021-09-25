#ifndef myPWMServo_h
#define myPWMServo_h

/*
  myPWMServo.h - Hardware Servo Timer Library
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

#include <inttypes.h>

class myPWMServo
{
private:
  PinName  _pin;
  HardwareTimer *_HW_TIM;
  uint32_t _channel;
  uint16_t _pulseWidth;
  uint16_t _pulseWidthMin;
  uint16_t _pulseWidthMax;
  uint16_t _pulseWidthMicroSeconds;

public:
  myPWMServo(PinName pinArg, TIM_TypeDef *InstanceArg, uint32_t channelArg);
  uint8_t attach(uint8_t pinArg, uint16_t freq, uint16_t min, uint16_t max);
  void write(uint16_t pulseWidthArg);
};

#endif