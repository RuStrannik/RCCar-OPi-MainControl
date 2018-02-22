/*
 * softPwm.h:
 *	Provide 2 channels of software driven PWM.
 *	Copyright (c) 2012 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#ifndef SOFT_PWM_H
#define SOFT_PWM_H

#define	PWM_MAX_SOFT_CHANNELS			8
#define	PWM_THREAD_PRIORITY				90
#define	PWM_THREAD_ACTIVATION_TIMOUT	10 //[ms]

#define PWM_MAX_VAL 1000
#define PWM_MAX_FREQ 10000

#ifdef __cplusplus
extern "C" {
#endif

int8_t pwmSoftChanStart(uint8_t pin, uint16_t pwmFreqHZ);
int8_t pwmSoftChanSetVal(uint8_t ch, uint16_t newValue);
int8_t pwmSoftChanSetFreq(uint8_t ch, uint16_t newFrequency);
int8_t pwmSoftChanSetPin(uint8_t ch, uint16_t pin);
int8_t pwmSoftChanStop(uint8_t ch);
int8_t getPwmSoftChanByPin (uint16_t pin);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SOFT_PWM_H
