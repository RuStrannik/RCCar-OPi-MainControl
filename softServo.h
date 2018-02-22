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


#ifndef SOFT_SERVO_H
#define SOFT_SERVO_H

#define	SERVO_MAX_SOFT_CHANNELS			8
#define	SERVO_THREAD_PRIORITY			99
#define	SERVO_THREAD_ACTIVATION_TIMOUT	10 //[ms]

#define SERVO_CHANGE_LEN				300	// [mS]
#define SERVO_MAX_SPACE_LEN				20000	// [uS]
#define SERVO_MAX_PULSE_LEN				2500	// [uS]
#define SERVO_MIN_PULSE_LEN				500		// [uS]

#ifdef __cplusplus
extern "C" {
#endif

int8_t servoSoftChanStart(uint8_t pin);
int8_t servoSoftChanSetPulse(uint8_t ch, uint16_t newValue);
int8_t servoSoftChanStop(uint8_t ch);
int8_t servoSoftChanStopOnPin(uint16_t pin);
int8_t getServoChannelByPin (uint16_t pin);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SOFT_SERVO_H
