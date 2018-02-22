/*
 * softServo.cpp:
 *	Provides 8 channels of software driven PWM.
 *	Copyright (c) 2018 Nikita Kuchinskiy nik.pda@gmail.com
 */

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>


#include "sunxi_gpio.h"
#include "softServo.h"

#define	MAX_SERVOS	8

struct ServoParamStruct {
	uint16_t  pulse_len;
	pthread_t thread;
};

static int16_t servoSoftChanPinMap[SERVO_MAX_SOFT_CHANNELS]={0};

static ServoParamStruct SERVO[SERVO_MAX_SOFT_CHANNELS];

//static uint16_t marks		[MAX_PINS];
//static uint16_t range		[MAX_PINS];
//static pthread_t threads	[MAX_PINS] = {0};

volatile int8_t NewServoChan = -1;

int8_t getServoChannelByPin (uint16_t pin) {
int8_t i;
	// Search for channel assigned to this pin
	for (i = 0; i<SERVO_MAX_SOFT_CHANNELS; ++i) {
		if (servoSoftChanPinMap[i] == pin) { break; };
	};//for(i)
	if (i >= SERVO_MAX_SOFT_CHANNELS) { return -1; }; // soft PWM on this pin doesn't exist
  return i;
};//()

/*
 * servoSoftChanThread:
 *	Thread to do the actual PWM output
 *********************************************************************************
 */

static void *servoSoftChanThread (void *par) {

	uint8_t ch;
	uint16_t value = 0;
	uint16_t space = SERVO_MAX_SPACE_LEN;
	uint16_t counter=0;

	//struct sched_param param;
	//param.sched_priority = sched_get_priority_max (SCHED_RR);
	//pthread_setschedparam (pthread_self(), SCHED_RR, &param);

	ch = NewServoChan;
	NewServoChan = -1;

	sched_param param = {0};
	param.sched_priority = sched_get_priority_max (SCHED_RR);
//	printf("Setting thread priority: %d (max %d)\r\n", val, param.sched_priority);
	if (SERVO_THREAD_PRIORITY < param.sched_priority)	param.sched_priority = SERVO_THREAD_PRIORITY;
	sched_setscheduler (0, SCHED_RR, &param);


	for (;;++counter) {
		if (value == SERVO[ch].pulse_len) {
			if (counter > (SERVO_CHANGE_LEN/(SERVO_MAX_SPACE_LEN/1000))) { delayMicrosecondsServo(SERVO_MAX_SPACE_LEN); continue; };
		} else {
			value = SERVO[ch].pulse_len;
			space = SERVO_MAX_SPACE_LEN - value;
			counter=0;
		};//else if()

		sunxi_digitalWrite(servoSoftChanPinMap[ch], HIGH);
		delayMicrosecondsServo(value); //a bit less noisy

		sunxi_digitalWrite(servoSoftChanPinMap[ch], LOW);
		delayMicrosecondsServo(space);

	};//inf loop
  //return NULL;
};//PI_THREAD()


/*
 * servoSoftChanWrite:
 *	Write a PWM value to the given channel
 *********************************************************************************
 */

int8_t servoSoftChanSetPulse(uint8_t ch, uint16_t newValue) {

	if (ch >= SERVO_MAX_SOFT_CHANNELS)  { return -1; };
	if (newValue > SERVO_MAX_PULSE_LEN) { newValue = SERVO_MAX_PULSE_LEN; return 1; };
	if (newValue < SERVO_MIN_PULSE_LEN) { newValue = SERVO_MIN_PULSE_LEN; return 1; };

	SERVO[ch].pulse_len = newValue;

  return 0;
};//()

/*
 * servoSoftChanCreate:
 *	Create a new servoSoftChan thread.
 *********************************************************************************
 */

int8_t servoSoftChanStart(uint8_t pin)
{
	int res;
	pthread_t myThread;
	int8_t ch = getServoChannelByPin(pin);

	//first, check if this pin is alreay used
	if (ch >= 0) { return -1; }; // already exist

	//first, check if this pin is alreay used
	for (ch=0; ch < SERVO_MAX_SOFT_CHANNELS; ++ch) {
		if (SERVO[ch].thread == 0) { servoSoftChanPinMap[ch] = pin; break; }; // found available spot
	};//for(i)
	if (ch >= SERVO_MAX_SOFT_CHANNELS) { return -2; }; // all sofw pwm channels are busy

	sunxi_set_gpio_mode(pin, OUTPUT);
	sunxi_digitalWrite (pin, LOW);

	SERVO[ch].pulse_len = 1500;

	NewServoChan = ch;
	if ((res = pthread_create (&myThread, NULL, servoSoftChanThread, NULL)) != 0) { return -3; }; // error creatig thread
	SERVO[ch].thread = myThread;

	//res has to be 0 here
	for (; (NewServoChan == -1) && (res < SERVO_THREAD_ACTIVATION_TIMOUT); ++res) { delayMicroseconds(1000); };

	if (res >= SERVO_THREAD_ACTIVATION_TIMOUT) {
		servoSoftChanStop(ch);
		return -4; // timeout pwm thread activation [10ms]
	};//if()

	//usleep(1000);

  return ch;
};//servoSoftChanStart()


/*
 * servoSoftChanStop:
 *	Stop an existing servoSoftChan thread
 *********************************************************************************
 */

int8_t servoSoftChanStop(uint8_t ch) {

	if (ch >= SERVO_MAX_SOFT_CHANNELS) { return -1; };

	if (SERVO[ch].thread != 0) {
		//printf("\t cancel...\r\n");
		pthread_cancel(SERVO[ch].thread);
		//printf("\t join...\r\n");
		//printf("\t Pars: %d, 0x%X\r\n", SERVO[ch].pulse_len, SERVO[ch].thread);

		pthread_join  (SERVO[ch].thread, NULL);
		SERVO[ch].pulse_len = 1500;
		SERVO[ch].thread = 0;
		sunxi_digitalWrite (servoSoftChanPinMap[ch], LOW);
		servoSoftChanPinMap[ch] = -1;
	} else {
		return -2; // thread was already destroyed or wasn't created, or something else went wrong
	};//else

  return 0;
};//servoSoftChanStopChan()

int8_t servoSoftChanStopOnPin(uint16_t pin) {
  int8_t ch = getServoChannelByPin(pin);

	if (ch < 0) { return -2; } // PWM channel not found
	else { return servoSoftChanStop(ch); };

};//servoSoftChanStop()






