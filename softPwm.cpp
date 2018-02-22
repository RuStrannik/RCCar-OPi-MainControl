/*
 * softPwm.cpp:
 *	Provides 8 channels of software driven PWM.
 *	Copyright (c) 2018 Nikita Kuchinskiy nik.pda@gmail.com
 */

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include "sunxi_gpio.h"
#include "softPwm.h"

// As frequency increases, resolution drops, so at max frequency of 10kHz you might only have
// resolution of 3: 0%, 50% and 100%. It's possible to get a higher resolutions at higher
// frequencies, however, CPU usage will be too high, since you'll have to start using a hard-loop
// to delay for periods under 100µS. This is because the Linux timer calls are not very accurate
// and have an overhead.

struct pwmParamStruct {
//	uint16_t  value; // 1000 = 100%, 0 = 0%
	uint32_t  len_hi;
	uint32_t  len_lo;
	uint16_t  freq;
	pthread_t thread;
};

static int16_t pwmPinMap[PWM_MAX_SOFT_CHANNELS]={-1, -1, -1, -1, -1, -1, -1, -1};

static pwmParamStruct PWM[PWM_MAX_SOFT_CHANNELS];

//static uint16_t marks		[MAX_PINS];
//static uint16_t range		[MAX_PINS];
//static pthread_t threads	[MAX_PINS] = {0};

volatile int8_t NewPwmChan = -1;

int8_t getPwmSoftChanByPin (uint16_t pin) {
  int8_t i;
	// Search for channel assigned to this pin
	for (i = 0; i<PWM_MAX_SOFT_CHANNELS; ++i) {
		if (pwmPinMap[i] == pin) { break; };
	};//for(i)
	if (i >= PWM_MAX_SOFT_CHANNELS) { return -1; }; // soft PWM on this pin doesn't exist
  return i;
};//()

/*
 * softPwmThread:
 *	Thread to do the actual PWM output
 *********************************************************************************
 */

//static PI_THREAD(SoftPwmThread) {
static void *SoftPwmThread (void *par) {

	uint8_t ch;
	uint32_t value, space;

	//struct sched_param param;
	//param.sched_priority = sched_get_priority_max (SCHED_RR);
	//pthread_setschedparam (pthread_self(), SCHED_RR, &param);

	ch = NewPwmChan;
	NewPwmChan = -1;
	sched_param param = {0};

	param.sched_priority = sched_get_priority_max (SCHED_RR);
//	printf("Setting thread priority: %d (max %d)\r\n", val, param.sched_priority);
	if (PWM_THREAD_PRIORITY < param.sched_priority)	param.sched_priority = PWM_THREAD_PRIORITY;
	sched_setscheduler (0, SCHED_RR, &param);

	for (;;) {
		value = PWM[ch].len_hi;
		space = PWM[ch].len_lo;

		if (value != 0) {
			sunxi_digitalWrite(pwmPinMap[ch], HIGH);
			delayMicroseconds (value);
		};

		if (space != 0) {
			sunxi_digitalWrite(pwmPinMap[ch], LOW);
			delayMicroseconds (space);
		};
	};//inf loop

	return NULL;
};//PI_THREAD()


/*
 * softPwmWrite:
 *	Write a PWM value to the given channel
 *********************************************************************************
 */

int8_t pwmSoftChanSetPin(uint8_t ch, uint16_t pin) {
	if (ch > PWM_MAX_SOFT_CHANNELS) { return -1; };
	pwmPinMap[ch] = pin;
	return 0;
};//pwmSoftChanSetPin()

int8_t pwmSoftChanSetVal(uint8_t ch, uint16_t newValue) {
  uint32_t window;
  float ratio;

	if (ch >= PWM_MAX_SOFT_CHANNELS) { return -2; };
	if ((PWM[ch].thread == 0) || (PWM[ch].freq == 0)) { return -1; };
	if (newValue > PWM_MAX_VAL) { newValue = PWM_MAX_VAL; return 1; };

	window = (PWM_MAX_FREQ / PWM[ch].freq); //[100*us]
	ratio = ((float)newValue) / PWM_MAX_VAL;

	// lroundf() // ceilf()
	PWM[ch].len_hi = (window * (1000000/PWM_MAX_FREQ)) * (             ratio);
	PWM[ch].len_lo = (window * (1000000/PWM_MAX_FREQ)) * ((float)1.0 - ratio);

//	printf("\tDEBUG1:\r\n"
//		   "\twindow = %d [us]\r\n"
//		   "\tratio = %2.2f\r\n"
//		   "\tlen_hi = %d[us]\r\n"
//		   "\tlen_lo = %d[us]\r\n",
//		   window*100, ratio*100, PWM[ch].len_hi, PWM[ch].len_lo);

  return 0;
};//()

int8_t pwmSoftChanSetFreq(uint8_t ch, uint16_t newFrequency) {

	if (ch >= PWM_MAX_SOFT_CHANNELS)  { return -2; };
	if (newFrequency > PWM_MAX_FREQ) { newFrequency = PWM_MAX_FREQ; return 2; };

	PWM[ch].freq = newFrequency;
	return pwmSoftChanSetVal(ch, 0);
};//()


/*
 * softPwmCreate:
 *	Create a new softPWM thread.
 *********************************************************************************
 */

int8_t pwmSoftChanStart(uint8_t pin, uint16_t pwmFreqHZ)
{
	int res;
	pthread_t myThread;
	int8_t ch = getPwmSoftChanByPin(pin);

	//first, check if this pin is alreay used
	if (ch >= 0) { return -1; }; // already exist

	//first, check if this pin is alreay used
	for (ch=0; ch < PWM_MAX_SOFT_CHANNELS; ++ch) {
		if (PWM[ch].thread == 0) { pwmPinMap[ch] = pin; break; }; // found available spot
	};//for(i)
	if (ch >= PWM_MAX_SOFT_CHANNELS) { return -2; }; // all sofw pwm channels are busy

	sunxi_set_gpio_mode(pin, OUTPUT);
	sunxi_digitalWrite (pin, LOW);

	PWM[ch].freq  = pwmFreqHZ;
	PWM[ch].thread = 1;
	pwmSoftChanSetVal(ch,0);
	//PWM[ch].len_hi= 0;
	//PWM[ch].len_lo= 100;

	NewPwmChan = ch;
	if ((res = pthread_create (&myThread, NULL, SoftPwmThread, NULL)) != 0) { return -3; }; // error creatig thread
	PWM[ch].thread = myThread;


	//res has to be 0 here
	for (; (NewPwmChan == -1) && (res < PWM_THREAD_ACTIVATION_TIMOUT); ++res) { delayMicroseconds(1000); };

	if (res >= PWM_THREAD_ACTIVATION_TIMOUT) {
		pwmSoftChanStop(ch);
		return -4; // timeout pwm thread activation [10ms]
	};//if()

	//usleep(1000);

  return ch;
};//SoftPwmStart()


/*
 * softPwmStop:
 *	Stop an existing softPWM thread
 *********************************************************************************
 */

int8_t pwmSoftChanStop(uint8_t ch) {

	if (ch >= PWM_MAX_SOFT_CHANNELS) return -1;

	if (PWM[ch].thread != 0) {
		//printf("\t cancel...\r\n");
		pthread_cancel(PWM[ch].thread);
		//printf("\t join...\r\n");
		//printf("\t Pars: %dHz, %dms HI, %dms LO, 0x%X\r\n", PWM[ch].freq, PWM[ch].len_hi, PWM[ch].len_lo, PWM[ch].thread);
		pthread_join  (PWM[ch].thread, NULL);
		PWM[ch].freq   = 0;
		PWM[ch].len_hi = 0;
		PWM[ch].len_lo = 100;
		PWM[ch].thread = 0;
		sunxi_digitalWrite (pwmPinMap[ch], LOW);
		pwmPinMap[ch] = -1;
	} else {
		return -2; // thread was already destroyed or wasn't created, or something else went wrong
	};//else

  return 0;
};//SoftPwmStopChan()

int8_t SoftPwmStopOnPin(uint16_t pin) {
  int8_t ch = getPwmSoftChanByPin(pin);

	if (ch < 0) { return -2; } // PWM channel not found
	else { return pwmSoftChanStop(ch); };

};//softPwmStop()






