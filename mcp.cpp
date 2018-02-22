#include "mcp.h"
#include "sunxi_gpio.h"
#include "softPwm.h"
#include "softServo.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>

//#include <termios.h>		//Used for UART
//#include <asm/termios.h>		//Used for UART
#include <sys/ioctl.h>
#include <asm/termbits.h>		//Used for UART

#define _BSD_SOURCE
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include "TCPServer.h"
//TCPServer tcp;

const char *version="0.0.1";
volatile static uint8_t ctrl_loop_upd=0;

int8_t SERVO_CH[SERVO_MAX_SOFT_CHANNELS];
int8_t PWM_CH[PWM_MAX_SOFT_CHANNELS];
//===========================================================
void control_loop(int newSocket, char *buffer, char *user_ip);
static void *control_loop_WD(void *par);
static void *gstreamer_thread(void *par);
void mcp_demo(void);
uint8_t pwm_test1(void);
uint8_t servo_test1(void);
uint8_t servo_test2(void);
void Init_GPIO(void);
void Init_Ctrl_Server(void);
uint8_t Init_PWMs(void);
uint8_t Init_SERVOS(void);
uint8_t DeInit_PWMs(void);
uint8_t DeInit_SERVOS(void);
void uart4servo(void);
//===========================================================
// cd MCP; gst-launch-1.0 -v v4l2src device=/dev/video0 ! "image/jpeg,width=800, height=600,framerate=30/1" ! rtpjpegpay ! udpsink host=192.168.1.3 port=5001 & ./mcp -r & echo 240000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq
// cd MCP; ./mcp -r &
// start /B d:\gstreamer\1.0\x86\bin\gst-launch-1.0 -e -v udpsrc port=5001 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec !  autovideosink
// e:\x86\Windows\WinsocTCP\build-WinsocTCP-Desktop-Debug\qtc_Desktop_Debug\install-root\client.exe 192.168.87.104

int main(int argc, char **argv) {

	printf("%s ===== Main Control Program =====\r\n", mcp_msg_prefix);
	printf("%s Configuring GPIO Direct Access...", mcp_msg_prefix);
	setup_io_access();
	printf("\t\t[OK]\r\n");
	sunxi_digitalWrite(PIN_STATUS_LED, 1);

//	printf("%s Configuring TCP/IP CTRL Server...", mcp_msg_prefix);
//	printf("\t\t[OK]\r\n");

	if (argc > 1) {
		printf("%s Startup args:\r\n", mcp_msg_prefix);
		for (uint8_t i = 1; i < argc; ++i) {
			printf("\t%s:\t", argv[i]);
			if		  (strncmp("-i", argv[i], 2) == 0) { // Init
				printf("Init mode\r\n");
				Init_GPIO();
			} else if (strncmp("-st",argv[i], 3) == 0) { // ServoTest
				//Init_GPIO();
				printf("uart4servo test mode\r\n");
				uart4servo();
			} else if (strncmp("-h", argv[i], 2) == 0) { // Help
				printf("Help\r\n");
				goto hlp;
			} else if (strncmp("-d", argv[i], 2) == 0) { // Demo
				printf("Demo mode\r\n");
				mcp_demo();
			} else if (strncmp("-r", argv[i], 2) == 0) { // Remote
				Init_GPIO();
				printf("Remote control mode\r\n");
				Init_Ctrl_Server();
			} else if (strncmp("-v", argv[i], 2) == 0) { // Version
				printf("Version: %s\r\n", version);
			} else if (strncmp("-f", argv[i], 2) == 0) { // Fan
				printf("Fan switch: ");
				sunxi_set_gpio_mode(PIN_FAN_ON,OUTPUT);
				if (argv[++i][0] == '1') {
					printf("ON");
					sunxi_digitalWrite(PIN_FAN_ON, 1);
				} else {
					printf("OFF");
					sunxi_digitalWrite(PIN_FAN_ON, 0);
				};//if()
				printf("\r\n");
			} else if (strncmp("-l", argv[i], 2) == 0) { // Lights
				printf("Lights switch: ");
				sunxi_set_gpio_mode(PIN_LIGHTS_ON,OUTPUT);
				if (argv[++i][0] == '1') {
					printf("ON");
					sunxi_digitalWrite(PIN_LIGHTS_ON, 1);
				} else {
					printf("OFF");
					sunxi_digitalWrite(PIN_LIGHTS_ON, 0);
				};//if()
				printf("\r\n");
			} else {
				printf("Unrecognized option\r\n");
			};//else if()
		};//for(i)
	} else {
hlp:	printf("%s Usage:\r\n"
			   "\t -h: Help\r\n"
			   "\t -v: Version\r\n"
			   "\t -i: Init GPIO\r\n"
			   "\t -d: Demo mode\r\n"
			   "\t -r: Remote control mode\r\n"
			   "\t -q: Quiet mode (not implemented)\r\n"
			   "\t -f [1|0]: Fan ON/OFF\r\n"
			   "\t -l [1|0]: Lights NO/OFF\r\n"
		, mcp_msg_prefix);
	};//if(argc)

	printf("%s ========== Program End =========\r\n", mcp_msg_prefix);
	sunxi_digitalWrite(PIN_STATUS_LED, 0);
  return 0;
};//main()
//===========================================================
void Init_Ctrl_Server(void) {
  int welcomeSocket, newSocket=1;
  char buffer[1024]={0};
  struct sockaddr_in serverAddr;
  struct sockaddr_in serverStorage;
  socklen_t addr_size;
  const char* correct_client_resp="Remote Control Client";
//	const char* user_ip="000.000.000.000";

	//---- Configure settings of the server address struct ----
	serverAddr.sin_family = AF_INET; // Address family = Internet
	serverAddr.sin_port = htons(CONRTOL_PORT); // Set port number, using htons function to use proper byte order
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Set IP address to localhost //inet_addr("127.0.0.1");
	memset(serverAddr.sin_zero, 0x00, sizeof serverAddr.sin_zero); // Set all bits of the padding field to 0

	// Create the socket. The three arguments are: Internet domain, Stream socket, Default protocol (TCP in this case)
	if ((welcomeSocket = socket(PF_INET, SOCK_STREAM, 0)) < 0) { printf("ERROR opening socket"); return; };
	if (setsockopt(welcomeSocket, SOL_SOCKET, SO_REUSEADDR, &newSocket, sizeof(newSocket)) < 0) { printf("setsockopt(SO_REUSEADDR) failed!\r\n"); return; };

	// Bind the address struct to the socket
	bind(welcomeSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));

	// Listen on the socket, with 1 max connection requests queued
	for (;;) {
		if(listen(welcomeSocket, 1) == 0) { printf("%s Waiting for USER Connection...\r\n", mcp_msg_prefix); }
		else { printf("Error listening!\n"); return; };

		//---- Accept call creates a new socket for the incoming connection ----
		addr_size = sizeof(serverStorage);
		newSocket = accept(welcomeSocket, (struct sockaddr *) &serverStorage, &addr_size);

		printf("%s USER connected! IP: %s:%d\n", mcp_msg_prefix, inet_ntoa(serverStorage.sin_addr), (int) ntohs(serverStorage.sin_port));

		//---- Send message to the socket of the incoming connection ----
		int n=0;
		n=recv(newSocket,buffer,1024,0);
		if (n>0) printf("%s Client sent: [%s] (%d Bytes)\r\n", mcp_msg_prefix, buffer, n);
		if (strncmp(buffer, correct_client_resp, strlen(correct_client_resp)) == 0) {
			strcpy(buffer,"Remote Control Server v1");
			send(newSocket,buffer,strlen(buffer),0);
			Init_SERVOS();
			Init_PWMs();
			control_loop(newSocket, buffer, inet_ntoa(serverStorage.sin_addr));
			DeInit_SERVOS();
			DeInit_PWMs();
		} else { printf("Incorrect Client Response!\r\n"); }

		printf("%s Connection closed.\r\n", mcp_msg_prefix);

		printf("%s Stopping video stream...", mcp_msg_prefix);
		if (system("pkill gst-launch-1.0") < 0) { printf("Error!\r\n"); } else { printf("\t\t\t[OK]\r\n"); };
		close(newSocket);
	};//for() inf loop
	close(welcomeSocket);

};//Init_Ctrl_Server()
void Init_GPIO(void) {
	sunxi_set_gpio_mode(PIN_PWM_FWD,OUTPUT);
	sunxi_digitalWrite(PIN_PWM_FWD, LOW);

	sunxi_set_gpio_mode(PIN_PWM_REV,OUTPUT);
	sunxi_digitalWrite(PIN_PWM_REV, LOW);

	sunxi_set_gpio_mode(PIN_SERVO1_CTL,OUTPUT);
	sunxi_digitalWrite(PIN_SERVO1_CTL, LOW);

	sunxi_set_gpio_mode(PIN_PWM_REV,OUTPUT);
	sunxi_digitalWrite(PIN_PWM_REV, LOW);

	sunxi_set_gpio_mode(PIN_FAN_ON,OUTPUT);
	sunxi_digitalWrite(PIN_FAN_ON, LOW);

	sunxi_set_gpio_mode(PIN_LIGHTS_ON,OUTPUT);
	sunxi_digitalWrite(PIN_LIGHTS_ON, LOW);

	sunxi_set_gpio_mode(PIN_STATUS_LED,OUTPUT);
	sunxi_digitalWrite(PIN_STATUS_LED, LOW);
};//Init_GPIO()
uint8_t servo_test1(void) {
#define max_rep 400
  uint16_t i;
  uint16_t pulse_len = 1000; //[uS]

	sunxi_set_gpio_mode(PIN_SERVO1_CTL,OUTPUT);

	for (i=0; i<=max_rep; i++) {
		//printf("Stage %d...\r\n",rep);
		sunxi_digitalWrite(PIN_SERVO1_CTL, HIGH);
		//printf("PIN%d: %d\r\n",TEST_PIN,sunxi_digitalRead(TEST_PIN));
		if		(i == max_rep*1/4 || i == max_rep*2/4)	{ pulse_len += 500; }
		else if (i == max_rep*3/4)				{ pulse_len = 1500; };
		usleep(pulse_len);
		sunxi_digitalWrite(PIN_SERVO1_CTL, LOW);
		//printf("PIN%d: %d\r\n",TEST_PIN,sunxi_digitalRead(TEST_PIN));
		usleep(20000-pulse_len);
	};//for(rep)
  return 0;
};//servo_test1()
uint8_t pwm_test1(void) {

	Init_PWMs();

	printf("PWM Test 1...\r\n");
	//sunxi_digitalWrite(PIN_PWM_FWD,0);
	printf("\t -> 10%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 100);	// 10%
	sleep(3);

	printf("\t -> 30%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 300);	// 30%
	sleep(3);

	printf("\t -> 60%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 600);	// 60%
	sleep(3);

	printf("\t ->100%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 1000);	// 100%
	sleep(3);

	printf("\t ->  0%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 0);		// OFF
	sleep(1);

	pwmSoftChanSetPin(PWM_CH[0], PIN_PWM_REV);
	//sunxi_digitalWrite(PIN_PWM_FWD,0);

	printf("\t<-  90%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 100);	// 10%
	sleep(3);

	printf("\t<-  60%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 300);	// 40%
	sleep(3);

	printf("\t<-  30%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 600);	// 70%
	sleep(3);

	printf("\t<-  10%%\r\n");
	pwmSoftChanSetVal(PWM_CH[0], 1000);	// 90%
	sleep(3);

	printf("\t ->  0%%\r\n");
	//sunxi_digitalWrite(PIN_PWM_FWD, 0);
	pwmSoftChanSetVal(PWM_CH[0], 0);		// OFF
	sleep(1);

	printf("DONE\r\n");

	printf("%s Stopping PWM CTRL processes...", mcp_msg_prefix);
	pwmSoftChanStop(PWM_CH[0]);
	printf("\t\t[OK]\r\n");

  return 0;
};//pwm_test1()
uint8_t servo_test2(void) {
	Init_SERVOS();
	printf("SERVO Test 2...\r\n");
	servoSoftChanSetPulse(SERVO_CH[0], 1200);
	sleep(3);
	servoSoftChanSetPulse(SERVO_CH[0], 1500);
	sleep(3);
	servoSoftChanSetPulse(SERVO_CH[0], 1800);
	sleep(3);
	servoSoftChanSetPulse(SERVO_CH[0], 1500);
	sleep(1);
	printf("DONE\r\n");
	printf("%s Stopping Servo CTRL processes...", mcp_msg_prefix);
	servoSoftChanStop(SERVO_CH[0]);
	printf("\t\t[OK]\r\n");
  return 0;
};//servo_test2()
void mcp_demo(void) {
	printf("Lights and fan...\r\n");
	sunxi_digitalWrite(PIN_FAN_ON, 1);
	sunxi_digitalWrite(PIN_LIGHTS_ON, 1);
	sleep(3);
	sunxi_digitalWrite(PIN_FAN_ON, 0);
	sunxi_digitalWrite(PIN_LIGHTS_ON, 0);
//	servo_test1();
	Init_GPIO();
	servo_test2();
	pwm_test1();

};//mcp_demo()
void control_loop(int newSocket, char *buffer, char *user_ip) {
  uint8_t n;
  uint8_t car_ctrl_byte=0;
  uint8_t car_ctrl_curr_state=0;
  uint8_t cam_ctrl_byte=0;
  uint8_t cam_ctrl_curr_state=0;
  pthread_t watchDogThreadfd,gstThreadfd;
//  char ctrl_str[8]={0};
	printf("\r\nControl func started\r\n");
	printf("%s Starting video stream to IP: %s:%d\r\n", mcp_msg_prefix, user_ip, GST_PORT);
//	if (pthread_create(&gstThreadfd, NULL, gstreamer_thread, (void *)"nik.noip.me") != 0) {
	if (pthread_create(&gstThreadfd, NULL, gstreamer_thread, (void *) user_ip) != 0) {
		printf("Error creatig GStreamer Thread!\r\n");
		return;
	};//if()

	printf("%s Limitting processor max freq to 240MHz...", mcp_msg_prefix);
	if (system("echo 240000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq") < 0) { printf("\tERROR!\r\n"); } else { printf("\t[OK]\r\n"); };

	ctrl_loop_upd = 1;
	if (pthread_create(&watchDogThreadfd, NULL, control_loop_WD, (void *) &car_ctrl_curr_state) != 0) {
		printf("Error creatig WatchDog Thread!\r\n");
		return;
	};//if()
	//strncpy(ctrl_str,"--------",8);
	for (;;) {
//		if (car_ctrl_byte&(1<<W_POS)) { ctrl_str[7-W_POS] = 'W'; } else { ctrl_str[7-W_POS] = '-'; };
//		if (car_ctrl_byte&(1<<A_POS)) { ctrl_str[7-A_POS] = 'A'; } else { ctrl_str[7-A_POS] = '-'; };
//		if (car_ctrl_byte&(1<<S_POS)) { ctrl_str[7-S_POS] = 'S'; } else { ctrl_str[7-S_POS] = '-'; };
//		if (car_ctrl_byte&(1<<D_POS)) { ctrl_str[7-D_POS] = 'D'; } else { ctrl_str[7-D_POS] = '-'; };
//		if (car_ctrl_byte&(1<<L_POS)) { ctrl_str[7-L_POS] = 'L'; } else { ctrl_str[7-L_POS] = '-'; };

		if (car_ctrl_byte&(1<<L_POS)) {
			if ((car_ctrl_curr_state&(1<<L_POS)) == 0) { sunxi_digitalWrite(PIN_LIGHTS_ON,1); car_ctrl_curr_state |= 1<<L_POS; };
		} else {
			if ((car_ctrl_curr_state&(1<<L_POS)) != 0) { sunxi_digitalWrite(PIN_LIGHTS_ON,0); car_ctrl_curr_state &= ~(1<<L_POS);};
		};//else()

		n = (1<<W_POS)|(1<<S_POS);
		if (((car_ctrl_byte & n) == n) || ((car_ctrl_byte & n) == 0)) {
			pwmSoftChanSetVal(PWM_CH[0],0);
		} else if (car_ctrl_byte&(1<<W_POS)) {
			pwmSoftChanSetPin(PWM_CH[0],PIN_PWM_FWD);
			//pwmSoftChanSetVal(PWM_CH[0],PWM_MAX_VAL*MOTOR_POWER_LIMIT);
			pwmSoftChanSetVal(PWM_CH[0], ((PWM_MAX_VAL*2/10) + ((car_ctrl_byte&0x03)*(PWM_MAX_VAL*8/10)/3)));
		} else if (car_ctrl_byte&(1<<S_POS)) {
			pwmSoftChanSetPin(PWM_CH[0],PIN_PWM_REV);
			//pwmSoftChanSetVal(PWM_CH[0],PWM_MAX_VAL*MOTOR_POWER_LIMIT);
			pwmSoftChanSetVal(PWM_CH[0], ((PWM_MAX_VAL*2/10) + ((car_ctrl_byte&0x03)*(PWM_MAX_VAL*8/10)/3)));
		};//if()

		n = (1<<A_POS)|(1<<D_POS);
		if (((car_ctrl_byte & n) == n) || ((car_ctrl_byte & n) == 0)) {
			servoSoftChanSetPulse(SERVO_CH[0], SERVO0_NEYTRAL_VAL);
		} else if (car_ctrl_byte&(1<<A_POS)) {
			servoSoftChanSetPulse(SERVO_CH[0], SERVO0_LEFT_VAL_MAX);
		} else if (car_ctrl_byte&(1<<D_POS)) {
			servoSoftChanSetPulse(SERVO_CH[0], SERVO0_RIGHT_VAL_MAX);
		};//if()

		n = (1<<UP_POS)|(1<<DN_POS);
		if (((cam_ctrl_byte & n) == n) || ((cam_ctrl_byte & n) == 0)) {
		} else if (cam_ctrl_byte&(1<<UP_POS)) {
			n=system("uvcdynctrl -s \"Tilt (relative)\" -- -100 &"); // move up
		} else if (cam_ctrl_byte&(1<<DN_POS)) {
			n=system("uvcdynctrl -s \"Tilt (relative)\" -- +100 &"); // move down
		};//if()

		n = (1<<LT_POS)|(1<<RT_POS);
		if (((cam_ctrl_byte & n) == n) || ((cam_ctrl_byte & n) == 0)) {
		} else if (cam_ctrl_byte&(1<<LT_POS)) {
			n=system("uvcdynctrl -s \"Pan (relative)\" -- +200 &"); // move left
		} else if (cam_ctrl_byte&(1<<RT_POS)) {
			n=system("uvcdynctrl -s \"Pan (relative)\" -- -200 &"); // move right
		};//if()

/*
		n = (1<<UP_POS)|(1<<DN_POS);
		if (((cam_ctrl_byte & n) == n) || ((cam_ctrl_byte & n) == 0)) {
			if ((cam_ctrl_curr_state & (1<<UP_POS)) != 0) {
				n=system("uvcdynctrl -s \"Tilt (relative)\" -- +1700 &"); // move down
				cam_ctrl_curr_state &= ~(1<<UP_POS);
			} else if ((cam_ctrl_curr_state & (1<<DN_POS)) != 0) {
				n=system("uvcdynctrl -s \"Tilt (relative)\" -- -1700 &"); // move up
				cam_ctrl_curr_state &= ~(1<<DN_POS);
			};//if()
		} else if (cam_ctrl_byte&(1<<UP_POS)) {
			if ((cam_ctrl_curr_state & (1<<UP_POS)) == 0) {
				n=system("uvcdynctrl -s \"Tilt (relative)\" -- -1700 &"); // move up
				cam_ctrl_curr_state |= (1<<UP_POS);
			};//if()
		} else if (cam_ctrl_byte&(1<<DN_POS)) {
			if ((cam_ctrl_curr_state & (1<<DN_POS)) == 0) {
				n=system("uvcdynctrl -s \"Tilt (relative)\" -- +1700 &"); // move down
				cam_ctrl_curr_state |= (1<<DN_POS);
			};//if()
		};//if()

		n = (1<<LT_POS)|(1<<RT_POS);
		if (((cam_ctrl_byte & n) == n) || ((cam_ctrl_byte & n) == 0)) {
			if ((cam_ctrl_curr_state & (1<<LT_POS)) != 0) {
				n=system("uvcdynctrl -s \"Pan (relative)\" -- -11000 &"); // move right
				cam_ctrl_curr_state &= ~(1<<LT_POS);
			} else if ((cam_ctrl_curr_state & (1<<RT_POS)) != 0) {
				n=system("uvcdynctrl -s \"Pan (relative)\" -- +11000 &"); // move left
				cam_ctrl_curr_state &= ~(1<<RT_POS);
			};//if()
		} else if (cam_ctrl_byte&(1<<LT_POS)) {
			if ((cam_ctrl_curr_state & (1<<LT_POS)) == 0) {
				n=system("uvcdynctrl -s \"Pan (relative)\" -- +11000 &"); // move left
				cam_ctrl_curr_state |= (1<<LT_POS);
			};//if()
		} else if (cam_ctrl_byte&(1<<RT_POS)) {
			if ((cam_ctrl_curr_state & (1<<RT_POS)) == 0) {
				n=system("uvcdynctrl -s \"Pan (relative)\" -- -11000 &"); // move right
				cam_ctrl_curr_state |= (1<<RT_POS);
			};//if()
		};//if()
*/

		if (cam_ctrl_byte&(1<<HM_POS)) {
			if ((cam_ctrl_curr_state&(1<<HM_POS)) == 0) {
				n=system("uvcdynctrl -s \"Pan Reset\" 1 &");
				cam_ctrl_curr_state |= (1<<HM_POS);
			};//if()
		} else {
			if ((cam_ctrl_curr_state&(1<<HM_POS)) != 0) {
				n=system("uvcdynctrl -s \"Tilt Reset\" 1 &");
				cam_ctrl_curr_state &= ~(1<<HM_POS);
			};//if()
		};//else()

		//printf("%.*s\r\n", 8, ctrl_str); // print keys pressed
		n=recv(newSocket,buffer,1024,0);
		ctrl_loop_upd = 2;
		if (n == 0) { break; };
		//memcpy(buffer,ctrl_byte,2);
		car_ctrl_byte = (uint8_t) buffer[0];
		cam_ctrl_byte = (uint8_t) buffer[1];
		if (car_ctrl_byte&(1<<T_POS)) { break; }; // ESC pressed

		//printf("\b\b\b\b\b\b\b\b");
	};//for(var)

	printf("%s Stopping WatchDog thread...", mcp_msg_prefix);
	pthread_cancel(gstThreadfd);
	pthread_join  (gstThreadfd, NULL);
	pthread_cancel(watchDogThreadfd);
	pthread_join  (watchDogThreadfd, NULL);
	printf("\t\t[OK]\r\n");
	printf("\r\nControl func ended\r\n");
};//control_loop()

static void *gstreamer_thread(void *par) {
	char buf[256]={0};
	if (system("uvcdynctrl -s \"Backlight Compensation\" 2") < 0) { printf("Error!\r\n"); };
	sprintf(buf,"%s udpsink host=%s port=%d > /dev/null &", GST_START_COMMAND_BASE, (char *)par, GST_PORT);
	//printf("Launching GST with args: %s\r\n",buf);
	if (system(buf) < 0) { printf("Error!\r\n"); };
//	FILE *handle = popen(buf, "r");
//	if (handle == NULL) {
//		return NULL;
//	};//if(error)

//	size_t readn;
//	while ((readn = fread(buf, 1, sizeof(buf), handle)) > 0) {
//		fwrite(buf, 1, readn, stdout);
//	};//while()
	//sleep(15);

//	pclose(handle);
//	printf("GST Thread end\r\n");
	return NULL;
};//gstreamer_thread()

static void *control_loop_WD(void *par) {
	printf("Control Loop WatchDog process started.\r\n");
	sched_param param = {0};
	param.sched_priority = sched_get_priority_max (SCHED_RR);
	if (CTRL_LOOP_WD_TH_PRIO < param.sched_priority)	param.sched_priority = PWM_THREAD_PRIORITY;
	sched_setscheduler (0, SCHED_RR, &param);

	for (;;) {
		if (ctrl_loop_upd == 0) {
			printf("Control Loop WatchDog: Timeout Detected! GPIO Reset!\r\n");
			servoSoftChanSetPulse(SERVO_CH[0], SERVO0_NEYTRAL_VAL);
			pwmSoftChanSetVal(PWM_CH[0], 0);
			sunxi_digitalWrite(PIN_LIGHTS_ON,0);
			*((uint8_t *) par) = 0; // car_ctrl_curr_state = 0;
			//break;
		} else { ctrl_loop_upd = 0; };
		usleep(CTRL_LOOP_WD_TIMEOUT*1000);
	};//for(var)

	//DeInit_PWMs();
	//DeInit_SERVOS();
	return NULL;
	//exit(-1);
};//control_loop_WD()
void uart4servo(void) {
	int fd;
	int ret;
	char msg[2]={ 0b10000000, 0b00000000 };
	//#define msglen 2
	//memset(msg,0xFF,msglen);


//	sunxi_set_gpio_mode(PIN_USART1_TX,2); // Set pin to USART1_TX mode

	if ((fd = open("/dev/ttyUSB0", O_WRONLY | O_NOCTTY | O_NONBLOCK)) < 0) {
		printf ("Unable to open serial device!\n") ;
		return;
	}

	// 100kbps = 100bits per ms,
	// 155 ones => 19 x 0xFF + 1 x 0x07 + usleep(10)
	// 256 ones => 32 x 0xFF + usleep(10)

	// 115.2 kbps => 115 bits per ms, hence, to get 1.55ms delay, we need to send ~130 ones and then get to sleep
	// 130 ones => 16 x 0xFF + 1 x 0x07 + usleep(10)

	struct termios2 options;
	ioctl(fd, TCGETS2, &options);
	options.c_cflag &= ~CBAUD;    //Remove current BAUD rate
	options.c_cflag |= BOTHER;    //Allow custom BAUD rate using int input
	options.c_ispeed = 1000;    //Set the input BAUD rate
	options.c_ospeed = 1000;    //Set the output BAUD rate
	ioctl(fd, TCSETS2, &options);

//	struct termios options;
//	tcgetattr(fd, &options);
//	options.c_cflag = B9600 | CS8 | CLOCAL;		//<Set baud rate
//	options.c_iflag = IGNPAR;
//	options.c_oflag = 0;
//	options.c_lflag = 0;
//	tcflush(fd, TCIFLUSH);
//	tcsetattr(fd, TCSANOW, &options);

	for (int i = 0; i < 10; ++i) {
		ret=write(fd, msg, 1);
		if (ret < 0) { printf("Error writing byte\r\n"); };
		//usleep(20000);
	};//for(i)

	close(fd);

};//uart4servo()

uint8_t DeInit_PWMs(void) {
	printf("%s Stopping PWM CTRL processes...", mcp_msg_prefix);
	printf("\t\t[OK]\r\n");
	return pwmSoftChanStop(PWM_CH[0]);
};//DeInit_PWMs()
uint8_t DeInit_SERVOS(void) {
	printf("%s Stopping Servo CTRL processes...", mcp_msg_prefix);
	printf("\t\t[OK]\r\n");
	return servoSoftChanStop(SERVO_CH[0]);
};//DeInit_SERVOS()

uint8_t Init_PWMs(void) {
	printf("%s Configuring PWM CTRL processes...", mcp_msg_prefix);
	PWM_CH[0] = pwmSoftChanStart(PIN_PWM_FWD, 100);
	if (PWM_CH[0] >= 0)	{ printf("\t\t[OK]\r\n"); }
	else {  printf("\t\t[FAIL]\r\nError: %d\r\n",PWM_CH[0]); return -1; };
  return 0;
};//Init_PWMs()
uint8_t Init_SERVOS(void) {
	printf("%s Configuring Servo CTRL processes...", mcp_msg_prefix);
	SERVO_CH[0] = servoSoftChanStart(PIN_SERVO1_CTL);
	if (SERVO_CH[0] >= 0)	{ printf("\t[OK]\r\n"); }
	else {  printf("\t[FAIL]\r\nError: %d\r\n",SERVO_CH[0]); return -1; };
  return 0;
};//Init_SERVOS()

/* // Servo tests
	uint16_t pulse_len = 1000;
	for (rep=0; rep<1000; rep++) {
		//printf("Stage %d...\r\n",rep);
		sunxi_digitalWrite(TEST_PIN,1);
		//printf("PIN%d: %d\r\n",TEST_PIN,sunxi_digitalRead(TEST_PIN));
		if (rep == 300 || rep == 600) { pulse_len += 500; }
		else if (rep == 900) { pulse_len = 1500; };
		usleep(pulse_len);
		sunxi_digitalWrite(TEST_PIN,0);
		//printf("PIN%d: %d\r\n",TEST_PIN,sunxi_digitalRead(TEST_PIN));
		usleep(20000-pulse_len);
	};//for(rep)
 //PWM tests
	printf("%s Reading current PWM config:\r\n",mcp_prefix);
	printf("\tCH0 ACT   : %d\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH0_PERIOD));
	printf("\tCH0 PERIOD: %d\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH0_PERIOD));
	printf("\tCH1 ACT   : %d\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH1_PERIOD));
	printf("\tCH1 PERIOD: %d\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH1_PERIOD));
//	printf("%s Configuring PWM on PIN%d...\r\n",mcp_prefix,TEST_PIN);
	//sunxi_pwm_set_clk();
	printf("%s Setting GPIO PIN%d to PWM_OUTPUT mode...\r\n",mcp_prefix,TEST_PIN);
	sunxi_set_gpio_mode(PWRL_PIN,INPUT);

	printf("%s Reading current PWM config:\r\n",mcp_prefix);
//	printf("\tCH0 ACT   : %d (%d)\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH0_PERIOD),PWM->CH0.ENTIRE_ACT_CYC);
//	printf("\tCH0 PERIOD: %d (%d)\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH0_PERIOD),PWM->CH0.ENTIRE_CYC);
//	printf("\tCH1 ACT   : %d (%d)\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH1_PERIOD),PWM->CH1.ENTIRE_ACT_CYC);
//	printf("\tCH1 PERIOD: %d (%d)\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH1_PERIOD),PWM->CH1.ENTIRE_CYC);
	printf("\tCH0 ACT   : %d\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH0_PERIOD));
	printf("\tCH0 PERIOD: %d\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH0_PERIOD));
	printf("\tCH1 ACT   : %d\r\n",sunxi_pwm_get_act(   SUNXI_PWM_CH1_PERIOD));
	printf("\tCH1 PERIOD: %d\r\n",sunxi_pwm_get_period(SUNXI_PWM_CH1_PERIOD));
*/
