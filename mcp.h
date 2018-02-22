#ifndef MCP_H
#define MCP_H

const char *mcp_msg_prefix="[MCP]";

#define _POSIX_THREADS

#define PIN_FAN_ON				198	// PG06 // EXP0	   /!\	CHANGE TO 199 // PG07 // EXP0
#define PIN_SERVO1_CTL			6	// PA06 // EXP1
#define PIN_USART1_TX			198	// PG06 // EXP1
#define PIN_SERVO2_CTL			-
#define PIN_SERVO3_CTL			-
#define PIN_LIGHTS_ON			12	// PA12 // EXP2
#define PIN_STATUS_LED			11	// PA11 // EXP3
#define PIN_PWM_FWD				0	// PA00 // EXP4
#define PIN_PWM_REV				1	// PA01 // EXP5

#define T_POS 7
#define L_POS 6
#define W_POS 5
#define A_POS 4
#define S_POS 3
#define D_POS 2

#define UP_POS 0
#define LT_POS 1
#define DN_POS 2
#define RT_POS 3
#define HM_POS 4

#define MOTOR_POWER_LIMIT		(1.0)
#define SERVO0_NEYTRAL_VAL		1575
#define SERVO0_LEFT_VAL_MAX		(SERVO0_NEYTRAL_VAL-300)
#define SERVO0_RIGHT_VAL_MAX	(SERVO0_NEYTRAL_VAL+300)

#define CTRL_LOOP_WD_TIMEOUT	500	//[ms]
#define CTRL_LOOP_WD_TH_PRIO	90

#define CONRTOL_PORT			5001 // 11999
#define GST_PORT				5001
#define GST_START_COMMAND_BASE	"gst-launch-1.0 -v v4l2src device=/dev/video0 ! \"image/jpeg,width=800, height=600,framerate=30/1\" ! rtpjpegpay !" // udpsink host=XXX.XXX.XXX.XXX port=XXXX

#endif // MCP_H




