#ifndef SUNXI_GPIO_H
#define SUNXI_GPIO_H

#include <stdint.h>

#define MAX_PIN_NUM						(0x40)  //64
#define GPIO_BASE_BP					(0x01C20000)
#define SUNXI_GPIO_BASE					(0x01C20800)

#define PAGE_SIZE						(0x2000)//(4*1024)
#define BLOCK_SIZE						(0x2000)//(4*1024)

#define MAP_SIZE						BLOCK_SIZE//(0x2000)//(4096*2)
#define MAP_MASK						(MAP_SIZE - 1)

#define GPIO_PWM_BP						(0x01c20000)  //need 4k*n
#define SUNXI_PWM_BASE					(0x01c21400) //(0x01c20e00) //
#define SUNXI_PWM_CTRL_REG				(SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD			(SUNXI_PWM_BASE + 0x4)
#define SUNXI_PWM_CH1_PERIOD			(SUNXI_PWM_BASE + 0x8)

#define SUNXI_PWM_CH0_EN				(1 << 4)
#define SUNXI_PWM_CH0_ACT_STA			(1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING		(1 << 6)
#define SUNXI_PWM_CH0_MS_MODE			(1 << 7) //pulse mode
#define SUNXI_PWM_CH0_PUL_START			(1 << 8)

#define SUNXI_PWM_CH1_EN				(1 << 19)
#define SUNXI_PWM_CH1_ACT_STA			(1 << 20)
#define SUNXI_PWM_SCLK_CH1_GATING		(1 << 21)
#define SUNXI_PWM_CH1_MS_MODE			(1 << 22) //pulse mode
#define SUNXI_PWM_CH1_PUL_START			(1 << 23)

#define PWM_CLK_DIV_120					0
#define PWM_CLK_DIV_180					1
#define PWM_CLK_DIV_240					2
#define PWM_CLK_DIV_360					3
#define PWM_CLK_DIV_480					4
#define PWM_CLK_DIV_12K					8
#define PWM_CLK_DIV_24K					9
#define PWM_CLK_DIV_36K					10
#define PWM_CLK_DIV_48K					11
#define PWM_CLK_DIV_72K					12

#define	LOW								0
#define	HIGH							1

// Pull up/down/none

#define	PUD_OFF							0
#define	PUD_DOWN						1
#define	PUD_UP							2

#define PWM_MODE_MS						0
#define PWM_MODE_BAL					1

#define INPUT							0
#define OUTPUT							1
#define PWM_OUTPUT						2
#define	SOFT_PWM_OUTPUT					8
#define	SOFT_SERVO_OUTPUT				9

typedef struct {
	volatile uint16_t ENTIRE_CYC;
	volatile uint16_t ENTIRE_ACT_CYC;
} CH_PERIOD_Struct;

typedef struct {
	volatile uint32_t COMM_CTRL;
	CH_PERIOD_Struct CH0;
	CH_PERIOD_Struct CH1;
} PWM_CTRL_Struct;

//#define PWM               ((PWM_CTRL_Struct *) SUNXI_PWM_BASE)

//#define	PI_THREAD(X)	void *X (void *par)

extern int threadSetPriority (const int pri);

void setup_io_access(void);
void delayMicrosecondsServo (uint32_t len_ms);
void delayMicroseconds (uint32_t howLong);
uint32_t readl(uint32_t addr);
void	 writel(uint32_t val, uint32_t addr);
void sunxi_pwm_CH0_enable(uint8_t en);
void sunxi_pwm_CH1_enable(uint8_t en);
void sunxi_pwm_CH0_set_mode(uint8_t mode);
void sunxi_pwm_CH1_set_mode(uint8_t mode);
void sunxi_pwm_set_clk(int clk);
uint16_t sunxi_pwm_get_period(uint32_t channel_reg);
uint16_t sunxi_pwm_get_act(uint32_t channel_reg);
void sunxi_pwm_set_period(int period_cys, uint32_t channel_reg);
void sunxi_pwm_set_act(int act_cys, uint32_t channel_reg);
void sunxi_set_gpio_mode(int pin, int mode);
void sunxi_digitalWrite(int pin, int value);
int sunxi_digitalRead(int pin);

#endif // SUNXI_GPIO_H
