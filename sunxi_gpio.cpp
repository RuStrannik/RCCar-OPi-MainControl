#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <pthread.h>
#include "sunxi_gpio.h"

static int pwmmode=0;
/*
static int BP_PIN_MASK[12][32] = { //[BANK]  [INDEX]
	{ 0, 1, 2, 3,-1,-1, 6, 7,-1,-1,10,11,12,13,14,15,16,17,18,19,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
	{ 0, 1, 2, 3, 4,-1,-1, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
	{-1,-1,-1,-1,-1,-1, 6, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PJ
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PK
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PL
};
*/
//int  mem_fd;
//void *gpio_map;
static volatile uint32_t *gpio;
//static volatile uint32_t *pwm;

void delayMicrosecondsServo (uint32_t len_ms) {
	timespec deadline;
	deadline.tv_sec = len_ms / 1000000;
	deadline.tv_nsec = (len_ms % 1000000) * 1000L;
	clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
//	nanosleep(&deadline, NULL);
};//delayMicrosecondsServo()

void delayMicroseconds (uint32_t howLong) {
	if (howLong  > 100) {
		uint32_t uSecs = howLong % 1000000;
		struct timespec sleeper;
		sleeper.tv_sec  = howLong / 1000000;
		sleeper.tv_nsec = (long)uSecs * 1000L;
		nanosleep (&sleeper, NULL);
	};//if()
};//delayMicroseconds()
void setup_io_access(void) {
  int mem_fd;
	// open /dev/mem
	if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0) {
		printf("ERROR: Could not open /dev/mem\r\n");
		exit(-1);
	}

	// GPIO
	gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE_BP);
	if ((int32_t)gpio == -1) {
		printf("ERROR: Setup: mmap (GPIO) failed\r\n");
		exit(-1);
	};
/*
	// PWM
	pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_PWM_BP);
	if ((int32_t)pwm == -1) {
		printf("ERROR: Setup: mmap (PWM) failed\r\n");
		exit(-1);
	};
*/
//	gpio_map = mmap(
//				NULL,             //Any adddress in our space will do
//				BLOCK_SIZE,       //Map length
//				PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
//				MAP_SHARED,       //Shared with other processes
//				mem_fd,           //File to map
//				GPIO_BASE   //Offset to GPIO peripheral
//				);

	close(mem_fd); //No need to keep mem_fd open after mmap

//	if (gpio_map == MAP_FAILED) {
//		printf("mmap error %d\n", (int32_t)gpio_map);//errno also set!
//		exit(-1);
//	}

	// Always use volatile pointer!
//	gpio = (volatile unsigned *)gpio_map;

};//setup_io()
uint32_t readl(uint32_t addr) {
	uint32_t val = 0;
	uint32_t mmap_base = (addr & ~MAP_MASK);
	uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	val = *(gpio + mmap_seek);
	return val;
};//readl()
void writel(uint32_t val, uint32_t addr) {
	uint32_t mmap_base = (addr & ~MAP_MASK);
	uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	*(gpio + mmap_seek) = val;
};//writel()
//pwm for BananaPro only for pwm1
void sunxi_pwm_CH0_enable(uint8_t en) {
	int val = 0;
	val = readl(SUNXI_PWM_CTRL_REG);
	if(en)
	{
		val |= (SUNXI_PWM_CH0_EN | SUNXI_PWM_SCLK_CH0_GATING);
	}
	else
	{
		val &= ~(SUNXI_PWM_CH0_EN | SUNXI_PWM_SCLK_CH0_GATING);
	}
	writel(val, SUNXI_PWM_CTRL_REG);
	usleep(200);
}
void sunxi_pwm_CH1_enable(uint8_t en) {
	int val = 0;
	val = readl(SUNXI_PWM_CTRL_REG);
	if(en)
	{
		val |= (SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
	}
	else
	{
		val &= ~(SUNXI_PWM_CH1_EN | SUNXI_PWM_SCLK_CH1_GATING);
	}
	writel(val, SUNXI_PWM_CTRL_REG);
	usleep(200);
}
void sunxi_pwm_CH0_set_mode(uint8_t mode) {
	int val = 0;
	val = readl(SUNXI_PWM_CTRL_REG);
	//mode &= 1; //cover the mode to 0 or 1
	if(mode)
	{ //pulse mode
		val |= ( SUNXI_PWM_CH0_MS_MODE|SUNXI_PWM_CH0_PUL_START);
		pwmmode=1;
	}
	else
	{  //cycle mode
		val &= ~( SUNXI_PWM_CH0_MS_MODE);
		pwmmode=0;
	}
	val |= ( SUNXI_PWM_CH0_ACT_STA);
	writel(val, SUNXI_PWM_CTRL_REG);
	usleep(200);
}
void sunxi_pwm_CH1_set_mode(uint8_t mode) {
	int val = 0;
	val = readl(SUNXI_PWM_CTRL_REG);
	//mode &= 1; //cover the mode to 0 or 1
	if(mode)
	{ //pulse mode
		val |= ( SUNXI_PWM_CH1_MS_MODE|SUNXI_PWM_CH1_PUL_START);
		pwmmode=1;
	}
	else
	{  //cycle mode
		val &= ~( SUNXI_PWM_CH1_MS_MODE);
		pwmmode=0;
	}
	val |= ( SUNXI_PWM_CH1_ACT_STA);
	writel(val, SUNXI_PWM_CTRL_REG);
	usleep(200);
}
void sunxi_pwm_set_clk(int clk) {
	int val = 0;

	// sunxi_pwm_set_enable(0);
	val = readl(SUNXI_PWM_CTRL_REG);
	//clear clk to 0
	val &= 0xf801f0;
	val |= ((clk & 0xf) << 15);  //todo check wether clk is invalid or not
	writel(val, SUNXI_PWM_CTRL_REG);
	//sunxi_pwm_set_enable(1);
	usleep(200);
}
// ch0 and ch1 set the same,16 bit period and 16 bit act
uint16_t sunxi_pwm_get_period(uint32_t channel_reg) {
	uint32_t period_cys = 0;
	period_cys = readl(channel_reg);//get ch1 period_cys
	//period_cys &= 0xffff0000;//get period_cys
	period_cys = period_cys >> 16;
	//sleep (1);
	return period_cys;
}
uint16_t sunxi_pwm_get_act(uint32_t channel_reg) {
	uint32_t period_act = 0;
	period_act = readl(channel_reg);//get ch1 period_cys //SUNXI_PWM_CH1_PERIOD
	//period_act &= 0xffff;//get period_act
	//sleep (1);
	return period_act;
}
void sunxi_pwm_set_period(int period_cys,uint32_t channel_reg) {
	uint32_t val = 0;
	//all clear to 0
	period_cys &= 0xffff; //set max period to 2^16
	period_cys = period_cys << 16;
	val = readl(channel_reg);
	val &=0x0000ffff;
	period_cys |= val;
	writel(period_cys, channel_reg); //SUNXI_PWM_CH1_PERIOD
	usleep(200);
}
void sunxi_pwm_set_act(int act_cys,uint32_t channel_reg) {
	uint32_t per0 = 0;
	//keep period the same, clear act_cys to 0 first
	per0 = readl(channel_reg);
	per0 &= 0xffff0000;
	act_cys &= 0xffff;
	act_cys |= per0;
	writel(act_cys,channel_reg);
	usleep(200);
}
void sunxi_set_gpio_mode(int pin,int mode) {
  uint32_t regval = 0;
  uint32_t bank = pin >> 5;
  uint32_t index = pin - (bank << 5);
  uint32_t offset = ((index - ((index >> 3) << 3)) << 2);
  uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
	//printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , mode,bank,index,phyaddr);
	//if(BP_PIN_MASK[bank][index] != -1) {
	regval = readl(phyaddr);
	//printf("read reg val: 0x%x offset:%d\n",regval,offset);
	if(INPUT == mode) {
		regval &= ~(7 << offset);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
		//printf("Input mode set over reg val: 0x%x\n",regval);
	} else if(OUTPUT == mode) {
		regval &= ~(7 << offset);
		regval |=  (1 << offset);
		//printf("Out mode ready set val: 0x%x\n",regval);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
		//printf("Out mode set over reg val: 0x%x\n",regval);
//	} else if(PWM_OUTPUT == mode) {
//		// set pin PWMx to pwm mode
//		regval &= ~(7 << offset);
//		regval |=  (0x2 << offset);
//		//0b010 0x2 = ALT5
//		//0b011 0x3 = ALT4
//		//0b100 0x4 = ALT0
//		//0b101 0x5 = ALT1
//		//0b110 0x6 = ALT2
//		//0b111 0x7 = ALT3

//		writel(regval, phyaddr);
//		usleep (200);
//		regval = readl(phyaddr);
//		//clear all reg
//		writel(0,SUNXI_PWM_CTRL_REG);
//		writel(0,SUNXI_PWM_CH0_PERIOD);
//		writel(0,SUNXI_PWM_CH1_PERIOD);

//		//set default M:S to 1/2
//		sunxi_pwm_CH0_set_mode(PWM_MODE_MS);
//		sunxi_pwm_set_period(1023,SUNXI_PWM_CH0_PERIOD);
//		sunxi_pwm_set_act(512,SUNXI_PWM_CH0_PERIOD);
//		usleep (200);

//		sunxi_pwm_CH1_set_mode(PWM_MODE_MS);
//		sunxi_pwm_set_period(4095,SUNXI_PWM_CH1_PERIOD);
//		sunxi_pwm_set_act(3072,SUNXI_PWM_CH1_PERIOD);

//		sunxi_pwm_set_clk(PWM_CLK_DIV_120);//default clk:24M/120
//		sunxi_pwm_CH0_enable(1);
//		sunxi_pwm_CH1_enable(1);
//		usleep (200);
	} else {
		regval &= ~(7 << offset);
		regval |=  (mode << offset);
		//printf("ready set val: 0x%x\n", regval);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
		//printf("set over reg val: 0x%x\n", regval);
	};
//	} else {
//		printf("line:%dpin number error\n",__LINE__);
//	};
};//sunxi_set_gpio_mode()
/*
void sunxi_digitalWrite(int pin, int value) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	//printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr);
	if(BP_PIN_MASK[bank][index] != -1) {
		regval = readl(phyaddr);
		//printf("befor write reg val: 0x%x,index:%d\n",regval,index);
		if(0 == value) {
			regval &= ~(1 << index);
			writel(regval, phyaddr);
			regval = readl(phyaddr);
			//printf("LOW val set over reg val: 0x%x\n",regval);
		} else {
			regval |= (1 << index);
			writel(regval, phyaddr);
			regval = readl(phyaddr);
			//printf("HIGH val set over reg val: 0x%x\n",regval);
		};
	} else {
		printf("pin number error\n");
	};

  return;
};//sunxi_digitalWrite()
int sunxi_digitalRead(int pin) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	//printf("func:%s pin:%d,bank:%d index:%d phyaddr:0x%x\n",__func__, pin,bank,index,phyaddr);
	if(BP_PIN_MASK[bank][index] != -1) {
		regval = readl(phyaddr);
		regval = regval >> index;
		regval &= 1;
		//printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n",regval,bank,index,__LINE__);
		return regval;
	} else {
		printf("Sunxi_digitalRead() pin - number error\n");
		return regval;
	};
};//sunxi_digitalRead()
*/
void sunxi_digitalWrite(int pin, int value) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	//printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr);
	regval = readl(phyaddr);
	//printf("befor write reg val: 0x%x,index:%d\n",regval,index);
	if(value == 0) {
		regval &= ~(1 << index);
		writel(regval, phyaddr);
		//regval = readl(phyaddr);
		//printf("LOW val set over reg val: 0x%x\n",regval);
	} else {
		regval |= (1 << index);
		writel(regval, phyaddr);
		//regval = readl(phyaddr);
		//printf("HIGH val set over reg val: 0x%x\n",regval);
	};
};//sunxi_digitalWrite()
int sunxi_digitalRead(int pin) {
	uint32_t regval = 0;
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg
	//printf("func:%s pin:%d,bank:%d index:%d phyaddr:0x%x\n",__func__, pin,bank,index,phyaddr);
	regval = readl(phyaddr);
	//regval = regval >> index;
	//regval &= 1<<index;
	//printf("***** read reg val: 0x%x,bank:%d,index:%d,line:%d\n",regval,bank,index,__LINE__);
	return ((regval & (1<<index))?(1):(0));
};//sunxi_digitalRead()
