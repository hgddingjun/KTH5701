#ifndef __KTH5701_H__
#define __KTH5701_H__

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>


#define KTH5701_I2C_NAME "kth5701"

#define KTH5701_DRIVER_VERSION "V1.0.0"

#define AW_I2C_RETRIES 3
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

#define REG_CHIPID 0x0d
#define KTH5701_CHIPID 0x0203

//读写寄存器
#define		READ_REGISTER  0x50
#define		WRITE_REGISTER  0x60

#define KTH5701_ON        1
#define KTH5701_OFF       0

#define GPIO_STATE_HIG    1
#define GPIO_STATE_LOW    0

struct kth5701 {
	struct i2c_client *i2c;
	struct device *dev;
	int enable_gpio;
	int hall_eint;
	int hall_eint_gpio;
	unsigned long halleint_swdebounce;
	unsigned long halleint_hwdebounce;
	struct delayed_work halleint_delaywork;
	int i2csck_gpio;
	int i2csda_gpio;
	//struct task_struct *thread;
};

extern struct kth5701* gp_Kth5701;

enum hall_state {
	LOW_LEVEL,
	HIG_LEVEL
};

typedef enum {
	WAKEUP_SLEEP_MODE          = 0,
	CONTINUOUS_SENSING_MODE,
	SINGLE_CONVERSION_MODE,
	IDLE_MODE,
	RESET_MODE,
	WORK_MODE_MAX
} KTH5701_WORK_MODE;

#ifdef _GPIO_IIC_
void KTH5701_CLK_HIG(void);

void KTH5701_CLK_LOW(void);

void KTH5701_SDA_HIG(void);

void KTH5701_SDA_LOW(void);

int KTH5701_SDA_RD(void);

void KTH5701_IIC_In(void);

void KTH5701_IIC_Out(void);
#endif

#endif /* __KTH5701_H__ */