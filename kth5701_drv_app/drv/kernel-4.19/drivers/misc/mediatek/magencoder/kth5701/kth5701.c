/*
 * Copyright (C) 2024 DuoWei Inc.
 * Magnetic encoder driver.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 ************************************************************************
 *
 *	[Create by Dingjun 2024.12.10]
 *	[modify by Dingjun 2025.01.04]
 *
 *************************************************************************
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/math64.h>

#include "kth5701.h"

#include "KTH57XXIIC.h"

/* USER CODE BEGIN PV */
uint8_t RegisterData[3]={0};//存放读取寄存器时返回的值 RegisterData[0]：status RegisterData[1]：寄存器高八位 RegisterData[2]：寄存器低八位
uint8_t DataReadFrame[9]={0};//存放测量结束读回的数据

char buf[6] = {'s', 't', 'a', 'r', 't', 0};
float angle = 0,calib_angle = 0;

/* USER CODE BEGIN 1 */
int16_t x, y;
uint16_t xTemp,yTemp;

uint8_t get_sample = 0,ang_get_l = 0,ang_get_h = 0;
uint8_t calib_sample_flag = 0;

int16_t x_max,x_min,y_max,y_min;
/* USER CODE END 1 */

/*static*/ struct kth5701* gp_Kth5701 = NULL;

static KTH5701_WORK_MODE s_WorkMode = WAKEUP_SLEEP_MODE;

static enum hall_state s_hall_state = HIG_LEVEL;

//测量数据回读帧（data read frame）
extern uint8_t KTH57XXDataRead(uint8_t axis);

//寄存器初始化函数
extern void KTH57XXInitial(uint8_t A1, uint8_t A0);

//四种测量模式
extern uint8_t KTH57XXContinuousSensing(uint8_t axis);
extern uint8_t KTH57XXWakeupSleep(uint8_t axis);
extern uint8_t KTH57XXSingleConversion(uint8_t axis);
extern uint8_t KTH57XXIdle(void);

void KTH5701_CLK_HIG(void)
{
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csck_gpio)) {
		gpio_set_value_cansleep(gp_Kth5701->i2csck_gpio, GPIO_STATE_HIG);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
	}
#endif
}

void KTH5701_CLK_LOW(void)
{
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csck_gpio)) {
		gpio_set_value_cansleep(gp_Kth5701->i2csck_gpio, GPIO_STATE_LOW);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
	}
#endif
}

void KTH5701_SDA_HIG(void)
{
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csda_gpio)) {
		gpio_set_value_cansleep(gp_Kth5701->i2csda_gpio, GPIO_STATE_HIG);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
	}
#endif
}

void KTH5701_SDA_LOW(void)
{
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csda_gpio)) {
		gpio_set_value_cansleep(gp_Kth5701->i2csda_gpio, GPIO_STATE_LOW);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
	}
#endif
}

int KTH5701_SDA_RD(void)
{
	int ret = 0;
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csda_gpio)) {
		ret = gpio_get_value_cansleep(gp_Kth5701->i2csda_gpio);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
		ret = -1;
	}
#endif
	return ret;
}

void KTH5701_IIC_In(void)
{
#ifdef _GPIO_IIC_
	if (gp_Kth5701 && gpio_is_valid(gp_Kth5701->i2csda_gpio)) {
		gpio_direction_input(gp_Kth5701->i2csda_gpio);
	} else {
		dev_err(gp_Kth5701->dev, "%s:  failed\n", __func__);
	}
#endif
}

void KTH5701_IIC_Out(void)
{
#ifdef _GPIO_IIC_
#endif
}


static void hall_detect_eint_work_callback(struct work_struct *work)
{
	if (s_hall_state == LOW_LEVEL) {
		s_hall_state = HIG_LEVEL;
		pr_info("%s,%d-[hall eint detect HIG_LEVEL ]\n", __func__, __LINE__);

		//msleep(100);

		irq_set_irq_type(gp_Kth5701->hall_eint, IRQF_TRIGGER_HIGH);
		enable_irq(gp_Kth5701->hall_eint);

	} else {
		s_hall_state = LOW_LEVEL;
		pr_info("%s,%d-[hall eint detect LOW_LEVEL ]\n", __func__, __LINE__);

		//msleep(100);

		irq_set_irq_type(gp_Kth5701->hall_eint, IRQF_TRIGGER_LOW);
		enable_irq(gp_Kth5701->hall_eint);
	}
}


static irqreturn_t halleint_detect_eint_isr(int irq, void *data)
{

	pr_info("%s,%d-irq=%d", __func__, __LINE__, irq);
	disable_irq_nosync(irq);
	schedule_delayed_work(&gp_Kth5701->halleint_delaywork,
		msecs_to_jiffies(gp_Kth5701->halleint_swdebounce));

	return IRQ_HANDLED;
}


/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int kth5701_parse_dt(struct device *dev, struct kth5701 *kth5701,
			    struct device_node *np)
{
	int ret=0;
	u32 ints[2] = {0};

	kth5701->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (kth5701->enable_gpio < 0) {
		dev_err(dev,
			"%s: no enable gpio provided, will not HW enable device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: enable gpio provided ok\n", __func__);
	}


	kth5701->hall_eint_gpio = of_get_named_gpio(np, "hall-eint", 0);
	ret = gpio_request(kth5701->hall_eint_gpio, "hall-eint control pin");
	printk("%s,%d ret=%d\n", __func__, __LINE__, ret);
	if (ret<0)
		pr_err("hall_eint_gpio pin, failure of setting\n");


	kth5701->hall_eint = gpio_to_irq(kth5701->hall_eint_gpio);
	if (!kth5701->hall_eint)
	{
		printk("kth5701->hall_eint irq_of_parse_and_map(..) fail\n");
		ret = -EINVAL;
		return ret;
	}

	pr_info("%s,%d-request handler for hall_eint ID IRQ: %d\n",__func__, __LINE__, kth5701->hall_eint);
	ret = request_irq(kth5701->hall_eint, halleint_detect_eint_isr, IRQ_TYPE_LEVEL_LOW, "KTH5701-hall-eint", kth5701);
	if (ret) {
	    pr_err("kth5701->hall_eint irq thread request failed, ret=%d\n", ret);
	    return -3;
    }

	enable_irq_wake(kth5701->hall_eint);

	ret = of_property_read_u32_array(np, "halleint-debounce",
		ints, ARRAY_SIZE(ints));
	if (!ret)
		kth5701->halleint_hwdebounce = ints[1];

	kth5701->halleint_swdebounce = msecs_to_jiffies(10);

	INIT_DELAYED_WORK(&kth5701->halleint_delaywork, hall_detect_eint_work_callback);

#ifdef _GPIO_IIC_
	kth5701->i2csck_gpio = of_get_named_gpio(np, "i2csck-gpio", 0);
	if (kth5701->i2csck_gpio < 0) {
		dev_err(dev,
			"%s: no i2c sck gpio provided, will not HW enable device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: i2c sck gpio provided ok\n", __func__);
	}

	kth5701->i2csda_gpio = of_get_named_gpio(np, "i2csda-gpio", 0);
	if (kth5701->i2csda_gpio < 0) {
		dev_err(dev,
			"%s: no i2c sda gpio provided, will not HW enable device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: i2c sda gpio provided ok\n", __func__);
	}
#endif

	return 0;
}


/*****************************************************
 *
 * power on/off
 *
 *****************************************************/
 static int kth5701_power_onoff(struct kth5701 *kth5701, int onoff) {
	pr_info("%s: enter\n", __func__);
	if (kth5701 && gpio_is_valid(kth5701->enable_gpio)) {
		gpio_set_value_cansleep(kth5701->enable_gpio, onoff);
		msleep(1);
	} else {
		dev_err(kth5701->dev, "%s:  failed\n", __func__);
	}
	 return 0;
 }

static int kth5701_get_chip_ID(struct kth5701 *kth5701) {

	unsigned char cnt = 0;

	while (cnt++ < AW_READ_CHIPID_RETRIES) {
		KTH57XXReadRegister(REG_CHIPID);

		if ( (RegisterData[1]<<8 | RegisterData[2])  == KTH5701_CHIPID ) {
			pr_info("This Chip is  KTH5701    REG_ID: 0x%02x 0x%02x 0x%02x\n",
				RegisterData[0], RegisterData[1], RegisterData[2]);
			return 0;
		} else {
			pr_err("ERROR: RegisterData: 0X%02X, 0X%02X, 0X%02X\n", RegisterData[0], RegisterData[1], RegisterData[2]);
		}
		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}
	return -EINVAL;
}


static ssize_t kth5701_show_work_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Work mode: %d\n", s_WorkMode);
}

static ssize_t kth5701_store_work_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	s_WorkMode = (KTH5701_WORK_MODE)simple_strtol(buf, NULL, 10);
	switch( s_WorkMode )
	{
		case WAKEUP_SLEEP_MODE:
			KTH57XXWakeupSleep(0x0f);
			pr_info("WAKEUP_SLEEP_MODE\n");
			break;

		case CONTINUOUS_SENSING_MODE:
			KTH57XXContinuousSensing(0x0f);
			pr_info("CONTINUOUS_SENSING_MODE\n");
			break;

		case SINGLE_CONVERSION_MODE:
			KTH57XXSingleConversion(0x0f);
			pr_info("SINGLE_CONVERSION_MODE\n");
			break;

		case IDLE_MODE:
			KTH57XXIdle();
			pr_info("IDLE_MODE\n");
			break;

		case RESET_MODE:
			//KTH57XXReset(); /* 不要轻易使用 reset*/
			pr_info("RESET_MODE\n");
			break;

		default:
			KTH57XXWakeupSleep(0x0f);
			pr_info("default\n");
			break;
	}
    return count;
}


static DEVICE_ATTR(workmode, 0664, kth5701_show_work_mode, kth5701_store_work_mode);


static ssize_t kth5701_show_register(struct device *dev, struct device_attribute *attr, char *buf)
{
	KTH57XXDataRead(0x0F);
    return sprintf(buf, "Read Back Data: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
				DataReadFrame[0],DataReadFrame[1],DataReadFrame[2],
				DataReadFrame[3],DataReadFrame[4],DataReadFrame[5],
				DataReadFrame[6],DataReadFrame[7],DataReadFrame[8]);
}

static ssize_t kth5701_store_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}


static DEVICE_ATTR(kthReg, 0664, kth5701_show_register, kth5701_store_register);


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int kth5701_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct kth5701 *kth5701;
	struct device_node *np = i2c->dev.of_node;
	int ret;

	pr_info("%s: enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	kth5701 = devm_kzalloc(&i2c->dev, sizeof(struct kth5701), GFP_KERNEL);
	if (kth5701 == NULL)
		return -ENOMEM;

	kth5701->dev = &i2c->dev;
	kth5701->i2c = i2c;

	i2c_set_clientdata(i2c, kth5701);

	gp_Kth5701 = kth5701;

	/* kth5701 enable pin */
	if (np) {
		ret = kth5701_parse_dt(&i2c->dev, kth5701, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		kth5701->enable_gpio = -1;
		kth5701->hall_eint_gpio = -1;
	#ifdef _GPIO_IIC_
		kth5701->i2csck_gpio = -1;
		kth5701->i2csda_gpio = -1;
	#endif
	}

	if (gpio_is_valid(kth5701->enable_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, kth5701->enable_gpio,
					    GPIOF_OUT_INIT_LOW, "kth5701_enable");
		if (ret) {
			dev_err(&i2c->dev, "%s: enable pin request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}


#ifdef _GPIO_IIC_
	if (gpio_is_valid(kth5701->i2csck_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, kth5701->i2csck_gpio,
					    GPIOF_OUT_INIT_LOW, "i2csck_gpio");
		if (ret) {
			dev_err(&i2c->dev, "%s: i2c sck pin request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(kth5701->i2csda_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, kth5701->i2csda_gpio,
					    GPIOF_OUT_INIT_LOW, "i2csda_gpio");
		if (ret) {
			dev_err(&i2c->dev, "%s: i2c sda pin request failed\n",
				__func__);
			goto err_gpio_request;
		}
	}
#endif

	dev_set_drvdata(&i2c->dev, kth5701);

	gp_Kth5701 = kth5701;


	kth5701_power_onoff(kth5701, KTH5701_ON);
	/* kth5701 chip id */
	ret = kth5701_get_chip_ID(kth5701);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: kth5701_get_chip_ID failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}


	/* 第一步：初始化IIC地址和寄存器配置 */
	mdelay(10);

	KTH57XXInitial(0,0);

	mdelay(10);

	/*第二步：开启持续感应模式*/

	//KTH57XXContinuousSensing(0X0f);
	KTH57XXWakeupSleep(0X0f);
	//KTH57XXSingleConversion(0x0f);

	/* USER CODE END 2 */

    ret = device_create_file(&i2c->dev, &dev_attr_kthReg);
    if (ret) {
        dev_err(&i2c->dev, "Failed to create sysfs kthReg node file\n");
        goto free_class;
    }

	ret = device_create_file(&i2c->dev, &dev_attr_workmode);
    if (ret) {
        dev_err(&i2c->dev, "Failed to create sysfs workmode node file\n");
        goto free_class;
    }

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

free_class:
err_id:
	devm_gpio_free(&i2c->dev, kth5701->enable_gpio);

err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, kth5701);
	kth5701 = NULL;
	gp_Kth5701 = NULL;
	return ret;
}

static int kth5701_i2c_remove(struct i2c_client *i2c)
{

	struct kth5701 *kth5701 = i2c_get_clientdata(i2c);

	pr_info("%s: enter\n", __func__);

	kth5701_power_onoff(kth5701, KTH5701_OFF);

	device_remove_file(&i2c->dev, &dev_attr_workmode);

	device_remove_file(&i2c->dev, &dev_attr_kthReg);

	if (gpio_is_valid(kth5701->enable_gpio))
		devm_gpio_free(&i2c->dev, kth5701->enable_gpio);

	devm_kfree(&i2c->dev, kth5701);
	kth5701 = NULL;

	if(NULL != gp_Kth5701) {
		gp_Kth5701 = NULL;
	}
	return 0;
}


static const struct i2c_device_id kth5701_i2c_id[] = {
	{KTH5701_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kth5701_i2c_id);

static const struct of_device_id kth5701_dt_match[] = {
	{.compatible = "magencoder,kth5701"},
	{},
};

static struct i2c_driver kth5701_i2c_driver = {
	.driver = {
		   .name = KTH5701_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(kth5701_dt_match),
		   },
	.probe = kth5701_i2c_probe,
	.remove = kth5701_i2c_remove,
	.id_table = kth5701_i2c_id,
};

static int __init kth5701_i2c_init(void)
{
	int ret = 0;

	pr_info("kth5701 driver version %s\n", KTH5701_DRIVER_VERSION);

	ret = i2c_add_driver(&kth5701_i2c_driver);
	if (ret) {
		pr_err("fail to add kth5701 device into i2c\n");
		return ret;
	}
	return 0;
}

module_init(kth5701_i2c_init);

static void __exit kth5701_i2c_exit(void)
{
	i2c_del_driver(&kth5701_i2c_driver);
}

module_exit(kth5701_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("dingjun@sz-duowei.com");
MODULE_DESCRIPTION("I2C Driver with Magnetic Encoder.");
