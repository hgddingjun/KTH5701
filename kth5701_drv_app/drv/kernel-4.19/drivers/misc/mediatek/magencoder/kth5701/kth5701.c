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

#define USE_KTH5701_THREAD (0)

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


#if 0
/**
  * @brief  读取寄存器数据
  *         寄存器中的值以及芯片工作状态存放在RegisterData数组中
  * @param  Register：寄存器
  * @retval IIC是否通信成功
  */
void KTH57XXReadRegister(uint8_t Register)
{

	uint8_t registerName;
	uint8_t com[2];
	uint8_t writeLen, readLen;

	registerName = Register <<2 ; //读取时寄存器要左移两位
	com[0] = READ_REGISTER;
	com[1] = registerName;
	writeLen = 2;
	readLen = 3;

	i2c_master_send(gp_Kth5701->i2c, com, writeLen);
    i2c_master_recv(gp_Kth5701->i2c, RegisterData, readLen);

}
#endif


/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int kth5701_parse_dt(struct device *dev, struct kth5701 *kth5701,
			    struct device_node *np)
{

	kth5701->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (kth5701->enable_gpio < 0) {
		dev_err(dev,
			"%s: no enable gpio provided, will not HW enable device\n",
			__func__);
		return (-1);
	} else {
		dev_info(dev, "%s: enable gpio provided ok\n", __func__);
	}

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

#if USE_KTH5701_THREAD
/** 泰勒展开式求 arctan 的值
	x: 反三角函数输入值
	terms: 泰勒展开级数
 */
static double arctan_taylor(double x, int terms) {
    double result = 0.0;
    double term = x; // 当前项值
    int n;

    for (n = 0; n < terms; n++) {
        if (n > 0) {
            term *= -x * x; // 更新当前项的幂次和符号
        }
        result += term / (2 * n + 1); // 加入当前项
    }

    return result;
}
#endif


#if USE_KTH5701_THREAD /*将此部分移动到hardware层去实现*/
static int kth5701_sense_thread(void *data)
{
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
    while (!kthread_should_stop()) {
        pr_info("kth5701_sense_thread is running...\n");


	    /*第三步：开启离轴模式*/
		if(strcmp(buf,"start")==0)
		{

			/*采样点采样*/
			if(calib_sample_flag == 0)
			{
				KTH57XXDataRead(0X0f);

				x = ( DataReadFrame[3] << 8) + DataReadFrame[4] - 0x7fff;
				y = ( DataReadFrame[5] << 8) + DataReadFrame[6] - 0x7fff;

				//angle = atan2(y,x)*57.3 + 180; //kernel里面没有找个数学库，使用泰勒级数进行展开求解;
				angle = arctan_taylor(y/x, 10)*57.3 + 180;

				if(get_sample == 0)
				{
					pr_info("开始离轴校准，请缓慢旋转磁铁。\n");
					x_max = x;
					x_min = x;
					y_max = y;
					y_min = y;

					ang_get_l = angle/10;
					ang_get_h = ang_get_l + 1;

					get_sample++;
					pr_info("已采样%d个点\n",get_sample);
				}
				else if(get_sample < 36)
				{
					if (ang_get_l == 0)
						ang_get_l = 36;
					if (ang_get_h == 36)
						ang_get_h = 0;

					angle = angle/10;
					if((uint8_t)angle == ang_get_l -1)
					{
						if(x_max < x)
						{
							x_max = x;
						}
						if(x_min > x)
						{
							x_min = x;
						}

						if(y_max < y)
						{
							y_max = y;
						}
						if(y_min > y)
						{
							y_min = y;
						}
						ang_get_l--;
						get_sample++;
						pr_info("已采样%d个点\n",get_sample);
					}
					else if((uint8_t)angle == ang_get_h)
					{
						if(x_max < x)
						{
							x_max = x;
						}
						if(x_min > x)
						{
							x_min = x;
						}

						if(y_max < y)
						{
							y_max = y;
						}
						if(y_min > y)
						{
							y_min = y;
						}
						ang_get_h++;
						get_sample++;
						pr_info("已采样%d个点\n",get_sample);
					}
				}
				else if(get_sample == 36)
				{
						x_max = (x_max - x_min)/2;
						y_max = (y_max - y_min)/2;

						calib_sample_flag = 1;
						pr_info("离轴校准结束\n");
				}
			}
			else//离轴校准完成后输出校准前后角度
			{
				KTH57XXDataRead(0X0f);
				x = ( DataReadFrame[3] << 8) + DataReadFrame[4] - 0x7fff;
				y = ( DataReadFrame[5] << 8) + DataReadFrame[6] - 0x7fff;

				//angle = atan2(y,x)*57.3 + 180;
				angle = arctan_taylor(y/x, 10)*57.3 + 180;

				xTemp = (float)x/x_max;
				yTemp = (float)y/y_max;
				calib_angle = arctan_taylor(yTemp/xTemp, 10)*57.3 + 180;
				//calib_angle = atan2((float)y/y_max, (float)x/x_max)*57.3 + 180;

				pr_info("较准前角度:%0.1f,校准后角度:%0.1f \n",angle,calib_angle);
			}
		}
		else
		{
			pr_info("请发送'start'开始离轴校准 \n");
			//scanf("%s",buf);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        msleep(10); // Sleep for 10 ms
    }
    pr_info("Kernel thread stopping\n");
    return 0;
}
#endif




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

	KTH57XXContinuousSensing(0X0f);

	/* USER CODE END 2 */
#if USE_KTH5701_THREAD
	gp_Kth5701->thread = kthread_run(kth5701_sense_thread, NULL, "kth5701_thread");
	if (IS_ERR(gp_Kth5701->thread)) {
        pr_err("Failed to create kth5701 sense thread\n");
        return PTR_ERR(gp_Kth5701->thread);
    }
#endif

    ret = device_create_file(&i2c->dev, &dev_attr_kthReg);
    if (ret) {
        dev_err(&i2c->dev, "Failed to create sysfs file\n");
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

	device_remove_file(&i2c->dev, &dev_attr_kthReg);

#if 0
    if (gp_Kth5701 && gp_Kth5701->thread) {
        kthread_stop(gp_Kth5701->thread);
        pr_info("kth5701_i2c_remove Thread stopped\n");
    }
#endif

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
