#include "KTH57XXIIC.h"

#include "kth5701.h"

extern uint8_t IIC_ADDR; //IIC地址

/**
  * @brief  写入IIC地址，初始化相应寄存器
  * @param  A0脚电平与A1脚电平
  * @retval 无
  */
void KTH57XXInitial(uint8_t A1, uint8_t A0)//写入i2c地址
{

	mdelay(4);
	//IIC_ADDR = I2C_BASE_ADDR | (A1?2:0) | (A0?1:0);
	//IIC_ADDR = IIC_ADDR << 1;

	KTH57XXRegInitial();

}

/**
  * @brief  开启持续感应模式
  * @param  axis：ZYXT 设置对哪个测量项进行测量
  * @retval 芯片工作状态（status）
  */
uint8_t KTH57XXContinuousSensing(uint8_t axis)
{
	uint8_t sta;
	uint8_t com;
	uint8_t writeLen, readLen;

	com = CONTINUOUS_SENSING | axis;
	writeLen = 1;
	readLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, &sta, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);
    i2c_master_recv(gp_Kth5701->i2c, &sta, readLen);

	return sta;
}

/**
  * @brief  开启唤醒睡眠模式
  * @param  axis：ZYXT 设置对哪个测量项进行测量
  * @retval 芯片工作状态（status）
  */
uint8_t KTH57XXWakeupSleep(uint8_t axis)
{
	uint8_t sta;
	uint8_t com;
	uint8_t writeLen, readLen;

	com = WAKEUP_SLEEP | axis;
	writeLen = 1;
	readLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, &sta, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);
    i2c_master_recv(gp_Kth5701->i2c, &sta, readLen);

	return sta;
}

/**
  * @brief  开启单次测量模式
  * @param  axis：ZYXT 设置对哪个测量项进行测量
  * @retval 芯片工作状态（status）
  */
uint8_t KTH57XXSingleConversion(uint8_t axis)
{
	uint8_t sta;
	uint8_t com;
	uint8_t writeLen, readLen;

	com= SINGLE_CONVERSION | axis;
	writeLen = 1;
	readLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, &sta, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);
	i2c_master_recv(gp_Kth5701->i2c, &sta, readLen);

	return sta;
}

/**
  * @brief  开启空闲模式
  * @param  无
  * @retval 芯片工作状态(status)
  */
uint8_t KTH57XXIdle(void)
{
	uint8_t sta;
	uint8_t com;
	uint8_t writeLen, readLen;

	com = IDLE;
	writeLen = 1;
	readLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, &sta, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);
	i2c_master_recv(gp_Kth5701->i2c, &sta, readLen);

	return sta;
}



/**
  * @brief  芯片重置
  * @param  无
  * @retval IIC是否通信成功
  */
void KTH57XXReset(void)
{
	uint8_t com;
	uint8_t writeLen;

	com = RESET;
	writeLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);

}

/**
  * @brief  读取芯片测量结果 芯片测量结果放在 DataReadFrame 数组中
  * @param  axis：ZYXT 设置对哪个测量项进行输出
  * @retval 返回的字节数
  */
uint8_t KTH57XXDataRead(uint8_t axis)
{
	uint8_t i;
	uint8_t com;
	uint8_t writeLen;
	uint8_t readLen = 1;//根据axis来计算一共需要读回多少测量数据，存放在counter中

	com = DATA_READ | axis;
	writeLen = 1;

	for(i=0;i<4;i++)
	{
		readLen = readLen + ( axis & 0x01 )*2;
		axis = axis >> 1;
	}

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, &com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, DataReadFrame, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, &com, writeLen);
	i2c_master_recv(gp_Kth5701->i2c, DataReadFrame, readLen);

	return readLen;

}


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

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, RegisterData, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, com, writeLen);
	i2c_master_recv(gp_Kth5701->i2c, RegisterData, readLen);

}

/**
  * @brief  向寄存器中写入配置
  * @param  writeData：写入寄存器的数据
  *         Register ：需写入的寄存器
  * @retval 芯片工作状态（status）
  */
uint8_t KTH57XXWriteRegister (uint16_t writeData,uint8_t Register)
{
	uint8_t registerName;
	uint8_t sta;
	uint8_t writeLen, readLen;
	uint8_t com[4];

	registerName = Register <<2 ;
	com[0] = WRITE_REGISTER;
	com[1] = writeData >> 8;
	com[2] = writeData & 0xff;
	com[3] = registerName;
	writeLen = 4;
	readLen = 1;

	//HAL_I2C_Master_Transmit(&hi2c1, IIC_ADDR, com, writeLen, 0xff);
	//HAL_I2C_Master_Receive(&hi2c1, IIC_ADDR, &sta, readLen, 0xff);

	i2c_master_send(gp_Kth5701->i2c, com, writeLen);
	i2c_master_recv(gp_Kth5701->i2c, &sta, readLen);

	return sta;

}

void KTH57XXRegInitial(void)
{
	KTH57XXWriteRegister (0x1636,28);
	KTH57XXWriteRegister (0x0002,29);
	KTH57XXWriteRegister (0x8000,30); /*add 20241224*/
}

