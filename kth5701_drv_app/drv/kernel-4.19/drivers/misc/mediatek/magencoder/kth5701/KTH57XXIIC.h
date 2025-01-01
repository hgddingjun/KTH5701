#ifndef __KTH57XXIIC_H
#define __KTH57XXIIC_H

#include <linux/i2c.h>
#include <linux/types.h>

#define I2C_BASE_ADDR 0x68

#define STA_IIC_ACK   0
#define STA_IIC_NACK 1

#define NACK  0
#define ACK   1

//四种测量模式
#define		CONTINUOUS_SENSING  0x10 //持续感应模式(continuous sensing mode)
#define		WAKEUP_SLEEP  0x20  //唤醒睡眠模式(wake-up & sleep mode)
#define		SINGLE_CONVERSION  0x30  //单次测量模式(single conversino mode)
#define		IDLE  0x80  //空闲模式(idle mode)

//测量数据回读帧(data read frame)
#define		DATA_READ  0x40

//读写寄存器
#define		READ_REGISTER  0x50
#define		WRITE_REGISTER  0x60

//芯片重置
#define		RESET  0xf0

extern uint8_t RegisterData[3];//存放读取寄存器时返回的值 RRData[0]：status RRData[1]：寄存器高八位 RRData[2]：寄存器低八位
extern uint8_t DataReadFrame[9];//存放测量结束读回的数据



//四种测量模式
uint8_t KTH57XXContinuousSensing(uint8_t axis);
uint8_t KTH57XXWakeupSleep(uint8_t axis);
uint8_t KTH57XXSingleConversion(uint8_t axis);
uint8_t KTH57XXIdle(void);

//测量数据回读帧（data read frame）
uint8_t KTH57XXDataRead(uint8_t axis);

//读写寄存器
void KTH57XXReadRegister(uint8_t Register);
uint8_t KTH57XXWriteRegister (uint16_t writeData,uint8_t Register);

//芯片重置
void KTH57XXReset(void);

//寄存器初始化函数
void KTH57XXInitial(uint8_t A1, uint8_t A0);
void KTH57XXRegInitial(void);

//芯片发送测量命令
uint8_t KTH57XXModeCOM(uint8_t *com, uint8_t num);
uint8_t KTH57XXReadSta(void);
uint8_t KTH57XXReadData(uint8_t *data,uint8_t num);


#endif
