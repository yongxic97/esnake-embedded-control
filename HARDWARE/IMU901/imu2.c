#include "imu2.h"
#include "usart3.h"
#include "delay.h"
#include "imu_share.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

/* 模块主动上传的数据(串口解析后) */
attitude_t		imu2_attitude;		/*!< 姿态角 */
quaternion_t	imu2_quaternion;
gyroAcc_t 		imu2_gyroAccData;
mag_t			imu2_magData;
baro_t			imu2_baroData;
ioStatus_t		imu2_iostatus;

/* 模块寄存器参数值 */
regValue_t  	imu2_Param;

/* 串口接收解析成功的数据包 */
atkp_t 			imu2_rxPacket;


/* 陀螺仪加速度量程表 */
const uint16_t imu2_gyroFsrTable[4] = {250, 500, 1000, 2000};
const uint8_t  imu2_accFsrTable[4] = {2, 4, 8, 16};

/*其余参数*/
uint8_t ch2;
uint32_t time2 = 0;

/**
  * @brief  接收串口数据解包流程
  */
static enum 
{
    waitForStartByte1,
    waitForStartByte2,
    waitForMsgID,
    waitForDataLength,
    waitForData,
    waitForChksum1,
} imu2_State = waitForStartByte1;


/**
  * @brief  imu901模块串口数据解析（串口接收的每一个数据需传入处理）
  *	@note	此函数需要实时调用
  * @param  ch: 串口接收的单个数据
  * @retval uint8_t: 0 无包 1 有效包
  */
uint8_t imu2_unpack(uint8_t ch)
{
    static uint8_t cksum = 0, dataIndex = 0;

    switch (imu2_State)
    {
        case waitForStartByte1:
            if (ch == UP_BYTE1)
            {
                imu2_State = waitForStartByte2;
                imu2_rxPacket.startByte1 = ch;
            }

            cksum = ch;
            break;

        case waitForStartByte2:
            if (ch == UP_BYTE2 || ch == UP_BYTE2_ACK)
            {
                imu2_State = waitForMsgID;
                imu2_rxPacket.startByte2 = ch;
            }
            else
            {
                imu2_State = waitForStartByte1;
            }

            cksum += ch;
            break;

        case waitForMsgID:
            imu2_rxPacket.msgID = ch;
            imu2_State = waitForDataLength;
            cksum += ch;
            break;

        case waitForDataLength:
            if (ch <= ATKP_MAX_DATA_SIZE)
            {
                imu2_rxPacket.dataLen = ch;
                dataIndex = 0;
                imu2_State = (ch > 0) ? waitForData : waitForChksum1;	/*ch=0,数据长度为0，校验1*/
                cksum += ch;
            }
            else
            {
                imu2_State = waitForStartByte1;
            }

            break;

        case waitForData:
            imu2_rxPacket.data[dataIndex] = ch;
            dataIndex++;
            cksum += ch;

            if (dataIndex == imu2_rxPacket.dataLen)
            {
                imu2_State = waitForChksum1;
            }

            break;

        case waitForChksum1:
            if (cksum == ch)	/*!< 校准正确返回1 */
            {
                imu2_rxPacket.checkSum = cksum;

                return 1;
            }
            else	/*!< 校验错误 */
            {
                imu2_State = waitForStartByte1;
            }

            imu2_State = waitForStartByte1;
            break;

        default:
            imu2_State = waitForStartByte1;
            break;
    }

    return 0;
}



/**
  * @brief  ATKP数据包解析
  * @param  packet: atkp数据包
  * @retval None
  */
void imu2_atkpParsing(atkp_t *packet)
{
    /* 姿态角 */
    if (packet->msgID == UP_ATTITUDE)
    {
        int16_t data = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_attitude.roll = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_attitude.pitch = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[5] << 8) | packet->data[4];
        imu2_attitude.yaw = (float) data / 32768 * 180;
    }

    /* 四元数 */
    else if (packet->msgID == UP_QUAT)
    {
        int16_t data = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_quaternion.q0 = (float) data / 32768;

        data = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_quaternion.q1 = (float) data / 32768;

        data = (int16_t) (packet->data[5] << 8) | packet->data[4];
        imu2_quaternion.q2 = (float) data / 32768;

        data = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu2_quaternion.q3 = (float) data / 32768;
    }

    /* 陀螺仪加速度数据 */
    else if (packet->msgID == UP_GYROACCDATA)
    {
        imu2_gyroAccData.acc[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_gyroAccData.acc[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_gyroAccData.acc[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        imu2_gyroAccData.gyro[0] = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu2_gyroAccData.gyro[1] = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu2_gyroAccData.gyro[2] = (int16_t) (packet->data[11] << 8) | packet->data[10];

        imu2_gyroAccData.faccG[0] = (float)imu2_gyroAccData.acc[0] / 32768 * imu2_accFsrTable[imu2_Param.accFsr]; 		/*!< 4代表4G，上位机设置好的量程 */
        imu2_gyroAccData.faccG[1] = (float)imu2_gyroAccData.acc[1] / 32768 * imu2_accFsrTable[imu2_Param.accFsr];
        imu2_gyroAccData.faccG[2] = (float)imu2_gyroAccData.acc[2] / 32768 * imu2_accFsrTable[imu2_Param.accFsr];

        imu2_gyroAccData.fgyroD[0] = (float)imu2_gyroAccData.gyro[0] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr]; 	/*!< 2000代表2000°/S，上位机设置好的量程 */
        imu2_gyroAccData.fgyroD[1] = (float)imu2_gyroAccData.gyro[1] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr];
        imu2_gyroAccData.fgyroD[2] = (float)imu2_gyroAccData.gyro[2] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr];
    }

    /* 磁场数据 */
    else if (packet->msgID == UP_MAGDATA)
    {
        imu2_magData.mag[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_magData.mag[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_magData.mag[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu2_magData.temp = (float) data / 100;
    }

    /* 气压计数据 */
    else if (packet->msgID == UP_BARODATA)
    {
        imu2_baroData.pressure = (int32_t) (packet->data[3] << 24) | (packet->data[2] << 16) |
                            (packet->data[1] << 8) | packet->data[0];

        imu2_baroData.altitude = (int32_t) (packet->data[7] << 24) | (packet->data[6] << 16) |
                            (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu2_baroData.temp = (float) data / 100;
    }

    /* 端口状态数据 */
    else if (packet->msgID == UP_D03DATA)
    {
        imu2_iostatus.d03data[0] = (uint16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_iostatus.d03data[1] = (uint16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_iostatus.d03data[2] = (uint16_t) (packet->data[5] << 8) | packet->data[4];
        imu2_iostatus.d03data[3] = (uint16_t) (packet->data[7] << 8) | packet->data[6];
    }
}


/**
  * @brief  写寄存器
  * @param  reg: 寄存器列表地址
  * @param  data: 数据
  * @param  datalen: 数据的长度只能是 1或2
  * @retval None
  */
void imu2_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg;
    buf[3] = datalen; 	/*!< datalen只能是1或者2 */
    buf[4] = data;

    if (datalen == 2)
    {
        buf[5] = data >> 8;
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        imu2_uart_send(buf);
    }
    else
    {
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        imu2_uart_send(buf);
    }
}


/**
  * @brief  发送读寄存器命令
  * @param  reg: 寄存器列表地址
  * @retval None
  */
static void imu2_atkpReadRegSend(enum regTable reg)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg | 0x80;
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    imu2_uart_send(buf);
}



/**
  * @brief  读寄存器
  * @param  reg: 寄存器地址
  * @param  data: 读取到的数据
  * @retval uint8_t: 0读取失败（超时） 1读取成功
  */
uint8_t imu2_atkpReadReg(enum regTable reg, int16_t *data)
{
    uint8_t ch;
    uint16_t timeout = 0;

    imu2_atkpReadRegSend(reg);

    while (1)
    {
        if (imu2_uart_receive(&ch, 1)) 	/*!< 获取串口fifo一个字节 */
        {
            if (imu2_unpack(ch)) 			/*!< 有有效数据包 */
            {
                if (imu2_rxPacket.startByte2 == UP_BYTE2) /*!< 主动上传包 */
                {
                    imu2_atkpParsing(&imu2_rxPacket);
                }
                else if (imu2_rxPacket.startByte2 == UP_BYTE2_ACK) /*!< 读寄存器应答包 */
                {
                    if (imu2_rxPacket.dataLen == 1)
                        *data = imu2_rxPacket.data[0];
                    else if (imu2_rxPacket.dataLen == 2)
                        *data = (imu2_rxPacket.data[1] << 8) | imu2_rxPacket.data[0];

                    return 1;
                }
            }
        }
        else
        {
            delay_ms(1);
            timeout++;

            if (timeout > 200) /*!< 超时返回 */
                return 0;
        }
    }
}



/**
  * @brief  模块初始化
  * @param  None
  * @retval None
  */
void imu2_init(void)
{
    int16_t data;

    /**
      *	 写入寄存器参数（测试）
      *	 这里提供写入引用例子，用户可以在这写入一些默认参数，
      *  如陀螺仪加速度量程、带宽、回传速率、PWM输出等。
      */
    imu2_atkpWriteReg(REG_GYROFSR, 3, 1);
    imu2_atkpWriteReg(REG_ACCFSR, 1, 1);
		imu2_atkpWriteReg(REG_SAVE, 0, 1); 	/* 发送保存参数至模块内部Flash，否则模块掉电不保存 */

    /* 读出寄存器参数（测试） */
    imu2_atkpReadReg(REG_GYROFSR, &data);
    imu2_Param.gyroFsr = data;

    imu2_atkpReadReg(REG_ACCFSR, &data);
    imu2_Param.accFsr = data;

    imu2_atkpReadReg(REG_GYROBW, &data);
    imu2_Param.gyroBW = data;

    imu2_atkpReadReg(REG_ACCBW, &data);
    imu2_Param.accBW = data;
}

/*获取解算数据*/
void imu2_get_data(Pose * pose1){
	while(time2<10){
		if (imu2_uart_receive(&ch2, 1)) {	/*!< 获取串口fifo一个字节 */
			if (imu2_unpack(ch2)){ 			/*!< 解析出有效数据包 */
				if (imu2_rxPacket.startByte2 == UP_BYTE2){ 			/*!< 主动上传的数据包 */
					imu2_atkpParsing(&imu2_rxPacket);
				}
			}
		}
		else{
			delay_ms(1);
			time2++;
		}	
	}
	
	pose1->roll  = imu2_attitude.roll;
	pose1->pitch = imu2_attitude.pitch;
	pose1->yaw   = imu2_attitude.yaw;

	time2 = 0;
}

/*******************************END OF FILE************************************/



