#include "imu2.h"
#include "usart3.h"
#include "delay.h"
#include "imu_share.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

/* ģ�������ϴ�������(���ڽ�����) */
attitude_t		imu2_attitude;		/*!< ��̬�� */
quaternion_t	imu2_quaternion;
gyroAcc_t 		imu2_gyroAccData;
mag_t			imu2_magData;
baro_t			imu2_baroData;
ioStatus_t		imu2_iostatus;

/* ģ��Ĵ�������ֵ */
regValue_t  	imu2_Param;

/* ���ڽ��ս����ɹ������ݰ� */
atkp_t 			imu2_rxPacket;


/* �����Ǽ��ٶ����̱� */
const uint16_t imu2_gyroFsrTable[4] = {250, 500, 1000, 2000};
const uint8_t  imu2_accFsrTable[4] = {2, 4, 8, 16};

/*�������*/
uint8_t ch2;
uint32_t time2 = 0;

/**
  * @brief  ���մ������ݽ������
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
  * @brief  imu901ģ�鴮�����ݽ��������ڽ��յ�ÿһ�������贫�봦��
  *	@note	�˺�����Ҫʵʱ����
  * @param  ch: ���ڽ��յĵ�������
  * @retval uint8_t: 0 �ް� 1 ��Ч��
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
                imu2_State = (ch > 0) ? waitForData : waitForChksum1;	/*ch=0,���ݳ���Ϊ0��У��1*/
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
            if (cksum == ch)	/*!< У׼��ȷ����1 */
            {
                imu2_rxPacket.checkSum = cksum;

                return 1;
            }
            else	/*!< У����� */
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
  * @brief  ATKP���ݰ�����
  * @param  packet: atkp���ݰ�
  * @retval None
  */
void imu2_atkpParsing(atkp_t *packet)
{
    /* ��̬�� */
    if (packet->msgID == UP_ATTITUDE)
    {
        int16_t data = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_attitude.roll = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_attitude.pitch = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[5] << 8) | packet->data[4];
        imu2_attitude.yaw = (float) data / 32768 * 180;
    }

    /* ��Ԫ�� */
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

    /* �����Ǽ��ٶ����� */
    else if (packet->msgID == UP_GYROACCDATA)
    {
        imu2_gyroAccData.acc[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_gyroAccData.acc[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_gyroAccData.acc[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        imu2_gyroAccData.gyro[0] = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu2_gyroAccData.gyro[1] = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu2_gyroAccData.gyro[2] = (int16_t) (packet->data[11] << 8) | packet->data[10];

        imu2_gyroAccData.faccG[0] = (float)imu2_gyroAccData.acc[0] / 32768 * imu2_accFsrTable[imu2_Param.accFsr]; 		/*!< 4����4G����λ�����úõ����� */
        imu2_gyroAccData.faccG[1] = (float)imu2_gyroAccData.acc[1] / 32768 * imu2_accFsrTable[imu2_Param.accFsr];
        imu2_gyroAccData.faccG[2] = (float)imu2_gyroAccData.acc[2] / 32768 * imu2_accFsrTable[imu2_Param.accFsr];

        imu2_gyroAccData.fgyroD[0] = (float)imu2_gyroAccData.gyro[0] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr]; 	/*!< 2000����2000��/S����λ�����úõ����� */
        imu2_gyroAccData.fgyroD[1] = (float)imu2_gyroAccData.gyro[1] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr];
        imu2_gyroAccData.fgyroD[2] = (float)imu2_gyroAccData.gyro[2] / 32768 * imu2_gyroFsrTable[imu2_Param.gyroFsr];
    }

    /* �ų����� */
    else if (packet->msgID == UP_MAGDATA)
    {
        imu2_magData.mag[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_magData.mag[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_magData.mag[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu2_magData.temp = (float) data / 100;
    }

    /* ��ѹ������ */
    else if (packet->msgID == UP_BARODATA)
    {
        imu2_baroData.pressure = (int32_t) (packet->data[3] << 24) | (packet->data[2] << 16) |
                            (packet->data[1] << 8) | packet->data[0];

        imu2_baroData.altitude = (int32_t) (packet->data[7] << 24) | (packet->data[6] << 16) |
                            (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu2_baroData.temp = (float) data / 100;
    }

    /* �˿�״̬���� */
    else if (packet->msgID == UP_D03DATA)
    {
        imu2_iostatus.d03data[0] = (uint16_t) (packet->data[1] << 8) | packet->data[0];
        imu2_iostatus.d03data[1] = (uint16_t) (packet->data[3] << 8) | packet->data[2];
        imu2_iostatus.d03data[2] = (uint16_t) (packet->data[5] << 8) | packet->data[4];
        imu2_iostatus.d03data[3] = (uint16_t) (packet->data[7] << 8) | packet->data[6];
    }
}


/**
  * @brief  д�Ĵ���
  * @param  reg: �Ĵ����б��ַ
  * @param  data: ����
  * @param  datalen: ���ݵĳ���ֻ���� 1��2
  * @retval None
  */
void imu2_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg;
    buf[3] = datalen; 	/*!< datalenֻ����1����2 */
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
  * @brief  ���Ͷ��Ĵ�������
  * @param  reg: �Ĵ����б��ַ
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
  * @brief  ���Ĵ���
  * @param  reg: �Ĵ�����ַ
  * @param  data: ��ȡ��������
  * @retval uint8_t: 0��ȡʧ�ܣ���ʱ�� 1��ȡ�ɹ�
  */
uint8_t imu2_atkpReadReg(enum regTable reg, int16_t *data)
{
    uint8_t ch;
    uint16_t timeout = 0;

    imu2_atkpReadRegSend(reg);

    while (1)
    {
        if (imu2_uart_receive(&ch, 1)) 	/*!< ��ȡ����fifoһ���ֽ� */
        {
            if (imu2_unpack(ch)) 			/*!< ����Ч���ݰ� */
            {
                if (imu2_rxPacket.startByte2 == UP_BYTE2) /*!< �����ϴ��� */
                {
                    imu2_atkpParsing(&imu2_rxPacket);
                }
                else if (imu2_rxPacket.startByte2 == UP_BYTE2_ACK) /*!< ���Ĵ���Ӧ��� */
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

            if (timeout > 200) /*!< ��ʱ���� */
                return 0;
        }
    }
}



/**
  * @brief  ģ���ʼ��
  * @param  None
  * @retval None
  */
void imu2_init(void)
{
    int16_t data;

    /**
      *	 д��Ĵ������������ԣ�
      *	 �����ṩд���������ӣ��û���������д��һЩĬ�ϲ�����
      *  �������Ǽ��ٶ����̡������ش����ʡ�PWM����ȡ�
      */
    imu2_atkpWriteReg(REG_GYROFSR, 3, 1);
    imu2_atkpWriteReg(REG_ACCFSR, 1, 1);
		imu2_atkpWriteReg(REG_SAVE, 0, 1); 	/* ���ͱ��������ģ���ڲ�Flash������ģ����粻���� */

    /* �����Ĵ������������ԣ� */
    imu2_atkpReadReg(REG_GYROFSR, &data);
    imu2_Param.gyroFsr = data;

    imu2_atkpReadReg(REG_ACCFSR, &data);
    imu2_Param.accFsr = data;

    imu2_atkpReadReg(REG_GYROBW, &data);
    imu2_Param.gyroBW = data;

    imu2_atkpReadReg(REG_ACCBW, &data);
    imu2_Param.accBW = data;
}

/*��ȡ��������*/
void imu2_get_data(Pose * pose1){
	while(time2<10){
		if (imu2_uart_receive(&ch2, 1)) {	/*!< ��ȡ����fifoһ���ֽ� */
			if (imu2_unpack(ch2)){ 			/*!< ��������Ч���ݰ� */
				if (imu2_rxPacket.startByte2 == UP_BYTE2){ 			/*!< �����ϴ������ݰ� */
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



