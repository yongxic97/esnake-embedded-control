#include "imu1.h"
#include "usart2.h"
#include "delay.h"
#include "imu_share.h"
#include "usart.h"


/* ģ�������ϴ�������(���ڽ�����) */
attitude_t		imu1_attitude;		/*!< ��̬�� */
quaternion_t	imu1_quaternion;
gyroAcc_t 		imu1_gyroAccData;
mag_t			imu1_magData;
baro_t			imu1_baroData;
ioStatus_t		imu1_iostatus;

/* ģ��Ĵ�������ֵ */
regValue_t  	imu1_Param;

/* ���ڽ��ս����ɹ������ݰ� */
atkp_t 			imu1_rxPacket;


/* �����Ǽ��ٶ����̱� */
const uint16_t imu1_gyroFsrTable[4] = {250, 500, 1000, 2000};
const uint8_t  imu1_accFsrTable[4] = {2, 4, 8, 16};

/*�������*/
uint8_t ch1;
uint32_t time1 = 0;

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
} imu1_State = waitForStartByte1;


/**
  * @brief  imu901ģ�鴮�����ݽ��������ڽ��յ�ÿһ�������贫�봦��
  *	@note	�˺�����Ҫʵʱ����
  * @param  ch: ���ڽ��յĵ�������
  * @retval uint8_t: 0 �ް� 1 ��Ч��
  */
uint8_t imu1_unpack(uint8_t ch)
{
    static uint8_t cksum = 0, dataIndex = 0;

    switch (imu1_State)
    {
        case waitForStartByte1:
            if (ch == UP_BYTE1)
            {
                imu1_State = waitForStartByte2;
                imu1_rxPacket.startByte1 = ch;
            }

            cksum = ch;
            break;

        case waitForStartByte2:
            if (ch == UP_BYTE2 || ch == UP_BYTE2_ACK)
            {
                imu1_State = waitForMsgID;
                imu1_rxPacket.startByte2 = ch;
            }
            else
            {
                imu1_State = waitForStartByte1;
            }

            cksum += ch;
            break;

        case waitForMsgID:
            imu1_rxPacket.msgID = ch;
            imu1_State = waitForDataLength;
            cksum += ch;
            break;

        case waitForDataLength:
            if (ch <= ATKP_MAX_DATA_SIZE)
            {
                imu1_rxPacket.dataLen = ch;
                dataIndex = 0;
                imu1_State = (ch > 0) ? waitForData : waitForChksum1;	/*ch=0,���ݳ���Ϊ0��У��1*/
                cksum += ch;
            }
            else
            {
                imu1_State = waitForStartByte1;
            }

            break;

        case waitForData:
            imu1_rxPacket.data[dataIndex] = ch;
            dataIndex++;
            cksum += ch;

            if (dataIndex == imu1_rxPacket.dataLen)
            {
                imu1_State = waitForChksum1;
            }

            break;

        case waitForChksum1:
            if (cksum == ch)	/*!< У׼��ȷ����1 */
            {
                imu1_rxPacket.checkSum = cksum;

                return 1;
            }
            else	/*!< У����� */
            {
                imu1_State = waitForStartByte1;
            }

            imu1_State = waitForStartByte1;
            break;

        default:
            imu1_State = waitForStartByte1;
            break;
    }

    return 0;
}



/**
  * @brief  ATKP���ݰ�����
  * @param  packet: atkp���ݰ�
  * @retval None
  */
void imu1_atkpParsing(atkp_t *packet)
{
    /* ��̬�� */
    if (packet->msgID == UP_ATTITUDE)
    {
        int16_t data = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu1_attitude.roll = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu1_attitude.pitch = (float) data / 32768 * 180;

        data = (int16_t) (packet->data[5] << 8) | packet->data[4];
        imu1_attitude.yaw = (float) data / 32768 * 180;
    }

    /* ��Ԫ�� */
    else if (packet->msgID == UP_QUAT)
    {
        int16_t data = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu1_quaternion.q0 = (float) data / 32768;

        data = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu1_quaternion.q1 = (float) data / 32768;

        data = (int16_t) (packet->data[5] << 8) | packet->data[4];
        imu1_quaternion.q2 = (float) data / 32768;

        data = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu1_quaternion.q3 = (float) data / 32768;
    }

    /* �����Ǽ��ٶ����� */
    else if (packet->msgID == UP_GYROACCDATA)
    {
        imu1_gyroAccData.acc[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu1_gyroAccData.acc[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu1_gyroAccData.acc[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        imu1_gyroAccData.gyro[0] = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu1_gyroAccData.gyro[1] = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu1_gyroAccData.gyro[2] = (int16_t) (packet->data[11] << 8) | packet->data[10];

        imu1_gyroAccData.faccG[0] = (float)imu1_gyroAccData.acc[0] / 32768 * imu1_accFsrTable[imu1_Param.accFsr]; 		/*!< 4����4G����λ�����úõ����� */
        imu1_gyroAccData.faccG[1] = (float)imu1_gyroAccData.acc[1] / 32768 * imu1_accFsrTable[imu1_Param.accFsr];
        imu1_gyroAccData.faccG[2] = (float)imu1_gyroAccData.acc[2] / 32768 * imu1_accFsrTable[imu1_Param.accFsr];

        imu1_gyroAccData.fgyroD[0] = (float)imu1_gyroAccData.gyro[0] / 32768 * imu1_gyroFsrTable[imu1_Param.gyroFsr]; 	/*!< 2000����2000��/S����λ�����úõ����� */
        imu1_gyroAccData.fgyroD[1] = (float)imu1_gyroAccData.gyro[1] / 32768 * imu1_gyroFsrTable[imu1_Param.gyroFsr];
        imu1_gyroAccData.fgyroD[2] = (float)imu1_gyroAccData.gyro[2] / 32768 * imu1_gyroFsrTable[imu1_Param.gyroFsr];
    }

    /* �ų����� */
    else if (packet->msgID == UP_MAGDATA)
    {
        imu1_magData.mag[0] = (int16_t) (packet->data[1] << 8) | packet->data[0];
        imu1_magData.mag[1] = (int16_t) (packet->data[3] << 8) | packet->data[2];
        imu1_magData.mag[2] = (int16_t) (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[7] << 8) | packet->data[6];
        imu1_magData.temp = (float) data / 100;
    }

    /* ��ѹ������ */
    else if (packet->msgID == UP_BARODATA)
    {
        imu1_baroData.pressure = (int32_t) (packet->data[3] << 24) | (packet->data[2] << 16) |
                            (packet->data[1] << 8) | packet->data[0];

        imu1_baroData.altitude = (int32_t) (packet->data[7] << 24) | (packet->data[6] << 16) |
                            (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t) (packet->data[9] << 8) | packet->data[8];
        imu1_baroData.temp = (float) data / 100;
    }

    /* �˿�״̬���� */
    else if (packet->msgID == UP_D03DATA)
    {
        imu1_iostatus.d03data[0] = (uint16_t) (packet->data[1] << 8) | packet->data[0];
        imu1_iostatus.d03data[1] = (uint16_t) (packet->data[3] << 8) | packet->data[2];
        imu1_iostatus.d03data[2] = (uint16_t) (packet->data[5] << 8) | packet->data[4];
        imu1_iostatus.d03data[3] = (uint16_t) (packet->data[7] << 8) | packet->data[6];
    }
}


/**
  * @brief  д�Ĵ���
  * @param  reg: �Ĵ����б��ַ
  * @param  data: ����
  * @param  datalen: ���ݵĳ���ֻ���� 1��2
  * @retval None
  */
void imu1_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen)
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
        imu1_uart_send(buf);
    }
    else
    {
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        imu1_uart_send(buf);
    }
}


/**
  * @brief  ���Ͷ��Ĵ�������
  * @param  reg: �Ĵ����б��ַ
  * @retval None
  */
static void imu1_atkpReadRegSend(enum regTable reg)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg | 0x80;
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    imu1_uart_send(buf);
}



/**
  * @brief  ���Ĵ���
  * @param  reg: �Ĵ�����ַ
  * @param  data: ��ȡ��������
  * @retval uint8_t: 0��ȡʧ�ܣ���ʱ�� 1��ȡ�ɹ�
  */
uint8_t imu1_atkpReadReg(enum regTable reg, int16_t *data)
{
    uint8_t ch;
    uint16_t timeout = 0;

    imu1_atkpReadRegSend(reg);

    while (1)
    {
        if (imu1_uart_receive(&ch, 1)) 	/*!< ��ȡ����fifoһ���ֽ� */
        {
            if (imu1_unpack(ch)) 			/*!< ����Ч���ݰ� */
            {
                if (imu1_rxPacket.startByte2 == UP_BYTE2) /*!< �����ϴ��� */
                {
                    imu1_atkpParsing(&imu1_rxPacket);
                }
                else if (imu1_rxPacket.startByte2 == UP_BYTE2_ACK) /*!< ���Ĵ���Ӧ��� */
                {
                    if (imu1_rxPacket.dataLen == 1)
                        *data = imu1_rxPacket.data[0];
                    else if (imu1_rxPacket.dataLen == 2)
                        *data = (imu1_rxPacket.data[1] << 8) | imu1_rxPacket.data[0];

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
void imu1_init(void)
{
    int16_t data;

    /**
      *	 д��Ĵ������������ԣ�
      *	 �����ṩд���������ӣ��û���������д��һЩĬ�ϲ�����
      *  �������Ǽ��ٶ����̡������ش����ʡ�PWM����ȡ�
      */
    imu1_atkpWriteReg(REG_GYROFSR, 3, 1);
    imu1_atkpWriteReg(REG_ACCFSR, 1, 1);
	imu1_atkpWriteReg(REG_SAVE, 0, 1); 	/* ���ͱ��������ģ���ڲ�Flash������ģ����粻���� */

    /* �����Ĵ������������ԣ� */
    imu1_atkpReadReg(REG_GYROFSR, &data);
    imu1_Param.gyroFsr = data;

    imu1_atkpReadReg(REG_ACCFSR, &data);
    imu1_Param.accFsr = data;

    imu1_atkpReadReg(REG_GYROBW, &data);
    imu1_Param.gyroBW = data;

    imu1_atkpReadReg(REG_ACCBW, &data);
    imu1_Param.accBW = data;
}

void imu1_get_data(Pose * pose1){
	while(time1<10){
		if (imu1_uart_receive(&ch1, 1)) {	/*!< ��ȡ����fifoһ���ֽ� */
			if (imu1_unpack(ch1)){ 			/*!< ��������Ч���ݰ� */
				if (imu1_rxPacket.startByte2 == UP_BYTE2){ 			/*!< �����ϴ������ݰ� */
					imu1_atkpParsing(&imu1_rxPacket);
				}
			}
		}
		else{
			delay_ms(1);
			time1++;			
		}	
	}
	pose1->roll  = imu1_attitude.roll;
	pose1->pitch = imu1_attitude.pitch;
	pose1->yaw   = imu1_attitude.yaw;

	time1 = 0;
}
/*******************************END OF FILE************************************/



