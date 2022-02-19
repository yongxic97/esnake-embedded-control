#ifndef _IMU2_H_
#define _IMU2_H_


#include "sys.h"
#include "usart3.h"
#include "imu_share.h"

#define imu2_uart_receive(data, len)	usart3_getRxData(data, len) /*!< ���ڻ�ȡ��������API */
#define imu2_uart_send(data)		usart3_sendData(data)	/*!< ����i��������API */


///* ģ�������ϴ������� */
//extern attitude_t		imu2_attitude;		/*!< ��̬�� */
//extern quaternion_t	imu2_quaternion;
//extern gyroAcc_t 		imu2_gyroAccData;
//extern mag_t			imu2_magData;
//extern baro_t			imu2_baroData;
//extern ioStatus_t		imu2_iostatus;

///* ģ��Ĵ�������ֵ */
//extern regValue_t  	imu2_Param;

///* ���ڽ��ս����ɹ������ݰ� */
//extern atkp_t 			imu2_rxPacket;

void imu2_init(void);

uint8_t imu2_unpack(uint8_t ch);
void imu2_atkpParsing(atkp_t *packet);

void imu2_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen);
uint8_t imu2_atkpReadReg(enum regTable reg, int16_t *data);
void imu2_get_data(Pose * pose);

#endif /* _IMU901_H_ */

/*******************************END OF FILE************************************/


