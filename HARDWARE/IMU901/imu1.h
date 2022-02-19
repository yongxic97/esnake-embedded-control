#ifndef _IMU1_H_
#define _IMU1_H_


#include "sys.h"
#include "usart2.h"
#include "imu_share.h"

#define imu1_uart_receive(data, len)	usart2_getRxData(data, len) /*!< ���ڻ�ȡ��������API */
#define imu1_uart_send(data)		usart2_sendData(data)	/*!< ����i��������API */


///* ģ�������ϴ������� */
//extern attitude_t		imu1_attitude;		/*!< ��̬�� */
//extern quaternion_t	imu1_quaternion;
//extern gyroAcc_t 		imu1_gyroAccData;
//extern mag_t			imu1_magData;
//extern baro_t			imu1_baroData;
//extern ioStatus_t		imu1_iostatus;

///* ģ��Ĵ�������ֵ */
//extern regValue_t  	imu1_Param;

///* ���ڽ��ս����ɹ������ݰ� */
//extern atkp_t 			imu1_rxPacket;

void imu1_init(void);

uint8_t imu1_unpack(uint8_t ch);
void imu1_atkpParsing(atkp_t *packet);

void imu1_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen);
uint8_t imu1_atkpReadReg(enum regTable reg, int16_t *data);
void imu1_get_data(Pose * pose1);

#endif /* _IMU901_H_ */

/*******************************END OF FILE************************************/


