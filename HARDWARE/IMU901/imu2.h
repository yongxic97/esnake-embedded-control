#ifndef _IMU2_H_
#define _IMU2_H_


#include "sys.h"
#include "usart3.h"
#include "imu_share.h"

#define imu2_uart_receive(data, len)	usart3_getRxData(data, len) /*!< 串口获取接收数据API */
#define imu2_uart_send(data)		usart3_sendData(data)	/*!< 串口i发送数据API */


///* 模块主动上传的数据 */
//extern attitude_t		imu2_attitude;		/*!< 姿态角 */
//extern quaternion_t	imu2_quaternion;
//extern gyroAcc_t 		imu2_gyroAccData;
//extern mag_t			imu2_magData;
//extern baro_t			imu2_baroData;
//extern ioStatus_t		imu2_iostatus;

///* 模块寄存器参数值 */
//extern regValue_t  	imu2_Param;

///* 串口接收解析成功的数据包 */
//extern atkp_t 			imu2_rxPacket;

void imu2_init(void);

uint8_t imu2_unpack(uint8_t ch);
void imu2_atkpParsing(atkp_t *packet);

void imu2_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen);
uint8_t imu2_atkpReadReg(enum regTable reg, int16_t *data);
void imu2_get_data(Pose * pose);

#endif /* _IMU901_H_ */

/*******************************END OF FILE************************************/


