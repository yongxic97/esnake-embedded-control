#ifndef _IMU1_H_
#define _IMU1_H_


#include "sys.h"
#include "usart2.h"
#include "imu_share.h"

#define imu1_uart_receive(data, len)	usart2_getRxData(data, len) /*!< 串口获取接收数据API */
#define imu1_uart_send(data)		usart2_sendData(data)	/*!< 串口i发送数据API */


///* 模块主动上传的数据 */
//extern attitude_t		imu1_attitude;		/*!< 姿态角 */
//extern quaternion_t	imu1_quaternion;
//extern gyroAcc_t 		imu1_gyroAccData;
//extern mag_t			imu1_magData;
//extern baro_t			imu1_baroData;
//extern ioStatus_t		imu1_iostatus;

///* 模块寄存器参数值 */
//extern regValue_t  	imu1_Param;

///* 串口接收解析成功的数据包 */
//extern atkp_t 			imu1_rxPacket;

void imu1_init(void);

uint8_t imu1_unpack(uint8_t ch);
void imu1_atkpParsing(atkp_t *packet);

void imu1_atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen);
uint8_t imu1_atkpReadReg(enum regTable reg, int16_t *data);
void imu1_get_data(Pose * pose1);

#endif /* _IMU901_H_ */

/*******************************END OF FILE************************************/


