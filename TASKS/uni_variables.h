#ifndef _UNI_VARIABLES_H
#define _UNI_VARIABLES_H


/******** globally used typedefs **********/

/* Sensor messages of the manipulator part */
typedef struct ManipulatorData{
	float InnerGroup1; 						// pressure of way 1
	float InnerGroup2; 						// pressure of way 2
	float InnerGroup3; 						// pressure of way 3
	float CenterLen;   						// length of the centerline
}ManipulatorData;	

/* Sensor messages of the end-effector part */
typedef struct EndeffectorData{
	float LinearOuterPres; 				// pressure of bottom linear DoF
	float LinearInnerPres;  			// pressure of upper inner linear DoF
	float RotPres;  							// pressure of rotational DoF
}EndeffectorData;

/* Configuration of a single IMU */
typedef struct Pose{
/* around global Z-axis: roll  *
 * around global Y-axis: pitch *
 * around global X-axis: yaw   */
	float pitch,roll,yaw; 				// defined as Euler's angle
}Pose;

// /* raw data of a single IMU, MAY NOT BE USED */
// typedef struct imuRawData{
// 	 short aacx,aacy,aacz;			//加速度传感器原始数据
//	 short gyrox,gyroy,gyroz;		//陀螺仪原始数据
//	 short magx, magy, magz;   	// 磁力计原始数据
// }imuRawData;	

/*********** end of typedefs ************/

#endif
