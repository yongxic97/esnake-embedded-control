-- 基于STM32F407ZGT6，为充电机器人提供传感，通信，控制等功能。
    使用FreeRTOS。
    for Project eSnake

-- v1.0.0 2021.8.10 更新，yongxi

-- IO备忘
D0 - OLED_SCL
D1 - OLED_SDA
PD4 - OLED_RES
PD15 - OLED_DC
PD1 - OLED_CS

-- v1.0.3 2021.8.30 update, yongxi
-加入了FreeRTOS的队列以进行任务间通信
-进一步封装了单步控制，加入了通气时间作为一个传参
-加入FreeRTOS静态内存分配和管理，实现ADC->静态内存的DMA

-- v1.0.8 2021.9.15 update, yongxi
-将加入第三个任务，实现jetson nano与STM32的串口通信

-注意：mpl的初始化流程存在很多问题，第一个就是，传入的IMU选择最好是一个结构体参数，方便调整数量
-在调试完整个流程后最好应当修改。