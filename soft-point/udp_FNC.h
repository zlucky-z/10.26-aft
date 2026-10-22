
#ifndef _UDP_FNC_H
#define _UDP_FNC_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>

#include <stddef.h>



// 帧格式定义
#define FRAME_HEADER_SIZE     			4
#define FRAME_LENGTH_SIZE     			2
#define FRAME_CRC_SIZE        			2
#define FRAME_HEADER         			0xAA55AA55  // 4字节帧头
#define MAX_DATA_SIZE         			4096
#define MAX_FRAME_SIZE          		(FRAME_HEADER_SIZE + FRAME_LENGTH_SIZE + MAX_DATA_SIZE + FRAME_CRC_SIZE)
#define DEFAULT_PORT            		8080

#define  COM_ConnectDevice_CMD									0X11    // 连接设备
#define  COM_disConnectDevice_CMD								0X12    // 连接设备
#define  COM_PointCloudData_CMD									0X20	// 点云数据
#define  COM_dwaPlan_CMD										0X21	// 点云数据

int udp_FNC_INT(void);
int udp_sendData(uint8_t *databuf,size_t datalen);

#endif
