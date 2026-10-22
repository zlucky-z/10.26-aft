



#include "udp_FNC.h"

// CRC-16/CCITT 计算
// 多项式: x^16 + x^12 + x^5 + 1 (0x1021)
// 初始值: 0xFFFF
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// 构建发送帧
static bool build_frame(const uint8_t *data, size_t data_len, uint8_t *frame, size_t *frame_len)
{
    if (data_len > MAX_DATA_SIZE)
	{
        printf("Error: Data too large (max %d bytes)\n", MAX_DATA_SIZE);
        return false;
    }
    size_t offset = 0;
	uint32_t magic,m_magic;
	magic =FRAME_HEADER;
    // 1. 帧头 (4字节)
    m_magic = htonl(magic); 
    memcpy(frame + offset, &m_magic, FRAME_HEADER_SIZE);
    offset += FRAME_HEADER_SIZE;;
    // 2. 数据长度 (2字节，网络字节序)
    uint16_t net_len = htons(data_len);
    memcpy(frame + offset, &net_len, FRAME_LENGTH_SIZE);
    offset += FRAME_LENGTH_SIZE;
    // 3. 数据
    memcpy(frame + offset, data, data_len);
    offset += data_len;
    // 4. CRC校验 (2字节，对帧头+长度+数据计算)
    uint16_t crc = crc16_ccitt(frame, offset);
    uint16_t net_crc = htons(crc);
    memcpy(frame + offset, &net_crc, FRAME_CRC_SIZE);
    offset += FRAME_CRC_SIZE;
    *frame_len = offset;
    return true;
}

// 解析接收帧
static bool parse_frame(const uint8_t *frame, size_t frame_len, uint8_t *data, size_t *data_len)
{
	uint32_t magic,m_magic;

    if (frame_len < FRAME_HEADER_SIZE + FRAME_LENGTH_SIZE + FRAME_CRC_SIZE) {
        printf("Error: Frame too short\n");
        return false;
    }
    size_t offset = 0;
    
    // 1. 检查帧头
	memcpy(&magic, frame , FRAME_HEADER_SIZE);
	m_magic = (uint32_t)(ntohl(magic));
	if(m_magic != FRAME_HEADER)
	{
		printf("Error: HEADER m_magic= %X  magic = %X \n",m_magic,magic);	
		return false;
		
	}
    offset += FRAME_HEADER_SIZE;
    // 2. 获取数据长度
    uint16_t net_len;
    memcpy(&net_len, frame + offset, FRAME_LENGTH_SIZE);
    uint16_t data_len_raw = ntohs(net_len);
    offset += FRAME_LENGTH_SIZE;
    
    if (data_len_raw > MAX_DATA_SIZE) {
        printf("Error: Data length too large\n");
        return false;
    }
    
    // 3. 检查帧长度是否匹配
    size_t expected_len = FRAME_HEADER_SIZE + FRAME_LENGTH_SIZE + data_len_raw + FRAME_CRC_SIZE;
    if (frame_len != expected_len) {
        printf("Error: Frame length mismatch (expected %zu, got %zu)\n", 
               expected_len, frame_len);
        return false;
    }
    
    // 4. 校验CRC (对整个帧头+长度+数据计算，不包括CRC本身)
    uint16_t calc_crc = crc16_ccitt(frame, offset + data_len_raw);
    
    uint16_t recv_crc;
    memcpy(&recv_crc, frame + offset + data_len_raw, FRAME_CRC_SIZE);
    recv_crc = ntohs(recv_crc);
    
    if (calc_crc != recv_crc) {
        printf("Error: CRC mismatch (calc=0x%04X, recv=0x%04X)\n", calc_crc, recv_crc);
        return false;
    }
    
    // 5. 提取数据
    memcpy(data, frame + offset, data_len_raw);
    *data_len = data_len_raw;
    
    return true;
}

// 打印十六进制数据
static  void print_hex(const uint8_t *data, size_t len, const char *prefix)
{
    printf("%s", prefix);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// UDP发送函数
static int udp_send(int sockfd, const char *ip, uint16_t port, const uint8_t *data, size_t data_len)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
        printf("Error: Invalid IP address\n");
        return -1;
    }
    uint8_t frame[MAX_FRAME_SIZE];
    size_t frame_len;
    
    if (!build_frame(data, data_len, frame, &frame_len)) {
		 printf("build_frame error\n");
		return -1;
    }
	#if 0
    print_hex(frame, frame_len, "Sending frame: ");
    printf("Data: %.*s\n", (int)data_len, data);
    printf("Frame length: %zu bytes\n", frame_len);
    #endif
    ssize_t sent = sendto(sockfd, frame, frame_len, 0, 
                          (struct sockaddr*)&addr, sizeof(addr));
    if (sent < 0) {
        perror("sendto error");
        return -1;
    }
    
    return sent;
}

// UDP接收函数
static int udp_recv(int sockfd, uint8_t *data, size_t *data_len, char *sender_ip, uint16_t *sender_port)
{
    uint8_t buffer[MAX_FRAME_SIZE];
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);
    
    memset(buffer, 0, sizeof(buffer));
    memset(&sender_addr, 0, sizeof(sender_addr));
    
    ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&sender_addr, &addr_len);
    if (recv_len < 0) {
        perror("recvfrom error");
        return -1;
    }
    
    //printf("\nReceived %zd bytes from %s:%d\n", recv_len,inet_ntoa(sender_addr.sin_addr), ntohs(sender_addr.sin_port));
    //print_hex(buffer, recv_len, "Received frame: ");

	// 解析帧
    if (!parse_frame(buffer, recv_len, data, data_len)) {
        return -1;
    }
    #if 1
    if (sender_ip) 
	{
        strcpy(sender_ip, inet_ntoa(sender_addr.sin_addr));
    }
    if (sender_port) {
        *sender_port = ntohs(sender_addr.sin_port);
    }
    // 解析帧
    if (!parse_frame(buffer, recv_len, data, data_len)) {
        return -1;
    }
  //printf("Parsed data (%zu bytes): %.*s\n", *data_len, (int)*data_len, data);
	#endif
    return recv_len;
}

struct _udp_remote_Type {

	char sender_ip[INET_ADDRSTRLEN];
	uint16_t sender_port;
	bool deviConnetFLAG;
	int sockfd;
};
struct _udp_remote_Type remote_Type;


static int udp_recv_dataAnalysis(int sockfd,uint8_t *dataBUF, uint16_t datalen,char *sender_ip, uint16_t sender_port)
{
	uint8_t CMD =0;
	uint8_t sendDataBUF[MAX_DATA_SIZE];
	CMD = dataBUF[0];
	//printf("cmd =%x\n",CMD);
	switch(CMD)
	{
		case COM_ConnectDevice_CMD:		
			sendDataBUF[0] = COM_ConnectDevice_CMD;
			sendDataBUF[1] = 0X01;
			int sent = udp_send(sockfd, sender_ip, sender_port, 
						   		(uint8_t*)sendDataBUF, 2);
			if(sent <=0)
			{
				printf("Send failed\n\n");
				return 0;
			}
			 remote_Type.deviConnetFLAG = true;
		break;

		case COM_disConnectDevice_CMD:
			remote_Type.deviConnetFLAG = false;
		break;
	}
	return 0;

}


// 接收线程函数
void* receive_thread_func(void* arg)
{
    //int sockfd = *(int*)arg;
    uint8_t data[MAX_DATA_SIZE];
    size_t data_len;
	//remote_Type.sockfd = sockfd;
    printf("UDP Receive thread started, waiting for data...sockfd = %d\n",remote_Type.sockfd);
    while (1) 
	{
        if (udp_recv(remote_Type.sockfd, data, &data_len, remote_Type.sender_ip, &remote_Type.sender_port) > 0) 
		{
            // 收到数据，可以在这里添加处理逻辑
            print_hex(data, data_len, "Received DATA: ");
            udp_recv_dataAnalysis(remote_Type.sockfd,data,data_len,remote_Type.sender_ip,remote_Type.sender_port);
        }
		usleep(1000*100);
    }
    return NULL;

}
int udp_sendData(uint8_t *databuf,size_t datalen)
{
	int ret =0; 
	if(remote_Type.deviConnetFLAG == true)
	{
		ret = udp_send(remote_Type.sockfd, remote_Type.sender_ip, remote_Type.sender_port, databuf,datalen);	
	}
	return ret;
}
int udp_FNC_INT(void)
{
    int sockfd;
    struct sockaddr_in addr;
    uint16_t port = DEFAULT_PORT;
    pthread_t recv_thread;
    // 创建UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        printf("udp_FNC_INT:socket creation failed\n");
        return -1;
    }
    // 设置地址重用
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) 
	{
        printf("udp_FNC_INT: setsockopt reuseaddr failed\n");
        close(sockfd);
        return -1;
    }
    // 服务器模式：绑定端口并接收数据
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
	{
        printf("udp_FNC_INT:bind failed");
        close(sockfd);
        return -1;
    }
    // 创建接收线程
	//ret = pthread_create(&id1,NULL,(void *)lidar_D500_left_thread_read,NULL);
	remote_Type.sockfd =sockfd;
    if (pthread_create(&recv_thread, NULL, receive_thread_func, &sockfd) != 0) 
	{
        printf("udp_FNC_INT:pthread_create failed");
        close(sockfd);
        return -1;
    }
	 printf("udp_FNC_INT: OK\n");
    return 0;
}



