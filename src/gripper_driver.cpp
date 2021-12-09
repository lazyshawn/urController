#include "../include/gripper_driver.h"

/*************************************************************************
 * @class: CRGGripper
*************************************************************************/
CRGGripper::CRGGripper(std::string targetIP, unsigned int port){
  memset(cmd, 0, sizeof(cmd));
  clientFd = socket(AF_INET, SOCK_STREAM, 0);
  /* 设置 sockaddr_in 结构体 */
  bzero(&targetAddr, sizeof(targetAddr));
  targetAddr.sin_family = AF_INET;
  targetAddr.sin_port = htons(10001);
  targetAddr.sin_addr.s_addr = inet_addr(targetIP.c_str());
  /* 连接夹爪 */
  msgFd = connect(clientFd, (struct sockaddr*)&targetAddr, sizeof(targetAddr));
  if(msgFd+1 == 0) {
    std::cout << "Connect CRGGripper failed!" << std::endl;
    return;
  }
  home();
}

CRGGripper::~CRGGripper(){
  disable();
  close(clientFd);
  std::cout << "Termination" << std::endl;
}

void CRGGripper::send_cmd(std::string str) {
  if(send(clientFd, str.c_str(), strlen(str.c_str()), 0) == -1) {
    std::cout << "Client send failed" << std::endl;
    return;
  }
  memset(msgRecv, 0, sizeof(msgRecv));
  int recvFlag = recv(clientFd, msgRecv, sizeof(msgRecv), 0);
}

void CRGGripper::home(){
  sprintf(cmd, "home(%d)\n", portIndex);
  send_cmd(cmd);
}

float CRGGripper::read_pos(){
  sprintf(cmd, "VALUE[%d][0]?\n", portIndex);
  send_cmd(cmd);
  pos = std::atof(&msgRecv[12])/1000;
  printf("%f\n", pos);
  return pos;
}

void CRGGripper::go(float destination){
  // 限位
  bool getDes = false;
  if(destination >= 85){
    destination = 85;
  }else if(destination <= 32){
    destination = 32;
  }
  grip = release = destination;
  sprintf(cmd, "GRIPCFG[0][0]=[\"\",%d,%d,%d,0,0,0,0,0]\n",  grip, release, 100);
  send_cmd(cmd);
  send_cmd("GRIP(0,0)\n");
  unsigned int status = 0;
  while(status != 4){
    send_cmd("DEVSTATE[0]?\n");
    status = std::atof(&msgRecv[12]);
    usleep(50);
  }
  send_cmd("RELEASE(0,0)\n");
}

void CRGGripper::disable(){
  sprintf(cmd, "DISABLE(%d)\n", portIndex);
  send_cmd(cmd);
}

void CRGGripper::read_recv(){
  printf("%s\n", msgRecv);
}



/*************************************************************************
 * @class: RobotiQ
*************************************************************************/
/* **************** 构造函数 **************** */
// @param : device-RobotiQ连接的端口号
RobotiQ::RobotiQ(char* in_device):device(in_device){
  udelay = 18000;
  slaveID = 0x09;
  readFC = 0x04;
  writeFC = 0x10;
  masterFC = 0x17;
  firstReadReg[0] = 0x07; firstReadReg[1] = 0xD0;
  firstWriteReg[0] = 0x03; firstWriteReg[1] = 0xE8;
  // 初始化串口
  nFd = port_init();
  // 夹爪初始化
  uint8_t initReg[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  write_registers(0, initReg, 6);
  usleep(udelay);
  read_port(rxData, 8);
  // for (int i=0; i<8; ++i) {
  //   printf("%02X ", rxData[i]);
  // }
};

RobotiQ::~RobotiQ(){
  close(nFd);
}


/* **************** 串口初始化 **************** */
int RobotiQ::port_init(){
  /* *** 打开串口 *** */
  // TODO: 遍历串口，直到打开
  // O_RDWR: 读写方式打开; O_NOCTTY: 不允许进程管理串口; O_NDELAY: 非阻塞
  nFd = open(device, O_RDWR|O_NOCTTY|O_NDELAY);
  if(-1 == nFd) {
    perror("Open Serial Port Error!\n");
    return -1;
  }
  // 恢复串口为阻塞状态
  if ((fcntl(nFd, F_SETFL, 0)) < 0) {
    perror("Fcntl F_SETFL Error!\n");
    return -1;
  }
  // 测试是否为终端设备
  if(isatty(STDIN_FILENO)==0) {
    printf("standard input is not a terminal device\n");
    return -1;
  }
  // 保存原先串口的配置
  if (tcgetattr(nFd, &settingBackup) != 0) {
    perror("tcgetattr error!\n");
    return -1;
  }

  /* *** 串口参数设置 *** */
  // 获取串口原来的参数设置
  tcgetattr(nFd, &serialSetting);
  // 设置终端为原始模式，该模式下全部的输入数据以字节为单位被处理
  cfmakeraw(&serialSetting);
  // 设置波特率，用户不能直接通过位掩码来操作
  cfsetispeed(&serialSetting, B115200);
  cfsetospeed(&serialSetting, B115200);

  // 本地连接 | 接收使能
  serialSetting.c_cflag |= (CLOCAL|CREAD);
  // 用数据位掩码清空数据位设置
  serialSetting.c_cflag &= ~CSIZE;
  // 数据位为8位
  serialSetting.c_cflag |= CS8;
  // 无校验位
  serialSetting.c_cflag &= ~PARENB;
  // 无奇偶校验位
  serialSetting.c_iflag &= ~INPCK; 
  // 清除CSTOPB，设置停止位为1 bits
  serialSetting.c_cflag &= ~CSTOPB;

  // 设置read()函数的调用方式
  // 指定读取每个字符之间的超时时间
  serialSetting.c_cc[VTIME]=2;
  // 指定所要读取字符的最小数量
  serialSetting.c_cc[VMIN]=1;

  // 清空终端未完毕的输入/输出请求及数据
  tcflush(nFd,TCIFLUSH);
  // 立即激活新配置
  if (tcsetattr(nFd, TCSANOW, &serialSetting) != 0) {
    perror("tcsetattr Error!\n");
    return -1;
  }
  return nFd;
};


/* **************** 串口通讯 **************** */
// 推送
int RobotiQ::write_port(uint8_t* tx, int txLen){
  /* *** 向串口发送 *** */
  int writeRet = write(nFd, tx, txLen);
  if (writeRet == txLen) {
    return writeRet;
  } else {
    tcflush(nFd, TCOFLUSH);
    return -1;
  }
}

// 接收
int RobotiQ::read_port(uint8_t* rx, int rxLen){
  /* *** 从串口接收 *** */
  fd_set fs_read;
  // 超时时间10ms
  struct timeval timeout = {0, 10};
  
  // 将指定的文件描述符集清空，每次循环都要清空集合，否则不能检测描述符变化
  FD_ZERO(&fs_read);
  // 在文件描述符集合中增加新的文件描述符
  FD_SET(nFd,&fs_read);
  
  // 使用select实现串口的多路通信
  int retSel = select(nFd+1,&fs_read,NULL,NULL,&timeout);
  if(retSel < 0){
    // 错误
    printf("\033[1;31;40m Error: \033[0m Read port failed!\n");
    return -1;
  }else if(retSel == 0) {
    // 等待超时，没有可读写文件
    return 0;
  }else {
    // 返回读到的字节数
    int rRet = read(nFd,rxData,rxLen);    
    return rRet;
  }
}

void RobotiQ::go(double width) {
  if (width <= 0) width = 0; 
  else if (width > 80) width = 80;

  uint8_t valToWrite[] = {0x09, 0x00, 0x00, uint8_t(width*255/80), 0x01, 0x00};
  write_registers(0, valToWrite, get_array_size(valToWrite));
  read_port(rxData, 8);
}

uint8_t RobotiQ::get_position() {
  // 清空串口
  read_port(rxData, 11);
  read_registers(2, 2);
  // 等待串口输入
  usleep(udelay);
  read_port(rxData, 7);
  return rxData[3];
}

/* **************** 向指定寄存器写入数据 **************** */
void RobotiQ::write_registers(int regIndex, uint8_t valueToWrite[], int numOfBytes) {
  int dataLen = numOfBytes+9;
  txData[0] = slaveID;
  txData[1] = writeFC;
  // address of the first register
  txData[2] = firstWriteReg[0]; txData[3] = firstWriteReg[1] + regIndex;
  // number of registers written to
  txData[4] = 0x00; txData[5] = numOfBytes/2;
  // number of data bytes
  txData[6] = numOfBytes;
  // value written to register
  for (int i=0; i<numOfBytes; ++i) {
    txData[i+7] = valueToWrite[i];
  }
  // crc frame
  uint16_t crc = get_crc16(txData, numOfBytes+7);
  txData[dataLen-2] = crc & 0x00FF;
  txData[dataLen-1] = (crc & 0xFF00)>>8;
  // push to port
  write_port(txData, dataLen);
}

/* **************** 读取指定寄存器 **************** */
// @param : regIndex-首位寄存器索引; numOfVal-写入的字节数(bytes)
void RobotiQ::read_registers(int regIndex, int numOfBytes) {
  txData[0] = slaveID;
  txData[1] = readFC;
  // address of the first register
  txData[2] = firstReadReg[0]; txData[3] = firstReadReg[1] + regIndex;
  // number of registers requested
  txData[4] = 0x00; txData[5] = numOfBytes/2;
  // crc frame
  uint16_t crc = get_crc16(txData, 6);
  txData[6] = crc & 0x00FF;
  txData[7] = (crc & 0xFF00)>>8;
  // push to port
  write_port(txData, 8);
}

/* **************** CRC16校验 **************** */
// @param : *arr_数据帧首地址(最高位); len_数据Bit位数;
// @return: uint16_t_两字节校验码(高位在左)。
uint16_t RobotiQ::get_crc16(uint8_t *arr, int len) {
  // 预置 CRC 寄存器
  uint16_t crc = 0xFFFF, lsb;

  for (int i = 0; i < len; ++i) {
    // 高8位与CRC寄存器异或
    crc ^= arr[i];
    // 逐位移出并检测
    for (int j = 0; j < 8; ++j) {
      lsb = crc & 0x0001;
      crc = crc >> 1;
      // 最低位检测
      if (lsb != 0) crc ^= 0xA001;
    }
  }
  return crc;
}

