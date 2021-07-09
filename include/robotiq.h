#ifndef ROBOTIQ_H
#define ROBOTIQ_H

#include <array>
/* *** 串口通讯头文件 *** */
#include <termios.h>    // POSIX 终端控制
#include <fcntl.h>      // 文件控制
#include <stdio.h>      // 标准输入输出
#include <stdlib.h>     // 标准函数库
#include <sys/stat.h>   // 定义状态相关的数据类型
#include <sys/types.h>  // 定义数据类型
#include <unistd.h>     // Unix 标准函数
#include <stdint.h>

// 自定义串口类
class RobotiQ {
private:
  // 打开串口获得的文件描述符
  int nFd;
  // 串口名称
  char* device;
  // 波特率
  int baudRate;
  // 传输延时(us): 发送指令到读取串口返回值的间隔
  int udelay;
  // Linux串口配置的结构体
  struct termios serialSetting;
  struct termios settingBackup;
  // 传输数据
  uint8_t rxData[11], txData[17];
  uint8_t slaveID, readFC, writeFC, masterFC;
  uint8_t firstWriteReg[2], firstReadReg[2];

public:
  // 构造函数
  RobotiQ(char* device);
  // 析构
  ~RobotiQ();
  // 串口初始化
  int port_init();
  // 串口通讯
  int write_port(uint8_t* tx, int txLen);
  int read_port(uint8_t* rx, int rxLen);
  // 用数组给传输数据赋值
  void set_tx(uint8_t* data, int len);
  void open_to_cmd(double width);
  uint8_t get_position(void);
  void write_registers(int regIndex, uint8_t valueToWrite[], int numOfVal);
  void read_registers(int regIndex, int numOfVal);
  /* *********************************************************************
  * ==>> Function:
  * 进行CRC-16/MODBUS计算
  * ==>> Parameters:
  * *data: 输入数据数组指针; len: 数组长度.
  * ==>> 测试网站
  * http://www.ip33.com/crc.html
  ***********************************************************************/
  uint16_t get_crc16(uint8_t *data, int len);
};

// 获取数组长度
template<class T>
int get_array_size(const T& array) {
  return sizeof(array)/sizeof(array[0]);
}

#endif

