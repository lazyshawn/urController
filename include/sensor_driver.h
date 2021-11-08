
#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H
#include "./sensor_can_driver.h"

#include <array>

class ForceSensor {
public:
  // float forceX, forceY, forceZ;
  // float force[2][8][3], timestamp[2][8];
  int devType, devIndex, canIndex, status, devNum;
  int ID[2];   // [SDA, No.]
  

  ForceSensor();
  ForceSensor(int devIndex, int canIndex);
  ~ForceSensor();

  int init_dev();
  int read_force(int forceMat[2][8][3], int timestamp[2][8]);

private:
  VCI_BOARD_INFO_EX CAN_BoardInfo;
  VCI_INIT_CONFIG_EX CAN_InitEx;
  VCI_FILTER_CONFIG CAN_FilterConfig;
  VCI_CAN_OBJ *pCAN_ReceiveData;  // 接受的消息内容(若干报文)
};

#endif

