#include "../include/sensor_driver.h"

/*************************************************************************
* @class: ForceSensor;
*************************************************************************/
ForceSensor::ForceSensor(int devIndex_, int canIndex_): devType(VCI_USBCAN2) {
  devIndex = devIndex_;
  canIndex = canIndex_;
  status = 0;
}

ForceSensor::ForceSensor(): devType(VCI_USBCAN2), devIndex(0), canIndex(0) {
  status = 0;
}

ForceSensor::~ForceSensor() {
  /* Free Received Data */
  free(pCAN_ReceiveData);
  /* Disconnect Sensor */
  VCI_CAN_OBJ CAN_SendData[1];
  for (int j = 0; j < 1; j++) {
    CAN_SendData[j].DataLen = 2;  // 报文长度
    CAN_SendData[j].Data[0] = 0x07;
    CAN_SendData[j].Data[1] = 0x01;
    CAN_SendData[j].ID = 0x201;  // 报文ID

    CAN_SendData[j].ExternFlag = 0;
    CAN_SendData[j].RemoteFlag = 0;
    CAN_SendData[j].SendType = 0;
  }
  status = VCI_Transmit(devType, devIndex, canIndex, CAN_SendData, 1);
  if (status == STATUS_ERR) {
    printf("Send CAN %d data failed!\n", canIndex);
    VCI_ResetCAN(devType, devIndex, canIndex);
  } else {
    printf("Sensor on CAN %d disconncted!\n", canIndex);
  }
  /* Stop receive can data */
  status = VCI_ResetCAN(devType, devIndex, canIndex);
  printf("VCI_ResetCAN %d\n", status);
  VCI_CloseDevice(devType, devIndex);
  printf("VCI_CloseDevice\n");
  std::cout << "== Devices disconncted\n" << std::endl;
}

int ForceSensor::init_dev() {
// int init_dev() {
  /* Scan device */
  devNum = VCI_ScanDevice(1);
  if (devNum > 0) {
    printf("Have %d device connected!\n", devNum);
  } else {
    printf("No device connected!\n");
    return 0;
  }

  /* Get board info */
  status = VCI_ReadBoardInfoEx(devIndex, &CAN_BoardInfo); // It will open device
  if (status == STATUS_ERR) {
    printf("Get board info failed!\n");
    return 0;
  } else {
    printf("--CAN_BoardInfo.ProductName = %s\n", CAN_BoardInfo.ProductName);
    printf("--CAN_BoardInfo.FirmwareVersion = V%d.%d.%d\n",
        CAN_BoardInfo.FirmwareVersion[1], CAN_BoardInfo.FirmwareVersion[2],
        CAN_BoardInfo.FirmwareVersion[3]);
    printf("--CAN_BoardInfo.HardwareVersion = V%d.%d.%d\n",
        CAN_BoardInfo.HardwareVersion[1], CAN_BoardInfo.HardwareVersion[2],
        CAN_BoardInfo.HardwareVersion[3]);
    printf("--CAN_BoardInfo.SerialNumber = %08X%08X%08X\n",
        (uint32_t)(*(uint32_t *)(&CAN_BoardInfo.SerialNumber[0])),
        (uint32_t)(*(uint32_t *)(&CAN_BoardInfo.SerialNumber[4])),
        (uint32_t)(*(uint32_t *)(&CAN_BoardInfo.SerialNumber[8])));
  }

  /* CAN init EX */
  // Config device
  CAN_InitEx.CAN_ABOM = 0;
  CAN_InitEx.CAN_Mode = 0;
  CAN_InitEx.CAN_NART = 0;
  CAN_InitEx.CAN_RFLM = 0;
  CAN_InitEx.CAN_TXFP = 1;
  CAN_InitEx.CAN_RELAY = 0;
  // Baurate 1M
  CAN_InitEx.CAN_BRP = 9;
  CAN_InitEx.CAN_BS1 = 2;
  CAN_InitEx.CAN_BS2 = 1;
  CAN_InitEx.CAN_SJW = 1;
  status = VCI_InitCANEx(devType, devIndex, canIndex, &CAN_InitEx);
  if (status == STATUS_ERR) {
    printf("Init device failed!\n");
    return 0;
  } else {
    printf("Init device success!\n");
  }

  /* Set filter */
  CAN_FilterConfig.FilterIndex = 0;
  CAN_FilterConfig.Enable = 1;
  CAN_FilterConfig.ExtFrame = 0;
  CAN_FilterConfig.FilterMode = 0;
  CAN_FilterConfig.ID_IDE = 0;
  CAN_FilterConfig.ID_RTR = 0;
  CAN_FilterConfig.ID_Std_Ext = 0;
  CAN_FilterConfig.MASK_IDE = 0;
  CAN_FilterConfig.MASK_RTR = 0;
  CAN_FilterConfig.MASK_Std_Ext = 0;
  status = VCI_SetFilter(devType, devIndex, canIndex, &CAN_FilterConfig);
  for (int i = 1; i <= 13; i++) {
    CAN_FilterConfig.FilterIndex = i;
    status = VCI_SetFilter(devType, devIndex, canIndex, &CAN_FilterConfig);
  }
  if (status == STATUS_ERR) {
    printf("Set filter failed!\n");
    return 0;
  } else {
    printf("can index:%d Set filter success!\n", canIndex);
  }

  /* Start CAN */
  status = VCI_StartCAN(devType, devIndex, canIndex);
  if (status == STATUS_ERR) {
    printf("Start CAN:%d failed!\n", canIndex);
    return 0;
  } else {
    printf("Start CAN%d success!\n", canIndex);
  }

  /* Send data */
  VCI_CAN_OBJ CAN_SendData[1];
  for (int j = 0; j < 1; j++) {
    CAN_SendData[j].DataLen = 2;  // 报文长度
    CAN_SendData[j].Data[0] = 0x07;
    CAN_SendData[j].Data[1] = 0x00;
    CAN_SendData[j].ID = 0x201;  // 报文ID

    CAN_SendData[j].ExternFlag = 0;
    CAN_SendData[j].RemoteFlag = 0;
    CAN_SendData[j].SendType = 0;
  }
  status = VCI_Transmit(devType, devIndex, canIndex, CAN_SendData, 2);
  if (status == STATUS_ERR) {
    printf("Send CAN %d data failed!\n", canIndex);
    VCI_ResetCAN(devType, devIndex, canIndex);
  } else {
    printf("Send CAN %d data success!\n", canIndex);
  }

  return 1;
}

int ForceSensor::read_force(int forceMat[2][8][3], int timestamp[2][8]) {
  int sda, taxel;   // 传感器模块编号
  int DataNum = 0;  // 报文数
  pCAN_ReceiveData = NULL;  // 接受的消息内容(若干报文)
  // 确保至少读到一组数据
  while (pCAN_ReceiveData == NULL || DataNum == 0) {
    DataNum = VCI_GetReceiveNum(devType, devIndex, canIndex);
    pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(DataNum * sizeof(VCI_CAN_OBJ));
  }
  int ReadDataNum = VCI_Receive(
      devType, devIndex, canIndex, pCAN_ReceiveData, DataNum);
  // 解析报文 | 保存数据
  for (int i = 0; i < ReadDataNum; i++) {
    sda = (pCAN_ReceiveData[i].ID & 0x0F0) >> 8;
    taxel = (pCAN_ReceiveData[i].ID & 0x00F);

    timestamp[sda][taxel] = pCAN_ReceiveData[i].TimeStamp;
    forceMat[sda][taxel][0] = ((pCAN_ReceiveData[i].Data[1] & 0x00ff) << 8) |
      pCAN_ReceiveData[i].Data[2];
    forceMat[sda][taxel][1] = ((pCAN_ReceiveData[i].Data[3] & 0x00ff) << 8) |
      pCAN_ReceiveData[i].Data[4];
    forceMat[sda][taxel][2] = ((pCAN_ReceiveData[i].Data[5] & 0x00ff) << 8) |
      pCAN_ReceiveData[i].Data[6];

    /* For debug */
    // printf("== Receive data:\n");
    // printf("%03X\n", pCAN_ReceiveData[i].ID);
    // printf("%d: %X, %X\n", pCAN_ReceiveData[i].TimeStamp, sda, taxel);
    // printf("force: %d\n", forceMat[sda][taxel][0]);
  }
  return 1;
}

