#include "../include/gripper_driver.h"

/*************************************************************************
 * @class: CRGGripper
*************************************************************************/
CRGGripper::CRGGripper(std::string targetIP, unsigned int port){
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
    close(clientFd);
    return;
  }
  home();
}

CRGGripper::~CRGGripper(){
  close(msgFd);
  close(clientFd);
  std::cout << "Termination" << std::endl;
}

void CRGGripper::send_cmd(std::string str) {
  memset(msgRecv, 0, sizeof(msgRecv));
  if(send(clientFd, str.c_str(), strlen(str.c_str()), 0) == -1) {
    std::cout << "Client send failed" << std::endl;
    return;
  }
  int recvFlag = recv(clientFd, msgRecv, sizeof(msgRecv), 0);
}

void CRGGripper::read_recv(){
  printf("%s\n", msgRecv);
}

void CRGGripper::home(){
  sprintf(cmd, "HOME(%d)\n", portIndex);
  send_cmd(cmd);
}

float CRGGripper::read_pos(){
  sprintf(cmd, "VALUE[%d][0]?\n", portIndex);
  send_cmd(cmd);
  pos = std::atof(&msgRecv[12])/1000;
  printf("%f\n", pos);
  return pos;
}

void CRGGripper::go(int destination) {
  // 限位
  bool getDes = false;
  if(destination >= 85){
    destination = 85;
  }else if(destination <= 0){
    destination = 0;
  }
  grip = release = destination;
  force = 100;
  sprintf(cmd, "GRIPCFG[0][0]=[\"\",%d,%d,%d,0,0,0,0,0]\n",  grip, release, force);
  send_cmd(cmd);
  send_cmd("GRIP(0,0)\n");
  // while(!getDes){
    send_cmd("DEVSTATE[0]?\n");
    read_recv();
    printf("%s\n", &msgRecv[12]);
  // }
  send_cmd("RELEASE(0,0)\n");
}

