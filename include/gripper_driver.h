/* Socket通信头文件 */
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <iostream>
#include <arpa/inet.h>

/*************************************************************************
 * @class: CRGGripper
*************************************************************************/
class CRGGripper {
private:
  int clientFd, msgFd, force;
  sockaddr_in targetAddr;
  float cfg[3], pos;
  int grip, release;
  std::string targetIP;
  unsigned int portIndex;
  char msgRecv[64], cmd[64];

public:
  CRGGripper(std::string targetIP, unsigned int port);
  ~CRGGripper();
  void send_cmd(std::string str);
  void read_recv();
  void home();
  float read_pos();
  void go(int destination);
};

