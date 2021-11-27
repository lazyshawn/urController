## ✨ Linux下C++的socket编程

## ✨ 通讯端口配置
### 头文件
`#include <sys/socket.h>`

### 涉及的结构体
```cpp
struct sockaddr {
    unsigned short sa_family; /* 地址家族, AF_XXX */
    char sa_data[14];         /* 14字节协议地址 */
};
```
其中`sa_family`为地址家族，如AF\_INET代表IPv4的地址家族，`sa_data`包含了套接字
中的目标地址和端口信息。由于兼容性问题，为了更好地处理`struct sockaddr`，于是有了并列的结构:
```cpp
struct sockaddr_in {
    short int sin_family;        /* 通信类型 */
    unsigned short int sin_port; /* 端口 */
    struct in_addr sin_addr;     /* internet 地址 */
    unsigned char sin_zeros[8];  /* 与sockaddr结构的长度相同 */
}：

struct in_addr {
    unsigned long s_addr;        /* 4字节的网络字节顺序的IP地址 */
}
```
其中，有几点需要特别注意：
* `sin_zeros`不使用，一般用 0 填充，应该使用`bzero()`或者`memset()`全部清零。
* 指向`sockaddr_in`结构体的指针可以被用来指向结构体`sockaddr`并且代替它。
之后即使`socket()`想要的是`struct sockaddr*`，仍可以使用`sruct sockaddr_in`，
并且在最后转换。
* `sin_family`和`sa_family`应当一致。
* `sin_port`和`sin_addr`必须使用网络字节顺序。
* `sockaddr_in.sin_addr.in_addr.s_addr`存储的是4字节的网络字节顺序的IP地址。

### 本机转换
网络和本机(如平时见到的`127.0.0.0`)字节顺序的转换，能够转换两种类型 (short两字节
和long四字节)。字节顺序类型和对应的缩写为 `{host:h, network:n, short:s, long:l}`，
所以所有转换函数名的集合是 `{htonl, htons, ntohs, ntohl}`。
* `sin_addr` 和 `sin_port` 需要转换成网络字节顺序，是因为两者分别封装在包的
IP层和UDP层，需要发送到网络；
* `sin_family`只是被内核使用来决定在数据结构中包含什么类型的地址，所以必须是本
机字节顺序，而且没有被发送到网络，可以是本机字节顺序。

### IP 地址
`inet_addr()` 能将本机字节顺序转换为网络字节顺序，因此设置 `sockaddr_in` 的IP
地址的时候可以这样设置，
`sockaddr_in.sin_addr.in_addr.s_addr = inet_addr("127.0.0.1);`。
设置为`INADDR_ANY`等效于`0.0.0.0`对应的网络字节顺序，表示主机上的任意网卡都可以
使用这个socket通信。
> 若要将IP地址转换点数格式，则使用函数`inet_ntoa()`，其返回的是指向一个字符的char*指针，是一个由inet_ntoa()控制的静态的固定的指针，所以每次调用inet_ntoa()都会覆盖上次调用时所得的IP地址。

### 例子
```cpp
struct sockaddr_in serv_addr;
memset(&serv_addr, 0, sizeof(serv_addr));            //先使用0元素对结构体进行初始化
serv_addr.sin_family = AF_INET;                      //使用IPv4
serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");  //定义IP地址
serv_addr.sin_port = htons(1234);                    //定义端口号
// 如果是服务器：
    bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))
// 客户端：
    connect(client_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
```


## ✨ 函数接口
### 1. 调用socket()创建套接字
```cpp
// 成功时返回文件描述符，失败时返回-1;
// af: 协议族(Protocol Family)信息;
// type: 数据传输方式;
// protocol: 通信协议;
int socket(int af, int type, int protocol);
```

### 2. 调用bind()函数分配IP地址和端口号
```cpp
// 成功返回0，失败返回-1;
// sockfd: 要分配地址信息(IP地址和端口号)的套接字文件描述符;
// myaddr: 存有地址信息的结构体变量地址值(struct sockaddr*);
// addrlen：第二个结构体变量的长度sizeof(myaddr);
int bind (int sockfd, struct sockaddr * myaddr, socklen_t addrlen);
```

### 3. 调用listen()函数将套接字转换为可接收连接状态
```cpp
// 成功返回0，失败返回-1;
// sockfd: 希望进入等待请求状态的套接字文件描述符，
//         传递的参数为服务端套接字(监听套接字);
// n: 连接请求等待队列(Queue)的长度，表示最多使n个连接请求进入队列
int listen (int sockfd, int n);
```


## ✨ 实例演示
```cpp
/*************************************************************************
 * 服务端程序
*************************************************************************/
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <iostream>
#include <arpa/inet.h>

int main(int argc, char** argv) {
  struct sockaddr_in serv_addr, cli_addr;
  int sServer = socket(AF_INET, SOCK_STREAM, 0);

  /* 设置 sockaddr_in 结构体 */
  // 将内存块serv_addr清零, 初始化地址信息
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  // htons: 将一个16位数从主机字节顺序转换为网络字节顺序。
  serv_addr.sin_port = htons(50007);

  bind(sServer, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  listen(sServer, 1024);

  /* 受理连接请求 */
  socklen_t cli_addr_len = sizeof(cli_addr);
  // 返回一个用于与客户端通信的套接字
  int new_sockfd_ = accept(sServer, (struct sockaddr *)&cli_addr, &cli_addr_len);
  if (new_sockfd_<0) {
    std::cout << "ERROR on accepting reverse communication" << std::endl;
    return 0;
  }

  /* 处理接受的消息 */
  char msg[1024];
  int rcv;
  while(true) {
    bzero(msg, sizeof(msg));
    rcv = recv(new_sockfd_, msg, sizeof(msg), 0);
    if (!rcv) break;
    send(new_sockfd_, msg, strlen(msg), 0);
  }
  close(sServer);
  close(new_sockfd_);

  return 0;
}
```

调试方法：安装Linux下常用的命令行网络测试工具`netcat`，可以用来读写TCP/UDP数据。
```bash
# 远程连接并开始发送数据
nc <ip_addr> <port>
```




## § Reference
1. [Socket\_Linux（Linux下C++ socket）][1]
1. [简单web服务器的实现（C++）][2]
1. [C++ | socket笔记, noayaud, 知乎][3]

[1]:https://blog.csdn.net/qq_27855393/article/details/108294588
[2]:https://blog.csdn.net/qq_22642239/article/details/106463313
[3]:https://zhuanlan.zhihu.com/p/137954595

