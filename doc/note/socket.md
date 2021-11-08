## § Linux下C++的socket编程
### 头文件
`#include <sys/socket.h>`

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




## § Reference
1. [Socket\_Linux（Linux下C++ socket）][1]
1. [简单web服务器的实现（C++）][2]

[1]:https://blog.csdn.net/qq_27855393/article/details/108294588
[2]:https://blog.csdn.net/qq_22642239/article/details/106463313

