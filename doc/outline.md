## § Outline
### Project structure
| 文件名              | 主要功能描述   |
| ---                 | ---            |
| main                | 主函数         |
| thread\_pool        | 线程库         |
| data\_exchange      | 数据交换及保存 |
| system\_time        | 系统时间       |
| user\_interface     | 用户交互       |
| trajectory          | 轨迹规划       |
| ur\_driver           | 机械臂驱动     |
| ur\_kinematics      | 运动学规划     |
| matrix              | 矩阵运算       |
| ur\_commuicaton     |                |
| ur\_commuicaton\_RT |                |
| ur\_state           |                |
| ur\_state\_RT       |                |


### Outline of the UrDriver
```cpp
├─ UrDriver::UrDriver()
│  ├─ UrRealtimeCommunication::UrRealtimeCommunication()
│  │  └─ RobotStateRT::RobotStateRT()
│  ├─ UrCommunication::UrCommunication()
│  │  └─ RobotState::RobotState()
│  └─ UrDriver::incoming_sockfd_
│
├─ UrDriver::start()
│  ├─ UrDriver::firmware_version_
│  └─ UrDriver::ip_addr_
│
├─ UrDriver::setServojTime()
│
├─ UrDriver::uploadProg()
│  ├─ RobotStateRT::addCommandToQueue()
│  └─ UrDriver::openServo
│     ├─ UrDriver::new_sockfd_
│     └─ UrDriver::reverse_connected_
│ 
```


