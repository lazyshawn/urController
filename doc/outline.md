## § Steps to Bringup UR
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


### Servo threads





