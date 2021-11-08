## ✨ urController
[![standard-readme compliant](
https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](
https://github.com/RichardLitt/standard-readme)

This is a servo controller for UR5e, which should be applied for other UR.
All the functions are validated on UR5e.

## ✨ Download
Download this project locally.
```bash
git clone https://github.com/lazyshawn/urController.git
```

## ✨ Usage
### Configuration of PC's Network
1. Add static addresses according to your robot's IP, Netmarsk, and Gateway.
Check them out at `setting -> system -> network` in your robot's Polyscope.
1. Set robot's IP as static address. Note that you can ignore the **not 
connectted error**.

### Dependence
1. Libraries for RobotiQ controlling is required, which could be derived from 
[lazyshawn/robotiQ-controller](https://github.com/lazyshawn/robotiQ-controller).
Put the derived `librobotiq.so` at `<urController>/lib`. Alternatively, adding
the source cpp file `<robotiQ-controller>/src/robotiq.cpp` is also available.

### Building
This project uses CMake. So you build it by running:
```bash
cd urController
mkdir build && cd build
cmake ..
make
```

### Bringup robot
1. Make sure everything is right and bringup your robot.
1. Switch robot controller to remote mode.


### Running
Run the `demo_bin`,
inside the terminal you should see the output `Connectted. Robot gets ready.`
```bash
sudo chmod +x demo_bin
sudo ./demo_bin
```


## ✨ Maintainers
[@lazyshawn](https://github.com/lazyshawn)

