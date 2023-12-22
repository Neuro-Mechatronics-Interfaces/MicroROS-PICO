![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Contributors: 
  Jonathan Shulgach jshulgac@andrew.cmu.edu

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

An all-in-one package for building a local MicroROS workspace supporting ROS2 and flashing a ROS2 node on a Raspberry Pi PICO.

These instructions were put together to make setting up a new PICO microcontroller and integrated into a larger ROS2 framework connected to a host server as painless as possible. Installation of this package has been tested with native Ubuntu 22.04 and Windows 11.

## Installation

 These instructions should work for ROS2 distributions [Foxy](https://docs.ros.org/en/foxy/Installation.html), [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), and [Iron](https://docs.ros.org/en/iron/Installation.html). These install steps have been successfully tested with a [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) (Ubuntu 22.04 on Windows) session running on Windows 11 in adition to native Ununtu 22.04 OS. 
 
 If any problems occur, the Raspberry Pi company has a detailed [setup manual](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf) for further help.

#### Additional steps: WSL

If using WSL to interface with physical usb devices, you will need to set up [USB pass-through](doc/markdown/installation-wsl.md) access.

### Install Micro-ROS 

Make sure your packages are up-to-date.
```bash
sudo apt-get update & sudo apt-get upgrade -y
```
Create the microROS workspace with a ready-to-use micro-ROS build system. This will download any required cross-compilation tools and build the apps for your system.
```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
cd $HOME
mkdir -p micro_ros_ws/src
cd micro_ros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

* Note: Be aware of "End-Of-Life" distributions. If using ROS2 Foxy which reached its EOL phase Summer of 2023, an additional parameter of `--include-eol-distros` is needed when performing the `rosdep` command
  
   ```bash
   rosdep install --from-paths src --include-eol-distros --ignore-src -y
   ```

Create the firmware workspace that targets all the required code and tools
```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
```
Finally create the microROS agent package and source the workspace
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
### Install Pico SDK
Now let's get the latest stable release of the Pico SDK installed and configured:
```bash
# Install dependencies
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/micro_ros_ws/src/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/micro_ros_ws/src/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```
* Note: The Pico SDK installation downloads a handful of submodules for TinyUSB which could take up quite a bit of space on your PC. If optimization isn't a concern, continue with the installation. Otherwise, you can delete folders from the `pico-sdk/lib/hw` path and remove the board types you won't be using. Be sure to keep at least these two folders:
   `pico-sdk/lib/hw/bsp/rp2040`
   `pico-sdk/lib/hw/mcu/raspberry_pi`

You may get a warning that the TinyUSB submodule isn't initialized yet. If so, navigate to the pico-sdk directory and initialize the submodule:
```
cd $HOME/micro_ros_ws/src/pico-sdk
git submodule update --init
```

### Install Pre-Compiled MicroROS Pico libraries
Once the Pico SDK is ready, compile the example:

```bash
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git $HOME/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk
cd $HOME/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk
mkdir build
cd build
cmake ..
make
```

### Flashing Code
To flash code onto a Pico, hold the reset button on the Pico and plug in the USB cable. The `.uf2` file that gets created in the `micro_ros_raspberrypi_pico_sdk/build` subdirectory can be moved to the RPI drive by either clicking and dragging, or with the command:
```
cp pico_micro_ros_example.uf2 /media/$USER/RPI-RP2
```
 
 
### Start MicroROS Agent
Assuming the Pico is connected on `dev/ttyACM0`, pass that in as an argument for the serial communication method.
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
If access to the port is denied, you may need to allow read/write access and add yourself to the `tty` and `dialout` groups (necessary for communicating with USB peripheral devices).
```
sudo chmod a+rw /dev/ttyACM0
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```

 Contributors: 
* Jonathan Shulgach (jshulgac@andrew.cmu.edu)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->


[Neuromechatronics Lab]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[pip install]: https://pip.pypa.io/en/stable/cli/pip_install/

[microROS]: https://micro.ros.org/

[microROS RaspberryPi Pre-Compiled Pico SDK]: https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/tree/iron

[Pico examples]: https://github.com/raspberrypi/pico-examples


