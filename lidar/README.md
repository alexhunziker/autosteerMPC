# LIDAR Sensor

## Hardware

YDLIDAR X4 (V1)

## Connection

- USB, via USB serial convertor

The Module assumes that the port of the LIDAR Device is `/dev/ttyUSB0`

## Setup

Prerequisite: cmake installed

- clone the YDLIDAR SDK available at `https://github.com/yangfuyuan/ydlidar_sdk`
- Now we have to fix the repository for Usage with a sensor connected to AMA0 and make it compile with raspberry pi:
  - Comment out `echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules` in `startup/initev.sh`
  - Replace all occurences of `<linux/types.h>` with `<sys/sysmacros.h>` in the project
- now we can build the project and test the sensor by:
  ```
  $ mkdir build
  $ cd build
  $ cmake ../
  $ make
  $ ./ydlidar_test
  ```

## Build the module

- Make a folder `build` and swich into it
- Run the following commands to build the module

```
cmake build ../
cmake --build .
```

It will create a shared object (.so) which is needed to include the module via ctypes with the rest of the project as well as an executable to test the module standalone.
The lidar.ini file configures the LIDAR sensor. It needs to be copied to the folder from which the program/shared object is called.
