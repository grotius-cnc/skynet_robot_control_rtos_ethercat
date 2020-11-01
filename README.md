# Look at the end of the .pro file how to install. There are a few dependencies to install by terminal.

I published a quite minimal program to keep it simple for starting C++ programmers.

This software is designed to be running in a RealTime environment.

The software is configured to control a robot over Ethercat and preview the robot position in the cad "opencascade" screen.

The operating system i use is Debian Ethercat RTos 64 bit. 
https://github.com/grotius-cnc/LINUX_RTOS

This OS "operating system" uses the HAL "hardware abstract layer" to perform the RT "realtime" operations and connect i-o very easy.

This program is developped with the QT C++ open source edition.

To run or modify this example, you have to download and install the OpenCascade library, 
https://github.com/grotius-cnc/oce

Also you have to install the KDL kinematics library. 
https://github.com/grotius-cnc/orocos_kinematics_dynamics


![skynet_robot_controller_hal_working](https://user-images.githubusercontent.com/44880102/97806781-59299180-1c2b-11eb-8744-dd5f13f865a4.png)

Have fun.

