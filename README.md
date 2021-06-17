https://user-images.githubusercontent.com/44880102/122046313-c0f0af00-cdac-11eb-92a3-1def8517ed62.mp4

Download the latest source code => ZIPFILE
See Tags. https://github.com/grotius-cnc/Skynet_Robot_Control_Rtos_Ethercat/releases/tag/1.0.23

Open in Qt-creator : Skynet_Project.pro (edit the include path's to your needs).
+1 : When compiling the program, all files are copied to the build directory. 

The machine control interface is quite complete. It can move in xyz, joints, euler, tooldirection etc.
The opencascade cad primitive function's are all inplemented 3d : points, lines, arc's, circle, wire, spline, etc. 

Libraries:
- Qt
External :
- Opencascade cad (machine visualisation)
- Kdl orocos kinematics (machine kinematic model)
- Linuxcnc, Hal (hardware abstract layer)
- Ethercat-hg master (ethercat bus)
- Linuxcnc-ethercat (beckhoff ethercat driver family Ek-El)
Internal :
- 3d Spline (3d spline algoritme)
- Dxfrw (read and write dxf files)
- Cavaliercontours (2d contour offset algoritme)
- Scurve motion (scurve motion planner)

My realtime filosofy when the sum of program calculations, kinematic calculations take longer then 1ms :

The hal (hardware abstract layer) is reading the commanded machine position's every 1ms. This 1ms thread is called the servo-thread.
In the kernel we use a realtime component called "streamer" This streamer read's the program position files and perform's a position every 1ms.
The program path is calculated and written to multiple textfiles before execution by the streamer input.
In this way the machine execution has a realtime performance. Through the tiny memory load during execution, program's can carry a huge complexity load.

The code in a nutshell :
1. Load the machine stepfiles.
2. Setup the kinematic model.
3. Teach-in, add primitives like lines or arc's to the bucketvec data.
4. Create a program from the bucketvec data. Create points for every 1ms and store them in multiple textfiles.
5. Play the program. (Consider this as watching a netflix movie).

Todo :
1. Motion reverse.
2. Store teach-in primitive data with the streamer files in a project. Then user can view a previous teach-in project.
3. When kinematic moves are outside the scope, add a solver sequence.
4. Test realtime streaming over the internet.
5. Realtime record option for ultra realistic moves.

Possible implemenations:
1. Robotic Surgery Systems, including realtime streaming over the internet.
2. Machine control system.
3. Machine retrofit.
4. Research and universety applications
5. Boston robotics realistic movements trough realtime record options.
6. Advanched military positioning systems.
7. Use the code as infocenter for opencascade- kinematic- realtime program implementations.

To be reviewed:
1. When the scurve has a initial(vo) or end(ve) motion value, the displacement graph's show a "kink". This "kink" is absorbed by the stepgen backend componenent.

If you have any problems loading the program:
A linux-pro quickstart:

[![Download Linux 11 Professional ](https://a.fsdn.com/con/app/sf-download-button)](https://sourceforge.net/projects/linux-11-pro/files/latest/download)
[![Download Linux 11 Professional ](https://img.shields.io/sourceforge/dt/linux-11-pro.svg)](https://sourceforge.net/projects/linux-11-pro/files/latest/download)

A linux-bullseye quickstart:

[![Download Linux Debian 11 Bullseye Rtos  ](https://a.fsdn.com/con/app/sf-download-button)](https://sourceforge.net/projects/linux-debian-bullseye-11-rtos/files/latest/download)
[![Download Linux Debian 11 Bullseye Rtos  ](https://img.shields.io/sourceforge/dt/linux-debian-bullseye-11-rtos.svg)](https://sourceforge.net/projects/linux-debian-bullseye-11-rtos/files/latest/download)

Not oploaded yet, follow the arc's

https://user-images.githubusercontent.com/44880102/122229068-6e83c100-ce86-11eb-9814-1199550dd265.mp4


