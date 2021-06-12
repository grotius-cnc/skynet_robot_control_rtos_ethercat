
Download the latest release code => ZIPFILE
See Tags.

Example of kuka robots. The program has loaded 2 ChainVectors (projects), each containing 8 Segments (joints). 
That brings this example with 16 controlled axis. It's nothing special. Just build up your program dynamicly.

![skynet_robot_controller_hal_working](https://user-images.githubusercontent.com/44880102/99879792-13823800-2bdd-11eb-8c40-29b79d8a18e5.png)

The ammount of controlled axis is dynamic. Can be rotational, translational.
It doens't matter. Thanks to the KDL coronos inverse kinematics and the OpenCascade libs !

The robot's are specified in the xml config file.
Each project needs a xml config file. Inside the config file, the machine stepfiles are pointed out. Also the kinematic model is defined in the xml file in
a simple way ! No dh parameters needed !

Documents and drawings are included.

This code can be used as a OpenCascade infocenter.

Have fun.

To be contineued.


https://user-images.githubusercontent.com/44880102/121789296-cf6b7a80-cba2-11eb-97d0-e38b01950fe4.mp4

