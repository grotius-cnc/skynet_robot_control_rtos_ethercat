Download the latest release code => ZIPFILE
See Tags.

Example of fighting kuka robots. The program has loaded 4 ChainVectors (projects), each containing 8 Segments (joints). 
That brings this example with 32 controlled axis. It's nothing special. Just build up your program dynamicly.

![skynet_robot_controller_hal_working](https://user-images.githubusercontent.com/44880102/99879078-175f8b80-2bd8-11eb-95e5-55183e9170d0.png)

The ammount of controlled axis is dynamic. Can be rotational, translational.
It doens't matter. Thanks to the KDL coronos inverse kinematics and the OpenCascade libs !

The robot's are specified in the xml config file.
Each project needs a xml config file. Inside the config file, the machine stepfiles are pointed out. Also the kinematic model is defined in the xml file in
a simple way ! No dh parameters needed !

Documents and drawings are included.

This code can be used as a OpenCascade infocenter.

Have fun.

To be contineued.

