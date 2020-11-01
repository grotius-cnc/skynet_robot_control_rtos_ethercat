#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>

struct FWKIN_DEG{
    double J1=0,J2=0,J3=0,J4=0,J5=0,J6=0;
    double Xee=0,Yee=0,Zee=0; //ee = end effector
    double EulerZ=0,EulerY=0,EulerX=0;
    double Xtrans=0,Ytrans=0,Ztrans=0;
};

class kinematics
{
public:
    struct FWKIN_DEG kinematics_fwd(struct FWKIN_DEG);
    struct FWKIN_DEG kinematics_inv(struct FWKIN_DEG);
};

#endif // KINEMATICS_H
