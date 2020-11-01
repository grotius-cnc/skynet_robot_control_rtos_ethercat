#include "kinematics.h"

#include <frames.hpp>
#include <frames_io.hpp>
#include <trajectory.hpp>
#include <trajectory_segment.hpp>
#include <trajectory_stationary.hpp>
#include <trajectory_composite.hpp>
#include <trajectory_composite.hpp>
#include <velocityprofile_trap.hpp>
#include <path_roundedcomposite.hpp>
#include <rotational_interpolation_sa.hpp>
#include <utilities/error.h>
#include <utilities/utility.h>
#include <trajectory_composite.hpp>

#include <chainiksolverpos_lma.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr_jl.hpp>

using namespace KDL;

FWKIN_DEG kinematics::kinematics_fwd(FWKIN_DEG F){

    //Robot type : KUKA 500_3
    //Value's are given with a robot in forward lying position for axis J1.
    //Chain chain = Chain();
    //chain.addSegment(Segment());                                                      //J1 = 0.0
    //chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(500.0,0.0,1045.0))));   //dist from 0.0 to J2
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(1300.0,0.0,0.0))));     //dist J3 to J4
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(1025.0,0.0,-55.0))));   //dist J3 to J5
    //chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0,0.0,0.0))));        //J4
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(290.0,0.0,0.0))));      //dist J5 tot end-effector
    //chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0,0.0,0.0))));        //J6
    //chain.addSegment(Segment());

    //Robot type : KUKA KR AGILUS sixx
    //Robot value's are given as on datasheet page 23.
    //Cartesian Z is robot height, X is robot seen from side, Y is robot in depth, only used for axis 4 en 6.

    //From datasheet the startup end effector position => X(25+420+80), Y(0), Z(855+35) => Result: X525, Y0, Z890

    Chain chain = Chain();
    chain.addSegment(Segment());                                                        //J0
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(25.0,0.0,400.0))));       //0.0 to J2
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0,0.0,455.0))));        //J2 to J3
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(420.0,0.0,35.0))));       //J3 to J5
    chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0,0.0,0.0))));          //J4 (J4 is in fact on top of J5)
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(80.0,0.0,0.0))));         //J5 to end-effector
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0,0.0,0.0))));          //J6
    chain.addSegment(Segment());

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    jointpositions(0)=(F.J1)*M_PI/180.0;
    jointpositions(1)=(F.J2)*M_PI/180.0;
    jointpositions(2)=(F.J3)*M_PI/180.0;
    jointpositions(3)=(F.J4)*M_PI/180.0;
    jointpositions(4)=(F.J5)*M_PI/180.0;
    jointpositions(5)=(F.J6)*M_PI/180.0;

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){

        std::cout << "Forward Kinemetics Status ok.." <<std::endl;

        std::cout << "X:"<<cartpos.p.x() << " Y:" << cartpos.p.y() << " Z:"<< cartpos.p.z() << std::endl;
        F.Xee=cartpos.p.x();
        F.Yee=cartpos.p.y();
        F.Zee=cartpos.p.z();

        //double roll,pitch,yaw;
        //cartpos.M.GetEulerZYX(yaw,pitch,roll);

        cartpos.M.GetEulerZYX(F.EulerZ,F.EulerY,F.EulerX);
        std::cout << "EULER Z"<<F.EulerZ*(180.0/M_PI) << " Y:"<<F.EulerY*(180.0/M_PI)<<" X:"<<F.EulerX*(180.0/M_PI)<<std::endl;

        //cartpos.M.GetEulerZYZ(alpha,beta,gamma);
        //std::cout << "EULER Z"<<alpha*(180.0/M_PI) << " Y:"<<beta*(180.0/M_PI)<<" Z:"<<gamma*(180.0/M_PI)<<std::endl;

        F.EulerZ*=180.0/M_PI;
        F.EulerY*=180.0/M_PI;
        F.EulerX*=180.0/M_PI;

    } else {std::cout <<"Forward Kinemetics Error"<< std::endl;}

    return F;
}

FWKIN_DEG kinematics::kinematics_inv(FWKIN_DEG F){

    //double roll=0,pitch=0,yaw=0;
    //double alpha=0,beta=0,gamma=0;

    //Chain chain = Chain();
    //Robot type : KUKA 500_3
    //Value's are given with a robot in forward lying position for axis J1.
    //Chain chain = Chain();
    //chain.addSegment(Segment());                                                      //J1 = 0.0
    //chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(500.0,0.0,1045.0))));   //dist from 0.0 to J2
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(1300.0,0.0,0.0))));     //dist J3 to J4
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(1025.0,0.0,-55.0))));   //dist J3 to J5
    //chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0,0.0,0.0))));        //J4
    //chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(290.0,0.0,0.0))));      //dist J5 tot end-effector
    //chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0,0.0,0.0))));        //J6
    //chain.addSegment(Segment());

    //Robot type : KUKA KR AGILUS sixx
    //Robot value's are given as on datasheet page 23.
    //Cartesian Z is robot height, X is robot seen from side, Y is robot in depth, only used for axis 4 en 6.
    Chain chain = Chain();
    chain.addSegment(Segment());                                                        //J0
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(25.0,0.0,400.0))));       //0.0 to J2
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.0,0.0,455.0))));        //J2 to J3
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(420.0,0.0,35.0))));       //J3 to J5
    chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0,0.0,0.0))));          //J4 (J4 is in fact on top of J5)
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(80.0,0.0,0.0))));         //J5 to end-effector
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0,0.0,0.0))));          //J6
    chain.addSegment(Segment());

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    jointpositions(0)=(F.J1)*M_PI/180.0;
    jointpositions(1)=(F.J2)*M_PI/180.0;
    jointpositions(2)=(F.J3)*M_PI/180.0;
    jointpositions(3)=(F.J4)*M_PI/180.0;
    jointpositions(4)=(F.J5)*M_PI/180.0;
    jointpositions(5)=(F.J6)*M_PI/180.0;

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << "Forward Kinemetics Status ok.." <<std::endl;

        std::cout << "Current toolpos / end effector, without translation X:"<<cartpos.p.x() << " Y:" << cartpos.p.y() << " Z:"<< cartpos.p.z() << std::endl;
        F.Xee=cartpos.p.x();
        F.Yee=cartpos.p.y();
        F.Zee=cartpos.p.z();

        //cartpos.M.GetEulerZYX(yaw,pitch,roll);
        cartpos.M.GetEulerZYX(F.EulerZ,F.EulerY,F.EulerX);
        std::cout << "euler angle's, yaw:"<<F.EulerZ *(180.0/M_PI) << " pitch:"<<F.EulerY *(180.0/M_PI)<<" roll:"<<F.EulerX *(180.0/M_PI)<<std::endl;

        //cartpos.M.GetEulerZYZ(alpha,beta,gamma);
        //std::cout << "EULER Z"<<alpha*(180.0/M_PI) << " Y:"<<beta*(180.0/M_PI)<<" Z:"<<gamma*(180.0/M_PI)<<std::endl;
    }

    // inverse kinematics secion

    //Creation of joint arrays q:
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    KDL::JntArray q_min(chain.getNrOfJoints());
    KDL::JntArray q_max(chain.getNrOfJoints());
    KDL::SetToZero(q_init); //initialize with zero positions
    //define q_init:
    q_init(0) = (0)*M_PI/180.0;
    q_init(1) = (0)*M_PI/180.0;
    q_init(2) = (0)*M_PI/180.0;
    q_init(3) = (0)*M_PI/180.0;
    q_init(4) = (0)*M_PI/180.0;
    q_init(5) = (0)*M_PI/180.0;
    q_init(5) = (0)*M_PI/180.0;
    nj = chain.getNrOfJoints();

    //define minimum and maximum joint positions, measure this values exactly!
    //KR500
    //q_min(0) = -185.0*(M_PI/180);
    //q_max(0) = 185*(M_PI/180);
    //q_min(1) = -130.0*(M_PI/180);
    //q_max(1) = 20*(M_PI/180);
    //q_min(2) = -100.0*(M_PI/180);
    //q_max(2) = 144.0*(M_PI/180);
    //q_min(3) = -350.0*(M_PI/180);
    //q_max(3) = 350.0*(M_PI/180);
    //q_min(4) = -120.0*(M_PI/180);
    //q_max(4) = 120.0*(M_PI/180);
    //q_min(5) = -350.0*(M_PI/180);
    //q_max(5) = 350.0*(M_PI/180);

    //KR10 datasheet page 35.
    q_min(0) = -360.0*(M_PI/180);
    q_max(0) = 360*(M_PI/180);
    q_min(1) = -190.0*(M_PI/180);
    q_max(1) = 190*(M_PI/180);
    q_min(2) = -120.0*(M_PI/180);
    q_max(2) = 156.0*(M_PI/180);
    q_min(3) = -185.0*(M_PI/180);
    q_max(3) = 185.0*(M_PI/180);
    q_min(4) = -120.0*(M_PI/180);
    q_max(4) = 120.0*(M_PI/180);
    q_min(5) = -350.0*(M_PI/180);
    q_max(5) = 350.0*(M_PI/180);

    //Creation of the solver:
    //KDL::ChainFkSolverPos_recursive fksolver(chain); //Forward kinematic position solver needed for IK
    KDL::ChainIkSolverVel_pinv iksolverv(chain); //Inverse velocity solver needed for IK
    //KDL::ChainIkSolverPos_NR iksolver(kdlChain, fksolver, iksolverv, 100, 1e-6); //max 100 iterations, stop at accuracy 1e-6
    KDL::ChainIkSolverPos_NR_JL iksolver(chain, q_min, q_max, fksolver, iksolverv, 100, 1e-6);

    //make calculations:

    //define target frame (rotation&translation)
    //rotation is in Tait-Bryan angles/ZYX Euler
    KDL::Frame F_dest(KDL::Rotation::EulerZYX(F.EulerZ,F.EulerY,F.EulerX), KDL::Vector(cartpos.p.x()+F.Xtrans, cartpos.p.y()+F.Ytrans, cartpos.p.z()+F.Ztrans));

    //Solve IK problem for F_dest
    int status = iksolver.CartToJnt(q_init, F_dest, q);

    //copy q to pointer
    if(status>=0){
        std::cout << "Inverse Kinemetics Status ok.." <<std::endl;

        //for(unsigned int i=0;i<nj;i++){
        //    std::cout<<"joint:"<<i<< " degrees:"<< q(i)* (180.0/M_PI)<< std::endl;
        //}

        F.Xee=cartpos.p.x()+F.Xtrans;
        F.Yee=cartpos.p.y()+F.Ytrans;
        F.Zee=cartpos.p.z()+F.Ztrans;

        F.EulerZ*=(180.0/M_PI);
        F.EulerY*=(180.0/M_PI);
        F.EulerX*=(180.0/M_PI);

        F.J1=q(0)*(180.0/M_PI);
        F.J2=q(1)*(180.0/M_PI);
        F.J3=q(2)*(180.0/M_PI);
        F.J4=q(3)*(180.0/M_PI);
        F.J5=q(4)*(180.0/M_PI);
        F.J6=q(5)*(180.0/M_PI);

         std::cout << "Toolpos / End effector, with translation X:"<<F.Xee << " Y:" << F.Yee<< " Z:"<< F.Zee<< std::endl;

    } else {std::cout <<"Inverse Kinemetics Error"<< std::endl;}

    return F;
}













