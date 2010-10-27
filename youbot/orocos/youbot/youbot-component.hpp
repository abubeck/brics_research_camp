#ifndef OROCOS_YOUBOT_COMPONENT_HPP
#define OROCOS_YOUBOT_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <iostream>
#include <cstdlib>
#include <youBotApi.h>
#include "youbot_chain.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <cmath>

using namespace std;
using namespace youbot;
using namespace KDL;

typedef struct{
  int jointID;
  int gearRatio;
  double maxJointValue;
  //double maxAxisValue;
  double minJointValue;
  //double minAxisValue;
  bool negative;
  double rpm_to_rad;
} YouJoint;



class Youbot
    : public RTT::TaskContext
{
   YouBotApi youBot;

   int encoderSteps;

   YouJoint joints[5];

   // Convert from joint value to encoder relative value
   double encoderValueToJointValue(int jointID, int encoderValue){
       return double(encoderValue)/(joints[jointID-1].gearRatio*encoderSteps)*360;
   }
   // Convert from encoder value to joint relative value
   int jointValueToEncoderValue(int jointID, double jointValue){
       return jointValue/360*(joints[jointID-1].gearRatio*encoderSteps);
   }

   //Convert from encoder speeds to joint speeds in radians per second
   double encoderSpeedToRadPerSec(int jointID, double encoderSpeed){
	   return encoderSpeed*joints[jointID-1].rpm_to_rad;
   }

   //Convert from joint speeds in radians per second to encoder speeds
   double radPerSecToEncoderSpeed(int jointID, double radPerSec){
   	   return radPerSec/joints[jointID-1].rpm_to_rad;
   }

   // Convert from Joint value to axis absolute value
   int getAxisAbsolutePosition(int jointID, double jointPosition){
     if(joints[jointID-1].negative)
         return jointValueToEncoderValue(jointID, -jointPosition + joints[jointID-1].minJointValue);
       else
         return jointValueToEncoderValue(jointID, jointPosition - joints[jointID-1].maxJointValue);
   }

   // Convert from Joint value to axis absolute value
   double getJointAbsolutePosition(int jointID, int axisPosition){
       double value = encoderValueToJointValue(jointID, axisPosition);
       if(joints[jointID-1].negative)
         return joints[jointID-1].minJointValue - value;
       else
         return value + joints[jointID-1].maxJointValue;
   }

   Chain chain;
   ChainFkSolverPos_recursive *solver;
   ChainIkSolverVel_wdls *iksolver;
   JntArray q;
   Frame eef;
   JntArray qdot;
   Eigen::MatrixXd Mq;

public:
    Youbot(string const& name)
        : TaskContext(name, PreOperational), youBot("/tmp/youBotMemMapFile", 123456),
          encoderSteps(4096)
    {
        std::cout << "Youbot constructed !" <<std::endl;

        addOperation("move",&Youbot::move, this)
        .doc("move axis to given position").arg("axis","axis 1..5").arg("degrees","degrees");

        addOperation("setSpeed",&Youbot::setSpeed, this)
        .doc("move axis at given speed").arg("axis","axis 1..5").arg("radPerSec","radians");

        addOperation("openGripper",&Youbot::openGripper, this);
        addOperation("closeGripper",&Youbot::closeGripper, this);

        addOperation("speedTest",&Youbot::speedTest, this);

        addProperty("eef",eef);

        chain = getYouBotKinematicChain();
        q.resize(8);
        solver = new ChainFkSolverPos_recursive(chain);
        iksolver = new ChainIkSolverVel_wdls(chain);

        this->configure();
    }

    bool configureHook() {
        std::cout << "Youbot configured !" <<std::endl;

    	// Joint 0 parameters
    	joints[0].jointID = 1;
    	joints[0].gearRatio = 156;
    	joints[0].minJointValue = -169;
    	joints[0].maxJointValue = 169;
    	joints[0].negative = true;
    	joints[0].rpm_to_rad = -0.0783;
    	//joints[0].minAxisValue = -585659;

    	// Joint 1 parameters
    	joints[1].jointID = 2;
    	joints[1].gearRatio = 156;
    	joints[1].minJointValue = -65;
    	joints[1].maxJointValue = 90;
    	joints[1].negative = true;
    	joints[1].rpm_to_rad = -0.0807;
    	//joints[1].minAxisValue = -268741;

    	// Joint 2 parameters
    	joints[2].jointID = 3;
    	joints[2].gearRatio = 100;
    	joints[2].minJointValue = -151;
    	joints[2].maxJointValue = 146;
    	joints[2].negative = false;
    	joints[2].rpm_to_rad = 0.0863;
    	//joints[2].minAxisValue = -325596;

    	// Joint 3 parameters
    	joints[3].jointID = 4;
    	joints[3].gearRatio = 71;
    	joints[3].minJointValue = -102.5;
    	joints[3].maxJointValue = 102.5;
    	joints[3].negative = true;
    	joints[3].rpm_to_rad = -0.0863;
    	//joints[3].minAxisValue = -162930;

    	// Joint 4 parameters
    	joints[4].jointID = 5;
    	joints[4].gearRatio = 71;
    	joints[4].minJointValue = -167.5;
    	joints[4].maxJointValue = 167.5;
    	joints[4].negative = true;
    	joints[4].rpm_to_rad = -0.0825;
    	//joints[4].minAxisValue = 131697;

    	/*// Compute max and min joints value
    	  for{int i=0; i<5;i++}{
    	  joints[i].minAxisValue = jointValueToEncoderValue(joints[i].minJointValue, joints[i].jointID);
    	  joints[i].maxAxisValue = jointValueToEncoderValue(joints[i].maxJointValue, joints[i].jointID);
    	  }*/
    	this->getActivity()->setPeriod(0.1);

    	// steer zero velocities (wheels):
    	for(int i=0;i < 4; i++)
    	{
    		youBot.setControllerMode(i,2);  //2: velocity, 1: position, 3: move by hand
    		youBot.setMotorPositionOrSpeed(i, 0);
    	}

    	// set arm in position mode:
    	for(int i=4;i < 9; i++)
    	{
    		youBot.setControllerMode(i,1);  //2: velocity, 1: position, 3: move by hand
    	}

    	int counter = 0;
    	bool trigger = false;

    	youBot.setAxisPosition(1, getAxisAbsolutePosition(1,0));
    	youBot.setAxisPosition(2, getAxisAbsolutePosition(2,0));
    	youBot.setAxisPosition(3, getAxisAbsolutePosition(3,0));
    	youBot.setAxisPosition(4, getAxisAbsolutePosition(4,0));
    	youBot.setAxisPosition(5, getAxisAbsolutePosition(5,0));

    	/* Disable base: */
    	//Mq = Eigen::
    	//setWeightJS(Mq);

        return true;
    }

//    bool movePose(const KDL::Frame& eef) {
//    	target_frame = eef;
//    }

    bool move(int axis, double degrees) {
    	youBot.setControllerMode(axis+3,1);
    	return youBot.setAxisPosition(axis, getAxisAbsolutePosition(axis, degrees)) == 0;
    }

    bool setSpeed(int axis, double radPerSec) {
    	youBot.setControllerMode(axis+3,2);
    	return youBot.setMotorPositionOrSpeed(axis+3, radPerSecToEncoderSpeed(axis, radPerSec)) == 0;
    }

    bool speedTest(int axis, int speed) {
    	double startPos = getJointAbsolutePosition(axis, youBot.getAxisPosition(axis)) / 180.* M_PI;
    	youBot.setControllerMode(axis+3,2);  //2: velocity, 1: position, 3: move by hand
    	youBot.setMotorPositionOrSpeed(axis+3,speed);
    	sleep(1);
    	double endPos = getJointAbsolutePosition(axis, youBot.getAxisPosition(axis)) / 180.* M_PI;
    	youBot.setMotorPositionOrSpeed(axis+3,0);
    	youBot.setControllerMode(axis+3,1);
    	youBot.setAxisPosition(axis, getAxisAbsolutePosition(axis,0));
    	cout << axis << ": " << endPos - startPos << endl;
    }

    bool getJointAngles(){
    	q(0) = 0;
    	q(1) = 0;
    	q(2) = 0;
    	for(int i=3; i < q.rows(); ++i) {
    		q(i) = getJointAbsolutePosition(i-3, youBot.getAxisPosition(i-3)) / 180.* M_PI;
    	}
    }

    bool moveToCartPos(const KDL::Frame& desired_eef, const KDL::Twist& eef_vel){
    	getJointAngles();
    	solver->JntToCart(q,eef); //FK: get end effector pos
    	Twist del_eef = diff(desired_eef, eef);
    	Twist del_vel = diff(eef_vel, del_eef);
        iksolver->CartToJnt(q, del_vel, qdot); //IK: get joint velocities, from current joint angles and desired end effector velocity
        for(int i=3; i < qdot.rows(); ++i) {
        	setSpeed(i-2, qdot(i));
        }

    }

    void closeGripper() {
    	youBot.setGripper(2);
    }

    void openGripper() {
    	youBot.setGripper(1);
    }

    bool startHook() {
        std::cout << "Youbot started !" <<std::endl;

        //

        return true;
    }

    void updateHook() {
        //std::cout << "Youbot executes updateHook !" <<std::endl;
        //moveToCartPos();
    }

    void stopHook() {
        std::cout << "Youbot executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Youbot cleaning up !" <<std::endl;
    }
};

#endif
