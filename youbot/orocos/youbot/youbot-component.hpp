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
   double encoderValueToJointValue(int encoderValue, int jointID){
       return double(encoderValue)/(joints[jointID].gearRatio*encoderSteps)*360;
   }
   // Convert from encoder value to joint relative value
   int jointValueToEncoderValue(double jointValue, int jointID){
       return jointValue/360*(joints[jointID].gearRatio*encoderSteps);
   }

   //Convert from encoder speeds to joint speeds in radians per second
   double encoderSpeedToRadPerSec(double encoderSpeed, int jointID){
	   return encoderSpeed*joints[jointID].rpm_to_rad;
   }

   //Convert from joint speeds in radians per second to encoder speeds
   double radPerSecToEncoderSpeed(double radPerSec, int jointID){
   	   return radPerSec/joints[jointID].rpm_to_rad;
   }

   // Convert from Joint value to axis absolute value
   int getAxisAbsolutePosition(double jointPosition, int jointID){
     if(joints[jointID].negative)
         return jointValueToEncoderValue(-jointPosition + joints[jointID].minJointValue,jointID);
       else
         return jointValueToEncoderValue(jointPosition - joints[jointID].maxJointValue,jointID);
   }

   // Convert from Joint value to axis absolute value
   double getJointAbsolutePosition(int axisPosition, int jointID){
       double value = encoderValueToJointValue(axisPosition,jointID);
       if(joints[jointID].negative)
         return joints[jointID].minJointValue - value;
       else
         return value + joints[jointID].maxJointValue;
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
        .doc("move axis to given position").arg("axis","axis 0..4").arg("degrees","degrees");

        addOperation("setSpeed",&Youbot::setSpeed, this)
        .doc("move axis at given speed").arg("axis","axis 0..4").arg("radPerSec","radians");

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
    	joints[0].jointID = 0;
    	joints[0].gearRatio = 156;
    	joints[0].minJointValue = -169;
    	joints[0].maxJointValue = 169;
    	joints[0].negative = true;
    	joints[0].rpm_to_rad = -0.0783;
    	//joints[0].minAxisValue = -585659;

    	// Joint 1 parameters
    	joints[1].jointID = 1;
    	joints[1].gearRatio = 156;
    	joints[1].minJointValue = -65;
    	joints[1].maxJointValue = 90;
    	joints[1].negative = true;
    	joints[1].rpm_to_rad = -0.0807;
    	//joints[1].minAxisValue = -268741;

    	// Joint 2 parameters
    	joints[2].jointID = 2;
    	joints[2].gearRatio = 100;
    	joints[2].minJointValue = -151;
    	joints[2].maxJointValue = 146;
    	joints[2].negative = false;
    	joints[2].rpm_to_rad = 0.0863;
    	//joints[2].minAxisValue = -325596;

    	// Joint 3 parameters
    	joints[3].jointID = 3;
    	joints[3].gearRatio = 71;
    	joints[3].minJointValue = -102.5;
    	joints[3].maxJointValue = 102.5;
    	joints[3].negative = true;
    	joints[3].rpm_to_rad = -0.0863;
    	//joints[3].minAxisValue = -162930;

    	// Joint 4 parameters
    	joints[4].jointID = 4;
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

    	int counter = 0;
    	bool trigger = false;

    	youBot.setAxisPosition(0, getAxisAbsolutePosition(0,0));
    	youBot.setAxisPosition(1, getAxisAbsolutePosition(0,1));
    	youBot.setAxisPosition(2, getAxisAbsolutePosition(0,2));
    	youBot.setAxisPosition(3, getAxisAbsolutePosition(0,3));
    	youBot.setAxisPosition(4, getAxisAbsolutePosition(0,4));

    	/* Disable base: */
    	//Mq = Eigen::
    	//setWeightJS(Mq);

        return true;
    }

//    bool movePose(const KDL::Frame& eef) {
//    	target_frame = eef;
//    }

    bool move(int axis, double degrees) {
    	youBot.setControllerMode(axis+4,1);
    	return youBot.setAxisPosition(axis, getAxisAbsolutePosition(degrees,axis)) == 0;
    }

    bool setSpeed(int axis, double radPerSec) {
    	youBot.setControllerMode(axis+4,2);
    	return youBot.setMotorPositionOrSpeed(axis+4, radPerSecToEncoderSpeed(axis, radPerSec)) == 0;
    }

    bool speedTest(int axis, int speed) {
    	double startPos = getJointAbsolutePosition(youBot.getAxisPosition(axis), axis) / 180.* M_PI;
    	youBot.setControllerMode(axis+4,2);  //2: velocity, 1: position, 3: move by hand
    	youBot.setMotorPositionOrSpeed(axis+4,speed);
    	sleep(10);
    	double endPos = getJointAbsolutePosition(youBot.getAxisPosition(axis), axis) / 180.* M_PI;
    	youBot.setMotorPositionOrSpeed(axis+4,0);
    	youBot.setControllerMode(axis+4,1);
    	youBot.setAxisPosition(axis, getAxisAbsolutePosition(0,axis));
    	cout << axis << ": " << endPos - startPos << endl;
    }

    bool getJointAngles(){
    	q(0) = 0;
    	q(1) = 0;
    	q(2) = 0;
    	for(int i=3; i < q.rows(); ++i) {
    		q(i) = getJointAbsolutePosition(youBot.getAxisPosition(i-3), i-3) / 180.* M_PI;
    	}
    }

    bool moveToCartPos(const KDL::Frame& desired_eef, const KDL::Twist& eef_vel){
    	getJointAngles();
    	solver->JntToCart(q,eef); //FK: get end effector pos
    	Twist del_eef = diff(desired_eef, eef);
    	Twist del_vel = diff(eef_vel, del_eef);
        iksolver->CartToJnt(q, del_vel, qdot); //IK: get joint velocities, from current joint angles and desired end effector velocity
        for(int i=3; i < qdot.rows(); ++i) {
        	setSpeed(i, qdot(i));
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


#if 0
        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        for(int i=3; i < q.rows(); ++i) {
        	q(i) = getJointAbsolutePosition(youBot.getAxisPosition(i-3), i-3) / 180.* M_PI;
        	cout << " (" << q(i) * 180 / M_PI <<", "<< youBot.getAxisPosition(i-3) << ") ";
        }
        cout << endl;
        solver->JntToCart(q,eef); //FK: get end effector pos
#endif

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
