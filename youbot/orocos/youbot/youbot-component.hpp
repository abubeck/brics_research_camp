#ifndef OROCOS_YOUBOT_COMPONENT_HPP
#define OROCOS_YOUBOT_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <iostream>
#include <cstdlib>
#include <youBotApi.h>
#include "youbot_chain.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
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
   JntArray q;
   Frame eef;

public:
    Youbot(string const& name)
        : TaskContext(name, PreOperational), youBot("/tmp/youBotMemMapFile", 123456),
          encoderSteps(4096)
    {
        std::cout << "Youbot constructed !" <<std::endl;

        addOperation("move",&Youbot::setAxisPosition, this)
        .doc("move axis to given position").arg("axis","axis 0..4").arg("degrees","degrees");

        addOperation("openGripper",&Youbot::openGripper, this);
        addOperation("closeGripper",&Youbot::closeGripper, this);

        addProperty("eef",eef);

        chain = getYouBotKinematicChain();
        q.resize(8);
        solver = new ChainFkSolverPos_recursive(chain);

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
    	//joints[0].minAxisValue = -585659;

    	// Joint 1 parameters
    	joints[1].jointID = 1;
    	joints[1].gearRatio = 156;
    	joints[1].minJointValue = -65;
    	joints[1].maxJointValue = 90;
    	joints[1].negative = true;
    	//joints[1].minAxisValue = -268741;

    	// Joint 2 parameters
    	joints[2].jointID = 2;
    	joints[2].gearRatio = 100;
    	joints[2].minJointValue = -151;
    	joints[2].maxJointValue = 146;
    	joints[2].negative = false;
    	//joints[2].minAxisValue = -325596;

    	// Joint 3 parameters
    	joints[3].jointID = 3;
    	joints[3].gearRatio = 71;
    	joints[3].minJointValue = -102.5;
    	joints[3].maxJointValue = 102.5;
    	joints[3].negative = true;
    	//joints[3].minAxisValue = -162930;

    	// Joint 4 parameters
    	joints[4].jointID = 4;
    	joints[4].gearRatio = 71;
    	joints[4].minJointValue = -167.5;
    	joints[4].maxJointValue = 167.5;
    	joints[4].negative = true;
    	//joints[4].minAxisValue = 131697;

    	/*// Compute max and min joints value
    	  for{int i=0; i<5;i++}{
    	  joints[i].minAxisValue = jointValueToEncoderValue(joints[i].minJointValue, joints[i].jointID);
    	  joints[i].maxAxisValue = jointValueToEncoderValue(joints[i].maxJointValue, joints[i].jointID);
    	  }*/


    	// steer zero velocities:
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

        return true;
    }

    bool setAxisPosition(int axis, double degrees) {
    	return youBot.setAxisPosition(axis, getAxisAbsolutePosition(degrees,axis)) == 0;
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
        this->getActivity()->setPeriod(0.1);
        return true;
    }

    void updateHook() {
        //std::cout << "Youbot executes updateHook !" <<std::endl;
        q(0) = 0;
        q(1) = 0;
        q(2) = 0;
        for(int i=3; i < q.rows(); ++i) {
        	q(i) = getJointAbsolutePosition(youBot.getAxisPosition(i-3), i-3) / 180.* M_PI;
        	cout << " (" << q(i) * 180 / M_PI <<", "<< youBot.getAxisPosition(i-3) << ") ";
        }
        cout << endl;
        solver->JntToCart(q,eef);
    }

    void stopHook() {
        std::cout << "Youbot executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Youbot cleaning up !" <<std::endl;
    }
};

#endif
