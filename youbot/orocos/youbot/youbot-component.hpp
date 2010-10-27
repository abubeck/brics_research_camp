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

class Youbot
    : public RTT::TaskContext
{
   YouBotApi youBot;

   Chain chain;
   ChainFkSolverPos_recursive *solver;
   ChainIkSolverVel_wdls *iksolver;
   JntArray q;
   Frame eef;
   JntArray qdot;
   Eigen::MatrixXd Mq;

public:
    Youbot(string const& name)
        : TaskContext(name, PreOperational), youBot("/tmp/youBotMemMapFile", 123456)
    {
        std::cout << "Youbot constructed !" <<std::endl;

        addOperation("moveDegrees",&Youbot::moveDegrees, this)
        .doc("move axis to given position").arg("axis","axis 1..5").arg("degrees","degrees");

        addOperation("setSpeed",&Youbot::setSpeed, this)
        .doc("move axis at given speed").arg("axis","axis 1..5").arg("radPerSec","radians");

        addOperation("openGripper",&Youbot::openGripper, this);
        addOperation("closeGripper",&Youbot::closeGripper, this);

        addOperation("speedTest",&Youbot::speedTest, this).arg("axis","").arg("speed","speed in rpm");

        addProperty("eef",eef).doc("End effector frame");

        chain = getYouBotKinematicChain();
        q.resize(8);
        solver = new ChainFkSolverPos_recursive(chain);
        iksolver = new ChainIkSolverVel_wdls(chain);

        this->configure();
    }

    bool configureHook() {
        std::cout << "Youbot configured !" <<std::endl;

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

    	youBot.setAxisPosition(1, youBot.getAxisAbsolutePosition(1,0));
    	youBot.setAxisPosition(2, youBot.getAxisAbsolutePosition(2,0));
    	youBot.setAxisPosition(3, youBot.getAxisAbsolutePosition(3,0));
    	youBot.setAxisPosition(4, youBot.getAxisAbsolutePosition(4,0));
    	youBot.setAxisPosition(5, youBot.getAxisAbsolutePosition(5,0));

    	/* Disable base: */
    	//Mq = Eigen::
    	//setWeightJS(Mq);

        return true;
    }

//    bool movePose(const KDL::Frame& eef) {
//    	target_frame = eef;
//    }

    bool moveDegrees(int axis, double degrees) {
    	return youBot.setArmJointPosition(axis, degrees / 180.*M_PI) == 0;
    }

    bool setSpeed(int axis, double radPerSec) {
    	return youBot.setArmJointVelocity(axis, radPerSec) == 0;
    }

    bool speedTest(int axis, int speed) {
    	double startPos = youBot.getJointAbsolutePosition(axis, youBot.getAxisPosition(axis));
    	youBot.setControllerMode(axis+3,2);  //2: velocity, 1: position, 3: move by hand
    	youBot.setMotorPositionOrSpeed(axis+3,speed);
    	sleep(1);
    	double endPos = youBot.getJointAbsolutePosition(axis, youBot.getAxisPosition(axis));
    	youBot.setMotorPositionOrSpeed(axis+3,0);
    	youBot.setControllerMode(axis+3,1);
    	youBot.setAxisPosition(axis, youBot.getAxisAbsolutePosition(axis,0));
    	cout << axis << ": rad/s: " << endPos - startPos << endl;
    }

    bool getJointAngles(){
    	q(0) = 0;
    	q(1) = 0;
    	q(2) = 0;
    	for(int i=3; i < q.rows(); ++i) {
    		q(i) = youBot.getJointAbsolutePosition(i-3, youBot.getAxisPosition(i-3)) / 180.* M_PI;
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
