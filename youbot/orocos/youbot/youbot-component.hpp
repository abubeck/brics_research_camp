#ifndef OROCOS_YOUBOT_COMPONENT_HPP
#define OROCOS_YOUBOT_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <iostream>
#include <cstdlib>
#include <youBotApi.h>
#include "youbot_chain.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
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
   ChainFkSolverVel_recursive *velsolver;
   ChainIkSolverVel_wdls *iksolver;
   Frame eef, desired_eef;
   JntArrayVel jointVals;
   JntArray qdot_out;
   Eigen::MatrixXd Mq;
   double K;

   //OutputPort<Position> pos_out;
   //InputPort<Position> pos_in;

public:
    Youbot(string const& name)
        : TaskContext(name, PreOperational), youBot("/tmp/youBotMemMapFile", 123456),
          K(0.01)
    {
        std::cout << "Youbot constructed !" <<std::endl;

        addProperty("K", K).doc("P controller gain.");
        addOperation("moveDegrees",&Youbot::moveDegrees, this)
        .doc("move axis to given position").arg("axis","axis 1..5").arg("degrees","degrees");

        addOperation("setSpeed",&Youbot::setSpeed, this)
        .doc("move axis at given speed").arg("axis","axis 1..5").arg("radPerSec","radians");

        addOperation("openGripper",&Youbot::openGripper, this);
        addOperation("closeGripper",&Youbot::closeGripper, this);

        addOperation("getEEF",&Youbot::getEEF, this);
        addOperation("setDesiredEEF",&Youbot::setDesiredEEF, this);

        addOperation("speedTest",&Youbot::speedTest, this).arg("axis","").arg("speed","speed in rpm");

        addProperty("eef",eef).doc("End effector frame");

        chain = getYouBotKinematicChain();
        jointVals.q.resize(8);
        jointVals.qdot.resize(8);
        qdot_out.resize(8);
        solver = new ChainFkSolverPos_recursive(chain);
        velsolver = new ChainFkSolverVel_recursive(chain);
        iksolver = new ChainIkSolverVel_wdls(chain);

        this->configure();
    }

    bool configureHook() {
        std::cout << "Youbot configured !" <<std::endl;

    	this->getActivity()->setPeriod(1);

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

    	//youBot.setAxisPosition(1, youBot.getAxisAbsolutePosition(1,0));
    	//youBot.setAxisPosition(2, youBot.getAxisAbsolutePosition(2,0));
    	//youBot.setAxisPosition(3, youBot.getAxisAbsolutePosition(3,0));
    	//youBot.setAxisPosition(4, youBot.getAxisAbsolutePosition(4,0));
    	//youBot.setAxisPosition(5, youBot.getAxisAbsolutePosition(5,0));

    	/* Disable base: */
    	Mq = Eigen::MatrixXd::Identity(8,8);
    	Mq(0,0) = 0;
    	Mq(1,1) = 0;
    	Mq(2,2) = 0;

    	iksolver->setWeightJS(Mq);

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

    void readJoints(){
    	jointVals.q(0) = 0;
    	jointVals.q(1) = 0;
    	jointVals.q(2) = 0;
    	for(int i=3; i < jointVals.q.rows(); ++i) {
    	}
    	jointVals.qdot(0) = 0;
    	jointVals.qdot(1) = 0;
    	jointVals.qdot(2) = 0;
    	for(int i=3; i < jointVals.qdot.rows(); ++i) {
    		jointVals.qdot(i) = youBot.getArmJointVelocity(i - 2);
    		jointVals.q(i) = youBot.getArmJointPosition(i - 2);
    	}
    }

    Frame getEEF() {
    	readJoints();
    	solver->JntToCart(jointVals.q,eef); //FK: get end effector pos
    	return eef;
    }

    void setDesiredEEF(const KDL::Frame& target_eef) {
    	desired_eef = target_eef;
    }

    bool moveToCartPos() { //(const KDL::Frame& desired_eef ){
    	// measure position and speed:
    	readJoints();
    	// calculate cartesian space position and speed:
    	solver->JntToCart(jointVals.q,eef); //FK: get end effector pos
    	FrameVel framevel;
    	velsolver->JntToCart(jointVals.qdot,framevel);
    	Twist act_vel = framevel.deriv();
    	// desired velocity = (des pos - cur pos) / t
    	Twist des_vel = diff(desired_eef, eef, 1.0);
    	// P controller = K ( desired velocity - actual velocity )
    	Twist out_vel = K * diff(des_vel, act_vel);
        iksolver->CartToJnt(jointVals.q, out_vel, qdot_out); //IK: get joint velocities, from current joint angles and desired end effector velocity
        double sat_correction = 1.0;
        for(int i=3; i < qdot_out.rows(); ++i) {
        	if ( qdot_out(i) > 0.1 ) {
        		if ( qdot_out(i) / 0.1 > sat_correction )
        			sat_correction = qdot_out(i) / 0.1;
        	}
        }
        for(int i=3; i < qdot_out.rows(); ++i) {
        	//setSpeed(i-2, qdot_out(i) / sat_correction );
        	cout << " " << qdot_out(i) / sat_correction;
        }
        cout << endl;
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


    	moveToCartPos();
    }

    void stopHook() {
        std::cout << "Youbot executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Youbot cleaning up !" <<std::endl;
    }
};

#endif
