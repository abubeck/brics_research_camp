#ifndef OROCOS_YOUBOT_COMPONENT_HPP
#define OROCOS_YOUBOT_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <cstdlib>
#include <youBotApi.h>
#include <cmath>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>


using namespace std;
using namespace youbot;
using namespace RTT;

class Youbot
    : public RTT::TaskContext
{
   YouBotApi youBot;
   OutputPort<std::vector<double> > qdotPort;
   OutputPort<std::vector<double> > qPort;

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

        addPort("qPort", qPort);
        addPort("qdotPort", qdotPort);

        this->configure();
    }

    bool configureHook() {
        std::cout << "Youbot configured !" <<std::endl;

    	this->getActivity()->setPeriod(0.01);

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

        return true;
    }

    bool moveDegrees(int axis, double degrees) {
    	q(axis) += degrees / 180.*M_PI;
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
    	readJoints();
    	q_out = jointVals.q;
    }

    bool moveToCartPos() { //(const KDL::Frame& desired_eef ){
    	if (getPeriod() == 0) {
    		log(Error) <<"Can't do move without a period." << endlog();
    		return false;
    	}
    	// measure position and speed:
    	readJoints();
    	// calculate cartesian space position and speed:
    	solver->JntToCart(jointVals.q,eef); //FK: get end effector pos
    	FrameVel framevel;
    	//velsolver->JntToCart(jointVals,framevel);
    	//Twist act_vel = framevel.deriv();
//    	cout << " actvel: "<< act_vel;
    	// desired velocity = (des pos - cur pos) / t
    	// diff is (arg2 - arg1)
    	Twist des_vel = diff(eef, desired_eef, 1.0);
    	cout << " desvel: "<< des_vel;
    	// P controller = K ( desired velocity - actual velocity )
    	Twist out_vel = K * des_vel;
    	cout << " outvel: "<< out_vel << endl;
    	int ret = 0;
        if ( (ret = iksolver->CartToJnt(jointVals.q, out_vel, qdot_out)) < 0) {
        	cout << "IK Error: "<< ret <<endl; //IK: get joint velocities, from current joint angles and desired end effector velocity
        	return false;
        }
        JntArrayVel testjntvel(jointVals.q,qdot_out);
        velsolver->JntToCart(testjntvel,framevel);
    	cout << " outvel2: "<< framevel.deriv() <<endl;
        cout << "qdot:";
        for(int i=0; i < qdot_out.rows(); ++i) {
        	cout << " " << qdot_out(i);
        }
        cout << endl;
        qdotPort.write(qdot_out);
        qPort.write(jointVals.q);
        double sat_correction = 1.0;
        for(int i=3; i < qdot_out.rows(); ++i) {
//        	cout << " " << qdot_out(i) / sat_correction;
        	// check for nan:
        	if (!isnormal(qdot_out(i)) )
        		qdot_out(i) = 0;
        	if ( qdot_out(i) > 0.1 || qdot_out(i) < -0.1 ) {
        		if ( fabs( qdot_out(i) ) / 0.1 > sat_correction ) {
        			sat_correction = fabs(qdot_out(i)) / 0.1;
        			cout <<"Warning: saturating joint " << i <<endl;
        		}
        	}
        }
        for(int i=3; i < qdot_out.rows(); ++i) {
        	q_out(i) += qdot_out(i) / sat_correction * getPeriod();
        	//setSpeed(i-2, qdot_out(i) / sat_correction );
        	youBot.setArmJointPosition(i - 2, q_out(i));
        }
        cout << "qdotsat:";
        for(int i=0; i < qdot_out.rows(); ++i) {
        	cout << " " << qdot_out(i) / sat_correction;
        }
        cout << endl;
        return true;
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
    	// set arm in position mode:
    	for(int i=4;i < 9; i++)
    	{
    		youBot.setControllerMode(i,1);  //2: velocity, 1: position, 3: move by hand
    	}
    }

    void cleanupHook() {
        std::cout << "Youbot cleaning up !" <<std::endl;
    }
};

#endif
