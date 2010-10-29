#ifndef OROCOS_YOUBOT_JOINTSPACE_COMPONENT_HPP
#define OROCOS_YOUBOT_JOINTSPACE_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>


using namespace std;
using namespace RTT;

namespace youbot {

class JointSpaceComponent
    : public RTT::TaskContext
{
   OutputPort<std::vector<double> > qdotPort_out;
   OutputPort<std::vector<double> > qPort_out;

   InputPort<std::vector<double> > qdotPort_in;
   InputPort<std::vector<double> > qPort_in;

   std::vector<double> currentpos, currentvel;
public:
    JointSpaceComponent(string const& name)
        : TaskContext(name, PreOperational)
    {
        addOperation("moveDegrees",&JointSpaceComponent::moveDegrees, this)
        .doc("move axis to given position").arg("axis","axis 1..5").arg("degrees","degrees");

        addOperation("setSpeed",&JointSpaceComponent::setSpeed, this)
        .doc("move axis at given speed").arg("axis","axis 1..5").arg("radPerSec","radians");

        addOperation("speedTest",&JointSpaceComponent::speedTest, this).arg("axis","").arg("speed","speed in rpm");

        addPort("qPort_out", qPort_out);
        addPort("qdotPort_out", qdotPort_out);

        addPort("qPort_in", qPort_in);
        addPort("qdotPort_in", qdotPort_in);

        this->configure();
    }

    bool configureHook() {
    	this->getActivity()->setPeriod(0.01);

    	currentpos.resize(8);
    	currentvel.resize(8);
        return true;
    }

    bool moveDegrees(int axis, double degrees) {
    	if (this->isRunning()) {
    		currentpos[axis+2] = degrees / 180.*M_PI;
        	qPort_out.write(currentpos);
        	return true;
    	}
    	return false;
    }

    bool setSpeed(int axis, double radPerSec) {
    	if (this->isRunning()) {
    		currentvel[axis+2] = radPerSec;
        	qdotPort_out.write(currentvel);
        	return true;
    	}
    	return false;
    }

    bool speedTest(int axis, int speed) {
    	if ( !this->isRunning())
    		return false;
#if 0
    	double startPos = youBot.getJointAbsolutePosition(axis, youBot.getAxisPosition(axis));
    	youBot.setControllerMode(axis+3,2);  //2: velocity, 1: position, 3: move by hand
    	youBot.setMotorPositionOrSpeed(axis+3,speed);
    	sleep(1);
    	double endPos = youBot.getJointAbsolutePosition(axis, youBot.getAxisPosition(axis));
    	youBot.setMotorPositionOrSpeed(axis+3,0);
    	youBot.setControllerMode(axis+3,1);
    	youBot.setAxisPosition(axis, youBot.getAxisAbsolutePosition(axis,0));
    	cout << axis << ": rad/s: " << endPos - startPos << endl;
#endif
    	cout << "Not implemented anymore." <<endl;
    	return false;
    }

    bool startHook() {
        std::cout << "Youbot started !" <<std::endl;

        qPort_in.read(currentpos);
        currentvel = vector<double>(8,0);

        qdotPort_out.write( currentvel );

        return true;
    }

    void updateHook() {
    }

    void stopHook() {
    }

    void cleanupHook() {
    }
};

}

#endif
