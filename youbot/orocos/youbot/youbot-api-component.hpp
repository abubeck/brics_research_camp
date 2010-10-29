#ifndef OROCOS_YOUBOT_API_COMPONENT_HPP
#define OROCOS_YOUBOT_API_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <cstdlib>
#include <youBotApi.h>
#include <cmath>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>


using namespace std;
using namespace youbot;
using namespace RTT;
using namespace KDL;
using namespace sensor_msgs;

class YouBotAPIComponent
    : public RTT::TaskContext
{
   YouBotApi youBot;
   OutputPort<std::vector<double> > qdotVectorPort_out;
   OutputPort<std::vector<double> > qVectorPort_out;

   OutputPort< JointState > jointStatePort_out;
   OutputPort< JntArrayVel > jointArrayPort_out;
   
   InputPort<std::vector<double> > qdotVectorPort_in;
   InputPort<std::vector<double> > qVectorPort_in;

   InputPort< JointState > jointStatePort_in;
   InputPort< JntArray > jointArrayPort_in;
   
   JntArray jointArray;
   JntArrayVel jointArrayVel;
   JointState  jointState;
   std::vector<double> qvector;
   std::vector<double> qdotvector;
   std::vector<double> qdesvector;

   std::vector<double> qintvector,qdotmeas;

   double K;
   bool inposition;
public:
    YouBotAPIComponent(string const& name)
        : TaskContext(name, PreOperational), youBot("/tmp/youBotMemMapFile", 123456), K(0.5), inposition(true)
    {
        std::cout << "Youbot constructed !" <<std::endl;

        //Outputs:
        addPort("qVectorPort_out", qVectorPort_out);
        addPort("qdotVectorPort_out", qdotVectorPort_out);

        addPort("jointStatePort_out", jointStatePort_out);
        addPort("jointArrayPort_out", jointArrayPort_out);

        // Inputs:
        addPort("qVectorPort_in", qVectorPort_in);
        addPort("qdotVectorPort_in", qdotVectorPort_in);

        addPort("jointStatePort_in", jointStatePort_in);
        addPort("jointArrayPort_in", jointArrayPort_in);

        addOperation("openGripper",&YouBotAPIComponent::openGripper, this);
        addOperation("closeGripper",&YouBotAPIComponent::closeGripper, this);

        addProperty("K",K);

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

    	// Go to joint null positions
    	youBot.setAxisPosition(1, youBot.getAxisAbsolutePosition(1,0));
    	youBot.setAxisPosition(2, youBot.getAxisAbsolutePosition(2,0));
    	youBot.setAxisPosition(3, youBot.getAxisAbsolutePosition(3,0));
    	youBot.setAxisPosition(4, youBot.getAxisAbsolutePosition(4,0));
    	youBot.setAxisPosition(5, youBot.getAxisAbsolutePosition(5,0));

    	// Init data values - ROS
		jointState.name.resize(8);
		jointState.name[ 0 ] = "BaseX";
		jointState.name[ 1 ] = "BaseY";
		jointState.name[ 2 ] = "BaseTheta";

		for (int i=3; i < 8; ++i) {
			stringstream ss;
			ss << "Joint" << i-2;
			jointState.name[i] = ss.str();
		}
		jointState.position.resize(8, 0);
		jointState.velocity.resize(8, 0);
    	qvector.resize(8,0);
    	qdotvector.resize(8,0);
    	qintvector.resize(8,0);
    	qdesvector.resize(8,0);
    	qdotmeas.resize(8,0);

    	jointArrayVel.q.resize(8);
    	jointArrayVel.qdot.resize(8);
        return true;
    }

    void readJoints(){
    	timeval time;
    	youBot.getBasePositionCartesian(jointArrayVel.q(0), jointArrayVel.q(1), jointArrayVel.q(2),time);
    	youBot.getBaseVelocitiesCartesian(jointArrayVel.qdot(0), jointArrayVel.qdot(1), jointArrayVel.qdot(2),time);
    	for(unsigned int i=3; i < jointArrayVel.qdot.rows(); ++i) {
    		jointArrayVel.qdot(i) = youBot.getArmJointVelocity(i - 2);
    		jointArrayVel.q(i) = youBot.getArmJointPosition(i - 2);
    		qdotmeas[i] = jointArrayVel.qdot(i);// first slave of robot arm = 4
    	}
    	for(unsigned int i=0; i < jointArrayVel.q.rows(); ++i) {
    		qvector[i] = jointState.position[i] = jointArrayVel.q(i);
    		qdotvector[i] = jointState.velocity[i] = jointArrayVel.qdot(i);
    	}
    }


    void closeGripper() {
    	youBot.setGripper(2);
    }

    void openGripper() {
    	youBot.setGripper(1);
    }

    bool startHook() {
        std::cout << "Youbot API started !" <<std::endl;
        readJoints();
        qintvector = qvector;
        return true;
    }

    void updateHook() {

    	readJoints();
    	// publish all:
    	qdotVectorPort_out.write( qdotvector );
    	qVectorPort_out.write( qvector );

    	jointStatePort_out.write( jointState );
    	jointArrayPort_out.write( jointArrayVel );

    	// read ports:
    	// velocity vector
    	if ( qdotVectorPort_in.read( qdesvector ) == NewData ) {
    		youBot.setBaseVelocitiesCartesian(qdesvector[0],qdesvector[1],qdesvector[2]);
    		for(int i=3; i < 8; ++i) {
    			double vout = qdesvector[i] + K * (qdesvector[i] - 0.1 * qdotmeas[i] );
    			//cout << "vout = " << vout;
    			youBot.setArmJointVelocity(i-2, vout); //-> does not work properly in firmware
    			// emulate it here:
    			//qintvector[i] += qdesvector[i] * getPeriod();
    			//youBot.setArmJointPosition(i-2, qintvector[i]);
    		}
    		//cout << endl;
    		inposition = false;
    		return;
    	}

    	// position vector
    	if ( qVectorPort_in.read( qvector ) == NewData ) {
    		cout << "Moving to position ..."<<endl;
    		//youBot.setBasePositionCartesian(qvector[0],qvector[1],qvector[2]);
			qintvector = qvector;
			qdesvector[0] = qdesvector[1] = qdesvector[2] = 0;
    		for(int i=3; i < 8; ++i) {
    			youBot.setArmJointPosition(i-2, qvector[i]);
    			qdesvector[i] = 0;
    		}
    		inposition = true;
    		return;
    	}

    	// by default, if no new data comes, continue the interpolation:
		for(int i=3; i < 8; ++i) {
			if ( ! inposition ) {
				double vout = qdesvector[i] + K * (qdesvector[i] - 0.1 * qdotmeas[i] );
				//cout << " vout = " << vout;
				youBot.setArmJointVelocity(i-2, vout); //-> does not work properly in firmware
				//qintvector[i] += qdesvector[i] * getPeriod();
				//youBot.setArmJointPosition(i-2, qintvector[i]);
			}
		}
		//cout << endl;

#if 0
    	//JntVel
    	if ( jointVelPort_in.read( jointArray ) == NewData ) {
    		youBot.setBaseVelocitiesCartesian(jointArray.qdot(0),jointArray.qdot(1),jointArray.qdot(2));
    		for(int i=3; i < 8; ++i) {
    			youBot.setArmJointVelocity(i-2, jointArray[i]);
    		}
    		return;
    	}
#endif
    }

    void stopHook() {
        std::cout << "Youbot executes stopping !" <<std::endl;
    	// set arm in position mode:
    	for(int i=4;i < 9; i++)
    	{
    		youBot.setArmJointPosition(i, qvector[i]);  //2: velocity, 1: position, 3: move by hand

    	}
    }

    void cleanupHook() {
        std::cout << "Youbot cleaning up !" <<std::endl;
    }
};

#endif
