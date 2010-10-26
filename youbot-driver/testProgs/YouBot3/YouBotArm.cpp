/*
 * YouBotArm.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Luca Gherardi, Peter Soetens, Ben
 */

#include "YouBotArm.h"
#include <math.h>

using namespace youbot;

// Number of joints
const int jointNumber = 5;
// Number of steps per 2PI
const int encoderResolution = 4096; //number of steps per 2PI
// Array of joints
Joint joints[encoderResolution];

int semaphoreKey = 12345;

YouBotApi* youBot = new YouBotApi("/tmp/youBotMemMapFile", semaphoreKey);

YouBotArm::YouBotArm() {

	setJointsParameters();
	for(int i=0;i < 4; i++)
	{
		youBot->setControllerMode(i,2);  //2: velocity, 1: position, 3: move by hand
		youBot->setMotorPositionOrSpeed(i, 0);
	}

}

YouBotArm::~YouBotArm() {
	// TODO Auto-generated destructor stub
}

void YouBotArm::setJointAbsoluteValue(int jointID, double value){
	joints[jointID].currentValue = value;
	printf("Set value: %f to joint %d \n", value, jointID );
	youBot->setAxisPosition(jointID, getAxisAbsolutePosition(jointID,value));
}

void YouBotArm::setHomingPosition(){
	Configuration home;
	for(int i=0; i<jointNumber; i++){
		if(joints[i].negative)
			home.push_back(joints[i].minJointValue);
		else
			home.push_back(joints[i].maxJointValue);
	}
	setJointsConfiguration(home);
}

void YouBotArm::setEndEffectorCartesianPosition(Position pos){

}

void YouBotArm::setJointsParameters(){

	// Joint 0 parameters
	joints[0].jointID = 0;
	joints[0].gearRatio = 156;
	joints[0].minJointValue = -169/180*M_PI;
	joints[0].maxJointValue = 169/180*M_PI;
	joints[0].negative = true;
	//joints[0].minAxisValue = -585659;

	// Joint 1 parameters
	joints[1].jointID = 1;
	joints[1].gearRatio = 156;
	joints[1].minJointValue = -65/180*M_PI;
	joints[1].maxJointValue = 90/180*M_PI;
	joints[1].negative = true;
	//joints[1].minAxisValue = -268741;

	// Joint 2 parameters
	joints[2].jointID = 2;
	joints[2].gearRatio = 100;
	joints[2].minJointValue = -151/180*M_PI;
	joints[2].maxJointValue = 146/180*M_PI;
	joints[2].negative = false;
	//joints[2].minAxisValue = -325596;

	// Joint 3 parameters
	joints[3].jointID = 3;
	joints[3].gearRatio = 71;
	joints[3].minJointValue = -102.5/180*M_PI;
	joints[3].maxJointValue = 102.5/180*M_PI;
	joints[3].negative = true;
	//joints[3].minAxisValue = -162930;

	// Joint 4 parameters
	joints[4].jointID = 4;
	joints[4].gearRatio = 71;
	joints[4].minJointValue = -167.5/180*M_PI;
	joints[4].maxJointValue = 167.5/180*M_PI;
	joints[4].negative = true;
	//joints[4].minAxisValue = 131697;

}

//! Convert from angle value to encoder steps number
//! input:
//!		encoderValue:	the encoder relative value
//!		jointID:	the ID of the joint
//! return:
//!		the angle relative value
double YouBotArm::encoderStepsToJointValue(int jointID, int encoderSteps){
	// Angle in radiants:: TO BE TESTED
	return encoderSteps/(joints[jointID].gearRatio*encoderResolution)*2*M_PI;
	//return encoderSteps/(joints[jointID].gearRatio*encoderResolution)*360;
}

//! Convert from encoder steps to angle value
//! input:
//!		angleValue:	the angle relative value
//!		jointID:	the ID of the joint
//! return:
//!		the encoder relative value
int YouBotArm::jointValueToEncoderSteps(int jointID, double jointValue){

	// Angle in radiants:: TO BE TESTED
	return jointValue/(2*M_PI)(joints[jointID].gearRatio*encoderResolution);
	//return jointValue/360*(joints[jointID].gearRatio*encoderResolution);
}


//! Compute the joint value position corresponding to the input axis absolute position
//! input:
//!		axisPosition:	the axis absolute position
//!		jointID:	the ID of the joint
//! return:
//!		the joint angle value
double YouBotArm::getJointAbsolutePosition(int jointID, int axisPosition){
	double value = encoderStepsToJointValue(jointID, axisPosition);
	if(joints[jointID].negative)
		return joints[jointID].minJointValue + value;
	else
		return value - joints[jointID].maxJointValue;
}

//! Compute the axis absolute position corresponding to the input joint value
//! input:
//!		joinValue:	the joint value
//!		jointID:	the ID of the joint
//! return:
//!		the axis absolute position
int YouBotArm::getAxisAbsolutePosition(int jointID, double jointPosition){
	if(joints[jointID].negative)
		return jointValueToEncoderSteps(jointID, -jointPosition + joints[jointID].minJointValue);
	else
		return jointValueToEncoderSteps(jointID, jointPosition - joints[jointID].maxJointValue);
}
