/*
 * YouBotArm.h
 *
 *  Created on: Oct 25, 2010
 *      Author: luca
 */

#ifndef YOUBOTARM_H_
#define YOUBOTARM_H_

#include "Manipulator.h"
#include "youBotApi.h"

class YouBotArm: public Manipulator {

public:


	YouBotArm();
	YouBotArm(int semaphoreKey);
	virtual ~YouBotArm();
	void setJointAbsoluteValue(JointValue joint);
	//! Move the arm to the homing position
	void setHomingPosition();
	void setEndEffectorCartesianPosition(Position pos);

private:

	void setJointsParameters();
	void setJointValue(int jointID, double value);
	double encoderStepsToJointValue(int jointID, int encoderSteps);
	int jointValueToEncoderSteps(int jointID, double jointValue);
	double getJointAbsolutePosition(int jointID, int axisPosition);
	int getAxisAbsolutePosition(int jointID, double jointPosition);

};

#endif /* YOUBOTARM_H_ */
