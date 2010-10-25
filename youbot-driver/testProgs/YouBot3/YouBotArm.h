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
	virtual ~YouBotArm();
	int setJointAbsoluteValue(int jointID, double value);

private:

	void setJointsParameters();
	double encoderStepsToJointValue(int jointID, int encoderSteps);
	int jointValueToEncoderSteps(int jointID, double jointValue);
	double getJointAbsolutePosition(int jointID, int axisPosition);
	int getAxisAbsolutePosition(int jointID, double jointPosition);

};

#endif /* YOUBOTARM_H_ */
