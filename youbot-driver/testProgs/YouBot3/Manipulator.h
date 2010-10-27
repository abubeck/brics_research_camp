/*
 * Manipulator.h
 *
 *  Created on: Oct 25, 2010
 *      Author: Luca Gherardi, Peter Soetens,  Benjamin Rosman
 */

#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <cstdlib>
#include <stdlib.h>
#include <vector>
#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointValues.h"

using namespace brics_actuator;


typedef struct{

	int jointID; // Joint ID
	int gearRatio; // Joint gear ratio: number of motor
	double minJointValue; // Min angle value of the joint
	double maxJointValue; // Max angle value of the joint
	double currentValue; // Current angle value of the joint
	bool negative;  // true if in the homing position the joint value is the minJointValue

}Joint;

typedef struct{
	double x;
	double y;
	double z;
	double phi;
	double psi;
	double theta;
}Position;

typedef std::vector<double> Configuration;


class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();
	//! Set the joint value
			//! input:
			//!		value:	the joint value
			//!		jointID:	the ID of the joint
			//! return:
			//!		the joint angle value
	virtual void setJointAbsoluteValue( JointValue joint)=0;
	//! Set all the joints values
		//! input:
		//!		config:	the joints configuration
	virtual void setJointsConfiguration(JointValues configuration);
	//! Move the end effector to the input cartesian position
		//! input:
		//!		pos:	the cartesian position
	virtual void setEndEffectorCartesianPosition(Position pos)=0;
};



#endif /* MANIPULATOR_H_ */
