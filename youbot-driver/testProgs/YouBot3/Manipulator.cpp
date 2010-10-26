/*
 * Manipulator.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Luca Gherardi, Peter Soetens,  Benjamin Rosman
 */

#include "Manipulator.h"
#include <cstdlib>
#include <vector>
//#include <brics_actuator/JointValue.h>
//#include <brics_actuator/JointValues.h>

using namespace std;


Manipulator::Manipulator() {
	// TODO Auto-generated constructor stub
}

Manipulator::~Manipulator() {
	// TODO Auto-generated destructor stub
}

void Manipulator::setJointsConfiguration(JointValues configuration){


	for (std::vector<JointValue>::iterator it = configuration.values.begin();
			it < configuration.values.end(); it ++) {
		setJointAbsoluteValue(*it);
	}

}
