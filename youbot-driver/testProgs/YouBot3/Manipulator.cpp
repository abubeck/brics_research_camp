/*
 * Manipulator.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: luca
 */

#include "Manipulator.h"
#include <cstdlib>
#include <vector>

using namespace std;


Manipulator::Manipulator() {
	// TODO Auto-generated constructor stub
}

Manipulator::~Manipulator() {
	// TODO Auto-generated destructor stub
}

void Manipulator::setJointsConfiguration(Configuration config){

	for (unsigned int i = 0; i < config.size(); ++i) {
		setJointAbsoluteValue(config.at(i),i);
	}

}
