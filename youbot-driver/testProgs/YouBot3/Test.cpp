/*
 * Test.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: luca
 */

#include "Test.h"
#include <cstdlib>
#include "YouBotArm.h"

YouBotArm arm;

Test::Test() {
	// TODO Auto-generated constructor stub

}

Test::~Test() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {

	arm.setJointAbsoluteValue(0,0);
	arm.setJointAbsoluteValue(1,0);
	arm.setJointAbsoluteValue(2,0);
	arm.setJointAbsoluteValue(3,0);
	arm.setJointAbsoluteValue(4,0);

	return 0;
}
