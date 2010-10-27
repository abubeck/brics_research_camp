/* 
 * File:   main.cpp
 * Author: Özgür Sen
 *
 * Created on 24. Oktober 2010, 17:32
 */

#include <cstdlib>
#include <youBotApi.h>

using namespace std;
using namespace youbot;

YouBotApi* youBot;

void driveTo(double x, double y, double theta) {
	double maxTransSpeed = 0.1, maxRotSpeed = 0.1;

	while (true) {
		double curX, curY, curTheta;
		youBot->getBasePositionCartesian(curX, curY, curTheta);
		double distX = x - curX, distY = y - curY, distTheta = theta - curTheta;

		double time = max( fabs(distX / maxTransSpeed), max(fabs(distY / maxTransSpeed), fabs(distTheta / maxRotSpeed)));
		double velWorldX = distX / time, velWorldY = distY / time, velTheta = -distTheta / time;
		double velX = velWorldX * cos(-theta) + velWorldY * sin(-theta);
		double velY = -velWorldY * sin(-theta) + velWorldY * cos(-theta);
		timeval timestamp;
		double vx, vy, vtheta;
		youBot->getBaseVelocitiesCartesian(vx, vy, vtheta, timestamp);
		printf("x: %f, y: %f, theta: %f, remaining time: %f\n", curX, curY, curTheta, time);
//		printf("velX: %f, velY: %f, velTheta: %f, timestamp: %ld %ld\n", velX, velY, velTheta, timestamp.tv_sec, timestamp.tv_usec);
		youBot->setBaseVelocitiesCartesian(velX, velY, velTheta);
		usleep(4000);
		if(time < 0.01) break;
	}
	youBot->setBaseVelocitiesCartesian(0, 0, 0);
}

int main(int argc, char** argv) {

	//youBotApi
	int semaphoreKey = 222;

	if (argc != 1) {
		semaphoreKey = atoi(argv[1]);
	}

	youBot = new YouBotApi("/tmp/youBotMemMapFile", semaphoreKey);
	//	youBot->setBaseVelocitiesCartesian(0,0,0);
	//	sleep(1);
	//	double vx, vy, vtheta;
	//	youBot->getBaseVelocitiesCartesian(vx, vy, vtheta);
	//	printf("x: %f, y: %f, theta: %f\n", vx, vy, vtheta);
	//	printf("rpm: %i\n", youBot->getActualVelocity(0));
	//	printf("rpm: %i\n", youBot->getActualVelocity(1));
	//	printf("rpm: %i\n", youBot->getActualVelocity(2));
	//printf("rpm: %i\n", youBot->getActualVelocity(3));
	//return 0;
	/* wheels are 0..3 */
	for (int i = 0; i < 4; i++) {
		youBot->setControllerMode(i, 2); //2: velocity, 1: position, 3: move by hand
		youBot->setMotorPositionOrSpeed(i, 0);
	}

	driveTo(1,0,0);

	return 0;
}

