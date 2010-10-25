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

int main(int argc, char** argv) {

	//youBotApi
	int semaphoreKey = 12345;

	if(argc != 1) {
		semaphoreKey = atoi(argv[1]);
	}

	YouBotApi youBot("/tmp/youBotMemMapFile", semaphoreKey);

	for(int i=0;i < 4; i++)
	{
		youBot.setControllerMode(i,2);
		youBot.setMotorPositionOrSpeed(i, 0);
	}

	int counter = 0;
	bool trigger = false;

	while(true)
	{
		if( youBot.getAxisPosition(0) < -270000 )
		{
			for(int i = 0; i < 5; i++)
			{
				youBot.setAxisPosition(i,0);
			}
		}

		if (youBot.getAxisPosition(0) > -1000)
		{
			youBot.setAxisPosition(0, -275000);
			youBot.setAxisPosition(1, -110500);
			youBot.setAxisPosition(2, -165000);
			youBot.setAxisPosition(3, -85000) ;
			youBot.setAxisPosition(4, -60000) ;
		}

		if(counter % 5000 == 0)
		{
			if(trigger)
			{
				youBot.setGripper(2);
				trigger = false;
			}
			else
			{
				youBot.setGripper(1);
				trigger = true;
			}
		}
		//printf("counter: %i\n",counter);
		counter++;
	}

return 0;
}

