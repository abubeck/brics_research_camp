/*
 * File:   main.cpp
 * Author: ogs
 *
 * Created on 26. August 2010, 14:26
 */

#include <cstdlib>
#include <SDL/SDL.h>
#include <SDL/SDL_joystick.h>
#include <cstdlib>
#include <youBotApi.h>
#include <unistd.h>
#include <ios>

using namespace std;
using namespace youbot;

 int main(int argc, char *argv[])
 {
	printf("youBotJoypadApp gestartet\n");


	//youBotApi
	int semaphoreKey = 12345;

	if(argc != 1) {
		semaphoreKey = atoi(argv[1]);
	}

	YouBotApi api("/tmp/youBotMemMapFile", semaphoreKey);

	for(int i=0;i < 4; i++)
	{
		api.setControllerMode(i,2);
	}
	usleep(12000);

	//Joypad with SDL
	int done = 0;
	SDL_Event event;

     if (SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) == -1) {
         printf("Can't init SDL:  %s\n", SDL_GetError());
         exit(1);
     }
     atexit(SDL_Quit);

     SDL_EnableKeyRepeat (SDL_DEFAULT_REPEAT_DELAY, 1);

	 //Joystickpart
	 int anzJoy = SDL_NumJoysticks ();

	 printf(">>>> Joystickanzahl: %d  <<<<\n",anzJoy);
	 printf(">>>> Der Name lautet: %s <<<<\n",SDL_JoystickName (anzJoy-1) );

	 SDL_Joystick * myJoypad = SDL_JoystickOpen (anzJoy-1);

	 int anzAchs	= SDL_JoystickNumAxes (myJoypad);
//	 int anzBalls	= SDL_JoystickNumBalls(myJoypad);
	 int anzKopf	= SDL_JoystickNumHats (myJoypad);
	 int anzKnopf	= SDL_JoystickNumButtons (myJoypad);

//	 int relX, relY, mouseState;

	 if( myJoypad  )
	 {
		printf(">>>> Joypad gefunden!  <<<<\n");
                printf("Anzahl Hats: %i \n", anzKopf);
	 }


	 Uint8  axesNr;
	 Sint16 axesVal;
	 Uint8 hatIndex, hatVal = 0;
	 Uint8 butIndex, butState = 0;

	 int multiplicator = 1;
	 int addTurnSpeedOnLeft = 0, addTurnSpeedOnRight = 0;
	 int moveValue = 0;
	 int moveValueLeftWheels  = 0;
	 int moveValueRightWheels = 0;
         int prefixWheel0  = 1;
         int prefixWheel1  = 1;
         int prefixWheel2  = 1;
         int prefixWheel3  = 1;

//	 int counter = 0;

	 while (!done)
	 {
		while (SDL_PollEvent(&event))
         {	
			switch(event.type)
			{	
//				case SDL_QUIT:
//					done = 1;
//					break;
//				case SDL_KEYUP:
//					if (event.key.keysym.sym == SDLK_ESCAPE) done = 1;
//					break;
//				case SDL_KEYDOWN:
//					if (event.key.keysym.sym == SDLK_q) done = 1;
//					break;
				case  SDL_JOYAXISMOTION:
					printf("/**< Joystick axis motion; Anzahl %d*/\n",anzAchs);
					axesNr	= event.jaxis.axis;
					axesVal	= event.jaxis.value;
					printf("----> AxenNr.: %d <-> AxenWert: %d\n", axesNr, axesVal);
					break;

				case  SDL_JOYHATMOTION:
					printf("/**< Joystick HAT position change; Anzahl %d*/\n",anzKopf);
					hatIndex = event.jhat.hat;
					hatVal = event.jhat.value;
					printf("----> HatIndex.: %d <-> hatVal: %d\n", hatIndex, hatVal);
					printf("----> BUTTONIndex.: %d <-> butState: %d\n", butIndex, butState);
					break;

				case  SDL_JOYBUTTONDOWN:
					printf("/**< Joystick button pressed; Anzahl %d*/\n",anzKnopf);
					butIndex = event.jbutton.button;
					butState = event.jbutton.state;
					printf("----> BUTTONIndex.: %d <-> butState: %d\n", butIndex, butState);
					
					if( butIndex == 10 )
					{
						if( multiplicator > 0 ) multiplicator--;
					}

					if( butIndex == 11 )
					{
						if( multiplicator < 16 ) multiplicator++;
					}

					//if you want to drive left, the right wheels have to be faster
					//if you want to drive right, vice versa
					if( butIndex == 4 )	addTurnSpeedOnRight  = 1000;
					if( butIndex == 5 )	addTurnSpeedOnLeft   = 1000;

					break;

				case SDL_JOYBUTTONUP:
					printf("/**< Joystick button released; Anzahl %d*/\n",anzKnopf);
					butIndex = event.jbutton.button;
					butState = event.jbutton.state;
					printf("----> BUTTONIndex.: %d <-> butState: %d\n", butIndex, butState);

					if( butIndex == 4 )	addTurnSpeedOnRight = 0;
					if( butIndex == 5 )	addTurnSpeedOnLeft  = 0;

					break;
             }
         }


		moveValue = 500*multiplicator;

		if(moveValue < 7000)
		{
			moveValueRightWheels = moveValue + addTurnSpeedOnRight;
			moveValueLeftWheels  = moveValue + addTurnSpeedOnLeft;
		}
		else
		{
			moveValueRightWheels = moveValue - addTurnSpeedOnLeft;
			moveValueLeftWheels  = moveValue - addTurnSpeedOnRight;
		}


                if ( hatVal == 0 )
		{
//			printf("--> STOP <--\n");
                    int rotation = 0;
                    if(butIndex == 4 && butState)
                    {
                        prefixWheel0 = 1;
                        prefixWheel1 = 1;
                        prefixWheel2 = 1;
                        prefixWheel3 = 1;
                        rotation = moveValue;
                    }
                    if(butIndex == 5 && butState)
                    {
                        prefixWheel0 = -1;
                        prefixWheel1 = -1;
                        prefixWheel2 = -1;
                        prefixWheel3 = -1;
                        rotation = -1 * moveValue;
                    }

                    if( rotation != 0)
                    {

                        api.setMotorPositionOrSpeed(0,rotation);
                        api.setMotorPositionOrSpeed(1,rotation);
                        api.setMotorPositionOrSpeed(2,rotation);
                        api.setMotorPositionOrSpeed(3,rotation);
                    }
                    else
                    {
                        api.setMotorPositionOrSpeed(0,0);
                        api.setMotorPositionOrSpeed(1,0);
                        api.setMotorPositionOrSpeed(2,0);
                        api.setMotorPositionOrSpeed(3,0);
                    }
		}

		if ( SDL_HAT_UP == hatVal)
		{
//                  printf("--> UP <--\n");
                    prefixWheel0 = -1;
                    prefixWheel1 =  1;
                    prefixWheel2 = -1;
                    prefixWheel3 =  1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels);
		}

		if (( SDL_HAT_DOWN == hatVal))
		{
//                  printf("--> DOWN <--\n");
                    prefixWheel0 =  1;
                    prefixWheel1 = -1;
                    prefixWheel2 =  1;
                    prefixWheel3 = -1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels);
		}

		if (( SDL_HAT_RIGHT == hatVal ) )
		{
//                  printf("--> RIGHT <--\n");
                    prefixWheel0 = -1;
                    prefixWheel1 = -1;
                    prefixWheel2 =  1;
                    prefixWheel3 =  1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels);
		}

		if (( SDL_HAT_LEFT == hatVal) )
		{
//                  printf("--> LEFT <--\n");
                    prefixWheel0 =  1;
                    prefixWheel1 =  1;
                    prefixWheel2 = -1;
                    prefixWheel3 = -1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels );
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels);
		}

		if ( SDL_HAT_RIGHTUP == hatVal)
		{
//                  printf("--> UP'n'RIGHT <--\n");
                    prefixWheel0 = -1;
                    prefixWheel1 =  1;
                    prefixWheel2 = -1;
                    prefixWheel3 =  1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels     );
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * addTurnSpeedOnRight * 2);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * addTurnSpeedOnLeft  * 2);
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels    );
		}

		if (( SDL_HAT_RIGHTDOWN == hatVal))
		{
//                  printf("--> DOWN'n'RIGHT <--\n");
                    prefixWheel0 =  1;
                    prefixWheel1 = -1;
                    prefixWheel2 =  1;
                    prefixWheel3 = -1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * addTurnSpeedOnLeft  * 2);
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels    );
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels     );
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * addTurnSpeedOnRight * 2);
		}

		if (( SDL_HAT_LEFTDOWN == hatVal ) )
		{
//                  printf("--> DOWN'n'LEFT <--\n");
                    prefixWheel0 =  1;
                    prefixWheel1 = -1;
                    prefixWheel2 =  1;
                    prefixWheel3 = -1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * moveValueLeftWheels);
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * addTurnSpeedOnRight * 2);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * addTurnSpeedOnLeft  * 2);
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * moveValueRightWheels);
		}

		if (( SDL_HAT_LEFTUP == hatVal) )
		{
//                  printf("--> UP'n'LEFT <--\n");
                    prefixWheel0 = -1;
                    prefixWheel1 =  1;
                    prefixWheel2 = -1;
                    prefixWheel3 =  1;

                    api.setMotorPositionOrSpeed(0, prefixWheel0 * addTurnSpeedOnLeft  * 2);
                    api.setMotorPositionOrSpeed(1, prefixWheel1 * moveValueRightWheels);
                    api.setMotorPositionOrSpeed(2, prefixWheel2 * moveValueLeftWheels);
                    api.setMotorPositionOrSpeed(3, prefixWheel3 * addTurnSpeedOnRight * 2);
		}

		//to stop the joypad application
		if (butIndex == 9)
		{
			done = 1;
		}
                
                if (butIndex == 6)
		{
			api.setMotorPositionOrSpeed(4, -moveValue );
		}
                
                if (butIndex == 7)
		{
			api.setMotorPositionOrSpeed(4, moveValue );
		}
                
                if (butIndex == 0)
		{
			api.setMotorPositionOrSpeed(8, moveValue );
		}
                
                if (butIndex == 2)
		{
			api.setMotorPositionOrSpeed(8, -moveValue );
		}
                
                if (butIndex == 1)
		{
			api.setMotorPositionOrSpeed(7, -moveValue );
		}
                
                if (butIndex == 3)
		{
			api.setMotorPositionOrSpeed(7, moveValue );
		}

                if (axesNr == 1 && axesVal < 0)
		{
			api.setMotorPositionOrSpeed(5, -moveValue );
		}
                
                if (axesNr == 1 && axesVal > 0)
		{
			api.setMotorPositionOrSpeed(5,  moveValue );
		}

                if (axesNr == 3 && axesVal < 0)
		{
			api.setMotorPositionOrSpeed(6, -moveValue );
		}

                if (axesNr == 3 && axesVal > 0)
		{
			api.setMotorPositionOrSpeed(6,  moveValue );
		}

	}
	
     return 0;
 }

 /*
	* SDL_HAT_CENTERED
    * SDL_HAT_UP
    * SDL_HAT_RIGHT
    * SDL_HAT_DOWN
    * SDL_HAT_LEFT
    * SDL_HAT_RIGHTUP
    * SDL_HAT_RIGHTDOWN
    * SDL_HAT_LEFTUP
    * SDL_HAT_LEFTDOWN
*/
/*
 if ( hatVal == 0 && !butState)
		{
			printf("--> STOP <--\n");
			api.setMotorPositionOrSpeed(0,0);
			api.setMotorPositionOrSpeed(1,0);
			api.setMotorPositionOrSpeed(2,0);
			api.setMotorPositionOrSpeed(3,0);
		}

		if ( SDL_HAT_UP == hatVal)
		{
			printf("--> UP <--\n");
			api.setMotorPositionOrSpeed(0,-100*multiplicator);
			api.setMotorPositionOrSpeed(1,100*multiplicator);
			api.setMotorPositionOrSpeed(2,-100*multiplicator);
			api.setMotorPositionOrSpeed(3,100*multiplicator);
		}

		if (( SDL_HAT_DOWN == hatVal))
		{
			printf("--> DOWN <--\n");
			api.setMotorPositionOrSpeed(0,100*multiplicator);
			api.setMotorPositionOrSpeed(1,-100*multiplicator);
			api.setMotorPositionOrSpeed(2,100*multiplicator);
			api.setMotorPositionOrSpeed(3,-100*multiplicator);
		}

		if (( SDL_HAT_RIGHT == hatVal ) )
		{
			printf("--> RIGHT <--\n");
			api.setMotorPositionOrSpeed(0,100*multiplicator);
			api.setMotorPositionOrSpeed(1,100*multiplicator);
			api.setMotorPositionOrSpeed(2,-100*multiplicator);
			api.setMotorPositionOrSpeed(3,-100*multiplicator);
		}

		if (( SDL_HAT_LEFT == hatVal) )
		{
			printf("--> LEFT <--\n");
			api.setMotorPositionOrSpeed(0,-100*multiplicator);
			api.setMotorPositionOrSpeed(1,-100*multiplicator);
			api.setMotorPositionOrSpeed(2,100*multiplicator);
			api.setMotorPositionOrSpeed(3,100*multiplicator);
		}

		if ( SDL_HAT_RIGHTUP == hatVal)
		{
			printf("--> UP'n'RIGHT <--\n");
			api.setMotorPositionOrSpeed(0,0);
			api.setMotorPositionOrSpeed(1,100*multiplicator);
			api.setMotorPositionOrSpeed(2,-100*multiplicator);
			api.setMotorPositionOrSpeed(3,0);

		}

		if (( SDL_HAT_RIGHTDOWN == hatVal))
		{
			printf("--> DOWN'n'RIGHT <--\n");
			api.setMotorPositionOrSpeed(0,100*multiplicator);
			api.setMotorPositionOrSpeed(1,0);
			api.setMotorPositionOrSpeed(2,0);
			api.setMotorPositionOrSpeed(3,-100*multiplicator);
		}

		if (( SDL_HAT_LEFTDOWN == hatVal ) )
		{
			printf("--> DOWN'n'LEFT <--\n");
			api.setMotorPositionOrSpeed(0,0);
			api.setMotorPositionOrSpeed(1,-100*multiplicator);
			api.setMotorPositionOrSpeed(2,100*multiplicator);
			api.setMotorPositionOrSpeed(3,0);
		}

		if (( SDL_HAT_LEFTUP == hatVal) )
		{
			printf("--> UP'n'LEFT <--\n");
			api.setMotorPositionOrSpeed(0,-100*multiplicator);
			api.setMotorPositionOrSpeed(1,0);
			api.setMotorPositionOrSpeed(2,0);
			api.setMotorPositionOrSpeed(3,100*multiplicator);
		}

		if (( butIndex == 4 && butState ) )
		{
			printf("--> ROTATION LEFT <--\n");
			api.setMotorPositionOrSpeed(0,300);
			api.setMotorPositionOrSpeed(1,300);
			api.setMotorPositionOrSpeed(2,300);
			api.setMotorPositionOrSpeed(3,300);
		}

		if (( butIndex == 5 && butState) )
		{
//			api.setMotorPositionOrSpeed(3,-100*multiplicator);
			printf("--> ROTATION RIGHT <--\n");
			api.setMotorPositionOrSpeed(0,-300);
			api.setMotorPositionOrSpeed(1,-300);
			api.setMotorPositionOrSpeed(2,-300);
			api.setMotorPositionOrSpeed(3,-300);
		}


//		if (butIndex == 8)
//		{
//			api.setMotorPositionOrSpeed(0,1700);
//			api.setMotorPositionOrSpeed(1,-1700);
//			api.setMotorPositionOrSpeed(2,1700);
//			api.setMotorPositionOrSpeed(3,-1700);
//		}

		if (butIndex == 9)
		{
			done = 1;
		}
 */


 
