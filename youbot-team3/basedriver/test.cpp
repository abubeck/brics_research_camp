#include "YouBotOmniBaseDriver.h"

// temp main function for testing
int main(int argc, char** argv) {
    YouBotOmniBaseDriver youBot;

    youBot.Open(12345);

    // test
    youBot.SetVelocity(0.07*3.14, 0, 0);
    //youBot.SetVelocity(0, 0.07*3.14, 0);
    //youBot.SetVelocity(0, 0, 3.14f*2.0f);

    while (1)
    ;
}
