#!/bin/bash

echo "---------------------------"
echo "Installing memorymap/semaphore"
echo "---------------------------"
cd memmoyMappedFilesAndSemaphore/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-driver"
echo "---------------------------"
cd ../../youBotDriver/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-MasterApp"
echo "---------------------------"
cd ../../youBotMasterApp/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-Api"
echo "---------------------------"
cd ../../youBotApi/build/
cmake ..
make
sudo make install


echo "---------------------------"
echo "Installing youBot-Joypad"
echo "---------------------------"
cd ../../youBotJoypadApp/build/
cmake ..
make

echo "---------------------------"
echo "Installing TestProgramms"
echo "---------------------------"
cd ../../testProgs/ApiTest/build/
cmake ..
make

#create links to the master and joypad
cd ../../../
ln -s youBotMasterApp/youBotApp master
ln -s youBotJoypadApp/youBotJoypadApp joypad
ln -s testProgs/ApiTest/youBotApiTest apitest

