#!/bin/bash

echo "---------------------------"
echo "Installing memorymap/semaphore"
echo "---------------------------"
mkdir -p memmoyMappedFilesAndSemaphore/build/
cd memmoyMappedFilesAndSemaphore/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-driver"
echo "---------------------------"
mkdir -p ../../youBotDriver/build/
cd ../../youBotDriver/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-MasterApp"
echo "---------------------------"
mkdir -p ../../youBotMasterApp/build/
cd ../../youBotMasterApp/build/
cmake ..
make
sudo make install

echo "---------------------------"
echo "Installing youBot-Api"
echo "---------------------------"
mkdir -p ../../youBotApi/build/
cd ../../youBotApi/build/
cmake ..
make
sudo make install


echo "---------------------------"
echo "Installing youBot-Joypad"
echo "---------------------------"
mkdir -p ../../youBotJoypadApp/build/
cd ../../youBotJoypadApp/build/
cmake ..
make

echo "---------------------------"
echo "Installing TestProgramms"
echo "---------------------------"
mkdir -p ../../testProgs/ApiTest/build/
cd ../../testProgs/ApiTest/build/
cmake ..
make

#create links to the master and joypad
cd ../../../
ln -s youBotMasterApp/youBotApp master
ln -s youBotJoypadApp/youBotJoypadApp joypad
ln -s testProgs/ApiTest/youBotApiTest apitest

