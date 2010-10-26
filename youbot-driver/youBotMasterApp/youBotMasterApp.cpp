#include <soem_master.h>
#include <cstdio>
#include <iostream>
//#include <soem_driver_factory.h>

#include <MemoryMappedFiles.h>
#include <SemaphoreLock.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

//#include "EtherCATSlaves/youBotArm.h"
//#include "EtherCATSlaves/youBotGripper.h"
//#include "EtherCATSlaves/youBotPlatform.h"

//Wenn im CMAKE der Inlcude Pfad gesetzt ist, kann man den include so angeben
//SET(locationSlaveFiles ${CMAKE_SOURCE_DIR}/EtherCATSlaves)
//include_directories(${locationSlaveFiles})
//#include <youBotArm.h>
//#include <youBotGripper.h>
//#include <youBotPlatform.h>
//#include <soem_driver.h>
#include "youBotSlaveComponent.h"
#include "youBotArm.h"

using namespace soem_ethercat_drivers;
using namespace std;

using namespace memmap;
using namespace semlock;

int main(int argc, char *argv[]) {

	SoemMaster master = SoemMaster();

	int semaphoreKey = 123456;

	if(argc != 1) {
		semaphoreKey = atoi(argv[1]);
	}
//	cout << "Map-Grösse: " << SoemDriverFactory::Instance().sizeOfMap() << endl;

	char * eth = (char*) "eth0";
	
	if(argc == 3) {
		eth = argv[2];
	}

	if(master.init(eth)) {

			// the number of existing slaves
		int nrOfSlaves = master.drivers_.size();
		cout << "Master.drivers_-Grösse " << master.drivers_.size() << endl;
		cout << "Anzahl aller Slaves    " << ec_slavecount			<< endl;
		//create semaphore-objekt and a semaphore
		SemaphoreLock semLock;

		if ( semLock.createLock(semaphoreKey) < 0 ) {
			printf( "createLock failed\n");
			return 1;
		}

		//reserve memory for mapped file
		//MemmoryMappedFiles memMap(DEFAULT_DIR.c_str(), nrOfSlaves * sizeof(YouBotSlaveMsg) );
		MemmoryMappedFiles memMap("/tmp/youBotMemMapFile", 10*sizeof(YouBotSlaveMsg) );

		//create a msg-object-pointer and assign the address of the mapped file 
		//after that use the pointer like an array
		YouBotSlaveMsg * mappedMsg = (YouBotSlaveMsg *) memMap.getAddr();
		YouBotSlaveMsg msg;

		//Mailbox-Buffer
		ec_mbxbuft mbx;
//

			msg.stctOutput.positionOrSpeed = 1500;
			msg.stctOutput.controllerMode = 2 ;

		//Initialisierung von dem Arm:

		YouBotArmMsg joint[5];
		int startValue = 0;


///////////////////////////////////////
// Auf Null kallibrieren
///////////////////////////////////////
		for(int i = 0; i < 5; i++)
		{
//			joint[i].stctOutput.controllerMode = 4;
//			joint[i].stctOutput.positionOrSpeed = startValue;
			mappedMsg[i+4].stctOutput.controllerMode = 4;
			mappedMsg[i+4].stctOutput.positionOrSpeed = startValue;
		}

		for(int i = 0; i < 5; i++)
		{
//			((YouBotArm*)(master.drivers_[i+4]))->update(joint[i]);
			YouBotArmMsg * msg = (YouBotArmMsg*) &mappedMsg[i+4];
			((YouBotArm*)(master.drivers_[i+4]))->update(*msg);
		}

		try
		{
			master.update();
		}
		catch(const std::exception& e) {
			printf("Updating master failed: %s\n",e.what());
		}

/////////////////////////////////////////////////////////////////////////
// Auf Velocity Controll setzen, wegen Aktivierung der Sinus-Kommutierung
/////////////////////////////////////////////////////////////////////////
		for(int i = 0; i < 5; i++){
//			joint[i].stctOutput.controllerMode = 2;
//			joint[i].stctOutput.positionOrSpeed = -1;
			mappedMsg[i+4].stctOutput.controllerMode = 2;
			mappedMsg[i+4].stctOutput.positionOrSpeed = -1;
		}

		for(int i = 0; i < 1000; i++)
		{
			for(int i = 0; i < 5; i++)
			{
				YouBotArmMsg * msg = (YouBotArmMsg*) &mappedMsg[i+4];
				((YouBotArm*)(master.drivers_[i+4]))->update( *msg );
			}
			usleep(4000);
			try {
				master.update();
			}
			catch(const std::exception& e) {
				printf("Updating master failed: %s\n",e.what());
			}
		}

///////////////////////////////////////
// Auf Positionskontrolle setzen und NULLEN
///////////////////////////////////////
		for(int i = 0; i < 5; i++){
//			joint[i].stctOutput.controllerMode = 1;
//			joint[i].stctOutput.positionOrSpeed = 0;
			mappedMsg[i+4].stctOutput.controllerMode = 1;
			mappedMsg[i+4].stctOutput.positionOrSpeed = 0;
		}

		for(int i = 0; i < 5; i++)
		{
			YouBotArmMsg * msg = (YouBotArmMsg*) &mappedMsg[i+4];
			((YouBotArm*)(master.drivers_[i+4]))->update( *msg );
		}

		for(int i = 0; i < 1000; i++)
		{
			usleep(4000);
			try {
				master.update();
			}
			catch(const std::exception& e) {
				printf("Updating master failed: %s\n",e.what());
			}
		}

//////////////////////////////////
// Greifer initialisieren.
//////////////////////////////////
		mappedMsg[9].stctOutput.positionOrSpeed = 0;
		mappedMsg[9].stctOutput.controllerMode = 0;
////Initialsierung ENDE

//Nullfahrt
//		for(int i = 0; i < 5; i++)
//		{
//			mappedMsg[i+4].stctOutput.controllerMode = 3;
//			mappedMsg[i+4].stctOutput.positionOrSpeed = startValue;
//		}
//
//		for(int i = 0; i < 5; i++)
//		{
//			YouBotArmMsg * msg = (YouBotArmMsg*) &mappedMsg[i+4];
//			((YouBotArm*)(master.drivers_[i+4]))->update(*msg);
//		}

//		if(ec_mbxempty(6,4000) > 0)
//				printf("Mailbox is empty before setting mailbox\n");
//
//		int setValue = -10000;
//		int getValue = 0;
//		ec_mbxbuft mbx;
//		mbx[0] = 1;
//		mbx[1] = 4;
//		mbx[2] = 1;
//		mbx[3] = 0;
////		mbx[4] = 0;
////		mbx[5] = 255;
////		mbx[6] = 255;
////		mbx[7] = 255;
////		memcpy(&mbx[4], &setValue, 4);
//		char * little;
//		little = (char*)&setValue;
//		for(int i = 0; i < 4; i++)
//		{
//			printf("pos %i <-> little-Wert: %i | little-Adresse: %x\n", i, little[i], &little[i]);
////			mbx[4+i] = little[i];
//		}
//		mbx[4] = (char)0;
//		mbx[5] = (char)1;
//		mbx[6] = (char)10;
//		mbx[7] = (char)0;
//		mbx[4] = (char) 0;
//		mbx[5] = (char) 0;
//		mbx[6] = (char) 255;
//		mbx[7] = (char) 128;
//
//
//		int wct = 0;
////		memcpy(&getValue, &mbx[4], 4);
//		wct = ec_mbxsend(6, &mbx, 4000);
//
//		printf("working counter: %i | gesetzter Wert: %i\n", wct, getValue);

//		joint[0].stctOutput.positionOrSpeed = -100000;
//		joint[1].stctOutput.positionOrSpeed = -100000;
//		joint[2].stctOutput.positionOrSpeed = -100000;
//		joint[3].stctOutput.positionOrSpeed = -100000;
//		joint[4].stctOutput.positionOrSpeed = -100000;

//		int startValue = 0;
//		joint[0].stctOutput.positionOrSpeed = startValue;
//		joint[1].stctOutput.positionOrSpeed = startValue;
//		joint[2].stctOutput.positionOrSpeed = startValue;
//		joint[3].stctOutput.positionOrSpeed = startValue;
//		joint[4].stctOutput.positionOrSpeed = startValue;


		//Mailbox-Funktion
//for(int i=1; i<6;i++){
//		if(ec_mbxempty(6,4000) > 0)
//				printf("Mailbox is empty before setting mailbox\n");
//
//		int setValue = -10000;
//		int getValue = 0;
//		ec_mbxbuft mbx;
//		mbx[0] = 1;
//		mbx[1] = 4;
//		mbx[2] = 1;
//		mbx[3] = 0;
////		mbx[4] = 0;
////		mbx[5] = 255;
////		mbx[6] = 255;
////		mbx[7] = 255;
////		memcpy(&mbx[4], &setValue, 4);
//		char * little;
//		little = (char*)&setValue;
//		for(int i = 0; i < 4; i++)
//		{
//			printf("pos %i <-> little-Wert: %i | little-Adresse: %x\n", i, little[i], &little[i]);
////			mbx[4+i] = little[i];
//		}
//		mbx[4] = (char)0;
//		mbx[5] = (char)1;
//		mbx[6] = (char)10;
//		mbx[7] = (char)0;
////		mbx[4] = (char)-1;
////		mbx[5] = (char)-1;
////		mbx[6] = (char)-255;
////		mbx[7] = (char)-128;
//
//
//		int wct = 0;
//		memcpy(&getValue, &mbx[4], 4);
//		wct = ec_mbxsend(6, &mbx, 4000);
//
//		printf("working counter: %i | gesetzter Wert: %i\n", wct, getValue);

//		int ctr = 0;
//		while( true )
//		{
//			if(ec_mbxempty(i,4000) > 0)
//				printf("Mailbox is empty: %i\n", ctr++);
//			else
//					ec_mbxreceive(3, &mbx, 4000);
//					char mailbox[9];
//					for(int i = 0; i < 9; i++)
//					{
//						if(i == 8) mailbox[i] = '\0';
//						else mailbox[i] = mbx[i];
//					}
//
//					for(int i = 0; i < 10; i++)
//					{
//						printf("Werte einzeln aus Mailbox %c: \n", i, mbx[i]);
//					}
//
//
//					printf("MAILBOX ausgelesen: %s\n", mailbox );
//					printf("----------------------------------\n");
//					break;
//		}
//}
//			 first lock the memory
//			 commit message-objects from the memory map to the slaves --> typecast to correspon

		int counter = 1;
		while(true) 
		//for(int i = 0; i < 10000; i++)
		{
			usleep(4000);
			// first lock the memory
			// commit message-objects from the memory map to the slaves --> typecast to corresponding slave is neccessary
			// with this update method the input variable of the message-objects is set automatically with the last received data
			semLock.lock();
//			if(msg.stctOutput.currentMotorPwm < 1700) msg.stctOutput.currentMotorPwm++;
//			else msg.stctOutput.currentMotorPwm=0;
//			int x = mappedMsg[7].stctOutput.currentMotorPwm;
//			if (x > 0){
//				printf("Daten aus dem MappedMessage[7] --> PWM: %i\n", x);
//			}
//			for(int i = 0; i < nrOfSlaves; i++ ) {
//				((YouBotSlaveComponent*)(master.drivers_[i]))->update(msg);
//			}

//			if( i == 999){
//				msg.stctOutput.positionOrSpeed = 0;
//				msg.stctOutput.controllerMode = 2 ;
//			}

			for(int i = 0; i < 4; i++ ) {
//				printf("Daten aus dem MappedMessage[0] --> PWM: %i und i: %i\n", mappedMsg[0].stctOutput.currentMotorPwm, i);
//				printf("Daten aus dem MappedMessage[1] --> PWM: %i und i: %i\n", mappedMsg[1].stctOutput.currentMotorPwm, i);
//				printf("Daten aus dem MappedMessage[2] --> PWM: %i und i: %i\n", mappedMsg[2].stctOutput.currentMotorPwm, i);
//				printf("Daten aus dem MappedMessage[3] --> PWM: %i und i: %i\n", mappedMsg[3].stctOutput.currentMotorPwm, i);
//				printf("Daten aus dem MappedMessage[i] --> PWM: %i und i: %i\n", mappedMsg[i].stctOutput.currentMotorPwm, i);
				static_cast<YouBotSlaveComponent*>(master.drivers_[i])->update(mappedMsg[i]);
//				static_cast<YouBotSlaveComponent*>(master.drivers_[i])->update(msg);
			}


//			if( mappedMsg[4].stctInput.actualPosition < -586000 )
//			{
//				mappedMsg[4].stctOutput.positionOrSpeed = 0;
//				mappedMsg[5].stctOutput.positionOrSpeed = 0;
//				mappedMsg[6].stctOutput.positionOrSpeed = 0;
//				mappedMsg[7].stctOutput.positionOrSpeed = 0;
//				mappedMsg[8].stctOutput.positionOrSpeed = 0;
//			}
//
//			if (mappedMsg[4].stctInput.actualPosition > -1000)
//			{
//				mappedMsg[4].stctOutput.positionOrSpeed = -587000;
//				mappedMsg[5].stctOutput.positionOrSpeed = -110500;
//				mappedMsg[6].stctOutput.positionOrSpeed = -165000;
//				mappedMsg[7].stctOutput.positionOrSpeed = -85000;
//				mappedMsg[8].stctOutput.positionOrSpeed =  -60000;
//			}

			for(int i = 4; i < 9 ; i++)
			{
//				printf("Wert: %i an der Stelle: %i\n", mappedMsg[i].stctOutput.positionOrSpeed, i );
				YouBotArmMsg * msg = (YouBotArmMsg*) &mappedMsg[i];
				static_cast<YouBotArm*>(master.drivers_[i])->update( *msg );
				printf("Wert: %i an der Stelle: %i\n", mappedMsg[i].stctOutput.controllerMode, i );
			}
//*/
//			if( joint[0].stctInput.actualPosition < -1000 )
//			{
//				joint[0].stctOutput.positionOrSpeed = -404322;
//				joint[1].stctOutput.positionOrSpeed = -227928;
//				joint[2].stctOutput.positionOrSpeed = -61175;
//				joint[3].stctOutput.positionOrSpeed = -28332;
//				joint[4].stctOutput.positionOrSpeed = -131656;
//			}
//
//			if( joint[0].stctInput.actualPosition < -404000 )
//			{
//				joint[0].stctOutput.positionOrSpeed = -404322;
//				joint[1].stctOutput.positionOrSpeed = -268342;
//				joint[2].stctOutput.positionOrSpeed = -129756;
//				joint[3].stctOutput.positionOrSpeed = -57816;
//				joint[4].stctOutput.positionOrSpeed = -131656;
//			}
//
//			if( joint[2].stctInput.actualPosition < -129000 )
//			{
//				joint[0].stctOutput.positionOrSpeed = -404322;
//				joint[1].stctOutput.positionOrSpeed = -190082;
//				joint[2].stctOutput.positionOrSpeed = -68932;
//				joint[3].stctOutput.positionOrSpeed = -52000;
//				joint[4].stctOutput.positionOrSpeed = -131656;
//			}
				
			if(mappedMsg[9].stctOutput.controllerMode == 1)
			{
				mbx[0] = 1;
				mbx[1] = 4;
				mbx[2] = 1;
				mbx[3] = 0;

				if(mappedMsg[9].stctOutput.positionOrSpeed == 1)
				{
					mbx[4] = (char) -1;
					mbx[5] = (char) -1;
					mbx[6] = (char) -255;
					mbx[7] = (char) -255;
				}

				if(mappedMsg[9].stctOutput.positionOrSpeed == 2)
				{
					mbx[4] = (char) 0;
					mbx[5] = (char) 0;
					mbx[6] = (char) 255;
					mbx[7] = (char) 255;
				}

				ec_mbxsend(11, &mbx, 4000);

				mappedMsg[9].stctOutput.positionOrSpeed = 0;
				mappedMsg[9].stctOutput.controllerMode  = 0;

			}

//			

			semLock.unlock();

			// now send output data to slaves; new input data will be received at the same time
			try {
				master.update();
			}
			catch(const std::exception& e) {
				printf("Updating master failed: %s\n",e.what());
			}


//			printf("EC_MAXSM: %i\n", EC_MAXSM);
//			printf("MAILBOXDATEN: %X\n", master.drivers_[2]->datap->SM[0].StartAddr);
//			printf("MAILBOXDATEN: %X\n", master.drivers_[2]->datap->SM[1].SMlength);
//			printf("Manufacturer from EEprom: %i\n", master.drivers_[2]->datap->eep_man);
//			printf("revision from EEprom    : %i\n", master.drivers_[2]->datap->eep_rev);
//			for(int i=0; i < 4; i++)
//				printf("MailboxStructArray: %i\n", master.drivers_[2]->datap->SMtype[i]);

//			cout << "Slave Position :" << msg.stctInput.actualPosition	<< endl;
//			cout << "Slave Error    :" << msg.stctInput.errorFlags		<< endl;

//			printf("ec_slave[2].name: %s\n",ec_slave[2].name);
//			char * traveller = (char*)ec_slave[2].inputs;
//			*traveller = 255;
//			printf("(int32)ec_slave[2].inputs: %i\n",*((int32*)traveller));
//			char * traverser = traveller;
//			for(int i=0; i < 4; i++){
//				traverser++;
//			}
//			printf("actual current: %i\n", *((int16*)traverser));
			
			//some inputs
			cout << "--------------------------------" << endl;
//			cout << "Arm 0 Position :" << joint[0].stctInput.actualPosition		<< endl;
//			cout << "Arm 1 Position :" << joint[1].stctInput.actualPosition		<< endl;
//			cout << "Arm 2 Position :" << joint[2].stctInput.actualPosition		<< endl;
//			cout << "Arm 3 Position :" << joint[3].stctInput.actualPosition		<< endl;
//			cout << "Arm 4 Position :" << joint[4].stctInput.actualPosition		<< endl;

			cout << "Base 1 Position :" << mappedMsg[0].stctInput.actualPosition		<< endl;
			cout << "Base 2 Position :" << mappedMsg[1].stctInput.actualPosition		<< endl;
			cout << "Base 3 Position :" << mappedMsg[2].stctInput.actualPosition		<< endl;
			cout << "Base 4 Position :" << mappedMsg[3].stctInput.actualPosition		<< endl;
			cout << "Arm 0 Position :" << mappedMsg[4].stctInput.actualPosition		<< endl;
			cout << "Arm 1 Position :" << mappedMsg[5].stctInput.actualPosition		<< endl;
			cout << "Arm 2 Position :" << mappedMsg[6].stctInput.actualPosition		<< endl;
			cout << "Arm 3 Position :" << mappedMsg[7].stctInput.actualPosition		<< endl;
			cout << "Arm 4 Position :" << mappedMsg[8].stctInput.actualPosition		<< endl;

//			cout << "Slave 0 actualCurrent :" << mappedMsg[0].stctInput.actualCurrent		<< endl;
//			cout << "Slave 1 actualCurrent :" << mappedMsg[1].stctInput.actualCurrent		<< endl;
//			cout << "Slave 2 actualCurrent :" << mappedMsg[2].stctInput.actualCurrent		<< endl;
//			cout << "Slave 3 actualCurrent :" << mappedMsg[3].stctInput.actualCurrent		<< endl;
//			cout << "Slave 0 Error :" << mappedMsg[0].stctInput.errorFlags		<< endl;
//			cout << "Slave 1 Error :" << mappedMsg[1].stctInput.errorFlags		<< endl;
//			cout << "Slave 2 Error :" << mappedMsg[2].stctInput.errorFlags		<< endl;
//			cout << "Slave 3 Error :" << mappedMsg[3].stctInput.errorFlags		<< endl;
//			cout << "Slave 0 Position :" << mappedMsg[0].stctInput.actualPosition		<< endl;
//			cout << "Slave 1 Position :" << mappedMsg[1].stctInput.actualPosition		<< endl;
//			cout << "Slave 2 Position :" << mappedMsg[2].stctInput.actualPosition		<< endl;
//			cout << "Slave 3 Position :" << mappedMsg[3].stctInput.actualPosition		<< endl;
//			printf("Module 4 Values:\n");
//			printf("----------------\n");
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.bldcControllerMode);
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.commutationOffsetCw);
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.commutationOffsetCcw);
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.encoderMode);
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.maxCurrent);
//			printf("Module 4 Values: %i\n",mappedMsg[3].stctOutput.appCounter);
//			cout << "Slave 0 actualCurrent    :" << mappedMsg[0].stctInput.actualCurrent		<< endl;
//			cout << "Slave 1 actualCurrent    :" << mappedMsg[1].stctInput.actualCurrent		<< endl;
//			cout << "Slave 2 actualCurrent    :" << mappedMsg[2].stctInput.actualCurrent		<< endl;
//			cout << "Slave 3 actualCurrent    :" << mappedMsg[3].stctInput.actualCurrent		<< endl;
//			cout << "msg.stctInput.actualPosition   :" << msg.stctInput.actualPosition		<< endl;
//			cout << "msg/home/ogs/youBotNetbeans/masterApp/youBotMasterApp.cpp:171: error: redeclaration of ‘int rueck’.stctInput.driverTemperature:" << msg.stctInput.driverTemperature	<< endl;
//			cout << "msg.stctInput.supplyVoltage    :" << msg.stctInput.supplyVoltage		<< endl;

//			if(counter == 0)
//			{
//				if (ec_slave[0].state == EC_STATE_OPERATIONAL )
//				{
//					printf("Status in EC_STATE_OPERATIONAL: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_SAFE_OP;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_SAFE_OP )
//				{
//					printf("Status in EC_STATE_SAFE_OP: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_PRE_OP;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_PRE_OP )
//				{
//					printf("Status in EC_STATE_PRE_OP: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_INIT;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_INIT )
//				{
//					printf("Status in EC_STATE_INIT: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_PRE_OP;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_PRE_OP )
//				{
//					printf("Status in EC_STATE_PRE_OP: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_SAFE_OP;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_SAFE_OP )
//				{
//					printf("Status in EC_STATE_SAFE_OP: %i\n", ec_slave[0].state );
//					ec_slave[0].state = EC_STATE_OPERATIONAL;
//					ec_writestate(0);
//					ec_readstate();
//					ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
//				}
//				usleep(1000000);
//				if (ec_slave[0].state == EC_STATE_OPERATIONAL )
//				{
//					printf("Status in EC_STATE_OPERATIONAL: %i\n", ec_slave[0].state );
//				}
//				usleep(1000000);
//				for(int i=0; i < 4; i++)
//				{
//					printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//				}
//				counter++;
//			}
			
		}
	}

	return 0;
}


//	YouBotSlaveMsg msg;
//	cout << "Sizeof YouBotSlaveMsg: "<< sizeof(YouBotSlaveMsg) << endl;
//	cout << "Sizeof msg: "<< sizeof(msg) << endl;
//	cout << "----------------------------"<< endl;
//	cout << "sizeof outputbuffer    : " << sizeof(YouBotSlaveMsg::outputBuffer) << endl;
//	cout << "sizeof outputbuffer var: " << sizeof(msg.stctOutput)                  << endl;
//	cout << "----------------------------"<< endl;
//	cout << "sizeof inputbuffer    : " << sizeof(YouBotSlaveMsg::inputBuffer) << endl;
//	cout << "sizeof inputbuffer var: " << sizeof(msg.stctInput) << endl;


			//for(unsigned int i=0;i<master.drivers_.size();i++){
			//if(std::string(master.drivers_[0]->getName())=="EL2124")
			//	((SoemEL2124*)(master.drivers_[0]))->update(msg);
			//	((SoemEL1124*)(master.drivers_[1]))->update(msg4);

			//printf("out message: %i %i %i %i\n",msg.DO_5V[0],msg.DO_5V[1],msg.DO_5V[2],msg.DO_5V[3]);
			//printf("in message : %i %i %i %i\n",msg4.DI_5V[0],msg4.DI_5V[1],msg4.DI_5V[2],msg4.DI_5V[3]);
			//((SoemEL2124*)(master.drivers_[1]))->update(msg3);

			  /*
			  if(std::string(master.drivers_[i]->getName())=="EL2002")
				((SoemEL2002*)(master.drivers_[i]))->update(msg1);
			  if(std::string(master.drivers_[i]->getName())=="EL4034")
				((SoemEL4034*)(master.drivers_[i]))->update(msg2);
			  */
			//}
//
//if(counter == 0)
//			{
//					for(int i=0; i < 4; i++)
//					{
//						printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//
//						if (ec_slave[i+1].state == EC_STATE_SAFE_OP)
//						{
//							ec_slave[i+1].state = EC_STATE_PRE_OP;
//						}
//					}
//					//master.update();
//					ec_writestate(0);
//					ec_readstate();
//					int rueck = ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
//					printf("Rueckgabe: %i\n",rueck);
//					usleep(3000000);
//
//					for(int i=0; i < 4; i++)
//					{
//						printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//
//						if (ec_slave[i+1].state == EC_STATE_PRE_OP)
//						{
//							ec_slave[i+1].state = EC_STATE_INIT;
//							master.update();
//							ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE);
//						}
//					}
//					//master.update();
//					ec_writestate(0);
//					ec_readstate();
//					rueck = ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE);
//					printf("Rueckgabe: %i\n",rueck);
//					usleep(3000000);
//
//					for(int i=0; i < 4; i++)
//					{
//						printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//
//						if (ec_slave[i+1].state == EC_STATE_INIT )
//						{
//							ec_slave[i+1].state = EC_STATE_PRE_OP;
//						}
//					}
//					//master.update();
//					ec_writestate(0);
//					ec_readstate();
//					rueck = ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
//					printf("Rueckgabe: %i\n",rueck);
//					usleep(3000000);
//
//					for(int i=0; i < 4; i++)
//					{
//						printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//
//						if (ec_slave[i+1].state == EC_STATE_PRE_OP)
//						{
//							ec_slave[i+1].state = EC_STATE_SAFE_OP;
//						}
//					}
//					//master.update();
//					ec_writestate(0);
//					ec_readstate();
//					rueck = ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
//					printf("Rueckgabe: %i\n",rueck);
//					usleep(3000000);
//
//					for(int i=0; i < 4; i++)
//					{
//						printf("Status der Komponente %i: %i\n", i, ec_slave[i+1].state );
//
//						if (ec_slave[i+1].state == EC_STATE_SAFE_OP)
//						{
//							ec_slave[i+1].state = EC_STATE_OPERATIONAL;
//						}
//					}
//					//master.update();
//					ec_writestate(0);
//					ec_readstate();
//					rueck = ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE);
//					printf("Rueckgabe: %i\n",rueck);
//
//					counter++;
//					usleep(3000000);
//			}
