/*
* This is the Ethercat master class based on the SOEM package
*
* @author: Koen Buys, Ruben Smits
*
*
*
*/
#ifndef SOEM_MASTER_H
#define SOEM_MASTER_H

extern "C"{
#include <ethercattype.h>
#include <ethercatmain.h>
}

#include <vector>

#include <soem_driver.h>
//Erweiterung START
//#include <youBotArm.h>
//#include <youBotGripper.h>
//#include <youBotPlatform.h>
//Erweiterung STOP

namespace soem_ethercat_drivers{

class SoemMaster
{
    int nr_slaves_; //contains the number of found slave on the EC bus
    char IOmap_[4096];    
    
    //Erweiterung START
    /*
     * To check, which components of youBot are active. 
     * There are three states:
     *  - only arm with gripper     --> if 6  slaves were found (5 joints + 1 gripper)
     *  - only platform             --> if 4  slaves were found (4 wheels)
     *  - arm, gripper and platform --> if 10 slaves were found (4 wheels + 5 joints + 1 gripper)
     */
//    void setComponentsState();
    //Erweiterung STOP

 public:
    
    SoemMaster();
    ~SoemMaster();
    /** 
     * Initialise the master and bind the socket to ifname
     * 
     * @param ifname ethernet interface used to connect to ethercat slaves
     * 
     * @return error code
     */
    int init(const char *ifname);
    /** 
     * send updated data from all drivers to the slaves
     * 
     */
    bool send();
    /** 
     * receive data from slaves and update drivers data
     * 
     * @param waiting_time maximal time to wait for data
     */
    void receive();
    void update();
    void close();
    //void printstate();
    //should be protected, only public for developping
    std::vector<SoemDriver*> drivers_;

    
    //Erweiterung START
    /*To save the state of the components*/
//    bool      armOnline;
//    bool  gripperOnline;
//    bool platformOnline;
    //Erweiterung STOP

 protected:
    void createDrivers();
};
}

#endif
