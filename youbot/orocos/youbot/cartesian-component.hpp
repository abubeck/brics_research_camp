#ifndef OROCOS_CARTESIAN_COMPONENT_HPP
#define OROCOS_CARTESIAN_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <iostream>
#include <cstdlib>
#include "youbot_chain.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <cmath>

using namespace std;
using namespace KDL;
using namespace RTT;

namespace youbot
{

class Cartesian
    : public RTT::TaskContext
{
   Chain chain;
   ChainFkSolverPos_recursive *solver;
   ChainFkSolverVel_recursive *velsolver;
   ChainIkSolverVel_wdls *iksolver;
   Frame eef, desired_eef;
   JntArrayVel jointVals;
   JntArray qdot_out;
   JntArray q_out;
   vector<double> qdotvector_out;
   Eigen::MatrixXd Mq;
   double K;

   OutputPort<vector<double> > qdotPort_out;
   InputPort< JntArrayVel > jointArrayVel_in;

public:
    Cartesian(string const& name)
        : TaskContext(name, PreOperational),
          K(0.01)
    {
        std::cout << "Cartesian constructed !" <<std::endl;

        addProperty("K", K).doc("P controller gain.");

        addOperation("getEEF",&Cartesian::getEEF, this);
        addOperation("setDesiredEEF",&Cartesian::setDesiredEEF, this);

        addProperty("eef",eef).doc("End effector frame");
        addAttribute("desired_eef",desired_eef);

        addPort("jointArrayVel_in", jointArrayVel_in);
        addPort("qdotPort_out", qdotPort_out);

        chain = getYouBotKinematicChain();
        jointVals.q.resize(8);
        jointVals.qdot.resize(8);
        qdot_out.resize(8);
        qdotvector_out.resize(8);
        solver = new ChainFkSolverPos_recursive(chain);
        velsolver = new ChainFkSolverVel_recursive(chain);
        iksolver = new ChainIkSolverVel_wdls(chain, 0.1);

        this->configure();
    }

    bool configureHook() {
        std::cout << "Cartesian configured !" <<std::endl;

    	this->getActivity()->setPeriod(0.01);

    	/* Disable base: */
    	Mq = Eigen::MatrixXd::Identity(8,8);
    	Mq(0,0) = 0;
    	Mq(1,1) = 0;
    	Mq(2,2) = 0;

    	iksolver->setWeightJS(Mq);
    	iksolver->setLambda(0.1);

        return true;
    }

//    bool movePose(const KDL::Frame& eef) {
//    	target_frame = eef;
//    }

    void readJoints(){
    	jointArrayVel_in.read( jointVals );
    }

    Frame getEEF() {
    	readJoints();
    	solver->JntToCart(jointVals.q,eef); //FK: get end effector pos
    	return eef;
    }


    bool setDesiredEEF(const KDL::Frame& target_eef) {
    	if (!isRunning())
    		return false;
    	desired_eef = target_eef;
    	return true;
    }

    bool moveToCartPos() { //(const KDL::Frame& desired_eef ){
    	if (getPeriod() == 0) {
    		log(Error) <<"Can't do move without a period." << endlog();
    		return false;
    	}
    	// measure position and speed:
    	readJoints();
    	// calculate cartesian space position and speed:
    	solver->JntToCart(jointVals.q,eef); //FK: get end effector pos
    	FrameVel framevel;
    	//velsolver->JntToCart(jointVals,framevel);
    	//Twist act_vel = framevel.deriv();
//    	cout << " actvel: "<< act_vel;
    	// desired velocity = (des pos - cur pos) / t
    	// diff is (arg2 - arg1)
    	Twist des_vel = diff(eef, desired_eef, 1.0);
    	//cout << " desvel: "<< des_vel;
    	// P controller = K ( desired velocity - actual velocity )
    	Twist out_vel = K * des_vel;
    	//cout << " outvel: "<< out_vel << endl;
    	int ret = 0;
        if ( (ret = iksolver->CartToJnt(jointVals.q, out_vel, qdot_out)) < 0) {
        	cout << "IK Error: "<< ret <<endl; //IK: get joint velocities, from current joint angles and desired end effector velocity
        	return false;
        }
        JntArrayVel testjntvel(jointVals.q,qdot_out);
        velsolver->JntToCart(testjntvel,framevel);
#if 0
    	cout << " outvel2: "<< framevel.deriv() <<endl;
        cout << "qdot:";
        for(unsigned int i=0; i < qdot_out.rows(); ++i) {
        	cout << " " << qdot_out(i);
        }
        cout << endl;
#endif
        double sat_correction = 1.0;
        for(unsigned int i=3; i < qdot_out.rows(); ++i) {
//        	cout << " " << qdot_out(i) / sat_correction;
        	// check for nan:
        	if (!isnormal(qdot_out(i)) )
        		qdot_out(i) = 0;
        	if ( qdot_out(i) > 0.1 || qdot_out(i) < -0.1 ) {
        		if ( fabs( qdot_out(i) ) / 0.1 > sat_correction ) {
        			sat_correction = fabs(qdot_out(i)) / 0.1;
        			cout <<"Warning: saturating joint " << i <<endl;
        		}
        	}
        }
        // saturation:
        for(unsigned int i=3; i < qdot_out.rows(); ++i) {
        	qdot_out(i) = qdot_out(i) / sat_correction;
        }

        for(unsigned int i=0; i < qdot_out.rows(); ++i)
        	qdotvector_out[i] = qdot_out(i);
        qdotPort_out.write( qdotvector_out );
#if 0
        // debug:
        cout << "qdotsat:";
        for(int i=0; i < qdot_out.rows(); ++i) {
        	cout << " " << qdot_out(i) / sat_correction;
        }
        cout << endl;
#endif
        return true;
    }

    bool startHook() {
        std::cout << "Cartesian started !" <<std::endl;
        readJoints();
        return true;
    }

    void updateHook() {
        //std::cout << "Cartesian executes updateHook !" <<std::endl;
    	moveToCartPos();
    }

    void stopHook() {
    }

    void cleanupHook() {
    }
};

}

#endif
