#include <kdl/chain.hpp>
using namespace kdl;

KDL::Chain getYouBotKinematicChain(){
  //Initialize empty chain
  KDL::Chain chain();
  //Let's assume the base has two translational joints and a rotational joint:
  //Base joints
  chain.addSegment(Joint(Joint::TransX));
  chain.addSegment(Joint(Joint::TransY));
  //The base of the arm is located 15 cm in front of the center of the platform
  chain.addSegment(Joint(Joint::RotZ),Frame(Vector(0.15,0.0,0.0)));

  //Arm joints
  chain.addSegment(Joint(Joint::None),Frame(Vector(0.0,0.0,0.072)));
  chain.addSegment(Joint(Joint::RotZ),Frame(Vector(0.033,0.0,0.075)));
  chain.addSegment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.155)));
  chain.addSegment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.135)));
  chain.addSegment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.081)));
  chain.addSegment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.137)));
}
