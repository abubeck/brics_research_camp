#include <kdl/chain.hpp>
using namespace KDL;

KDL::Chain getYouBotKinematicChain(){
  //Initialize empty chain
  KDL::Chain chain;
  //Let's assume the base has two translational joints and a rotational joint:
  //Base joints
  chain.addSegment(Segment(Joint(Joint::TransX)));
  chain.addSegment(Segment(Joint(Joint::TransY)));
  chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.15,0.0,0.0))));
  //The base of the arm is located 15 cm in front of the center of the platform


  //Arm joints
  chain.addSegment(Segment(Joint(Joint::None),Frame(Rotation::RotY(M_PI),Vector(0.0,0.0,0.072))));
  chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(-0.033,0.0,-0.075))));
  chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,-0.155))));
  chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,-0.135))));
  chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,-0.081))));
  chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotY(M_PI),Vector(0.0,0.0,-0.137))));

  return chain;
}
