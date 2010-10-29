#include "ros/ros.h"
#include "brics_actuator/CartesianPose.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>

bool dual_arm_receive = true;

#define OFFSET_X 0.0
#define OFFSET_Y 0.0
#define OFFSET_Z 0.0

void dualArmCallback(const brics_actuator::CartesianPose::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform_ipa_lbr;

  transform_ipa_lbr.setOrigin( tf::Vector3(msg->position.x + OFFSET_X, msg->position.y + OFFSET_Y, msg->position.z + OFFSET_Z) );
  transform_ipa_lbr.setRotation(  tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));
  br.sendTransform(tf::StampedTransform(transform_ipa_lbr, ros::Time::now(), "/ipa_lbr_base", "/ipa_lbr_tcp"));
}

void dualArmCallback2(const brics_actuator::CartesianPose::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform_leuven_lbr;
  transform_leuven_lbr.setOrigin( tf::Vector3(msg->position.x + OFFSET_X, msg->position.y + OFFSET_Y, msg->position.z + OFFSET_Z) );
  transform_leuven_lbr.setRotation( tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));
  br.sendTransform(tf::StampedTransform(transform_leuven_lbr, ros::Time::now(), "/leuven_lbr_base", "/leuven_lbr_tcp"));
}



int main(int argc, char ** argv)
{

  ros::init(argc, argv, "dual_arm_relay");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/dual_arm/ipapose", 1, dualArmCallback);
  ros::Subscriber sub2 = nh.subscribe("/dual_arm/leuvenpose", 1, dualArmCallback2);

  ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cart_twist", 1);
  ros::Rate loop_rate(10);
  geometry_msgs::Twist twist;
  tf::TransformListener listener;
  tf::StampedTransform transform_cob3;
  
  static tf::TransformBroadcaster br;
  tf::Transform transform_map_leuven_lbr;
  transform_map_leuven_lbr.setOrigin( tf::Vector3(3.12,-2.22,0.78) );
  transform_map_leuven_lbr.setRotation( tf::Quaternion(1.57,0,0));

  tf::Transform transform_leuven_ipa_lbr;
  transform_leuven_ipa_lbr.setOrigin( tf::Vector3(0,-1.305,0) );
  transform_leuven_ipa_lbr.setRotation( tf::Quaternion(-1.57,0,0));



  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    if(dual_arm_receive)
    {
      try{
        listener.lookupTransform("arm_7_link", "ipa_lbr_tcp",  
                                 ros::Time(), transform_cob3);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }

      twist.linear.x = transform_cob3.getOrigin().x();
      twist.linear.y = transform_cob3.getOrigin().y();
      twist.linear.z = transform_cob3.getOrigin().z();

      if(twist.linear.x > 10)
        twist.linear.x = 10;
      if(twist.linear.x < -10)
        twist.linear.x = -10;
      if(twist.linear.y > 10)
        twist.linear.y = 10;
      if(twist.linear.y < -10)
        twist.linear.y = -10;
      if(twist.linear.z > 10)
        twist.linear.z = 10;
      if(twist.linear.z < -10)
        twist.linear.z = -10;
    }
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    pub_twist.publish(twist);
    br.sendTransform(tf::StampedTransform(transform_map_leuven_lbr, ros::Time::now(), "/map", "/leuven_lbr_base"));
    br.sendTransform(tf::StampedTransform(transform_leuven_ipa_lbr, ros::Time::now(), "/leuven_lbr_base", "/ipa_lbr_base"));
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
