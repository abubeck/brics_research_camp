#include <ros/ros.h>
#include "youbot_hal/youbot_movement_command.h"
#include <joy/Joy.h>


class TeleopYoubot
{
public:
  TeleopYoubot();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopYoubot::TeleopYoubot():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  //vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
  vel_pub_ = nh_.advertise<youbot_hal::youbot_movement_command>("youbot_movement_command", 1);

  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopYoubot::joyCallback, this);

}

void TeleopYoubot::joyCallback(const joy::Joy::ConstPtr& joy)
{
  youbot_hal::youbot_movement_command vel;
  vel.vel_x = joy->axes[0]*0.01;
  vel.vel_y = joy->axes[1]*0.01;
  vel.vel_theta = joy->axes[2]*0.01;
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_joy");
  TeleopYoubot teleop_youbot;

  ros::spin();
}
