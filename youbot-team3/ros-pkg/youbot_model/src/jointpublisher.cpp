#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "intial_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("youbot_robot_state", 1);
    ros::Rate loop_rate(10);

    double wheel = 0;

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(11);
        joint_state.position.resize(11);
        joint_state.name[0] ="j0";
        joint_state.position[0] = 0.5;
        joint_state.name[1] ="j1";
        joint_state.position[1] = 0.5;
        joint_state.name[2] ="j2";
        joint_state.position[2] = 0.5;
        joint_state.name[3] ="j3";
        joint_state.position[3] = 0.5;
        joint_state.name[4] ="j4";
        joint_state.position[4] = 0.5;

        joint_state.name[5] ="w1";
        joint_state.position[5] = wheel;
        joint_state.name[6] ="w2";
        joint_state.position[6] = wheel;
        joint_state.name[7] ="w3";
        joint_state.position[7] = wheel;
        joint_state.name[8] ="w4";
        joint_state.position[8] = wheel;
        joint_state.name[9] ="f1";
        joint_state.position[9] = 0.01;
        joint_state.name[10] ="f2";
        joint_state.position[10] = 0.01;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        wheel += 0.05;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
