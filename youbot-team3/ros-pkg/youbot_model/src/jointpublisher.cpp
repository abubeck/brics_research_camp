#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "intial_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
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

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
