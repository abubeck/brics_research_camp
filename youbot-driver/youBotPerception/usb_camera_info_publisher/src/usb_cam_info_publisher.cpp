#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <XmlRpcException.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "usb_cam_info_publisher");
  ros::NodeHandle node("~");
  ros::Publisher image_info_pub;

  sensor_msgs::CameraInfo usb_camera_info_msg;

  double publisher_rate;
  int image_width;
  int image_height;
  std::string frame_id;
  std::string camera_info_topic;

  XmlRpc::XmlRpcValue rosParam;
  boost::array<double, 5> tmpD = { { 0, 0, 0, 0, 0 } };
  boost::array<double, 9> tmpK = { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
  boost::array<double, 9> tmpR = { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
  boost::array<double, 12> tmpP = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

  /* get parameters */
  if (!node.getParam("publisher_rate", publisher_rate)) {
		publisher_rate = 10.0; // default
  }

  if (!node.getParam("image_width", image_width)) { // (used e.g. by usb_cam driver)
	  if (!node.getParam("width", image_width)) { // (used e.g. by uvc_cam dricer)
		  image_width = 640; // default
	  }
  }
  ROS_INFO ("\timage_width:\t %d", image_width);

  if (!node.getParam("image_height", image_height)) {
	  if (!node.getParam("height", image_height)) {
		  image_height = 480; // default
	  }
  }
  ROS_INFO ("\timage_height:\t %d", image_height);

  if (!node.getParam("camera_frame_id", frame_id)) {
	  frame_id = "usb_cam"; // default
  }
  ROS_INFO ("\tframe_id:\t %s", frame_id.c_str());

  if (!node.getParam("camera_info_topic", camera_info_topic)) {
	  camera_info_topic = "/usb_cam/camera_info"; // default
  }
  ROS_INFO ("\tcamera_info_topic:\t %s", camera_info_topic.c_str());

  try {
	  rosParam.clear();
	  node.param("D", rosParam, rosParam);
	  ROS_ASSERT(tmpD.size() == static_cast<unsigned int>(rosParam.size()));
	  for(int i =0; i < rosParam.size(); i++)
	  {
		  tmpD[i] = rosParam[i];
		  ROS_INFO ("\tD[%i] = \t %lf", i, tmpD[i]);
	  }
  } catch (XmlRpc::XmlRpcException e) {
	  ROS_WARN("\tNo rosparam D found.");
  }

  try {
	  rosParam.clear();
	  node.param("K", rosParam, rosParam);
	  ROS_ASSERT(tmpK.size() == static_cast<unsigned int>(rosParam.size()));
	  for(int i =0; i < rosParam.size(); i++)
	  {
		  tmpK[i] = rosParam[i];
		  ROS_INFO ("\tK[%i] = \t %lf", i, tmpK[i]);
	  }
  } catch (XmlRpc::XmlRpcException e) {
	  ROS_WARN("\tNo rosparam K found.");
  }

  try {
	  rosParam.clear();
	  node.param("R", rosParam, rosParam);
	  ROS_ASSERT(tmpR.size() == static_cast<unsigned int>(rosParam.size()));
	  for(int i =0; i < rosParam.size(); i++)
	  {
		  tmpR[i] = rosParam[i];
		  ROS_INFO ("\tR[%i] = \t %lf", i, tmpR[i]);
	  }
  } catch (XmlRpc::XmlRpcException e) {
	  ROS_WARN("\tNo rosparam R found.");
  }

  try {
	  rosParam.clear();
	  node.param("P", rosParam, rosParam);
	  ROS_ASSERT(tmpP.size() == static_cast<unsigned int>(rosParam.size()));
	  for(int i =0; i < rosParam.size(); i++)
	  {
		  tmpP[i] = rosParam[i];
		  ROS_INFO ("\tP[%i] = \t %lf", i, tmpP[i]);
	  }
  } catch (XmlRpc::XmlRpcException e) {
	  ROS_WARN("\tNo rosparam P found.");
  }

  
  usb_camera_info_msg.D = tmpD;
  usb_camera_info_msg.K = tmpK;
  usb_camera_info_msg.R = tmpR;
  usb_camera_info_msg.P = tmpP;
  
  usb_camera_info_msg.width = image_width;
  usb_camera_info_msg.height = image_height;
  
  usb_camera_info_msg.header.stamp = ros::Time::now();
  usb_camera_info_msg.header.frame_id = frame_id;

  image_info_pub = node.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);
  ros::Rate rate(publisher_rate);
  while (node.ok()){
	usb_camera_info_msg.header.stamp = ros::Time::now();
	image_info_pub.publish(usb_camera_info_msg);
    rate.sleep();
  }
  return 0;
};
