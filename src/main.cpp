#include "ros/ros.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "main");

    ROS_INFO("%s", "Running second_project test node");

  	while (ros::ok()){
    	ros::spinOnce();
    }
  	return 0;
}