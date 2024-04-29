#include <ros/ros.h>
#include <string>
#include <queue>
#include <mutex>
#include <vector>
#include <boost/thread/thread.hpp>



int main(int argc, char** argv){
    ros::init(argc, argv, "yaml_rw");

    ros::NodeHandle nh;
    
    
    std::string name, street;
    nh.getParam("/yaml_rw/person/name", name);

    ROS_INFO("Name: %s", name.c_str());
    
    ros::spinOnce();
    return 0;
}