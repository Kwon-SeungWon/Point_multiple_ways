#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

double x = 0.0;
double y = 0.0;
double z = 0.0;

std::string yaml_file_path = "/home/asura/turtlebot_ws/src/turtlebot3/turtlebot3_navigation/config/clicked_points.yaml";

void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Store the received point in a YAML node
    YAML::Node node;
    node["header"]["stamp"] = msg->header.stamp.toSec();
    node["header"]["frame_id"] = msg->header.frame_id;
    node["point"]["x"] = msg->point.x;
    node["point"]["y"] = msg->point.y;
    node["point"]["z"] = msg->point.z;

    // Open the YAML file in append mode and write the YAML node
    std::ofstream fout(yaml_file_path, std::ofstream::app);
    fout << node << std::endl;
    ROS_INFO("Saved point to clicked_points.yaml: x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_saver");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, callback);
    ros::spin();
    return 0;
}
