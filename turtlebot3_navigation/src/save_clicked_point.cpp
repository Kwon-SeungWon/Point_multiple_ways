#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

double x = 0.0;
double y = 0.0;
double z = 0.0;

std::string yaml_file_path = "/home/asura/turtlebot_ws/src/turtlebot3/turtlebot3_navigation/config/clicked_points.yaml";

// void ros::init(){
//     fout << Yam
// }

void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Create a YAML node for the point
    YAML::Node point_node;
    point_node["x"] = msg->point.x;
    point_node["y"] = msg->point.y;
    point_node["z"] = msg->point.z;

    // Create a YAML node for the header
    YAML::Node header_node;
    header_node["seq"] = msg->header.seq;
    header_node["stamp"] = msg->header.stamp.toSec();
    header_node["frame_id"] = msg->header.frame_id;

    // Create a YAML node for the waypoint containing the header and point nodes
    YAML::Node waypoint_node;
    waypoint_node["header"] = header_node;
    waypoint_node["point"] = point_node;

    // Append the waypoint node to the list of waypoints
    static YAML::Node waypoints_node;
    waypoints_node.push_back(waypoint_node);

    // Write the waypoints to the YAML file
    std::ofstream fout(yaml_file_path);
    fout << waypoints_node;
    ROS_INFO("Saved point to clicked_points.yaml: x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_saver");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, callback);
    ros::spin();
    return 0;
}
