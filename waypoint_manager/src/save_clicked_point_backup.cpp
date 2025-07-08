#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <fstream>


double x = 0.0;
double y = 0.0;
double z = 0.0;
bool subscribe_sip_node = false;
std::vector<geometry_msgs::Point> points;

std::string yaml_file_path = "/home/asura/turtlebot_ws/src/turtlebot3/waypoint/config/clicked_points.yaml";

ros::Publisher marker_pub;

void pointcallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // When you didn't subscribe the /sip/node which data is '1', callback must not work.
    if(subscribe_sip_node == false){
        return;
    }
    
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

    // Add the point to the vector of points
    points.push_back(msg->point);

    // Create a marker for the arrow
    if (points.size() > 1) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = msg->header.frame_id;
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "arrows";
        arrow.id = points.size(); // Ensure each marker has a unique ID
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        // Set the scale of the arrow
        arrow.scale.x = 0.1; // Shaft diameter
        arrow.scale.y = 0.2; // Head diameter
        arrow.scale.z = 0.2; // Head length

        // Set the color of the arrow
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        // Set the points of the arrow
        geometry_msgs::Point start = points[points.size() - 2];
        geometry_msgs::Point end = points[points.size() - 1];
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        // Publish the marker
        marker_pub.publish(arrow);
    }
}

void excuteCallback(const std_msgs::Int16::ConstPtr& msg){
    if (msg->data == 1) {
        subscribe_sip_node = true;
        ROS_INFO("Received signal to execute task.");
    } else {
        subscribe_sip_node = false;
        ROS_INFO("Received signal to stop task execution.");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_saver");
    ros::NodeHandle nh;

    // Initialize the marker publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber control_sub = nh.subscribe<std_msgs::Int16>("/sip/node", 1, excuteCallback);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, pointcallback);
    ros::spin();
    return 0;
}
