#ifndef SAVE_POLYGON_LOCAL_H
#define SAVE_POLYGON_LOCAL_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <vector>

class PolygonSaver {
public:
    PolygonSaver();

private:
    void executeCallback(const std_msgs::Int16::ConstPtr& msg);
    void polygonNameCallback(const std_msgs::String::ConstPtr& msg);
    void savePolygonToFile(const geometry_msgs::PointStamped::ConstPtr& msg);
    void publishAreas(const geometry_msgs::Pose::ConstPtr& point, double area_x, double area_y, double area_z);
    void checkAreas(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, YAML::Node& integrated_node);
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void readObjectsFile();

    void setParam();

    std::string polygon_file_path;
    std::string object_file_path;
    ros::NodeHandle nh;
    ros::Subscriber execute_sub;
    ros::Subscriber data_sub;
    ros::Subscriber point_sub;
    ros::Publisher marker_pub;
    interactive_markers::InteractiveMarkerServer server;

    std::vector<geometry_msgs::Point> published_points;
    bool subscribe_sip_node;
    bool first_point_set;
    ros::Time last_publish_time;

    geometry_msgs::Point first_point;
    std::string polygon_name;
    int polygon_counter;
    std::vector<float> PolygonArray;
};

#endif
