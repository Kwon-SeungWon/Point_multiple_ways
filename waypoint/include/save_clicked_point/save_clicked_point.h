#ifndef SAVE_CLICKED_POINT_H
#define SAVE_CLICKED_POINT_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class PointSaver {
public:
    PointSaver();

private:
    void savePointToFile(const geometry_msgs::PointStamped::ConstPtr& msg);
    void executeCallback(const std_msgs::Int16::ConstPtr& msg);
    void publishMarkers();

    void setParam();

    std::string yaml_file_path;
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    ros::Subscriber sub;
    ros::Publisher marker_pub;

    std::vector<geometry_msgs::Point> published_points;
    bool subscribe_sip_node;
    ros::Time last_publish_time;
    double min_distance_threshold;
    int Arrow_ID;
};

#endif
