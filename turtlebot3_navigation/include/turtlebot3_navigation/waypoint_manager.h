#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class WaypointManager {
public:
    WaypointManager(const std::string& file_path);

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg);

private:
    std::string yaml_file_path_;
    YAML::Node waypoints_node_;
};

#endif // WAYPOINT_MANAGER_H
