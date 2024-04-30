#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

class WaypointNavigation {
public:
    WaypointNavigation(const std::string& file_path);

    void navigate();

private:
    std::string file_path_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;

    void readWaypoints();
};

#endif // WAYPOINT_NAVIGATION_H
