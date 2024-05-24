#ifndef SELECTION_SAVER_H
#define SELECTION_SAVER_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

class SelectionSaver {
public:
    SelectionSaver();

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void publishPolygon() const;

private:
    ros::NodeHandle nh;
    ros::Publisher polygon_pub;
    geometry_msgs::PolygonStamped polygon;
    interactive_markers::InteractiveMarkerServer server;
    std::vector<geometry_msgs::Point> polygon_points;
    visualization_msgs::InteractiveMarker polygon_marker;
};

#endif // SELECTION_SAVER_H
