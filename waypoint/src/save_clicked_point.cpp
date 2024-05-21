#include "save_clicked_point/save_clicked_point.h"
#include <cmath>

PointSaver::PointSaver() : nh("~") {
    setParam();
    control_sub = nh.subscribe<std_msgs::Int16>("/sip/node", 1, &PointSaver::executeCallback, this);
    sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &PointSaver::savePointToFile, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    subscribe_sip_node = false;
    min_distance_threshold = 0.1; // Adjust as needed
    Arrow_ID = 1;
}

void PointSaver::setParam() {
    nh.param("yaml_file_path", this->yaml_file_path, std::string("/home/asura/turtlebot_ws/src/turtlebot3/waypoint/config/clicked_points.yaml"));
}

void PointSaver::savePointToFile(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!subscribe_sip_node) {
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
    std::ofstream fout(this->yaml_file_path);
    fout << waypoints_node;
    ROS_INFO("Saved point to clicked_points.yaml: x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);

    // Add the point to the vector of points
    published_points.push_back(msg->point);
    
    // Create a marker for the arrow
    // if (published_points.size() > 1) {
    //     visualization_msgs::Marker arrow;
    //     arrow.header.frame_id = msg->header.frame_id;
    //     arrow.header.stamp = ros::Time::now();
    //     arrow.ns = "arrows";
    //     arrow.id = published_points.size(); // Ensure each marker has a unique ID
    //     arrow.type = visualization_msgs::Marker::ARROW;
    //     arrow.action = visualization_msgs::Marker::ADD;

    //     // Set the scale of the arrow
    //     arrow.scale.x = 0.1; // Shaft diameter
    //     arrow.scale.y = 0.2; // Head diameter
    //     arrow.scale.z = 0.2; // Head length

    //     // Set the color of the arrow
    //     arrow.color.r = 1.0;
    //     arrow.color.g = 0.0;
    //     arrow.color.b = 0.0;
    //     arrow.color.a = 1.0;

    //     // Set the points of the arrow
    //     geometry_msgs::Point start = published_points[published_points.size() - 2];
    //     geometry_msgs::Point end = published_points[published_points.size() - 1];
    //     arrow.points.push_back(start);
    //     arrow.points.push_back(end);

    //     // Publish the marker
    //     marker_pub.publish(arrow);
    // }
    publishMarkers();
}

void PointSaver::executeCallback(const std_msgs::Int16::ConstPtr& msg) {
    if (msg->data == 1) {
        subscribe_sip_node = true;
        ROS_INFO("Received signal to execute task.");
    } else {
        subscribe_sip_node = false;
        ROS_INFO("Received signal to stop task execution.");
    }
}

void PointSaver::publishMarkers() {
    if (published_points.empty()) {
        return;
    }

    visualization_msgs::MarkerArray node_link_arr;
    ROS_INFO("Publishing markers for %lu points", published_points.size());


    if(published_points.size()> 1) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "map"; // Adjust the frame ID as needed
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "arrows";
        arrow.id = Arrow_ID; // Ensure each marker has a unique ID
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

        arrow.pose.orientation.w = 1.0;

        // Set the points of the arrow
        arrow.points.push_back(published_points[published_points.size()- 2]);
        arrow.points.push_back(published_points[published_points.size()- 1]);

        // Add the arrow to the marker array
        marker_pub.publish(arrow);

        //node_link_arr.markers.push_back(arrow);
        Arrow_ID ++ ;
    }
}
