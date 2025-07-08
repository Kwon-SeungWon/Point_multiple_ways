#include "save_object_local/save_object_local.h"

ObjectSaver::ObjectSaver() : nh("~"), server("area_marker_server") {
    setParam();
    execute_sub = nh.subscribe<std_msgs::Int16>("/sip/node", 1, &ObjectSaver::executeCallback, this);
    data_sub = nh.subscribe<std_msgs::String>("/sip/node/name", 1, &ObjectSaver::objectNameCallback, this);
    point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point2", 1, &ObjectSaver::saveObjectToFile, this);
    subscribe_sip_node = false;
    first_point_set = false;
}

void ObjectSaver::setParam() {
    nh.param("object_file_path", this->object_file_path, std::string("/home/asura/turtlebot_ws/src/turtlebot3/waypoint/config/save_objects.yaml"));
}

void ObjectSaver::saveObjectToFile(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!subscribe_sip_node) {
        return;
    }
    
    if(!first_point_set){
        first_point = msg->point;
        first_point_set = true;
        ROS_INFO("First point of diagonal - X: %f, Y: %f, Z: %f", first_point.x, first_point.y, first_point.z);
        return;
    }
    
    geometry_msgs::Point second_point = msg->point;

    ROS_INFO("Second point of diagonal - X: %f, Y: %f, Z: %f", second_point.x, second_point.y, second_point.z);

    geometry_msgs::Pose mean;
    mean.position.x = (first_point.x + second_point.x) / 2;
    mean.position.y = (first_point.y + second_point.y) / 2;
    mean.position.z = (first_point.z + second_point.z) / 2;

    double size_x = fabs(first_point.x - second_point.x);
    double size_y = fabs(first_point.y - second_point.y);
    double size_z = fabs(first_point.z - second_point.z);
    
    // Create a YAML node for the point
    YAML::Node point_node;
    point_node["x"] = mean.position.x;
    point_node["y"] = mean.position.y;
    point_node["z"] = mean.position.z;

    // Create a YAML node for the header
    YAML::Node header_node;
    header_node["seq"] = msg->header.seq;
    header_node["stamp"] = msg->header.stamp.toSec();
    header_node["frame_id"] = msg->header.frame_id;

    // Create a YAML node for the header
    YAML::Node object_node;
    object_node["name"] = object_name;
    object_node["size"] = 100;

    // Create a YAML node for the waypoint containing the header and point nodes
    YAML::Node integrate_node;
    integrate_node["header"] = header_node;
    integrate_node["object"] = object_node;
    integrate_node["point"] = point_node;

    // Append the waypoint node to the list of waypoints
    static YAML::Node integrated_node;
    integrated_node.push_back(integrate_node);

    // Write the waypoints to the YAML file
    std::ofstream fout(this->object_file_path);
    fout << integrated_node;

    first_point_set = false;

    boost::shared_ptr<geometry_msgs::Pose> mean_ptr(new geometry_msgs::Pose(mean));

    // View Area (in box)
    publishAreas(mean_ptr, size_x, size_y, size_z);
}

void ObjectSaver::publishAreas(const geometry_msgs::Pose::ConstPtr& point, double area_x, double area_y, double area_z){
    static int marker_count = 0;

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.name = "area_marker" + std::to_string(marker_count++);;
    int_marker.scale = 1;

    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = area_x;
    box_marker.scale.y = area_y;
    box_marker.scale.z = area_z;
    box_marker.color.r = 0.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 0.5;

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    int_marker.pose = *point;
    int_marker.controls.push_back(box_control);

    server.insert(int_marker, boost::bind(&ObjectSaver::processFeedback, this, _1));
    server.applyChanges();    
}

void ObjectSaver::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO_STREAM("Feedback from marker '" << feedback->marker_name << "' "
                        << " / control '" << feedback->control_name << "'");
}

void ObjectSaver::executeCallback(const std_msgs::Int16::ConstPtr& msg) {
    if (msg->data == 2) {
        subscribe_sip_node = true;
        ROS_INFO("Received signal to execute Object Saver.");
    } else {
        subscribe_sip_node = false;
        ROS_INFO("Received signal to stop Object Saver execution.");
    }
}

void ObjectSaver::objectNameCallback(const std_msgs::String::ConstPtr& msg) {
    object_name = msg->data;
    ROS_INFO("Received Object Name : %s", object_name.c_str());
}

