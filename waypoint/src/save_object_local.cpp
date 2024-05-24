#include "save_object_local/save_object_local.h"

ObjectSaver::ObjectSaver() : nh("~") {
    setParam();
    execute_sub = nh.subscribe<std_msgs::Int16>("/sip/node", 1, &ObjectSaver::executeCallback, this);
    data_sub = nh.subscribe<std_msgs::String>("/sip/node/name", 1, &ObjectSaver::objectNameCallback, this);
    point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point2", 1, &ObjectSaver::saveObjectToFile, this);
    subscribe_sip_node = false;
    click_subscribe_count = 0;
    point_x, point_y, point_z = 0;
}

void ObjectSaver::setParam() {
    nh.param("object_file_path", this->object_file_path, std::string("/home/asura/turtlebot_ws/src/turtlebot3/waypoint/config/save_objects.yaml"));
}

void ObjectSaver::saveObjectToFile(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!subscribe_sip_node) {
        return;
    }
    
    if(click_subscribe_count < 2){
        point_x += msg->point.x;
        point_y += msg->point.y;
        point_z += msg->point.z;

        click_subscribe_count++;
        ROS_INFO("Point: x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);

        if(click_subscribe_count == 1)  return;
    }
    
    float mean_x, mean_y, mean_z = 0.0;

    mean_x = point_x / 2.0;
    mean_y = point_y / 2.0;
    mean_z = point_z / 2.0;
    
    // Create a YAML node for the point
    YAML::Node point_node;
    point_node["x"] = mean_x;
    point_node["y"] = mean_y;
    point_node["z"] = mean_z;

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
    ROS_INFO("Saved point to clicked_points.yaml: x=%f, y=%f, z=%f", mean_x, mean_y, mean_z);

    // CLEAR
    click_subscribe_count = 0;
    point_x = 0.0;
    point_y = 0.0;
    point_z = 0.0;
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

