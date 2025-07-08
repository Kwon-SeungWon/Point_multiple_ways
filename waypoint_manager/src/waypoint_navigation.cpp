#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseStamped initial_pose;
bool initial_pose_received = false;

// Function to read waypoints from YAML file
void readWaypoints(const std::string& file_path, std::vector<geometry_msgs::PoseStamped>& waypoints) {
    ROS_INFO("Attempting to read waypoints from: %s", file_path.c_str());
    YAML::Node yaml_file = YAML::LoadFile(file_path);
    ROS_INFO("YAML file loaded successfully");
    
    for (const auto& node : yaml_file) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = node["point"]["x"].as<float>();
        pose.pose.position.y = node["point"]["y"].as<float>();
        pose.pose.position.z = node["point"]["z"].as<float>();
        
        // Set default orientation (facing forward)
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        waypoints.push_back(pose);
        ROS_INFO("Added waypoint: x=%.2f, y=%.2f, z=%.2f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
    ROS_INFO("Total waypoints loaded: %zu", waypoints.size());
}

// Function to send waypoints as goals in continuous loop
void sendWaypointsLoop(const std::vector<geometry_msgs::PoseStamped>& waypoints, MoveBaseClient& action_client) {
    while (ros::ok()) {
        for (const auto& waypoint : waypoints) {
            if (!ros::ok()) break;
            
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = waypoint;
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
            
            action_client.sendGoal(goal);
            action_client.waitForResult();
            if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Reached waypoint");
            } else {
                ROS_ERROR("Failed to reach waypoint");
                break;
            }
        }
        ROS_INFO("Completed one cycle of waypoints, starting again...");
    }
}

void saveInitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    initial_pose.pose.position = msg->pose.pose.position;
    initial_pose.pose.orientation = msg->pose.pose.orientation;
    initial_pose_received = true;
    ROS_INFO("Received initial pose");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigation");
    ros::NodeHandle nh("~");
    
    ROS_INFO("Waypoint navigation node started");

    std::vector<geometry_msgs::PoseStamped> waypoints;
    std::string file_path;
    nh.param<std::string>("file_path", file_path, "/home/san/catkin_ws/src/waypoint_manager/config/move_paths.yaml");
    ROS_INFO("File path parameter: %s", file_path.c_str());
    readWaypoints(file_path, waypoints);

    ROS_INFO("Connecting to move_base action server...");
    MoveBaseClient action_client("move_base", true);
    while (!action_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Connected to move_base action server");

    ROS_INFO("Subscribing to /initialpose topic...");
    ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, saveInitialPoseCallback);

    // Wait for the initial pose to be received
    ROS_INFO("Waiting for initial pose to be set...");
    while (ros::ok() && !initial_pose_received) {
        ros::spinOnce();
        ros::Duration(0.1).sleep(); // Sleep for a short duration
    }
    ROS_INFO("Initial pose received, starting waypoint navigation");

    if (!waypoints.empty()) {
        ROS_INFO("Starting waypoint loop with %zu waypoints", waypoints.size());
        sendWaypointsLoop(waypoints, action_client);
    } else {
        ROS_WARN("No waypoints loaded. Exiting...");
    }

    return 0;
}
