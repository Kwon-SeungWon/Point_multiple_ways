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
    YAML::Node yaml_file = YAML::LoadFile(file_path);
    for (const auto& node : yaml_file) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = node["point"]["x"].as<float>();
        pose.pose.position.y = node["point"]["y"].as<float>();
        pose.pose.position.z = 0.0;
        waypoints.push_back(pose);
    }
}

// Function to send waypoints as goals
void sendWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints, MoveBaseClient& action_client) {
    for (const auto& waypoint : waypoints) {
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
}

// Function to go back to initial pose
void goBackToInitialPose(const geometry_msgs::PoseStamped& initial_pose, MoveBaseClient& action_client) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = initial_pose;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";

    action_client.sendGoal(goal);
    action_client.waitForResult();
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Returned to initial pose");
    } else {
        ROS_ERROR("Failed to return to initial pose");
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

    std::vector<geometry_msgs::PoseStamped> waypoints;
    std::string file_path;
    nh.param<std::string>("file_path", file_path, "/home/asura/turtlebot_ws/src/turtlebot3/waypoint/config/clicked_points.yaml");
    readWaypoints(file_path, waypoints);

    MoveBaseClient action_client("move_base", true);
    while (!action_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, saveInitialPoseCallback);

    // Wait for the initial pose to be received
    while (ros::ok() && !initial_pose_received) {
        ros::spinOnce();
        ros::Duration(0.1).sleep(); // Sleep for a short duration
    }

    if (!waypoints.empty()) {
        sendWaypoints(waypoints, action_client);

        // Go back to initial pose if all waypoints are reached
        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            goBackToInitialPose(initial_pose, action_client);
        }
    } else {
        ROS_WARN("No waypoints loaded. Exiting...");
    }

    return 0;
}
