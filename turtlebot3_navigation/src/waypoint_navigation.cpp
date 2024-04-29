#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void readWaypoints(const std::string& file_path, std::vector<geometry_msgs::PoseStamped>& waypoints) {
    YAML::Node yaml_file = YAML::LoadFile(file_path);
    for (const auto& node : yaml_file) {
        geometry_msgs::PoseStamped pose;
        // pose.header.seq = node["header"]["seq"].as<uint32_t>();
        // pose.header.stamp = ros::Time::now();
        // pose.header.frame_id = "map";
        pose.pose.position.x = node["point"]["x"].as<float>();
        pose.pose.position.y = node["point"]["y"].as<float>();
        pose.pose.position.z = 0.0;
        waypoints.push_back(pose);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigation");
    ros::NodeHandle nh("~");

    // Load waypoints from YAML file
    std::string file_path;
    nh.param<std::string>("file_path", file_path, "/home/asura/turtlebot_ws/src/turtlebot3/turtlebot3_navigation/config/clicked_points.yaml");
    std::vector<geometry_msgs::PoseStamped> waypoints;
    readWaypoints(file_path, waypoints);

    // Initialize action client
    MoveBaseClient action_client("move_base", true);
    while (!action_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Send waypoints as goals
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
            ROS_INFO("Reached waypoint %d", waypoint.header.seq);
        } else {
            ROS_ERROR("Failed to reach waypoint %d", waypoint.header.seq);
            break;
        }
    }

    return 0;
}



// void SendGoal::poseMsgCallBack(const rudolph_msgs::web_rasp::ConstPtr &msg) {
//     middle_x = msg->mid_x, middle_y = msg->mid_y;
//     middle_z = msg->mid_z, middle_w = msg->mid_w;
//     dest_x = msg->fin_x, dest_y = msg->fin_y;
//     dest_z = msg->fin_z, dest_w = msg->fin_w;
//     start = msg->state;
//     std::cout << "dest_val 작동완료!!\n";
// }

// void SendGoal::setGoal(double x_pos, double y_pos, double z_pos, double w_pos) {
//     move_base_msgs::MoveBaseGoal goal;
//     goal.target_pose.header.stamp = ros::Time::now();
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.pose.position.x = x_pos;
//     goal.target_pose.pose.position.y = y_pos;
//     goal.target_pose.pose.position.z = 0.0;
//     goal.target_pose.pose.orientation.x = 0.0;
//     goal.target_pose.pose.orientation.y = 0.0;
//     goal.target_pose.pose.orientation.z = z_pos;
//     goal.target_pose.pose.orientation.w = w_pos;
//     return goal;
// }

// void SendGoal::sendGoal(const move_base_msgs::MoveBaseGoal &goal) {
//     MoveBaseClient client("move_base", true);
//     client.waitForServer();
//     client.sendGoal(goal);
//     ROS_INFO("Waiting for the result");
//     bool finished_before_timeout = client.waitForResult(ros::Duration(1.0));
//     if (finished_before_timeout) {
//         actionlib::SimpleClientGoalState state = client.getState();
//         ROS_INFO("Action finished: %s", state.toString().c_str());
//         start++;
//     } else {
//         ROS_INFO("Action did not finish before the time out.");
//     }
// }

// void SendGoal::sendMidArrive() {
//     if (start == 2) {
//         std::cout << "경유지 도착!!!!!!\n";
//         rudolph_msgs::rasp_arduino sig;
//         sig.mid_arrive = true;
//         sig.fin_arrive = false;
//         signal_pub_.publish(sig);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep();
//         if (Mid_Fin) {
//             std::cout << "경유지에서 목적지로!!!!!!\n";
//             start = 3;
//         }
//     }
// }

// void SendGoal::sendFinArrive() {
//     if (start == 7) {
//         std::cout << "목적지 도착!!!!!\n";
//         rudolph_msgs::rasp_arduino sig;
//         sig.mid_arrive = false;
//         sig.fin_arrive = true;
//         signal_pub_.publish(sig);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep();
//         if (Fin_Return) {
//             std::cout << "목적지에서 출발지로!!!!!!\n";
//             start = 8;
//         }
//     }
// }

// void SendGoal::sendGoals() {
//     if (start == 1) setGoal(middle_x, middle_y, middle_z, middle_w);
//     if (start == 3) setGoal(34.000, 12.12, 0.097, 0.99);
//     if (start == 4) setGoal(38.037, 12.000, -0.5148, 0.976);
//     if (start == 6) setGoal(dest_x, dest_y, dest_z, dest_w);
// }