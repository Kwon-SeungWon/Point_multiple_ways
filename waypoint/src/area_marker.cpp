#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

class AreaMarker {
public:
    AreaMarker() : nh_("~"), server_("area_marker_server") {
        pose_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point2", 10, &AreaMarker::poseCallback, this);
        // initInteractiveMarker();
    }

    // Function to initialize InteractiveMarker
    void initInteractiveMarker() {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = "area_marker";
        int_marker.scale = 1;

        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.scale.x = 1;
        box_marker.scale.y = 1;
        box_marker.scale.z = 0.01;
        box_marker.color.r = 0.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 0.5;

        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);

        int_marker.controls.push_back(box_control);

        server_.insert(int_marker, boost::bind(&AreaMarker::processFeedback, this, _1));
        server_.applyChanges();

    }

    // Callback function for handling clicked points
    void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (!first_point_set_) {
            first_point_ = msg->point;
            first_point_set_ = true;
            ROS_INFO("First point of diagonal - X: %f, Y: %f, Z: %f", first_point_.x, first_point_.y, first_point_.z);
        } else {
            geometry_msgs::Point second_point = msg->point;
            ROS_INFO("Second point of diagonal - X: %f, Y: %f, Z: %f", second_point.x, second_point.y, second_point.z);

            geometry_msgs::Pose pose;
            pose.position.x = (first_point_.x + second_point.x) / 2;
            pose.position.y = (first_point_.y + second_point.y) / 2;
            pose.position.z = (first_point_.z + second_point.z) / 2;

            double size_x = fabs(first_point_.x - second_point.x);
            double size_y = fabs(first_point_.y - second_point.y);
            double size_z = fabs(first_point_.z - second_point.z);

            visualization_msgs::InteractiveMarker int_marker;
            int_marker.header.frame_id = "map";
            int_marker.name = "area_marker";
            int_marker.scale = 1;

            visualization_msgs::Marker box_marker;
            box_marker.type = visualization_msgs::Marker::CUBE;
            box_marker.scale.x = size_x;
            box_marker.scale.y = size_y;
            box_marker.scale.z = size_z;
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.5;

            visualization_msgs::InteractiveMarkerControl box_control;
            box_control.always_visible = true;
            box_control.markers.push_back(box_marker);

            int_marker.pose = pose;
            int_marker.controls.push_back(box_control);

            server_.insert(int_marker, boost::bind(&AreaMarker::processFeedback, this, _1));
            server_.applyChanges();

            // Reset for next square
            first_point_set_ = false;
        }
    }

    // Function to handle InteractiveMarker feedback
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        ROS_INFO_STREAM("Feedback from marker '" << feedback->marker_name << "' "
                        << " / control '" << feedback->control_name << "'");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    interactive_markers::InteractiveMarkerServer server_;
    visualization_msgs::Marker marker;


    // Variables to track first clicked point
    geometry_msgs::Point first_point_;
    bool first_point_set_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "area_marker_node");

    AreaMarker area_marker;

    ros::spin();

    return 0;
}
