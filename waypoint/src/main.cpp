#include <ros/ros.h>
#include "save_clicked_point/save_clicked_point.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_clicked_point_node");
    PointSaver point_saver;
    ros::spin();
    return 0;
}
