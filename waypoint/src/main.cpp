#include <ros/ros.h>
#include "save_clicked_point/save_clicked_point.h"
#include "save_object_local/save_object_local.h"
#include "selection_saver/selection_saver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_data");
    ros::NodeHandle nh;
    PointSaver point_saver;
    ObjectSaver object_saver;
    //SelectionSaver selection_saver;
    ros::spin();
    return 0;
}
