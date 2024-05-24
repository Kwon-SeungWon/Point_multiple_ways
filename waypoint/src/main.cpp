#include <ros/ros.h>
#include "save_move_path/save_move_path.h"
#include "save_object_local/save_object_local.h"
#include "save_polygon_local/save_polygon_local.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_data");
    ros::NodeHandle nh;

    PointSaver point_saver;
    ObjectSaver object_saver;
    PolygonSaver polygon_saver;
    
    ros::spin();
    return 0;
}
