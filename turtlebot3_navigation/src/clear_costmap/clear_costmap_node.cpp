#include "clear_costmap/clear_costmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clear_costmap");
	ros::Time::init();
    ClearCostmap x;
    ros::spin();
}