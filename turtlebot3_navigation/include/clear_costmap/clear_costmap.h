#ifndef __CLEAR_COSTMAP_H__
#define __CLEAR_COSTMAP_H__

#include <ros/ros.h>
#include <std_srvs/Empty.h>

class ClearCostmap
{
private:
    ros::NodeHandle nh;
	ros::NodeHandle pnh;

    ros::ServiceClient clear_map_client_;

    ros::Timer loop_timer;

    double clear_hz;

    void setService();
    void getParam();
    void setTimer();
    void timerCallback(const ros::TimerEvent& event);

public:
    ClearCostmap();
    ~ClearCostmap();
    void init();


};

#endif