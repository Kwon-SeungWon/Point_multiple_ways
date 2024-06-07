#include "clear_costmap/clear_costmap.h"

ClearCostmap::ClearCostmap():pnh("~")
{
    init();
}

ClearCostmap::~ClearCostmap()
{

}

void ClearCostmap::setService()
{
    clear_map_client_ = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps"); 

}

void ClearCostmap::getParam()
{
    pnh.param("clear_hz",clear_hz,1.0);
}

void ClearCostmap::setTimer()
{
    loop_timer = nh.createTimer(ros::Duration(1/clear_hz),&ClearCostmap::timerCallback,this);
}

void ClearCostmap::timerCallback(const ros::TimerEvent& event)
{
    std_srvs::Empty srv;
    if(clear_map_client_.call(srv))
        ROS_INFO("[%s] Clear costmap",ros::this_node::getName().c_str());
    else
        ROS_ERROR("[%s] Cannot clear costmap",ros::this_node::getName().c_str());

}

void ClearCostmap::init()
{
    getParam();
    setService();
    setTimer();
}