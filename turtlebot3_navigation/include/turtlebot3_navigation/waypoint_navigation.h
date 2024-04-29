#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <boost/thread/thread.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

class SendGoal
{
  public:
    SendGoal();
    //~SendGoal();
    bool init();
    //bool GotoGoal();
    bool ClearCostmap();
    bool GoMidDestination();
    bool GoBackHome();

    bool SendMidArrive();
    bool SendFinArrive();
    
    bool Check();
    
    //int start = 0;
    
    int chk_point = 0;

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    // 로봇의 포지션 및 방향 
    ros::Subscriber dest_sub_;
    // 신호 sub 
    ros::Subscriber signal_sub_;
    // 신호 pub
    ros::Publisher signal_pub_;
    
    // 경유지 좌표
    double middle_x,middle_y,middle_z,middle_w;
    double dest_x,dest_y,dest_z,dest_w;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = -0.9;
    double initial_w = 0.9;  // 미정
    double x;
    double y;
    double theta;
    bool Mid_Fin = 0;
    bool Fin_Return = 0;
    int chk_point;
    
    void SetMidDestination(double x_pos,double y_pos,double z_pos,double w_pos);
    void SetFinalDestination(double x_pos,double y_pos,double z_pos,double w_pos);
    void SetInitialDestination(double x_pos,double y_pos,double z_pos,double w_pos);
}; 