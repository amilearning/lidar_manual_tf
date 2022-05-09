#include <ros/ros.h>
#include <dynamic_reconfigure_test/euler.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_test/dyn_reconfig_testConfig.h>


void msgcallBack(const dynamic_reconfigure_test::euler::ConstPtr& msg) {
    ROS_INFO("x : %f", msg->x);
    ROS_INFO("y : %f", msg->y);
    ROS_INFO("z : %f", msg->z);
    ROS_INFO("roll : %f", msg->roll);
    ROS_INFO("pitch : %f", msg->pitch);
    ROS_INFO("yaw : %f", msg->yaw);    
}

int main(int argc, char** argv)
{
    // ros node 초기화
    ros::init(argc, argv, "msg_sub_node");
    ros::NodeHandle nh;
 
    ros::Rate r(1);
 
    ros::Subscriber sub = nh.subscribe<dynamic_reconfigure_test::euler>("/euler_data", 1, msgcallBack);
    dynamic_reconfigure_test::euler msg;
 
    while(ros::ok())
    {
        ros::spinOnce();
        
        r.sleep();
    }
 
    return 0;
}
 