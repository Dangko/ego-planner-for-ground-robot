#include "ros/ros.h"

//ros msgs
#include <std_msgs/String.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle *n;

ros::Publisher cmd_vel_pub;
ros::Publisher cmd_flipper_pub;

geometry_msgs::Twist cmd_vel,cmd_flipper;
double max_vel,max_angular;

//joy手柄回调函数
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double vel = 0;
    double omega = 0;

   
    vel = joy->axes[1]*max_vel;
    omega = joy->axes[2]*max_angular;

    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;

    if(joy->buttons[6]==1)
    {
        cmd_flipper.linear.x+=0.1;
        cmd_flipper.linear.y+=0.1;
    }
    else if(joy->buttons[8]==1)
    {
        cmd_flipper.linear.x-=0.1;
        cmd_flipper.linear.y-=0.1;
    }

    if(joy->buttons[7]==1)
    {
        cmd_flipper.angular.x+=0.1;
        cmd_flipper.angular.y+=0.1;
    }
    else if(joy->buttons[9]==1)
    {
        cmd_flipper.angular.x-=0.1;
        cmd_flipper.angular.y-=0.1;
    }    

    cmd_vel_pub.publish(cmd_vel);
    cmd_flipper_pub.publish(cmd_flipper);
    
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "joy_control_node");
    ros::NodeHandle n;
	n.param<double>("max_vel", max_vel, 10.0);  
	n.param<double>("max_angular", max_angular, 2.0);  

    ros::Subscriber sub_joy = n.subscribe<sensor_msgs::Joy>("/joy",10,joy_callback);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel1",10);
    cmd_flipper_pub = n.advertise<geometry_msgs::Twist>("/cmd_flipper",10);

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
