#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;

    ros::Publisher pub{nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel",10)};

    geometry_msgs::TwistStamped vel;

    vel.twist.linear.x=3;
    vel.twist.angular.z=2;

    while (ros::ok()){
        vel.header.stamp=ros::Time::now();
        pub.publish(vel);

        ros::spinOnce();
    }

    return 0;
}