#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

ros::Publisher* twiststampedpub=NULL;

void doMsg(const geometry_msgs::TwistConstPtr& twist)
{
    geometry_msgs::TwistStamped twiststamped;
    twiststamped.header.stamp=ros::Time::now();
    twiststamped.twist.linear.x=twist->linear.x*0.3;
    twiststamped.twist.angular.z=twist->angular.z*0.3;
    twiststampedpub->publish(twiststamped);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"control");
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;

    ros::Publisher pub = nh1.advertise<geometry_msgs::TwistStamped>("/cmd_vel",10);
    twiststampedpub=&pub;

    ros::Subscriber sub{nh2.subscribe("/turtle1/cmd_vel",10,doMsg)};

    ros::spin();

    return 0;
}