#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

static ros::Publisher twistPub;

void joyCb(const sensor_msgs::JoyConstPtr &joy){
    geometry_msgs::Twist twist;
    twist.linear.x = joy->axes[4];
    twist.linear.y = joy->axes[3];
    twist.angular.z = joy->axes[0] * M_PI;

    twistPub.publish(twist);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "manual_control_node");

    ros::NodeHandle n;
    twistPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber odom_sub = n.subscribe("joy", 20, joyCb);

    ros::spin();
    return 0;
}