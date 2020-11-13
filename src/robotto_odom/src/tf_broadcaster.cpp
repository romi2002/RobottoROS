#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::OdometryConstPtr & msg){
    static tf::TransformBroadcaster odom_broadcaster;
    tf::Vector3 vec;
    vec.setX(msg->pose.pose.position.x);
    vec.setY(msg->pose.pose.position.y);
    vec.setZ(msg->pose.pose.position.z);

    tf::Quaternion quat;
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    quat.setW(msg->pose.pose.orientation.w);
    quat = tf::createQuaternionFromRPY(0,0,0);

    tf::StampedTransform trans(tf::Transform(quat, vec), ros::Time::now(), "odom", "base_link");
    odom_broadcaster.sendTransform(trans);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("odom", 20, odomCallback);

    ros::spin();
    return 0;
}