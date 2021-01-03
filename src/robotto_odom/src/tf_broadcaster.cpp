#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imuPub;
geometry_msgs::Vector3 accel, angVel;

void accelCallback(const geometry_msgs::Vector3ConstPtr &msg){
    accel = *msg;
}

void angVelCallback(const geometry_msgs::Vector3ConstPtr &msg){
    angVel = *msg;
}

void odomCallback(const geometry_msgs::PoseConstPtr & msg){
    static tf::TransformBroadcaster odom_broadcaster;
    tf::Vector3 vec;
    vec.setX(msg->position.x);
    vec.setY(msg->position.y);
    vec.setZ(msg->position.z);

    tf::Quaternion quat;
    quat.setX(msg->orientation.x);
    quat.setY(msg->orientation.y);
    quat.setZ(msg->orientation.z);
    quat.setW(msg->orientation.w);

    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Quaternion stab_quat = tf::createQuaternionFromRPY(0, 0, yaw);

    tf::Vector3 footVec;
    footVec.setZ(0);

    tf::StampedTransform trans(tf::Transform(quat, vec), ros::Time::now(), "odom", "base_link");
    tf::StampedTransform transStab(tf::Transform(stab_quat, vec), ros::Time::now(), "odom", "base_stabilized");
    tf::StampedTransform transFoot(tf::Transform(stab_quat, footVec), ros::Time::now(), "odom", "base_footprint");

    odom_broadcaster.sendTransform(trans);
    odom_broadcaster.sendTransform(transStab);
    odom_broadcaster.sendTransform(transFoot);

    sensor_msgs::Imu imu;
    imu.orientation_covariance[0] = -1;
    imu.angular_velocity_covariance[0] = -1;
    imu.linear_acceleration_covariance[0] = -1;

    imu.orientation = msg->orientation;
    imu.angular_velocity = angVel;
    imu.linear_acceleration = accel;

    imuPub.publish(imu);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle n;
    imuPub = n.advertise<sensor_msgs::Imu>("imu", 20);
    ros::Subscriber odom_sub = n.subscribe("pose", 20, odomCallback);
    ros::Subscriber ang_vel_sub = n.subscribe("imu_raw/angular_velocity", 20, angVelCallback);
    ros::Subscriber accel_sub = n.subscribe("imu_raw/linear_acceleration", 20, accelCallback);

    ros::spin();
    return 0;
}