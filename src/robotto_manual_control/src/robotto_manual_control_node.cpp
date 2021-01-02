#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

static ros::Publisher twistPub;
bool lastResetYaw = false;
double latestYaw = 0;
double zeroAngle = 0;
bool useFieldOriented = false;

double wrapAngle(double angle) {
    angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void joyCb(const sensor_msgs::JoyConstPtr &joy){
    geometry_msgs::Twist twist;
    twist.linear.x = joy->axes[4];
    twist.linear.y = joy->axes[3];
    twist.angular.z = joy->axes[0] * M_PI * 2;
    bool yawButton = joy->buttons[0];

    double yaw = latestYaw;

    if(lastResetYaw != yawButton and yawButton){
        zeroAngle = yaw;
    }

    yaw = wrapAngle(yaw - zeroAngle);

    if(useFieldOriented){
        double temp = twist.linear.x * std::cos(yaw) + twist.linear.y * std::sin(yaw);
        twist.linear.y = -twist.linear.x * std::sin(yaw) + twist.linear.y * std::cos(yaw);
        twist.linear.x = temp;
    }

    twistPub.publish(twist);
    lastResetYaw = joy->buttons[0];
}

void poseCb(const geometry_msgs::PoseConstPtr &msg){
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);

    double pitch, roll;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, latestYaw);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    if(nh.hasParam("enableFieldOriented")){
        nh.getParam("enableFieldOriented", useFieldOriented);
    }

    twistPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber joy_sub = n.subscribe("joy", 20, joyCb);
    ros::Subscriber odom_sub = n.subscribe("pose", 20, poseCb);

    ros::spin();
    return 0;
}