#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netdb.h>
#include <stdexcept>
#include <thread>
#include <cstring>
#include "cobs.h"
#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

using json = nlohmann::json;

ros::Publisher posePub, jointStatePub, imuPub;
geometry_msgs::Twist cmd_vel;

void twistCallback(const geometry_msgs::TwistConstPtr &msg){
    cmd_vel = *msg;
}

void addData(json &j){
    j["setpoint"]["dx"] = cmd_vel.linear.x;
    j["setpoint"]["dy"] = cmd_vel.linear.y;
    j["setpoint"]["dtheta"] = cmd_vel.angular.z;
}

void getData(const json &j){
    /**
     * Pose pub
     */
    geometry_msgs::Pose pose;

    pose.position.x = j["pose"]["x"];
    pose.position.y = j["pose"]["y"];
    pose.position.z = 0;

    pose.orientation.x = j["imu_quat"]["x"];
    pose.orientation.y = j["imu_quat"]["y"];
    pose.orientation.z = j["imu_quat"]["z"];
    pose.orientation.w = j["imu_quat"]["w"];

    posePub.publish(pose);

    /**
     * Tf Broadcast
     */
     static tf::TransformBroadcaster odom_broadcaster;
    tf::Vector3 vec;
    vec.setX(pose.position.x);
    vec.setY(pose.position.y);
    vec.setZ(pose.position.z);

    tf::Quaternion quat;
    quat.setX(pose.orientation.x);
    quat.setY(pose.orientation.y);
    quat.setZ(pose.orientation.z);
    quat.setW(pose.orientation.w);

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

    /**
     * IMU pub
     */
    sensor_msgs::Imu imu;
    imu.orientation_covariance[0] = -1;
    imu.angular_velocity_covariance[0] = -1;
    imu.linear_acceleration_covariance[0] = -1;

    imu.orientation = pose.orientation;

    //TODO
    //imu.angular_velocity = angVel;
    //imu.linear_acceleration = accel;
    imuPub.publish(imu);

    /**
     * Joint State
     */
    sensor_msgs::JointState jointState;
    jointState.position = std::vector<double>{
        j["wheel_pos"]["frontLeft"],
        j["wheel_pos"]["frontRight"],
        j["wheel_pos"]["backLeft"],
        j["wheel_pos"]["backRight"]
    };

    jointState.velocity = std::vector<double>{
            j["wheel_vel"]["frontLeft"],
            j["wheel_vel"]["frontRight"],
            j["wheel_vel"]["backLeft"],
            j["wheel_vel"]["backRight"]
    };

    jointState.effort = std::vector<double>{
            j["wheel_effort"]["frontLeft"],
            j["wheel_effort"]["frontRight"],
            j["wheel_effort"]["backLeft"],
            j["wheel_effort"]["backRight"]
    };

    jointStatePub.publish(jointState);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "robotto_tcp_comms");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    posePub = n.advertise<geometry_msgs::Pose>("pose", 10);
    jointStatePub = n.advertise<sensor_msgs::JointState>("joint_state", 10);
    imuPub = n.advertise<sensor_msgs::Imu>("imu", 10);

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 20, twistCallback);

    int sock = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (!sock) throw std::runtime_error("Error opening socket!");

    struct hostent *server = nullptr;
    std::string serverIp;
    while(!nh.getParam("server_ip", serverIp)){
        ROS_WARN("Needs a server_ip!");
    }
    server = gethostbyname(serverIp.c_str());
    if (server == nullptr) {
        throw std::runtime_error("Invalid host");
    }

    struct sockaddr_in servAddr;
    memset(&servAddr, 0, sizeof(servAddr));
    memcpy(&servAddr.sin_addr, server->h_addr_list[0], server->h_length);
    servAddr.sin_family = AF_INET;

    int port;
    if(!nh.getParam("server_port", port)){
        ROS_WARN("Using default port: 80");
        port = 80;
    }
    servAddr.sin_port = htons(port);

    int ret = connect(sock, (struct sockaddr *) &servAddr, sizeof(servAddr));
    if (ret == -1) {
        if (errno != EINPROGRESS)
            throw std::runtime_error("Could not connect to TCP Server " + std::to_string(errno));
    }

    int epfd = epoll_create(1);
    struct epoll_event ev;

    if (epfd == -1) {
        throw std::runtime_error("Error creating epoll");
    }

    ev.data.fd = sock;
    ev.events = EPOLLIN;

    if (epoll_ctl(epfd, EPOLL_CTL_ADD, sock, &ev) == -1) {
        throw std::runtime_error("Error at epoll_ctl()");
    }

    struct epoll_event evlist[5];

    const size_t bufferLen = 4096 * 10;
    uint8_t buffer[bufferLen];
    uint8_t cobsDecodeBuffer[bufferLen];
    uint8_t cobsEncodeBuffer[bufferLen];

    size_t currentLen = 0;

    std::thread writeLoop([&] {
        int write_epfd;
        struct epoll_event write_ev;
        struct epoll_event write_evlist[5];

        write_epfd = epoll_create(1);

        if (write_epfd == -1) {
            throw std::runtime_error("Error creating epoll");
        }

        write_ev.data.fd = sock;
        write_ev.events = EPOLLOUT;

        if (epoll_ctl(write_epfd, EPOLL_CTL_ADD, sock, &write_ev) == -1) {
            throw std::runtime_error("Error at epoll_ctl()");
        }

        unsigned int i = 0;

        while (ros::ok()) {
            int ret = epoll_wait(write_epfd, write_evlist, 5, -1);
            if(ret != -1){
                json j;
                j["i"] = i; i++;
                addData(j);

                auto data = json::to_msgpack(j);

                auto res = cobs_encode(cobsEncodeBuffer, sizeof(cobsEncodeBuffer), data.data(), data.size());
                if (res.status == COBS_ENCODE_OK) {
                    cobsEncodeBuffer[res.out_len++] = 0x00;
                    int ret = write(sock, cobsEncodeBuffer, res.out_len);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    });

    while (ros::ok()) {
        int ret = epoll_wait(epfd, evlist, 5, -1);
        if (ret != -1) {
            auto len = read(sock, buffer + currentLen, sizeof(buffer) - currentLen) + currentLen;

            for (int i = 0; i < len; ++i) {
                if (buffer[i] == 0x00) {
                    //Found end of package
                    auto res = cobs_decode(cobsDecodeBuffer, sizeof(cobsDecodeBuffer), buffer, i);
                    if (res.status == COBS_DECODE_OK && res.out_len > 2) {
                        std::vector<uint8_t> data( cobsDecodeBuffer + 2, cobsDecodeBuffer + res.out_len);
                        //std::string s((char *) cobsDecodeBuffer + 2, res.out_len - 2);
                        int dec_size = ((cobsDecodeBuffer[1] & 0xff) << 8) | ((cobsDecodeBuffer[0] & 0xff));

                        try {
                            json j = json::from_msgpack(data);
                            getData(j);
                            //std::cout << s << std::endl;
                        } catch (...){

                        }

                        //std::cout << j["hb"] << std::endl;
                    }

                    memmove(buffer, buffer + i + 1, (len + currentLen) - 1 - i);
                    currentLen = len - i - 1;
                    len = len - i - 1;
                    i = 0;
                }
            }
        } else {
            std::cout << errno << std::endl;
        }
        ros::spinOnce();
    }

    close(epfd);
    close(sock);
    writeLoop.join();

    return 0;
}
