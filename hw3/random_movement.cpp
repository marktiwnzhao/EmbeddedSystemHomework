#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "random_turtle_movement");
    ros::NodeHandle node;

    // 创建一个发布器来发布控制小乌龟移动的消息
    ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // 随机数生成器的初始化
    srand(time(0));

    ros::Rate loop_rate(1);  // 控制发布速率，每秒发布一次

    while (ros::ok()) {
        geometry_msgs::Twist msg;

        // 生成随机线速度和角速度
        msg.linear.x = double(rand()) / double(RAND_MAX);
        msg.angular.z = double(rand()) / double(RAND_MAX) - 1.0;

        // 发布移动消息
        vel_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}