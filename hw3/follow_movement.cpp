#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

// 记录目标乌龟的位置 
double x;
double y;
// 记录自身位置 
double sx;
double sy;
double yaw = 0.0;  // 偏航角，初始为0

void targetPoseGet(const turtlesim::Pose& msg) {
    x = msg.x;
    y = msg.y;
}	

void selfPoseGet(const turtlesim::Pose& msg) {
    sx = msg.x;
    sy = msg.y;
    yaw = msg.theta;  // 更新偏航角
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_turtle_movement");
    ros::NodeHandle n;
    // 接收主控乌龟的位姿 
    ros::Subscriber sub1 = n.subscribe("/turtle1/pose", 1000, targetPoseGet);
    // 接收跟随乌龟的位姿
    ros::Subscriber sub2 = n.subscribe("/turtle2/pose", 1000, selfPoseGet);
    
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);

    ros::Rate loopRate(10);
	ros::spinOnce();
    while (ros::ok()) {
        geometry_msgs::Twist msg;

        // 修改小乌龟的线速度和角速度

        // 计算两个乌龟之间的角度差
        double desired_angle = atan2(y - sy, x - sx);
        // 计算角度差
        double angle_diff = desired_angle - yaw;
        // 通过角度差调整角速度
        msg.angular.z = angle_diff;
        // 线速度
        msg.linear.x = 0.8;

        // 确定发送的消息
        vel_pub.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
