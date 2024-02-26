#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>

class WallFollow {
public:
    WallFollow() {
        n = ros::NodeHandle();
        // 订阅激光雷达数据和发布驱动消息的初始化
        lidar_subscriber = n.subscribe(lidarscan_topic, 1, &WallFollow::scan_callback, this);
        drive_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    }
private:
    // PID CONTROL PARAMS
    double kp = 0.5;
    double kd = 0.1;
    double ki = 0;

    double dt = 0.5;
    double prev_error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;

    double target_dist = 1.0;
    double angle_a = M_PI / 9 * 2;
    double angle_b = M_PI / 2;
    double len = 1.0;
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    ros::NodeHandle n;
    ros::Subscriber lidar_subscriber;
    ros::Publisher drive_publisher;


    void pid_control(double error, double velocity) {
        derivative = (error - prev_error) / dt;
        double angle = kp * error + kd * derivative;
        ROS_INFO("angle: %f", -angle);
        prev_error = error;
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = -angle;
        if(std::abs(angle) > 10 * (M_PI / 180.0)) {
            velocity = 0.8;
        } else if(std::abs(angle) > 7 * (M_PI / 180.0)) {
            velocity = 1.3;
        } else if(std::abs(angle) > 5 * (M_PI / 180.0)) {
            velocity = 1.5;
        } else if(std::abs(angle) > 2 * (M_PI / 180.0)) {
            velocity = 2.0;
        }
        ROS_INFO("velocity: %f", velocity);
        drive_msg.drive.speed = velocity;
        drive_publisher.publish(drive_msg);
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        double current_dist = 1.0;
        int index_a = (angle_a - scan_msg->angle_min) / scan_msg->angle_increment;
        int index_b = (angle_b - scan_msg->angle_min) / scan_msg->angle_increment;
        int index_c = (M_PI / 3 - scan_msg->angle_min) / scan_msg->angle_increment;

        double dist_a = scan_msg->ranges[index_a];
        double dist_b = scan_msg->ranges[index_b];
        double dist_c = scan_msg->ranges[index_c];
        

        double theta = angle_b - angle_a;
        double alpha = std::atan((dist_a * std::cos(theta) - dist_b) / (dist_a * std::sin(theta)));


        current_dist = dist_b * std::cos(alpha) + len * std::sin(alpha);

        ROS_INFO("current_dist: %f", current_dist);
        double velocity = 10;

        double dist_cal = dist_a * dist_b * std::sin(theta) / (dist_a * std::sin(M_PI / 9) + dist_b * std::sin(M_PI / 6));
        if(std::abs(dist_cal - dist_c) >= 0.028) {
            velocity = 4.0;
        }
        double error = target_dist - current_dist;
        ROS_INFO("error: %f", error);
        
        pid_control(error, velocity);
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follow;
    ros::spin();
    return 0;
}