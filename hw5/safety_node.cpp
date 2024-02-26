#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    ros::Publisher brake_publisher;
    ros::Publisher brake_bool_publisher;
    ros::Subscriber odom_subscriber;
    ros::Subscriber scan_subscriber;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        brake_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
        brake_bool_publisher = n.advertise<std_msgs::Bool>("/brake_bool", 1);
        odom_subscriber = n.subscribe("/odom", 1, &Safety::odom_callback, this);
        scan_subscriber = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        double ttc = 10.0;
        double obstacle_distance = scan_msg->ranges[0];
        double obstacle_angle = scan_msg->angle_min;
        double relative_speed = 0.0;
        for (int i = 0; i < scan_msg->ranges.size(); i++) {
            if(scan_msg->ranges[i] >= scan_msg->range_min && scan_msg->ranges[i] <= scan_msg->range_max) {
                obstacle_distance = scan_msg->ranges[i];
                obstacle_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                relative_speed = speed * std::cos(obstacle_angle);
            }
            if(relative_speed  > 0.0) {
                ttc = obstacle_distance / relative_speed;
            }
            if (ttc < 0.6) {
                // Publish emergency brake message
                ackermann_msgs::AckermannDriveStamped brake_msg;
                brake_msg.drive.speed = 0.0; // 设置速度为0
                brake_publisher.publish(brake_msg);
                // Publish boolean message as True
                std_msgs::Bool bool_msg;
                bool_msg.data = true;
                brake_bool_publisher.publish(bool_msg);
                break;
            }
        }
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}