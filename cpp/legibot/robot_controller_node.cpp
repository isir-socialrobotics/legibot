#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "legibot/robot_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_controller");
    ros::NodeHandle node;

    ROS_INFO("Robot controller node started");
    Robot robot("PepperBot");
    ROS_INFO("Robot controller node initialized");

    robot.cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);  // freq = 1000 Hz
    robot.odom_sub = node.subscribe("/odom", 100, &Robot::robotPoseCallback, &robot);
    robot.goals_sub = node.subscribe("/pepper/goals", 100, &Robot::robotGoalsCallback, &robot);

    ROS_INFO("Publishers created");

    // Parse goal coordinates from command-line arguments
    double Kpv = 2, Kpw = 3;  // fixme: To not be hardcoded
    robot.Kpv = Kpv;
    robot.Kpw = Kpw;

    robot.exec();
    ros::spin();
    return 0;
}
