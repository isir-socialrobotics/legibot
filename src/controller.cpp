#include "ros/ros.h"
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include "legibot/robot.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_controller");
    ros::NodeHandle n;

    Robot robot("turtlebot3_burger");

    // cmd_vel publisher
    robot.vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    robot.odometry = n.subscribe("odom", 100, &Robot::robotPoseCallback, &robot);


    robot.control_go_to_goal(-0.5, -7.1, 2, 3);
    

    ros::spin();

    return 0;
}


