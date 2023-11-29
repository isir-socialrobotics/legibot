#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

class Robot
{
private:
    std::string name;
    double x=INFINITY, y=INFINITY, z, roll, pitch, yaw;
    double t = 0;

    double getDistance(double x1, double y1, double x2, double y2);

    double saturate(double X, double Xmin, double Xmax);
//    bool test_publish(double v, double w);

public:

    Robot(std::string n);  // Constructor
    ros::Publisher cmd_vel_pub;

    ros::Subscriber odom_sub;
    ros::Subscriber goals_sub;
    geometry_msgs::PoseArray goals;

    std::vector<bool> goals_reached;

    double Kpv, Kpw;  // Kp coefficients for linear and angular velocity PID controllers
//    double x_d, y_d, yaw_d, roll_d, pitch_d;

//    void control(double xd, double yd);
    bool exec();

    void robotPoseCallback(const nav_msgs::OdometryConstPtr& msg);
    void robotGoalsCallback(const geometry_msgs::PoseArrayConstPtr& msg);
};

#endif // ROBOT_CONTROLLER_H