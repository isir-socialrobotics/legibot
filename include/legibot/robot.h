#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

class Robot
{
private:
    std::string name;
    double x, y, z, roll, pitch, yaw;
    double t = 0;

public:
    Robot(std::string n);

    ros::Publisher vel_pub;
    ros::Subscriber odometry;


    double x_d, y_d, yaw_d, roll_d, pitch_d;

    bool test_publish(double v, double w);
    double getDistance(double x1, double y1, double x2, double y2);
    void robotPoseCallback(const nav_msgs::OdometryConstPtr& msg);
    void control(double xd, double yd);
    bool control_go_to_goal(double xd, double yd, double Kpv, double Kpw);
    double saturate(double X, double Xmin, double Xmax);
};

#endif