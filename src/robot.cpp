#include "legibot/robot.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>  

Robot::Robot(std::string n)
{
    name = n;
    
}

bool Robot::test_publish(double v, double w){

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        Robot::vel_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}

void Robot::robotPoseCallback(const nav_msgs::OdometryConstPtr& msg){
    x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

}

int signnum_typical(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

void Robot::control(double xd, double yd){
    double linear_error, angular_error, angular_cmd, linear_cmd;

    ros::Rate loop_rate(30.3);
    while(ros::ok())
    {
        linear_error = getDistance(-3+0.7*sin(2*3.14*t/30), 4.3+0.7*sin(4*3.14*t/30), x, y);
        angular_error = atan2(4.3+0.7*sin(4*3.14*t/30) - y, -3+0.7*sin(2*3.14*t/30) - x) - yaw;

        angular_cmd = 3 * atan(tan(angular_error));
        linear_cmd = 2 * linear_error * signnum_typical(cos(angular_error));

        linear_cmd = saturate(linear_cmd, 0, 0.22);
        angular_cmd = saturate(angular_cmd, -2.5, 2.5);

        geometry_msgs::Twist msg;
        msg.linear.x = linear_cmd;
        msg.angular.z = angular_cmd;
        Robot::vel_pub.publish(msg);

        t = t + 0.033;

        ROS_INFO("Robot linear velocity: %f m/s", linear_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    Robot::vel_pub.publish(msg);
}

bool Robot::control_go_to_goal(double xd, double yd, double Kpv, double Kpw){
    double linear_error, angular_error, angular_cmd, linear_cmd;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        linear_error = getDistance(xd, yd, x, y);
        angular_error = atan2(yd - y, xd - x) - yaw;

        angular_cmd = Kpw * atan(tan(angular_error));
        linear_cmd = Kpv * linear_error * signnum_typical(cos(angular_error));

        linear_cmd = saturate(linear_cmd, -0.22, 0.22);
        angular_cmd = saturate(angular_cmd, -2.5, 2.5);

        //ROS_INFO("%f", linear_cmd);
        //ROS_INFO("%f", angular_error);

        geometry_msgs::Twist msg;
        msg.linear.x = linear_cmd;
        msg.angular.z = angular_cmd;
        Robot::vel_pub.publish(msg);

        if (getDistance(xd, yd, x, y) < 0.02){
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0;
            Robot::vel_pub.publish(msg);
            return true;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return false;
    
}


double Robot::saturate(double X, double Xmin, double Xmax){
    if(X > Xmax){
        X = Xmax;
    }
    if(X < Xmin){
        X = Xmin;
    }
    return X;
}

double Robot::getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

