/* generate reference path: generate all the path into a vector in which, 
each position contains the ref trajectory at time step */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <iostream>

#define PI 3.14159265

using namespace std;

bool path_generator(nav_msgs::Path& path, nav_msgs::Path& ref_path_derivative, nav_msgs::Path& ref_path_2derivative);
string frame_id, x_func, y_func;
double x_radius, y_radius, kx, ky;


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle nh("~");

    ros::Publisher ref_path_pub = nh.advertise<nav_msgs::Path>("/ref_path", 1);
    ros::Publisher ref_path_derivative_pub = nh.advertise<nav_msgs::Path>("/ref_path_derivative", 1);
    ros::Publisher ref_path_2derivative_pub = nh.advertise<nav_msgs::Path>("/ref_path_2derivative", 1);
    // ros::Publisher path_ref_pub = nh.advertise<geometry_msgs::Pose2D>("/ref_path", 1);

    /* ROS PARAMS */
    ros::param::get("~frame_id", frame_id);
    ros::param::get("~x_radius", x_radius);
    ros::param::get("~y_radius", y_radius);
    ros::param::get("~kx", kx);
    ros::param::get("~ky", ky);
    ros::param::get("~x_func", x_func);
    ros::param::get("~y_func", y_func);

    cout << frame_id << " " << x_radius << " " << y_radius << endl;

    nav_msgs::Path path;
    nav_msgs::Path ref_path_derivative;
    nav_msgs::Path ref_path_2derivative;
    

    while(!path_generator(path, ref_path_derivative, ref_path_2derivative)) {}
    ros::Rate r(10);

    while(ros::ok()) {
        ref_path_pub.publish(path);
        ref_path_derivative_pub.publish(ref_path_derivative);
        ref_path_2derivative_pub.publish(ref_path_2derivative);
        
        ros::spinOnce();
        r.sleep();
    }
}

bool path_generator(nav_msgs::Path& path, nav_msgs::Path& ref_path_derivative, nav_msgs::Path& ref_path_2derivative)
{
    // Generate the circle path
    double Ts = 0.1;
    double step = 1000;
    double TimePeriod = step*Ts;
    double ohmega = 2*PI/TimePeriod;
    // double SamplingTime = 1/step;
    double SamplingTime = 0.02; //sampling time has to be determined based on controller node.

    double x0 = 0;
    double y0 = 0;

    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    ref_path_derivative.header.frame_id = frame_id;
    ref_path_derivative.header.stamp = ros::Time::now();

    ref_path_2derivative.header.frame_id = frame_id;
    ref_path_2derivative.header.stamp = ros::Time::now();

    for (int i = 0; i < step; i++)
    {
        double x, y, yaw;
        double x_s, y_s, yaw_s; // x(k+1), y(k+1), yaw(k+1)
        double x_ss, y_ss, yaw_ss;
        double dx, dy, dyaw;
        double dxE, dyE, dyawE;
        double dx2E, dy2E, dyaw2E;
        
        
        yaw = 2*PI*i/step;
        yaw_s = 2*PI*(i+1)/step;
        yaw_ss = 2*PI*(i+2)/step;

        // correct yaw angle
        if(yaw > PI) {yaw = yaw - 2*PI;}
        if(yaw < -PI) {yaw = yaw + 2*PI;}
        if(yaw_s > PI) {yaw_s = yaw_s - 2*PI;}
        if(yaw_s < -PI) {yaw_s = yaw_s + 2*PI;}
        if(yaw_ss > PI) {yaw_ss = yaw_ss - 2*PI;}
        if(yaw_ss < -PI) {yaw_ss = yaw_ss + 2*PI;}
       
        x = x_radius*sin(yaw);   
        y = y_radius*cos(yaw);

        x_s = x_radius*sin(yaw_s);
        y_s = y_radius*cos(yaw_s);

        x_ss = x_radius*sin(yaw_ss);
        y_ss = y_radius*cos(yaw_ss);

        // derivative 
        dx = x_radius*2*PI*cos(yaw);
        dy = -y_radius*2*PI*sin(yaw);
        dyaw = 2*PI;

        // Euler derivative
        dxE = (x_s - x)/SamplingTime;
        dyE = (y_s - y)/SamplingTime;
        dyawE = (yaw_s - yaw)/SamplingTime;

        // Euler 2-order derivative
        dx2E = (x_ss - 2*x_s + x)/(SamplingTime*SamplingTime);
        dy2E = (y_ss - 2*y_s + y)/(SamplingTime*SamplingTime);
        dyaw2E = (yaw_ss - 2*yaw_s + yaw)/(SamplingTime*SamplingTime);

        std::cout << " x y w: [" << i << "]"
                                        << x
                                        << " "
                                        << y
                                        << " "
                                        << yaw
                                        << std::endl;

        std::cout << "Euler derivative x y w: [" << i << "]"
                                                << dxE
                                                << " "
                                                << dyE
                                                << " "
                                                << dyawE
                                                << std::endl;

        std::cout << "Euler 2-order derivative x y w: [" << i << "]"
                                                << dx2E
                                                << " "
                                                << dy2E
                                                << " "
                                                << dyaw2E
                                                << std::endl << std::endl;

        // provide data for pose
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = yaw;

        // provide data for pose's derivative
        geometry_msgs::PoseStamped ref_pose_derivative;
        ref_pose_derivative.header = ref_path_derivative.header;
        ref_pose_derivative.pose.position.x = dxE;
        ref_pose_derivative.pose.position.y = dyE;
        ref_pose_derivative.pose.orientation.z = dyawE;

        // provide data for pose's 2-order derivative
        geometry_msgs::PoseStamped ref_pose_2derivative;
        ref_pose_2derivative.header = ref_path_2derivative.header;
        ref_pose_2derivative.pose.position.x = dx2E;
        ref_pose_2derivative.pose.position.y = dy2E;
        ref_pose_2derivative.pose.orientation.z = dyaw2E;

        path.poses.push_back(pose);
        ref_path_derivative.poses.push_back(ref_pose_derivative);
        ref_path_2derivative.poses.push_back(ref_pose_2derivative);
    }

    return true;
}