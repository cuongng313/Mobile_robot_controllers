#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>  
#include <cmath>

#include <dynamic_reconfigure/server.h>
#include </home/cuongnguen/IVASTbot/Robot_control/devel/include/robot_kinematic_controllers/robotControllerReconfigureConfig.h>
#include <Eigen/Dense>

#define PI 3.14159265

/* global subscriber variables */
nav_msgs::Odometry odom;                    // robot odometry
geometry_msgs::Twist subTwistReference;     //
geometry_msgs::Twist pubTwist;              // Publish Control signal

// geometry_msgs::Pose2D pose2d;
Eigen::Vector3f pose2d(3); 

nav_msgs::Path ref_path;
nav_msgs::Path ref_path_derivative;
nav_msgs::Path ref_path_2derivative;
nav_msgs::Path odom_path;

double lambdaX = 3, lambdaY = 3, lambdaTheta = 3;    
double kx = 30, ky = 30, ktheta = 20; 
double kx2 = 30, ky2 = 30, ktheta2 = 20; 
double saturatedValue = 1;

/*-----------------------------------------*/

void callback(robot_controllers::robotControllerReconfigureConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  kx = config.kx;   ky = config.ky;   ktheta = config.ktheta;
  kx2 = config.kx2;   ky2 = config.ky2;   ktheta2 = config.ktheta2;
  lambdaX = config.lambdaX; lambdaY = config.lambdaY; lambdaTheta = config.lambdaTheta;
  saturatedValue = config.saturatedValue;
  std::cout << "kx, ky, ktheta: " << kx << ", " << ky << ", " << ktheta << std::endl;
}

/* Subscribers Callback */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msgOdom){
  odom.pose = msgOdom->pose;
  odom.twist = msgOdom->twist;

  // convert quaternion to RPY
  tf::Quaternion q(
    msgOdom->pose.pose.orientation.x,
    msgOdom->pose.pose.orientation.y,
    msgOdom->pose.pose.orientation.z,
    msgOdom->pose.pose.orientation.w);
  q.normalize();
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //////////////////////////////////////////////

  if(yaw > PI) {yaw = yaw - 2*PI;}
  if(yaw < -PI) {yaw = yaw + 2*PI;}
    
  pose2d(2) = yaw;
  pose2d(0) = msgOdom->pose.pose.position.x;
  pose2d(1) = msgOdom->pose.pose.position.y;
}

void refPathCallback(const nav_msgs::Path& msg) { ref_path = msg; }
void refPathDerivativeCallback(const nav_msgs::Path& msg) {ref_path_derivative = msg;}
void refPath2DerivativeCallback(const nav_msgs::Path& msg) {ref_path_2derivative = msg;}
/*------------------------------------------*/

int sgn(double v) {
  if(saturatedValue == 0) {
    return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
  }
  else {
    if (v < -saturatedValue) return -1;
    if (v > saturatedValue) return 1;
    return v/saturatedValue;
  }
  return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_controllers");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  // dynamic reconfigure server
  dynamic_reconfigure::Server<robot_controllers::robotControllerReconfigureConfig> server;
  dynamic_reconfigure::Server<robot_controllers::robotControllerReconfigureConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  /* ROS subscriber */
  ros::Subscriber subOdom = n.subscribe("/odom", 10, odomCallback);
  ros::Subscriber ref_path_sub = n.subscribe("/ref_path", 1, refPathCallback);
  ros::Subscriber ref_path_derivative_sub = n.subscribe("ref_path_derivative", 1, refPathDerivativeCallback);
  ros::Subscriber ref_path_2derivative_sub = n.subscribe("ref_path_2derivative", 1, refPath2DerivativeCallback);

  /* ROS publisher */
  ros::Publisher pubControl = n.advertise<geometry_msgs::Twist>("omni_robot/cmd_vel",10);
  ros::Publisher pubOdomPath = n.advertise<nav_msgs::Path>("/odom_path_pub", 10);
  ros::Publisher pubError = n.advertise<geometry_msgs::Vector3>("/control_error", 10);

  int i = 0;
  double t = 0.0;

  odom_path.header.frame_id = "/odom";
	odom_path.header.stamp = ros::Time::now();

  Eigen::Vector3f controlSignal_;    controlSignal_ << 0.0, 0.0, 0.0;
  Eigen::Vector3f controlError_;     controlError_ << 0.0, 0.0, 0.0;
  Eigen::Vector3f slidingSurface_;   slidingSurface_ << 0.0, 0.0, 0.0;

  Eigen::Vector3f refPath_;              refPath_.setZero();
  Eigen::Vector3f refPathDerivative_;    refPathDerivative_.setZero();
  Eigen::Vector3f refPath2Derivative_;   refPath2Derivative_.setZero();

  geometry_msgs::Vector3 controlError;


  while (ros::ok()) { 
    
    /* reference trajectory */
    if (ref_path.poses.size() == 0) {
      refPath_.setZero();
      refPathDerivative_.setZero();
      refPath2Derivative_.setZero();
		}
    else {
      if (i >= ref_path.poses.size() - 1) {
        i = 0; 
      }
      else {
        refPath_ << ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y, ref_path.poses[i].pose.orientation.z;
        refPathDerivative_ << ref_path_derivative.poses[i].pose.position.x, ref_path_derivative.poses[i].pose.position.y, ref_path_derivative.poses[i].pose.position.z;
        refPath2Derivative_ << ref_path_2derivative.poses[i].pose.position.x, ref_path_2derivative.poses[i].pose.position.y, ref_path_2derivative.poses[i].pose.position.z; 
        i++;
      }
    }
    //////////////////////////

    // current pose path visualize
    geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = odom_path.header;
		cur_pose.pose.position.x = pose2d(0);
		cur_pose.pose.position.y = pose2d(1);
		cur_pose.pose.orientation = odom.pose.pose.orientation;     // get quaternion

    if (i == 0) {
      odom_path.poses.clear();
    }
    
    odom_path.poses.push_back(cur_pose);
    pubOdomPath.publish(odom_path);
    //////////////////////////

    // tracking error
    controlError_ = pose2d - refPath_;
    // normalize angle error to avoid the reference angle switch from PI to -PI
    controlError_(2) = controlError_(2) - 2*PI*floor((controlError_(2) + PI)/(2*PI));
    
    Eigen::Matrix3f slidingGain;
    slidingGain << lambdaX, 0, 0,
                    0,  lambdaY, 0,
                    0, 0, lambdaTheta;

    slidingSurface_ = slidingGain*controlError_; 

    /* Backstepping and SMC control Signal */
    /* -----------------------------*/
    /* set the symbolic variable for math calculation */
    Eigen::Matrix3f inv_H;
    inv_H <<  cos(pose2d(2)), sin(pose2d(2)) , 0,
              -sin(pose2d(2)), cos(pose2d(2)) , 0,
              0,               0,                1;

    //Control gain matrix
    Eigen::Matrix3f gainK1;
    gainK1 << kx, 0, 0,
              0,  ky, 0,
              0, 0, ktheta;

    Eigen::Matrix3f gainK2;
    gainK2 << kx2, 0, 0,
              0,  ky2, 0,
              0, 0, ktheta2;

    Eigen::Vector3f signS;  signS << sgn(slidingSurface_(0)), sgn(slidingSurface_(1)), sgn(slidingSurface_(2));

    controlSignal_ = inv_H*(refPathDerivative_ - gainK1*slidingSurface_ - gainK2*signS);

    // if(abs(controlError.x) <= 0.001) {controlSignal.x = 0;}
    // if(abs(controlError.y) <= 0.001) {controlSignal.y = 0;}
    // if(abs(controlError.z) <= 0.001) {controlSignal.z = 0;}

    // const double SatValue = 10;

    // if (controlSignal.x > SatValue) {controlSignal.x = SatValue;}
    // if (controlSignal.x < -SatValue) {controlSignal.x = -SatValue;}

    // if (controlSignal.y > SatValue) {controlSignal.y = SatValue;};
    // if (controlSignal.y < -SatValue) {controlSignal.y = -SatValue;};

    // if (controlSignal.z > 3) {controlSignal.z = 3;}
    // if (controlSignal.z < -3) {controlSignal.z = -3;};

    std::cout << "ref x y theta: " << refPath_(0)
                                    << " "
                                    << refPath_(1)
                                    << " "
                                    << refPath_(2)
                                    << std::endl;

    std::cout << "x y theta: " << pose2d(0)
                                    << " "
                                    << pose2d(1)
                                    << " "
                                    << pose2d(2)
                                    << std::endl;

    std::cout << "error x y theta: " << controlError_(0)
                                    << " "
                                    << controlError_(1)
                                    << " "
                                    << controlError_(2)
                                    << std::endl;

    std::cout << "sliding surface x y theta: " << slidingSurface_(0)
                                            << " "
                                            << slidingSurface_(1)
                                            << " "
                                            << slidingSurface_(2)
                                            << std::endl;

    // std::cout << "ref w: " << refTrajectoryYaw << std::endl;
    // std::cout << "cur w: " << pose2d.theta << std::endl;

    std::cout << "control signal out x y w: " << controlSignal_(0)
                                    << " "
                                    << controlSignal_(1)
                                    << " "
                                    << controlSignal_(2)
                                    << std::endl << std::endl;

    pubTwist.linear.x = controlSignal_(0);
    pubTwist.linear.y = controlSignal_(1);
    pubTwist.angular.z = controlSignal_(2);

    pubControl.publish(pubTwist);

    controlError.x = controlError_(0);
    controlError.y = controlError_(1);
    controlError.z = controlError_(2);
    pubError.publish(controlError);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}