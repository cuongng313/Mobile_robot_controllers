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
#include </home/cuongnguen/IVASTbot/Robot_control/devel/include/robot_dynamic_controllers/robotControllerReconfigureConfig.h>
#include <Eigen/Dense>

#define PI 3.14159265

/* global subscriber variables */
nav_msgs::Odometry odom;                    // robot odometry
geometry_msgs::Twist subTwistReference;     //
geometry_msgs::Twist pubTwist;              // Publish Control signal

// geometry_msgs::Pose2D pose2d;
Eigen::Vector3f pose2d(3); 
Eigen::Vector3f localVelocity(3); 

nav_msgs::Path ref_path;
nav_msgs::Path ref_path_derivative;
nav_msgs::Path ref_path_2derivative;
nav_msgs::Path odom_path;

double lambdaX = 3, lambdaY = 3, lambdaTheta = 3;    
double kx = 30, ky = 30, ktheta = 20; 
double kx2 = 30, ky2 = 30, ktheta2 = 20; 
double kx3 = 30, ky3 = 30, ktheta3 = 20;
double saturatedValue = 1;

/*-----------------------------------------*/

void callback(robot_controllers::robotControllerReconfigureConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  kx = config.kx;   ky = config.ky;   ktheta = config.ktheta;
  kx2 = config.kx2;   ky2 = config.ky2;   ktheta2 = config.ktheta2;
  kx3 = config.kx3;   ky3 = config.ky3;   ktheta3 = config.ktheta3;

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

  localVelocity << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z;
}

void refPathCallback(const nav_msgs::Path& msg) { ref_path = msg; }
void refPathDerivativeCallback(const nav_msgs::Path& msg) {ref_path_derivative = msg;}
void refPath2DerivativeCallback(const nav_msgs::Path& msg) {ref_path_2derivative = msg;}
/*------------------------------------------*/

float sgn(double v) {
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
  ros::init(argc, argv, "robot_dynamic_controllers");
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
  double time_step = 0.0;

  odom_path.header.frame_id = "/odom";
	odom_path.header.stamp = ros::Time::now();

  Eigen::Vector3f controlSignal_;    controlSignal_ << 0.0, 0.0, 0.0;
  Eigen::Vector3f trackingError_;    trackingError_ << 0.0, 0.0, 0.0;
  Eigen::Vector3f trackingErrorDerivative_;    trackingErrorDerivative_ << 0.0, 0.0, 0.0;
  Eigen::Vector3f velocityError_;    velocityError_ << 0.0, 0.0, 0.0;
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
        time_step = 0; 
      }
      else {
        if(time_step == 1) {
          refPath_ << ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y, ref_path.poses[i].pose.orientation.z;
          refPathDerivative_ << ref_path_derivative.poses[i].pose.position.x, ref_path_derivative.poses[i].pose.position.y, ref_path_derivative.poses[i].pose.position.z;
          refPath2Derivative_ << ref_path_2derivative.poses[i].pose.position.x, ref_path_2derivative.poses[i].pose.position.y, ref_path_2derivative.poses[i].pose.position.z; 
          i++;
          time_step = 0;
        }
        time_step++;
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
    trackingError_ = pose2d - refPath_;
    // normalize angle error to avoid the reference angle switch from PI to -PI
    trackingError_(2) = trackingError_(2) - 2*PI*floor((trackingError_(2) + PI)/(2*PI));

    Eigen::Matrix3f gainK1;
    gainK1 << kx, 0, 0, 0,  ky, 0, 0, 0, ktheta;

    

    // global velocity
    Eigen::Matrix3f H;
    H <<  cos(pose2d(2)), -sin(pose2d(2)) , 0,
          sin(pose2d(2)), cos(pose2d(2)) , 0,
          0,               0,                1;

    Eigen::Vector3f refVel;     refVel.setZero();
    refVel = refPathDerivative_ - gainK1*trackingError_;        // virtual control signal

    // refVel = H.inverse()*(refPathDerivative_ - gainK1*trackingError_);

    Eigen::Vector3f globalVelocity;     globalVelocity.setZero();
    globalVelocity = H*localVelocity;

    trackingErrorDerivative_ = globalVelocity - refPathDerivative_;       // Derivative of the tracking error

    Eigen::Vector3f refVelDot;  refVelDot.setZero();
    refVelDot = refPath2Derivative_ - gainK1*trackingErrorDerivative_;    // Derivative of the virtual control signal

    velocityError_ = globalVelocity - refVel;                             // Virtual control signal tracking error

    /* Backstepping and SMC control Signal */
    /* -----------------------------*/

    //Control gain matrix
    Eigen::Matrix3f gainK2;
    gainK2 << kx2, 0, 0,
              0,  ky2, 0,
              0, 0, ktheta2;

    float theta = pose2d(2);
    float thetaDot = globalVelocity(2);
    const float m = 51;                   // robot mass (kg)
    const float J = 6.125;                 // robot inertia moment (z-axis)
    Eigen::Matrix3f M_;
    M_ << m*cos(theta), m*sin(theta), 0,
          -m*sin(theta), m*cos(theta), 0,
          0,            0,            J;
    Eigen::Matrix3f D;  
    D << -m*thetaDot*sin(theta), m*thetaDot*cos(theta), 0,
          -m*thetaDot*cos(theta), -m*thetaDot*sin(theta), 0,
          0,                      0,                      0;

    

    Eigen::Matrix3f gainSurface;
    gainSurface << lambdaX, 0, 0,
              0,  lambdaY, 0,
              0, 0, lambdaTheta;

    Eigen::Matrix3f gainK3;
    gainK3 << kx3, 0, 0,
              0,  ky3, 0,
              0, 0, ktheta3;

    velocityError_ = gainSurface*velocityError_;

    Eigen::Vector3f signS;    signS.setZero();
    signS << sgn(velocityError_(0)), sgn(velocityError_(1)), sgn(velocityError_(2));


    controlSignal_ = D*globalVelocity + M_*(refVelDot - trackingError_ - gainK2*velocityError_ - gainK3*signS);  


    if(abs(controlError.x) <= 0.001) {controlSignal_(0) = 0;}
    if(abs(controlError.y) <= 0.001) {controlSignal_(1) = 0;}
    if(abs(controlError.z) <= 0.001) {controlSignal_(2) = 0;}

    const double SatValue = 500;
    const double SatValueW = 300;

    if (controlSignal_(0) > SatValue) {controlSignal_(0) = SatValue;}
    if (controlSignal_(0) < -SatValue) {controlSignal_(0) = -SatValue;}

    if (controlSignal_(1) > SatValue) {controlSignal_(1) = SatValue;};
    if (controlSignal_(1)< -SatValue) {controlSignal_(1) = -SatValue;};

    if (controlSignal_(2) > SatValueW) {controlSignal_(2) = SatValueW;}
    if (controlSignal_(2) < -SatValueW) {controlSignal_(2) = -SatValueW;};

    std::cout << "ref x y theta: " << refPath_(0)
                                    << " "
                                    << refPath_(1)
                                    << " "
                                    << refPath_(2)
                                    << std::endl;

    std::cout << "ref dev x y theta: " << refPathDerivative_(0)
                                    << " "
                                    << refPathDerivative_(1)
                                    << " "
                                    << refPathDerivative_(2)
                                    << std::endl;                                

    std::cout << "ref 2dev x y theta: " << refPath2Derivative_(0)
                                    << " "
                                    << refPath2Derivative_(1)
                                    << " "
                                    << refPath2Derivative_(2)
                                    << std::endl;

    std::cout << "x y theta: " << pose2d(0)
                                    << " "
                                    << pose2d(1)
                                    << " "
                                    << pose2d(2)
                                    << std::endl;

    std::cout << "velocity x y theta: " << localVelocity(0)
                                    << " "
                                    << localVelocity(1)
                                    << " "
                                    << localVelocity(2)
                                    << std::endl;   

    std::cout << "global velocity x y theta: " << globalVelocity(0)
                                    << " "
                                    << globalVelocity(1)
                                    << " "
                                    << globalVelocity(2)
                                    << std::endl;                                               

    std::cout << "error x y theta: " << trackingError_(0)
                                    << " "
                                    << trackingError_(1)
                                    << " "
                                    << trackingError_(2)
                                    << std::endl;

    std::cout << "virtual control x y theta: " << refVel(0)
                                            << " "
                                            << refVel(1)
                                            << " "
                                            << refVel(2)
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

    controlError.x = trackingError_(0);
    controlError.y = trackingError_(1);
    controlError.z = trackingError_(2);
    pubError.publish(controlError);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}