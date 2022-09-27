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
#include </home/cuongnguen/IVASTbot/Robot_control/devel/include/robot_dynamic_controllers/SMCcontrollerReconfigureConfig.h>
#include <Eigen/Dense>

#define PI 3.14159265

/* global subscriber variables */
nav_msgs::Odometry odom;                    // robot odometry
geometry_msgs::Twist subTwistReference;     //
geometry_msgs::Twist pubTwist;              // Publish Control signal

// geometry_msgs::Pose2D q;
Eigen::Vector3f q(3); 
Eigen::Vector3f v(3); 

nav_msgs::Path ref_path;
nav_msgs::Path ref_path_derivative;
nav_msgs::Path ref_path_2derivative;
nav_msgs::Path odom_path;

double lambdaX = 3, lambdaY = 3, lambdaTheta = 3;    
double kx = 30, ky = 30, ktheta = 20; 
double kx2 = 30, ky2 = 30, ktheta2 = 20; 
double saturatedValue = 1;

/*-----------------------------------------*/

void callback(robot_controllers::SMCcontrollerReconfigureConfig &config, uint32_t level) {
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
  tf::Quaternion quat(
    msgOdom->pose.pose.orientation.x,
    msgOdom->pose.pose.orientation.y,
    msgOdom->pose.pose.orientation.z,
    msgOdom->pose.pose.orientation.w);
  quat.normalize();
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //////////////////////////////////////////////

  if(yaw > PI) {yaw = yaw - 2*PI;}
  if(yaw < -PI) {yaw = yaw + 2*PI;}
    
  q(2) = yaw;
  q(0) = msgOdom->pose.pose.position.x;
  q(1) = msgOdom->pose.pose.position.y;

  v << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z;
}

void refPathCallback(const nav_msgs::Path& msg) { ref_path = msg; }
void refPathDerivativeCallback(const nav_msgs::Path& msg) {ref_path_derivative = msg;}
void refPath2DerivativeCallback(const nav_msgs::Path& msg) {ref_path_2derivative = msg;}
/*------------------------------------------*/

float sat(double temp) {
  if(saturatedValue == 0) {
    return (temp < 0) ? -1 : ((temp > 0) ? 1 : 0);
  }
  else {
    if (temp < -saturatedValue) return -1;
    if (temp > saturatedValue) return 1;
    return temp/saturatedValue;
  }
  return 0;
}

int sgn(double temp) {
  return (temp < 0) ? -1 : ((temp > 0) ? 1 : 0);

}



int main(int argc, char** argv){
  ros::init(argc, argv, "robot_dynamic_controllers");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  // dynamic reconfigure server
  dynamic_reconfigure::Server<robot_controllers::SMCcontrollerReconfigureConfig> server;
  dynamic_reconfigure::Server<robot_controllers::SMCcontrollerReconfigureConfig>::CallbackType f;
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


  Eigen::Vector3f Feq;    Feq << 0.0, 0.0, 0.0;
  Eigen::Vector3f Fsw;    Fsw << 0.0, 0.0, 0.0;
  Eigen::Vector3f F;    F << 0.0, 0.0, 0.0;
  Eigen::Vector3f e;    e << 0.0, 0.0, 0.0;
  Eigen::Vector3f e_dot;    e_dot << 0.0, 0.0, 0.0;
  Eigen::Vector3f S;   S << 0.0, 0.0, 0.0;

  Eigen::Vector3f qd;         qd.setZero();
  Eigen::Vector3f qd_dot;    qd_dot.setZero();
  Eigen::Vector3f qd_2dot;   qd_2dot.setZero();

  geometry_msgs::Vector3 controlError;


  while (ros::ok()) { 
    
    /* reference trajectory */
    if (ref_path.poses.size() == 0) {
      qd.setZero();
      qd_dot.setZero();
      qd_2dot.setZero();
		}
    else {
      if (i >= ref_path.poses.size() - 1) {
        i = 0; 
      }
      else {
        qd << ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y, ref_path.poses[i].pose.orientation.z;
        qd_dot << ref_path_derivative.poses[i].pose.position.x, ref_path_derivative.poses[i].pose.position.y, ref_path_derivative.poses[i].pose.position.z;
        qd_2dot << ref_path_2derivative.poses[i].pose.position.x, ref_path_2derivative.poses[i].pose.position.y, ref_path_2derivative.poses[i].pose.position.z; 
        i++;
      }
    }
    //////////////////////////

    // current pose path visualize
    geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = odom_path.header;
		cur_pose.pose.position.x = q(0);
		cur_pose.pose.position.y = q(1);
		cur_pose.pose.orientation = odom.pose.pose.orientation;     // get quaternion

    if (i == 0) {
      odom_path.poses.clear();
    }
    
    odom_path.poses.push_back(cur_pose);
    pubOdomPath.publish(odom_path);
    //////////////////////////

    // tracking error
    e = q - qd;
    // normalize angle error to avoid the reference angle switch from PI to -PI
    e(2) = e(2) - 2*PI*floor((e(2) + PI)/(2*PI));

    Eigen::Matrix3f H;
    H <<  cos(q(2)), -sin(q(2)) , 0,
          sin(q(2)), cos(q(2)) , 0,
          0,               0,                1;

    Eigen::Vector3f q_dot;     q_dot.setZero();
    q_dot = H*v;

    e_dot = q_dot - qd_dot;

    Eigen::Matrix3f lambda;
    lambda << lambdaX, 0, 0, 0,  lambdaY, 0, 0, 0, lambdaTheta;

    // Sliding Surface
    S = e_dot + lambda*e;

  

    /* Backstepping and SMC control Signal */
    /* -----------------------------*/

    float theta = q(2);
    float thetaDot = q_dot(2);
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

        //Control gain matrix
    Eigen::Matrix3f gainK1;
    gainK1 << kx, 0, 0,
              0,  ky, 0,
              0, 0, ktheta;

    Eigen::Matrix3f gainK2;
    gainK2 << kx2, 0, 0,
              0,  ky2, 0,
              0, 0, ktheta2;

    Eigen::Vector3f satS;    satS.setZero();
    satS << sat(S(0)), sat(S(1)), sat(S(2));

    const float frictionGainX = 0.1;
    const float frictionGainY = 0.1;
    const float frictionGainW = 0.1;

    Eigen::Matrix3f FrictionGain;  
    FrictionGain << frictionGainX, 0, 0,
                    0, frictionGainY, 0,
                    0, 0, frictionGainW;

    Eigen::Vector3f signV;    signV.setZero();
    signV << sgn(v(0)), sgn(v(1)), sgn(v(2));

    // Feq = D*q_dot + FrictionGain*signV  + M_*(qd_2dot - lambda*e_dot);
    Feq = D*q_dot + FrictionGain*signV  + M_*(qd_2dot - lambda*e_dot);
    Fsw = -M_*(gainK1*satS + gainK2*S);

    F = Feq + Fsw;

    // if(abs(controlError.x) <= 0.00001) {F(0) = 0;}
    // if(abs(controlError.y) <= 0.00001) {F(1) = 0;}
    // if(abs(controlError.z) <= 0.00001) {F(2) = 0;}

    const double SatValue = 500;
    const double SatValueW = 300;

    if (F(0) > SatValue) {F(0) = SatValue;}
    if (F(0) < -SatValue) {F(0) = -SatValue;}

    if (F(1) > SatValue) {F(1) = SatValue;};
    if (F(1)< -SatValue) {F(1) = -SatValue;};

    if (F(2) > SatValueW) {F(2) = SatValueW;}
    if (F(2) < -SatValueW) {F(2) = -SatValueW;};

    std::cout << "ref x y theta: " << qd(0)
                                    << " "
                                    << qd(1)
                                    << " "
                                    << qd(2)
                                    << std::endl;

    std::cout << "ref dev x y theta: " << qd_dot(0)
                                    << " "
                                    << qd_dot(1)
                                    << " "
                                    << qd_dot(2)
                                    << std::endl;                                

    std::cout << "ref 2dev x y theta: " << qd_2dot(0)
                                    << " "
                                    << qd_2dot(1)
                                    << " "
                                    << qd_2dot(2)
                                    << std::endl;

    std::cout << "x y theta: " << q(0)
                                    << " "
                                    << q(1)
                                    << " "
                                    << q(2)
                                    << std::endl;

    std::cout << "velocity x y theta: " << v(0)
                                    << " "
                                    << v(1)
                                    << " "
                                    << v(2)
                                    << std::endl;   

    std::cout << "global velocity x y theta: " << q_dot(0)
                                    << " "
                                    << q_dot(1)
                                    << " "
                                    << q_dot(2)
                                    << std::endl;                                               

    std::cout << "error x y theta: " << e(0)
                                    << " "
                                    << e(1)
                                    << " "
                                    << e(2)
                                    << std::endl;

    std::cout << "sliding surface x y theta: " << S(0)
                                            << " "
                                            << S(1)
                                            << " "
                                            << S(2)
                                            << std::endl;

    // std::cout << "ref w: " << refTrajectoryYaw << std::endl;
    // std::cout << "cur w: " << q.theta << std::endl;

    std::cout << "control signal out x y w: " << F(0)
                                    << " "
                                    << F(1)
                                    << " "
                                    << F(2)
                                    << std::endl << std::endl;

    pubTwist.linear.x = F(0);
    pubTwist.linear.y = F(1);
    pubTwist.angular.z = F(2);

    pubControl.publish(pubTwist);

    controlError.x = e(0);
    controlError.y = e(1);
    controlError.z = e(2);
    pubError.publish(controlError);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}