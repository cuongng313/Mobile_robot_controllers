#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>  
#include <std_msgs/Int8.h> 
#include <PID.h>

nav_msgs::Odometry odom;
geometry_msgs::Twist subTwistReference;
geometry_msgs::Twist pubTwist;

PID_controller PIDxVel;
PID_controller PIDyVel;
PID_controller PIDwRot;

double refX{0}, refY{0};

double kPx{15.0}, kIx{0.0}, kDx{0.1}, maxx{50}, minx{-50};
double kPy{15.0}, kIy{0.0}, kDy{0.1}, maxy{50}, miny{-50};
double kPw{1.5}, kIw{0.0}, kDw{0.1}, maxw, minw;
double xVel{0.0}, yVel{0.0}, wRot{0.0};

geometry_msgs::Pose2D pose2d;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msgOdom){
  odom.pose = msgOdom->pose;
  odom.twist = msgOdom->twist;

  // convert quaternion to RPY
  pose2d.x = msgOdom->pose.pose.position.x;
  pose2d.y = msgOdom->pose.pose.position.y;
    
    tf::Quaternion q(
      msgOdom->pose.pose.orientation.x,
      msgOdom->pose.pose.orientation.y,
      msgOdom->pose.pose.orientation.z,
      msgOdom->pose.pose.orientation.w);
    q.normalize();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;
}

nav_msgs::Path path;
nav_msgs::Path odom_path;
void pathCallback(const nav_msgs::Path& msg) { path = msg; }

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_controllers");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
 
  /* ROS subscriber */
  ros::Subscriber subOdom = n.subscribe("/odom", 10, odomCallback);
  ros::Subscriber path_sub = n.subscribe("/ref_path", 1, pathCallback);

  /* ROS publisher */
  ros::Publisher pubControl = n.advertise<geometry_msgs::Twist>("omni_robot/cmd_vel",10);
  ros::Publisher odom_path_pub = n.advertise<nav_msgs::Path>("/odom_path_pub", 10);
  ros::Publisher pubError = n.advertise<geometry_msgs::Vector3>("/control_error", 10);

  // ROS parameters

  n.param("/robot_controllers/ref_x", refX, 0.0);
  n.param("/robot_controllers/ref_y", refY, 0.0);

  n.param("/robot_controllers/kPx", kPx, 15.0);
  n.param("/robot_controllers/kIx", kIx, 0.0);
  n.param("/robot_controllers/kDx", kDx, 0.1);
  n.param("/robot_controllers/maxx", maxx, 2.0);
  n.param("/robot_controllers/minx", minx, -2.0);

  n.param("/robot_controllers/kPy", kPy, 15.0);
  n.param("/robot_controllers/kIy", kIy, 0.0);
  n.param("/robot_controllers/kDy", kDy, 0.1);
  n.param("/robot_controllers/maxy", maxy, 2.0);
  n.param("/robot_controllers/miny", miny, -2.0);

  n.param("/robot_controllers/kPw", kPw, 1.5);
  n.param("/robot_controllers/kIw", kIw, 0.0);
  n.param("/robot_controllers/kDw", kDw, 0.0);
  n.param("/robot_controllers/maxw", maxw, 2.0);
  n.param("/robot_controllers/minw", minw, -2.0);

  PIDxVel.setParameters(kPx, kIx, kDx, maxx, minx);
  PIDyVel.setParameters(kPy, kIy, kDy, maxy, miny);
  PIDwRot.setParameters(kPw, kIw, kDw, maxw, minw);

  

  double refTrajectoryX;
  double refTrajectoryY;
  double refTrajectoryYaw;
  int i = 0;
  double t = 0.0;

  odom_path.header.frame_id = "/odom";
	odom_path.header.stamp = ros::Time::now();

  geometry_msgs::Vector3 controlError;
  double rollRef, pitchRef, yawRef;

  while (ros::ok()) { 
    // reference trajectory
    if (path.poses.size() == 0) {
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

    if (i >= path.poses.size() - 1) {
      i = 0;
      
    }
    else {
      refTrajectoryX = path.poses[i].pose.position.x;
      refTrajectoryY = path.poses[i].pose.position.y;
      refTrajectoryYaw = path.poses[i].pose.orientation.z;
      i++;
    }
    

    // current pose path visualize
    geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = odom_path.header;
		cur_pose.pose.position.x = odom.pose.pose.position.x;
		cur_pose.pose.position.y = odom.pose.pose.position.y;

		cur_pose.pose.orientation.w = 1.0;

    odom_path.poses.push_back(cur_pose);
    odom_path_pub.publish(odom_path);
    

    pubTwist.linear.x = PIDxVel.PIDcalculate(odom.pose.pose.position.x, refTrajectoryX);
    pubTwist.linear.y = PIDyVel.PIDcalculate(odom.pose.pose.position.y, refTrajectoryY);
    // pubTwist.angular.z = PIDwRot.PIDcalculate(pose2d.theta, refTrajectoryYaw);

    controlError.x = PIDxVel.error;
    controlError.y = PIDyVel.error;
    controlError.z = PIDwRot.error;
    pubError.publish(controlError);
 
    std::cout << "error x y w: " << PIDxVel.error
                                 << " "
                                 << PIDyVel.error
                                 << " "
                                 << PIDwRot.error
                                 << std::endl;
    std::cout << "ref w: " << refTrajectoryYaw << std::endl;
    std::cout << "cur w: " << pose2d.theta << std::endl;

    std::cout << "PID out x y w: " << PIDxVel.PIDout
                                    << " "
                                    << PIDyVel.PIDout
                                    << " "
                                    << PIDwRot.PIDout
                                    << std::endl;

    pubControl.publish(pubTwist);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}