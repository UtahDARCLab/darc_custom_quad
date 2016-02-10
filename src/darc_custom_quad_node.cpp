// This file has been created to try and fly the quadcopter using the joystick

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <Eigen/Geometry>

#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

float roll, pitch, yaw_rate, thrust;
const float MAX_ANGLE = 15.0 * M_PI / 180.0;
const float MAX_YAW_RATE = 30.0 * M_PI / 180.0;
void twist_callback(const geometry_msgs::Twist& twist_msg_in) { //this also scales the joystick reading
  //roll   = max_range - 0.5 * (max_range - min_range) * (1.0 + twist_msg_in.angular.x);  // Roll -- rotation about the x axis 
  //pitch  = max_range - 0.5 * (max_range - min_range) * (1.0 + twist_msg_in.angular.y);  // Pitch --rotation about the y axis
  //yaw    = max_range - 0.5 * (max_range - min_range) * (1.0 + twist_msg_in.angular.z);  // Yaw -- rotation about the z axis
  //thrust = min_range + (max_range - min_range) * twist_msg_in.linear.z;  // Thrust
  roll = twist_msg_in.angular.x * MAX_ANGLE;  // Roll -- rotation about x
  if (roll < -MAX_ANGLE) {
    roll = -MAX_ANGLE;
  } else if (roll > MAX_ANGLE) {
    roll = MAX_ANGLE;
  }
  
  pitch = twist_msg_in.angular.y * MAX_ANGLE;  // Pitch -- rotation about y
  if (pitch < -MAX_ANGLE) {
    pitch = -MAX_ANGLE;
  } else if (pitch > MAX_ANGLE) { 
    pitch = MAX_ANGLE;
  }
  
  yaw_rate = twist_msg_in.angular.z * MAX_YAW_RATE;  // Yaw rate -- rotation about z
  if (yaw_rate < -MAX_YAW_RATE) {
    yaw_rate = -MAX_YAW_RATE;
  } else if (yaw_rate > MAX_YAW_RATE) {
    yaw_rate = MAX_YAW_RATE;
  }
  
  thrust = twist_msg_in.linear.z;  // Thrust
  if (thrust < 0.0) {
    thrust = 0.0;
  } else if (thrust > 1.0) {
    thrust = 1.0;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "darc_custom_quad_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);

  ros::Subscriber twist_sub;
  twist_sub = node.subscribe("new_u",1,twist_callback);
  //twist_sub = node.subscribe("desired_u",1,twist_callback);

  ros::Publisher setpoint_pub;
  setpoint_pub = node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1);
  mavros_msgs::AttitudeTarget setpoint_out;
  setpoint_out.type_mask = 3;

  while (ros::ok()) {
    ros::spinOnce();

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    Eigen::Quaternionf q(m);
    setpoint_out.orientation.x = q.x();
    setpoint_out.orientation.y = q.y();
    setpoint_out.orientation.z = q.z();
    setpoint_out.orientation.w = q.w();

    setpoint_out.body_rate.x = 0.0;
    setpoint_out.body_rate.y = 0.0;
    setpoint_out.body_rate.z = yaw_rate;

    setpoint_out.thrust = thrust;

    setpoint_out.header.stamp = ros::Time::now();
    setpoint_pub.publish(setpoint_out);

    loop_rate.sleep();
  }
  return 0;
}
