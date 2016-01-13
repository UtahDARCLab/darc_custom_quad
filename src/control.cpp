// This file has been created to try and fly the quadcopter using the joystick

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <roscopter/RC.h>
#include <std_srvs/Empty.h>
#include <iostream>

int min_range = 1000, max_range = 2000;
int stable_mode = 1500;

roscopter::RC rc_out;
int roll, pitch, yaw, thrust = min_range;
void twist_callback(const geometry_msgs::Twist& twist_msg_in) { //this also scales the joystick reading
  roll   = max_range - 0.5*(max_range - min_range)*(1.0 + twist_msg_in.angular.x);  // Roll -- rotation about the x axis 
  pitch  = max_range - 0.5*(max_range - min_range)*(1.0 + twist_msg_in.angular.y);  // Pitch --rotation about the y axis
  yaw    = max_range - 0.5*(max_range - min_range)*(1.0 + twist_msg_in.angular.z);  // Yaw -- rotation about the z axis
  thrust = min_range + (max_range - min_range)*twist_msg_in.linear.z;  // Thrust
}

roscopter::RC rc_in;
void rc_callback(const roscopter::RC& rc_msg_in) {
  rc_in = rc_msg_in;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);

  ros::Subscriber twist_sub;
  //twist_sub = node.subscribe("new_u",1,twist_callback);
  twist_sub = node.subscribe("desired_u",1,twist_callback);

  ros::Subscriber rc_sub;
  rc_sub = node.subscribe("rc", 1, rc_callback);

  ros::Publisher rc_pub;
  rc_pub = node.advertise<roscopter::RC>("send_rc",1);

  while (ros::ok()) {
    ros::spinOnce();
    if (rc_in.channel[6] < 1250 || rc_in.channel[6] > 1750) {  // Gryo switch at 0, fly manually
      rc_out.channel[0] = 0;
      rc_out.channel[1] = 0;
      rc_out.channel[2] = 0;
      rc_out.channel[3] = 0;
      rc_out.channel[4] = 0;
    } else {
//      std::cout << "r: " << roll << ", p: " << pitch << ", y: " << yaw << ", t: " << thrust << std::endl;
      rc_out.channel[0] = roll;
      rc_out.channel[1] = pitch;
      rc_out.channel[2] = thrust;
      rc_out.channel[3] = yaw;
      rc_out.channel[4] = stable_mode;
    }
    rc_pub.publish(rc_out);
    loop_rate.sleep();
  }
  return 0;
}


