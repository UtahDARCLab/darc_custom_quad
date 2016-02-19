#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <Eigen/Geometry>
#include <std_msgs/Float32.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
#include <vector>
#include <numeric>

const float MAX_ANGLE = 20.0 * M_PI / 180.0;  // rad
const float MAX_YAW_RATE = 60 * M_PI / 180.0;

float rx, ry, vz, vw;
void twist_callback(const geometry_msgs::Twist& twist_msg_in) { //this also scales the joystick reading
  rx = twist_msg_in.angular.x;  // ROLL
  rx = ((rx < -1.0) ? -1.0 : ((rx > 1.0) ? 1.0 : rx)) * MAX_ANGLE;

  ry = twist_msg_in.angular.y;  // PITCH
  ry = ((ry < -1.0) ? -1.0 : ((ry > 1.0) ? 1.0 : ry)) * MAX_ANGLE;

  vw = twist_msg_in.angular.z;  // YAW RATE
  vw = ((vw < -1.0) ? -1.0 : ((vw > 1.0) ? 1.0 : vw)) * MAX_YAW_RATE;

  vz = 0.5*twist_msg_in.linear.z + 0.5;  // THROTTLE
  vz = ((vz < 0.0) ? 0.0 : ((vz > 1.0) ? 1.0 : vz));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "darc_custom_quad_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(100);

  ros::Subscriber twist_sub;
  twist_sub = node.subscribe("new_u", 1, twist_callback);

  ros::Publisher set_pub = node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
  mavros_msgs::AttitudeTarget set_out;
  set_out.type_mask = 7;  // Ignore rates

  float rz = 0.0;

  while (ros::ok()) {
    ros::spinOnce();

    rz += vw / 100.0;

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf q(m);
    set_out.orientation.x = q.x();
    set_out.orientation.y = q.y();
    set_out.orientation.z = q.z();
    set_out.orientation.w = q.w();

    set_out.thrust = vz;

    set_out.header.stamp = ros::Time::now();
    set_pub.publish(set_out);

    loop_rate.sleep();
  }
  return 0;
}
