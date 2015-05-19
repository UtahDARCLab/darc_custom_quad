/* This file has been created to try and fly the quadcopter using the joystic */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <roscopter/RC.h>
//#include <roscopter/APMCommand.h>
#include <std_srvs/Empty.h>


int roll, pitch, yaw, minRange = 1100,
midRange = 1500,
maxRange = 1900,
thrust = minRange,
toggle[2] = {1500,1099};

int joy_a_,joy_b_, joy_lb_;

bool armed = false;

roscopter::RC rc_out;

void twist_callback(const geometry_msgs::Twist& twist_msg_in) //this also scales the joystick reading
{
	pitch = 2000.0 - 500.0*(1.0 + twist_msg_in.angular.y);		//Pitc -- rotation about the y axis
	roll = 2000.0 - 500.0*(1.0 + twist_msg_in.angular.x);		//Roll -- rotation about the x axis
	yaw = 2000.0 - 500.0*(1.0 + twist_msg_in.angular.z);		//Yaw -- rotation about the z axis
	thrust = 1000.0 + 1000.0*(twist_msg_in.linear.z);		//Thrust 
}

void joy_callback(const sensor_msgs::Joy& joy_msg_in) //this also scales the joystick reading
{
        joy_a_ = joy_msg_in.buttons[0];         //arms the robot
        joy_b_ = joy_msg_in.buttons[1];         //disarms the robot
	joy_lb_ = toggle[ joy_msg_in.buttons[4]];     //toggle flight mode

}

void resetController(ros::Publisher rc_pub)
{
	rc_out.channel[0] = midRange;	//roll set back to midpoint	
	rc_out.channel[1] = midRange;	//pitc set back to midpoint
	rc_out.channel[2] = minRange; //thrust to min
	rc_out.channel[3] = midRange;	//yaw back to midpoint
	//rc_out.channel[4] = midRange;	//setting the mode to stabelize         
	rc_pub.publish(rc_out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Subscriber joy_sub;
	joy_sub = node.subscribe("joy",1,joy_callback);

	ros::Subscriber twist_sub;
	twist_sub = node.subscribe("new_u",1,twist_callback);

	ros::Publisher rc_pub;
	rc_pub = node.advertise<roscopter::RC>("send_rc",1);

	//ros::ServiceClient client = node.serviceClient<roscopter::APMCommand>("apm/command");
	//roscopter::APMCommand srv;
        ros::ServiceClient armClient = node.serviceClient<std_srvs::Empty>("/arm");
	ros::ServiceClient disarmClient = node.serviceClient<std_srvs::Empty>("/disarm");
        std_srvs::Empty srv;

	float dead_zone = .15;
	
	while(ros::ok())
	{
	ros::spinOnce();
		if(joy_a_ == true && !armed)
		{
			//resetController(rc_pub);
			//srv.request.command = CMD_ARM;
			//if(client.call(srv)){;}
			if(armClient.call(srv)){;}
			armed = !armed;
		}
		else if(joy_b_ == true && armed)
		{
			//resetController(rc_pub);
			//srv.request.command = CMD_DISARM;
			//if(client.call(srv)){;}
			if(disarmClient.call(srv)){;}
			armed = !armed;
		}
		else
		{
			rc_out.channel[0] = roll;
			rc_out.channel[1] = pitch;
			rc_out.channel[2] = thrust;
			rc_out.channel[3] =  yaw;
			rc_out.channel[4] = joy_lb_;
	}

	rc_pub.publish(rc_out);
	loop_rate.sleep();
	}
	return 0;
}


