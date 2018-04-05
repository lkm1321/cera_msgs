#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> 
#include <cera_msgs/PoseStampedArray.h> 
#include <iostream> 

void sub_callback(const cera_msgs::PoseStampedArrayConstPtr& msg)
{
	std::cout << "Message size was: " << msg->points.size() << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "message_test_node"); 

	ros::NodeHandle nh; 
	ros::Publisher msg_pub = nh.advertise<cera_msgs::PoseStampedArray>("plan", 100); 
	ros::Duration d(5.0);
	d.sleep();

	ros::Subscriber msg_sub = nh.subscribe<cera_msgs::PoseStampedArray>("plan_sub", 100, sub_callback); 

	cera_msgs::PoseStampedArray msg; 

	size_t count = 0; 
	geometry_msgs::PoseStamped point_to_add; 

	point_to_add.header.frame_id = "utm_56h"; // In ENU order. 

	point_to_add.pose.position.x = 334417.07; 
	point_to_add.pose.position.y = 6251354.86; 
	point_to_add.pose.position.z = 0.0; 

	ros::Rate r(10.0); 

	do
	{
		// Empty message on first call. 
		msg_pub.publish(msg); 

		point_to_add.header.seq++; 
		point_to_add.header.stamp = ros::Time::now() + ros::Duration(100.0); 
		point_to_add.pose.position.x += 10.0; 
		point_to_add.pose.position.y += 10.0; 

		msg.points.push_back(point_to_add); 

		ros::spinOnce();
		count++; 

		if ( (count % 100) == 0)
		{
			msg.points.clear();
		}

		r.sleep();
	} while (ros::ok()); 
}