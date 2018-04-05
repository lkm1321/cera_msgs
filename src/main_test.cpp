#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> 
#include <cera_msgs/PoseStampedArray.h> 
#include <cera_msgs/Target.h>
#include <cera_msgs/TargetArray.h> 
#include <iostream> 

void sub_callback(const cera_msgs::PoseStampedArrayConstPtr& msg)
{
	std::cout << "Message size was: " << msg->points.size() << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "message_test_node"); 

	ros::NodeHandle nh; 
	
	ros::Publisher pose_msg_pub = nh.advertise<cera_msgs::PoseStampedArray>("plan", 100); 
	ros::Publisher target_msg_pub = nh.advertise<cera_msgs::TargetArray>("targets", 100); 

	ros::Duration d(5.0);
	d.sleep();

	ros::Subscriber msg_sub = nh.subscribe<cera_msgs::PoseStampedArray>("plan_sub", 100, sub_callback); 

	cera_msgs::PoseStampedArray pose_msg; 

	size_t count = 0; 
	geometry_msgs::PoseStamped point_to_add; 

	point_to_add.header.frame_id = "utm_56h"; // In ENU order. 

	point_to_add.pose.position.x = 334417.07; 
	point_to_add.pose.position.y = 6251354.86; 
	point_to_add.pose.position.z = 0.0; 

	cera_msgs::TargetArray target_msg; 
	cera_msgs::Target target_to_add; 

	target_to_add.x = 334417.07; 
	target_to_add.y = 6251354.86; 
	target_to_add.cov = {1.0, 0.1, 0.1, 1.0}; 
	target_to_add.importance = 1.0; 

	target_msg.targets.push_back(target_to_add); 

	ros::Rate r(10.0); 

	do
	{
		// Empty message on first call. 
		pose_msg_pub.publish(pose_msg); 

		point_to_add.header.seq++; 
		point_to_add.header.stamp = ros::Time::now() + ros::Duration(100.0); 
		point_to_add.pose.position.x += 10.0; 
		point_to_add.pose.position.y += 10.0; 

		pose_msg.points.push_back(point_to_add); 

		ros::spinOnce();
		count++; 

		if ( (count % 100) == 0)
		{
			pose_msg.points.clear();
			target_to_add.x += 10.0; 
			target_to_add.y += 10.0; 
			target_msg.targets.push_back(target_to_add);
			target_msg_pub.publish(target_msg);
		}

		r.sleep();
	} while (ros::ok()); 
}