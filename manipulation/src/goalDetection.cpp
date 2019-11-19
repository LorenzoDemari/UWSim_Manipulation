
#include "ros/ros.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>

//#include <fstream>



int main (int argc, char **argv) 
{
    ros::init(argc, argv, "goalDetection");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform BwrtW;
    double B_x = 0.0;
    double B_y = 0.0;
    double B_z = 0.0;

    ros::Rate rate(10.0);

    while(n.ok()) {
	ros::Duration(1.0).sleep();
	try {
	    listener.waitForTransform("/world", "blackbox", ros::Time(0), ros::Duration(0.00005));
	    listener.lookupTransform("/world", "blackbox", ros::Time(0), BwrtW);

	    B_x = BwrtW.getOrigin().x();
	    B_y = BwrtW.getOrigin().y();
	    B_z = BwrtW.getOrigin().z();

	//    BwrtW.getRotation().x();
	//    BwrtW.getRotation().y();
	//    BwrtW.getRotation().z();
	//    BwrtW.getRotation().w();

	    nav_msgs::Odometry pos;

	    pos.pose.pose.position.x = B_x;
	    pos.pose.pose.position.y = B_y;
	    pos.pose.pose.position.z = B_z;

	    pos.pose.pose.orientation.x = 1.77;
	    pos.pose.pose.orientation.y = 3.14;
	    pos.pose.pose.orientation.z = 0.0;
	    pos.pose.pose.orientation.w = 0.0;

	    ros::param::set("/xbox", B_x);
	    ros::param::set("/ybox", B_y);
	    ros::param::set("/zbox", B_z + 2.0);
	    ros::param::set("/pitch", 3.14);
	    ros::param::set("/yaw", 3.14);
	    ros::param::set("/roll", 0.0);
	}

	catch (tf::TransformException &ex100) {
	ROS_ERROR("%s", ex100.what());
	}

	ros::spinOnce();

	rate.sleep();
    }

    return 0;

}
