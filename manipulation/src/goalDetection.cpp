
#include "ros/ros.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <cmath>
#include <iostream>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btMatrixX.h>
#include <armadillo>
#include <tf/transform_broadcaster.h>

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "goalDetection");
    ros::NodeHandle n;

    tf::TransformListener listener;
//    tf::TransformBroadcaster goal1;
//    tf::TransformBroadcaster goal2;

    tf::StampedTransform BwrtW1;
    tf::StampedTransform BwrtW2;

    double B_x1 = 0.0;
    double B_y1 = 0.0;
    double B_z1 = 0.0;

    double B_r_x1 = 0.0;
    double B_r_y1 = 0.0;
    double B_r_z1 = 0.0;
    double B_r_w1 = 0.0;

    double B_x2 = 0.0;
    double B_y2 = 0.0;
    double B_z2 = 0.0;

    double B_r_x2 = 0.0;
    double B_r_y2 = 0.0;
    double B_r_z2 = 0.0;
    double B_r_w2 = 0.0;


    tfScalar matrix[2];

    arma::Mat<double> goal_transf_matrix(4, 4, arma::fill::eye);
    //double d = det(goal_transf_matrix);
    //ROS_INFO("%f", d);

    ros::Rate rate(10.0);

    while(n.ok()) {
	ros::Duration(1.0).sleep();
	try {
	    listener.waitForTransform("/world", "goal1", ros::Time(0), ros::Duration(0.00005));
	    listener.lookupTransform("/world", "goal1", ros::Time(0), BwrtW1);

        B_x1 = BwrtW1.getOrigin().x();
        B_y1 = BwrtW1.getOrigin().y();
        B_z1 = BwrtW1.getOrigin().z();

        B_r_x1 = BwrtW1.getRotation().x();
        B_r_y1 = BwrtW1.getRotation().y();
        B_r_z1 = BwrtW1.getRotation().z();
        B_r_w1 = BwrtW1.getRotation().w();


	    ros::param::set("/xbox1", B_x1 + cos(-1*B_r_y1)*0.4);
	    ros::param::set("/ybox1", B_y1 + sin(-1*B_r_y1)*0.4);
	    ros::param::set("/zbox1", B_z1 + 1.9);
        ros::param::set("/pitch", 0.0);
        ros::param::set("/yaw", -1*B_r_y1);
        ros::param::set("/roll", 3.14);

	}

	catch (tf::TransformException &ex100) {
	ROS_ERROR("%s", ex100.what());
	}

    try {
        listener.waitForTransform("/world", "goal2", ros::Time(0), ros::Duration(0.00005));
        listener.lookupTransform("/world", "goal2", ros::Time(0), BwrtW2);

        B_x2 = BwrtW2.getOrigin().x();
        B_y2 = BwrtW2.getOrigin().y();
        B_z2 = BwrtW2.getOrigin().z();

        B_r_x2 = BwrtW2.getRotation().x();
        B_r_y2 = BwrtW2.getRotation().y();
        B_r_z2 = BwrtW2.getRotation().z();
        B_r_w2 = BwrtW2.getRotation().w();


        ros::param::set("/xbox2", B_x2 + cos(-1*B_r_y2)*0.4);
        ros::param::set("/ybox2", B_y2 + sin(-1*B_r_y2)*0.4);
        ros::param::set("/zbox2", B_z2 + 1.9);
//        ros::param::set("/pitch", 0.0);
//        ros::param::set("/yaw", -1*B_r_y2);
//        ros::param::set("/roll", 3.14);

    }

    catch (tf::TransformException &ex100) {
        ROS_ERROR("%s", ex100.what());
    }

	ros::spinOnce();

	rate.sleep();
    }

    return 0;

}
