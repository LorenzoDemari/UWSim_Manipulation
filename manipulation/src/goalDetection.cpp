
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
#include "/usr/include/armadillo"

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "goalDetection");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform BwrtW;
    double B_x = 0.0;
    double B_y = 0.0;
    double B_z = 0.0;

    double B_r_x = 0.0;
    double B_r_y = 0.0;
    double B_r_z = 0.0;
    double B_r_w = 0.0;


    tfScalar matrix[2];

    arma::Mat<double> goal_transf_matrix(4, 4, arma::fill::eye);
    //double d = det(goal_transf_matrix);
    //ROS_INFO("%f", d);

    ros::Rate rate(10.0);

    while(n.ok()) {
	ros::Duration(1.0).sleep();
	try {
	    listener.waitForTransform("/world", "blackbox", ros::Time(0), ros::Duration(0.00005));
	    listener.lookupTransform("/world", "blackbox", ros::Time(0), BwrtW);

        BwrtW.getOpenGLMatrix(matrix);

        goal_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
              << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
              << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
              << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

        arma::cout << goal_transf_matrix << arma::endl << arma::endl;


        B_x = BwrtW.getOrigin().x();
        B_y = BwrtW.getOrigin().y();
        B_z = BwrtW.getOrigin().z();

        B_r_x = BwrtW.getRotation().x();
        B_r_y = BwrtW.getRotation().y();
        B_r_z = BwrtW.getRotation().z();
        B_r_w = BwrtW.getRotation().w();

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
	    ros::param::set("/zbox", B_z + 2);
        ros::param::set("/pitch", 0.0);
        ros::param::set("/yaw", -1*B_r_y);
        ros::param::set("/roll", 3.14);
//	    ros::param::set("/pitch", 3.14);
//	    ros::param::set("/yaw", 3.14);
//	    ros::param::set("/roll", 0.0);


	}

	catch (tf::TransformException &ex100) {
	ROS_ERROR("%s", ex100.what());
	}

	ros::spinOnce();

	rate.sleep();
    }

    return 0;

}
