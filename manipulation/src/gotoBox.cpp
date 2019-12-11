/*
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */



/* Move a vehicle with a velocity proportional to the distance 
 * between the current pose and a desired absolute pose 
*/

#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>
#include <tf/transform_listener.h>

bool firstpass=false;
osg::Quat initialQ, goalQ, currentQ;
osg::Vec3d initialT, goalT, currentT;
double totalDistance, currentDistance;

void vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	if (!firstpass) {
		initialT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
		initialQ.set(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		totalDistance=(goalT-initialT).length();
		firstpass=true;
	} 
	currentT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	currentDistance=(goalT-currentT).length();
}

int main(int argc, char **argv) {

    if (argc < 2) {
        std::cerr << "USAGE: " << argv[0] << " <robotName>"<< std::endl;
        return 0;
    }


    std::string robot_name(argv[1]);
    std::string controlTopic = "/dataNavigator_G500" + robot_name;
    std::string poseTopic = "/uwsim/girona500_odom_" + robot_name;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;

	std::string nodeName = "gotoBox_" + robot_name;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform actual_pose;
	osg::Matrixd T, Rx, Ry, Rz, transform;

	ros::Publisher position_pub=nh.advertise<nav_msgs::Odometry>(controlTopic,1);
	ros::Subscriber position_sub = nh.subscribe(poseTopic, 1, vehiclePoseCallback);

	ros::Rate r(30);
    ros::Duration(3.0).sleep();

    std::string ready= "/ready"+ robot_name;

    bool touch = false;


    ros::param::set(ready, false);

    while (ros::ok()) {
        nh.getParam("/roll", roll);
        nh.getParam("/pitch", pitch);
        nh.getParam("/yaw", yaw);
        nh.getParam("/touch", touch);
        if (robot_name == "RAUVI1")
        {
            nh.getParam("/xbox1", x);
            nh.getParam("/ybox1", y);
            nh.getParam("/zbox1", z);
/*            if(touch)
	    {
		x = 0.0;
		y = 1.0;
		z = 0.0;
            }*/

        }
        else if (robot_name == "RAUVI2")
        {
            nh.getParam("/xbox2", x);
            nh.getParam("/ybox2", y);
            nh.getParam("/zbox2", z);
            if(touch)
            {
/*                x = -0.5;
                y = -1.0;
                z = 0.0;
		yaw = 1.57;
		pitch = 3.14;
		roll = 0.0;*/
		nh.shutdown();
            }
        }


//        ROS_INFO("%f %f %f \n", x, y, z);
//        ROS_INFO("%f %f %f", roll, pitch, yaw);

        T.makeTranslate(x,y,z);
        Rx.makeRotate(roll,1,0,0);
        Ry.makeRotate(pitch,0,1,0);
        Rz.makeRotate(yaw,0,0,1);
        transform=Rz*Ry*Rx*T;
        goalT=transform.getTrans();
        goalQ=transform.getRotate();

        if (firstpass) {
		osg::Vec3d vT=(goalT-currentT)*0.15;
		double vScale=(vT.length()>0.1) ? 0.1/vT.length() : 1;

		currentQ.slerp(1-currentDistance/totalDistance,initialQ, goalQ);
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=currentT.x()+vT.x()*vScale;
		odom.pose.pose.position.y=currentT.y()+vT.y()*vScale;
		odom.pose.pose.position.z=currentT.z()+vT.z()*vScale;
		odom.pose.pose.orientation.x=currentQ.x();
		odom.pose.pose.orientation.y=currentQ.y();
		odom.pose.pose.orientation.z=currentQ.z();
		odom.pose.pose.orientation.w=currentQ.w();

		odom.twist.twist.linear.x=0;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.linear.z=0;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=0;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		position_pub.publish(odom);
	   }

        try
        {
	listener.waitForTransform("/world", "girona500_" + robot_name +"/base_link", ros::Time(0),ros::Duration(0.00005));
	listener.lookupTransform("/world", "girona500_" + robot_name +"/base_link", ros::Time(0), actual_pose);

	bool x_check = actual_pose.getOrigin().x()>x-0.01 && actual_pose.getOrigin().x()<x+0.01;
	bool y_check = actual_pose.getOrigin().y()>y-0.01 && actual_pose.getOrigin().y()<y+0.01;
	bool z_check = actual_pose.getOrigin().z()>z-0.01 && actual_pose.getOrigin().z()<z+0.01;
	bool roll_check = actual_pose.getRotation().x()==roll;
	bool pitch_check = actual_pose.getRotation().y()==pitch;
	bool yaw_check = actual_pose.getRotation().z()==yaw;

      //  ROS_INFO("%d %d %d %d %d %d\n", x_check, y_check, z_check);//, roll_check, pitch_check, yaw_check);

/*
	ROS_INFO("ROLL  %f %f", actual_pose.getRotation().x(), roll);
	ROS_INFO("PITCH  %f %f", actual_pose.getRotation().y(), pitch);
	ROS_INFO("YAW  %f %f", actual_pose.getRotation().z(), yaw);
*/


	if(x_check && y_check && z_check ) // && roll_check && pitch_check && yaw_check)
	{
	    ros::param::set(ready, true);
	}


        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }

	ros::spinOnce();
	   r.sleep();
	}


	return 0;
}
