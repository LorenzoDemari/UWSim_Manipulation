
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
#include <sensor_msgs/JointState.h>

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "InverseKinematicProblem");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform BwrtW;
    double qdot[5];


    tfScalar matrix[2];

    arma::Mat<double> goal_transf_matrix(4, 4, arma::fill::eye);
    arma::Mat<double> j1_transf_matrix(4, 4, arma::fill::eye);
    arma::Mat<double> j2_transf_matrix(4, 4, arma::fill::eye);
    arma::Mat<double> j3_transf_matrix(4, 4, arma::fill::eye);
    arma::Mat<double> j4_transf_matrix(4, 4, arma::fill::eye);



    ros::Publisher velocity_pub;
    velocity_pub=n.advertise<sensor_msgs::JointState>("/uwsim/RAUVI_joint_state_command",1);

    ros::Rate rate(10.0);

    while(n.ok()) {
        ros::Duration(1.0).sleep();
        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "blackbox", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "blackbox", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            goal_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                  << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                  << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                  << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << goal_transf_matrix << arma::endl << arma::endl;

        }
        catch (tf::TransformException &ex100) {
        ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part1", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part1", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            j1_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                             << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                             << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                             << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << j1_transf_matrix << arma::endl << arma::endl;

        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part2", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part2", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            j2_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                             << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                             << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                             << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << j1_transf_matrix << arma::endl << arma::endl;

        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part3", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part3", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            j3_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                             << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                             << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                             << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << j1_transf_matrix << arma::endl << arma::endl;

        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part4", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part4", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            j4_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                             << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                             << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                             << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << j1_transf_matrix << arma::endl << arma::endl;

        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }




        sensor_msgs::JointState js;
        js.name.push_back(std::string("Slew"));
        js.velocity.push_back(qdot[0]);
        js.name.push_back(std::string("Shoulder"));
        js.velocity.push_back(qdot[1]);
        js.name.push_back(std::string("Elbow"));
        js.velocity.push_back(qdot[2]);
        js.name.push_back(std::string("JawRotate"));
        js.velocity.push_back(qdot[3]);
//        js.name.push_back(std::string("JawOpening"));
//        js.velocity.push_back(qdot[4]);

        velocity_pub.publish(js);



        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}
