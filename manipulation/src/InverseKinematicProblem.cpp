
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

void VersorLemma(arma::Mat<double> endeff, arma::Mat<double> goal, arma::Col<double> ang_error);

int main (int argc, char **argv) 
{
//    std::string goal(argv[1]);
//    std::string controlTopic(argv[2]);
//    std::string nodeName=controlTopic;
//    nodeName.replace(0,6,"IKP_");
    std::string nodeName="IKP";
    std::string goal="blackbox";

    ros::init(argc, argv, nodeName);
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
    arma::Mat<double> endeff_transf_matrix(4, 4, arma::fill::eye);

    arma::Col<double> trasl_e(3);
    arma::Col<double> trasl_g(3);

    arma::Col<double> linear_error(3);
    arma::Col<double> angular_error(3);
    arma::Col<double> error(6);
    arma::Col<double> qdot_arma(4);

    arma::Mat<double> jacobian(6, 4, arma::fill::zeros);


    ros::Publisher velocity_pub;
    velocity_pub=n.advertise<sensor_msgs::JointState>("/uwsim/RAUVI_joint_state_command",1);

    ros::Rate rate(10.0);

    while(n.ok()) {
        ros::Duration(1.0).sleep();
        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "blackbox", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "blackbox", ros::Time(0), BwrtW);

            tf::Vector3 new_goal;
            new_goal.setX(BwrtW.getOrigin().x());
            new_goal.setY(BwrtW.getOrigin().y());
            new_goal.setZ(BwrtW.getOrigin().z()+2.0);
            BwrtW.setOrigin(new_goal);
            BwrtW.getOpenGLMatrix(matrix);

            goal_transf_matrix << matrix[0] << matrix[4] << 0 << matrix[12] << arma::endr
                               << matrix[1] << matrix[5] << 0 << matrix[13] << arma::endr
                               << matrix[2] << matrix[6] << -1 << matrix[14] << arma::endr
                               << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;
//            goal_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
//                  << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
//                  << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
//                  << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << goal_transf_matrix << arma::endl << arma::endl;

            trasl_g << goal_transf_matrix(0,3) << goal_transf_matrix(1,3) << goal_transf_matrix(2,3);


        }
        catch (tf::TransformException &ex100) {
        ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/end_effector", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/end_effector", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            endeff_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                               << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                               << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                               << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << goal_transf_matrix << arma::endl << arma::endl;

            trasl_e << endeff_transf_matrix(0,3) << endeff_transf_matrix(1,3) << endeff_transf_matrix(2,3);

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

            arma::cout << "part1" << arma::endl << j1_transf_matrix << arma::endl << arma::endl;

            arma::Col<double> ki_1(3);
            ki_1 << j1_transf_matrix(0,2) << j1_transf_matrix(1,2) << j1_transf_matrix(2,2);
            for (int i=0; i<3; i++){
                jacobian(i,0)=j1_transf_matrix(i,2);
            }
            arma::Col<double> trasl_1(3);
            trasl_1 << j1_transf_matrix(0,3) << j1_transf_matrix(1,3) << j1_transf_matrix(2,3);
            arma::Col<double> trasl_diff_1(3);
            trasl_diff_1 = trasl_e - trasl_1;
            arma::Col<double> cross_prod_1(3);
            cross_prod_1 = arma::cross(ki_1, trasl_diff_1);
            for (int i=3; i<6; i++){
                jacobian(i,0)=cross_prod_1(i-3);
            }
            //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;

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

            arma::cout << "part2" << arma::endl << j2_transf_matrix << arma::endl << arma::endl;

            arma::Col<double> ki_2(3);
            ki_2 << j2_transf_matrix(0,2) << j2_transf_matrix(1,2) << j2_transf_matrix(2,2);
            for (int i=0; i<3; i++){
                jacobian(i,1)=j2_transf_matrix(i,2);
            }
            arma::Col<double> trasl_2(3);
            trasl_2 << j2_transf_matrix(0,3) << j2_transf_matrix(1,3) << j2_transf_matrix(2,3);
            arma::Col<double> trasl_diff_2(3);
            trasl_diff_2 = trasl_e - trasl_2;
            arma::Col<double> cross_prod_2(3);
            cross_prod_2 = arma::cross(ki_2, trasl_diff_2);
            for (int i=3; i<6; i++){
                jacobian(i,1)=cross_prod_2(i-3);
            }
            //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
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

            arma::cout << "part3" << arma::endl << j3_transf_matrix << arma::endl << arma::endl;

            arma::Col<double> ki_3(3);
            ki_3 << j3_transf_matrix(0,2) << j3_transf_matrix(1,2) << j3_transf_matrix(2,2);
            for (int i=0; i<3; i++){
                jacobian(i,2)=j3_transf_matrix(i,2);
            }
            arma::Col<double> trasl_3(3);
            trasl_3 << j3_transf_matrix(0,3) << j3_transf_matrix(1,3) << j3_transf_matrix(2,3);
            arma::Col<double> trasl_diff_3(3);
            trasl_diff_3 = trasl_e - trasl_3;
            arma::Col<double> cross_prod_3(3);
            cross_prod_3 = arma::cross(ki_3, trasl_diff_3);
            for (int i=3; i<6; i++) {
                jacobian(i, 2) = cross_prod_3(i - 3);
            }
            //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }

        try {
            listener.waitForTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part4_base", ros::Time(0), ros::Duration(0.00005));
            listener.lookupTransform("girona500_RAUVI/kinematic_base", "girona500_RAUVI/part4_base", ros::Time(0), BwrtW);

            BwrtW.getOpenGLMatrix(matrix);

            j4_transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                             << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                             << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                             << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

            arma::cout << "part4" << arma::endl << j4_transf_matrix << arma::endl << arma::endl;

            arma::Col<double> ki_4(3);
            ki_4 << j4_transf_matrix(0,2) << j4_transf_matrix(1,2) << j4_transf_matrix(2,2);
            for (int i=0; i<3; i++){
                jacobian(i,3)=j4_transf_matrix(i,2);
            }
            arma::Col<double> trasl_4(3);
            trasl_4 << j4_transf_matrix(0,3) << j4_transf_matrix(1,3) << j4_transf_matrix(2,3);
            arma::Col<double> trasl_diff_4(3);
            trasl_diff_4 = trasl_e - trasl_4;
            arma::Col<double> cross_prod_4(3);
            cross_prod_4 = arma::cross(ki_4, trasl_diff_4);
            for (int i=3; i<6; i++) {
                jacobian(i, 3) = cross_prod_4(i - 3);
            }
            //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
        }
        catch (tf::TransformException &ex100) {
            ROS_ERROR("%s", ex100.what());
        }



        arma::cout << "jacobian" << arma::endl << jacobian << arma::endl << arma::endl;


        VersorLemma(endeff_transf_matrix, goal_transf_matrix, angular_error);
        angular_error = 0.2*angular_error;

        linear_error = 0.2*(trasl_g - trasl_e);

        for (int i = 0; i<3; i++)
            error(i) = angular_error(i);

        for (int j = 3; j<6; j++)
            error(j) = linear_error(j-3);



        qdot_arma = pinv(jacobian)*error;

        for (int k = 0; k<4; k++)
            qdot[k] = qdot_arma(k);
        qdot[4] = -1.0;


        sensor_msgs::JointState js;
        js.name.push_back(std::string("Slew"));
        js.velocity.push_back(qdot[0]);
        js.name.push_back(std::string("Shoulder"));
        js.velocity.push_back(qdot[1]);
        js.name.push_back(std::string("Elbow"));
        js.velocity.push_back(qdot[2]);
        js.name.push_back(std::string("JawRotate"));
        js.velocity.push_back(qdot[3]);
        js.name.push_back(std::string("JawOpening"));
        js.velocity.push_back(qdot[4]);

        velocity_pub.publish(js);



        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}


void VersorLemma(arma::Mat<double> endeff, arma::Mat<double> goal, arma::Col<double> ang_error)
{
    arma::Col<double> ia;
    arma::Col<double> ja;
    arma::Col<double> ka;
    arma::Col<double> ib;
    arma::Col<double> jb;
    arma::Col<double> kb;
    double d = 0.0;
    double delta = 0.0;
    arma::Col<double> cross_vector;
    arma::Col<double> sigma;
    double theta = 0.0;
    arma::Col<double> vector;


    ia << endeff(0,0) << endeff(1,0) << endeff(2,0);
    ja << endeff(0,1) << endeff(1,1) << endeff(2,1);
    ka << endeff(0,2) << endeff(1,2) << endeff(2,2);

    ib << goal(0,0) << goal(1,0) << goal(2,0);
    jb << goal(0,1) << goal(1,1) << goal(2,1);
    kb << goal(0,2) << goal(1,2) << goal(2,2);

    d = dot(ia,ib) + dot(ja,jb) + dot(ka,kb);

    delta = (d-1)/2;

    cross_vector = cross(ia,ib) + cross(ja,jb) + cross(ka,kb);
    sigma = cross_vector/2;
    theta = atan2(norm(sigma), delta);

    vector = (sigma)/(norm(sigma));

    if (delta == -1)
    {
        theta = M_PI;
        vector = (ia+ib+ja+jb+ka+kb)/norm(ia+ib+ja+jb+ka+kb);
    }
    ang_error = theta*vector;
    if (delta == 0)
    {
        ang_error.fill(0.0);
    }
}
