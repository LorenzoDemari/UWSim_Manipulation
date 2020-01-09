
#include "ros/ros.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Bool.h"
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
#include "sensor_msgs/Range.h"
#include "geometry_msgs/WrenchStamped.h"
//#include <asdl.h>

bool touch1 = false;
bool touch2 = false;


arma::Col<double> JacobianColumn(arma::Mat<double> j_transf_matrix, arma::Col<double> trasl_e);
arma::Mat<double> InitTransfMatrix(tfScalar matrix[16]);
void VersorLemma(arma::Mat<double> endeff, arma::Mat<double> goal, arma::Col<double> ang_error);
void ContactCallback1(geometry_msgs::WrenchStamped msg)
{
	//ROS_INFO("***************************** %f", msg.range);
    if (msg.wrench.force.x != 0)
    {
        touch1 = true;
    }
}

void ContactCallback2(geometry_msgs::WrenchStamped msg)
{
    //ROS_INFO("***************************** %f", msg.range);
    if (msg.wrench.force.x != 0)
    {
        touch2 = true;
    }
}

int main (int argc, char **argv) 
{
    std::string robot_name(argv[1]);
    std::string goal(argv[2]);

    std::string nodeName="InverseKinematicProblem_" + robot_name;
    std::string topic="/uwsim/" + robot_name + "_joint_state_command";
    std::string joint="girona500_" + robot_name;
    std::string contact_topic1 = "/g500" + robot_name + "/ForceSensor1";
    std::string contact_topic2 = "/g500" + robot_name + "/ForceSensor2";

    ros::init(argc, argv, nodeName);
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform transformation;
    ros::Subscriber contact1 = n.subscribe(contact_topic1, 1, ContactCallback1);
    ros::Subscriber contact2 = n.subscribe(contact_topic2, 1, ContactCallback2);

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
    arma::Col<double> Jac1(6);
    arma::Col<double> Jac2(6);
    arma::Col<double> Jac3(6);
    arma::Col<double> Jac4(6);

    arma::Mat<double> jacobian(6, 4, arma::fill::zeros);
    arma::Mat<double> jacobian2(6, 4, arma::fill::zeros);

    arma::Mat<double> Jac12(6, 2, arma::fill::zeros);
    arma::Mat<double> Jac34(6, 2, arma::fill::zeros);


    bool ready1 = false;
    bool ready2 = false;
    bool graspPar1 = false;
    bool graspPar2 = false;
    bool touch_param = false;
    double norm = 0.0;

    std::string grasp= "/grasp"+ robot_name;
    ros::param::set(grasp, false);



    ros::Publisher velocity_pub;
    velocity_pub=n.advertise<sensor_msgs::JointState>(topic,1);

    ros::Rate rate(10.0);

    while(n.ok()) {
        n.getParam("/readyRAUVI1", ready1);
        n.getParam("/readyRAUVI2", ready2);

        if (ready1 && ready2) {

            try {
                listener.waitForTransform(joint + "/kinematic_base", goal, ros::Time(0), ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", goal, ros::Time(0), transformation);

                tf::Vector3 new_goal;
                new_goal.setX(transformation.getOrigin().x());
                new_goal.setY(transformation.getOrigin().y());
                new_goal.setZ(transformation.getOrigin().z() - 0.00);
                transformation.setOrigin(new_goal);
                transformation.getOpenGLMatrix(matrix);

                goal_transf_matrix << matrix[0] << matrix[4] << 0 << matrix[12] << arma::endr
                                   << matrix[1] << matrix[5] << 0 << matrix[13] << arma::endr
                                   << matrix[2] << matrix[6] << -1 << matrix[14] << arma::endr
                                   << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

               // arma::cout << goal_transf_matrix << arma::endl << arma::endl;

                trasl_g << goal_transf_matrix(0, 3) << goal_transf_matrix(1, 3) << goal_transf_matrix(2, 3);


            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }

            try {
                listener.waitForTransform(joint + "/kinematic_base", joint + "/end_effector", ros::Time(0),ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", joint + "/end_effector", ros::Time(0), transformation);

                transformation.getOpenGLMatrix(matrix);

                endeff_transf_matrix = InitTransfMatrix(matrix);
            //    arma::cout << goal_transf_matrix << arma::endl << arma::endl;

                trasl_e << endeff_transf_matrix(0, 3) << endeff_transf_matrix(1, 3) << endeff_transf_matrix(2, 3);

            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }


            try {
                listener.waitForTransform(joint + "/kinematic_base", joint + "/part1", ros::Time(0), ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", joint + "/part1", ros::Time(0), transformation);

                transformation.getOpenGLMatrix(matrix);

                j1_transf_matrix = InitTransfMatrix(matrix);

                Jac1 = JacobianColumn(j1_transf_matrix, trasl_e);

                //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;

            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }

            try {
                listener.waitForTransform(joint + "/kinematic_base", joint + "/part2", ros::Time(0), ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", joint + "/part2", ros::Time(0), transformation);

                transformation.getOpenGLMatrix(matrix);

                j2_transf_matrix = InitTransfMatrix(matrix);

                Jac2 = JacobianColumn(j2_transf_matrix, trasl_e);
                //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }

            try {
                listener.waitForTransform(joint + "/kinematic_base", joint + "/part3", ros::Time(0), ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", joint + "/part3", ros::Time(0), transformation);

                transformation.getOpenGLMatrix(matrix);

                j3_transf_matrix = InitTransfMatrix(matrix);

                Jac3 = JacobianColumn(j3_transf_matrix, trasl_e);
                //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }

            try {
                listener.waitForTransform(joint + "/kinematic_base", joint + "/part4_base", ros::Time(0), ros::Duration(0.00005));
                listener.lookupTransform(joint + "/kinematic_base", joint + "/part4_base", ros::Time(0), transformation);

                transformation.getOpenGLMatrix(matrix);

                j4_transf_matrix = InitTransfMatrix(matrix);

                Jac4 = JacobianColumn(j4_transf_matrix, trasl_e);
                //arma::cout << "cross_product" << arma::endl << cross_prod_1 << arma::endl << arma::endl;
            }
            catch (tf::TransformException &ex100) {
                ROS_ERROR("%s", ex100.what());
            }


            Jac12 = join_horiz(Jac1,Jac2);
            Jac34 = join_horiz(Jac3,Jac4);
            jacobian = join_horiz(Jac12, Jac34);


//            arma::cout << "jacobian" << arma::endl << jacobian << arma::endl << arma::endl;


            VersorLemma(endeff_transf_matrix, goal_transf_matrix, angular_error);
            angular_error = 0.8 * angular_error;

            linear_error = 0.8 * (trasl_g - trasl_e);

            for (int i = 0; i < 3; i++)
                error(i) = angular_error(i);

            for (int j = 3; j < 6; j++)
                error(j) = linear_error(j - 3);


            qdot_arma = pinv(jacobian) * error;

            for (int k = 0; k < 4; k++)
                qdot[k] = qdot_arma(k);

            norm = arma::norm(error, 2);

            ROS_INFO("errore %f",norm);
            if (norm < 0.004)
            {
                ros::param::set(grasp, true);
                ros::param::get ("/graspRAUVI1",graspPar1);
                ros::param::get ("/graspRAUVI2",graspPar2);

                if (graspPar1 && graspPar2)
                {
                    //ROS_INFO("chiudo");
                    if (touch1 && touch2)
                    {
                        qdot[4] = 0.0;
                    }
                    else
                    {
                        qdot[4] = -0.5;
                    }
                }
            }

            else
            {
                ros::param::set(grasp, false);
		        touch1 = false;
                touch2 = false;
                qdot[4] = 1.0;
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
            js.name.push_back(std::string("JawOpening"));
            js.velocity.push_back(qdot[4]);

            velocity_pub.publish(js);

            if (touch1 && touch2)
            {
                ros::Duration(2.0).sleep();
                n.setParam("/touch" + robot_name, true);
            }


        }



        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}

arma::Mat<double> InitTransfMatrix(tfScalar matrix[16]){
    arma::Mat<double> transf_matrix(4, 4, arma::fill::eye);

    transf_matrix << matrix[0] << matrix[4] << matrix[8] << matrix[12] << arma::endr
                         << matrix[1] << matrix[5] << matrix[9] << matrix[13] << arma::endr
                         << matrix[2] << matrix[6] << matrix[10] << matrix[14] << arma::endr
                         << matrix[3] << matrix[7] << matrix[11] << matrix[15] << arma::endr;

    return transf_matrix;
}

arma::Col<double> JacobianColumn(arma::Mat<double> j_transf_matrix, arma::Col<double> trasl_e){
    arma::Col<double> column(6);
    arma::Col<double> ki(3);
    arma::Col<double> trasl(3);
    arma::Col<double> trasl_diff(3);
    arma::Col<double> cross_prod(3);

    ki << j_transf_matrix(0, 2) << j_transf_matrix(1, 2) << j_transf_matrix(2, 2);

    trasl << j_transf_matrix(0, 3) << j_transf_matrix(1, 3) << j_transf_matrix(2, 3);
    trasl_diff = trasl_e - trasl;
    cross_prod = arma::cross(ki, trasl_diff);

    column << ki(0) << ki(1) << ki(2) << cross_prod(0) << cross_prod(1) << cross_prod(2);
    return column;
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
