//
// Created by dango on 2021/11/3.
//

#include "MPC.hpp"
#include "uniform_bspline.h"
#include "math.h"
#include <vector>

////ros include
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#define resolution 0.05
#define x_offset -1.44
#define y_offset -4.97
#define PI 3.1415926
#define N 300

using namespace Eigen;
using namespace std;

double points_template[20][2] = {{59,95},{68.2612,96.7822},{76.9734,100.147},{85.3319,104.507},{93.5119,109.348},
                                 {101.426,114.926},{109.109,121.228},{116.881,127.309},{125.255,131.749},{134.643,133.213},
                                 {145.082,131.581},{156.158,128.073},{166.946,125.402},{176.635,125.987},{184.754,131.046},
                                 {191.54,139.917},{198.267,148.922},{206.042,154.98},{214.88,158.017},{224.557,158.64}};

ego_planner::UniformBspline B_spline_trajectory;
double ts=0.5,t_step=0.01;
double t,tmp;

ros::Publisher trajectory_pub;
ros::Publisher trajectory_predict_pub;
ros::Publisher start_point_pub;
ros::Publisher predict_end_point_pub;
nav_msgs::Path b_spline_trajectory;
nav_msgs::Path trajectory_predict;
geometry_msgs::PointStamped start_point;
geometry_msgs::PointStamped predict_end_point;

MPC_controller mpc_controller;

int main(int argc, char** argv) {

    ros::init(argc, argv, "mpc_test_node");
    ros::NodeHandle n("mpc_test");
    ros::Rate loop_rate(500);

    start_point.header.frame_id = "/map";
//    start_point.point.x = 1.98;
//    start_point.point.y = 0.472;
    start_point.point.x = 68.2612*resolution+x_offset;
    start_point.point.y = 96.7822*resolution + y_offset;

    MatrixXd control_points = MatrixXd::Zero(2, 20);
    for(int i=0;i<20;i++)
    {
        Vector2d current_point(points_template[i][0],points_template[i][1]);
        control_points.col(i) = current_point;
    }
    trajectory_pub = n.advertise<nav_msgs::Path>("/Bspline_trajectory",10);
    trajectory_predict_pub = n.advertise<nav_msgs::Path>("/trajectory_predict",10);
    start_point_pub = n.advertise<geometry_msgs::PointStamped>("/start_point",10);
    predict_end_point_pub = n.advertise<geometry_msgs::PointStamped>("/predict_point",10);
    B_spline_trajectory.setUniformBspline(control_points,3,ts);
    B_spline_trajectory.getTimeSpan(t,tmp);

    b_spline_trajectory.header.frame_id = "map";
    trajectory_predict.header.frame_id = "map";
    predict_end_point.header.frame_id = "map";

    for(double t_c=t;t_c<tmp;t_c+=t_step)
    {
        Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = point_temp(0)*resolution+x_offset;
        current_pose.pose.position.y = point_temp(1)*resolution+y_offset;
        current_pose.header.frame_id = "/map";
        b_spline_trajectory.poses.push_back(current_pose);
    }



    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::Vector2d pos_r,pos_r_1,pos_r_2,u_r,v_r_1,v_r_2;
    Eigen::Vector3d x_r,x_r_1,x_r_2,X_k;
    MatrixXd u_k;
    double v_linear_1,v_linear_2,w;
    double t_c = t;
    double t_k,t_k_1,t_k_2;

    ros::Time t_start = ros::Time::now();
    Eigen::Vector2d pos_first = B_spline_trajectory.evaluateDeBoor(t);
    Eigen::Vector2d pos_second = B_spline_trajectory.evaluateDeBoor(t+t_step);
    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));
    for(int i=0;i<N;i++)
    {
        t_k = t_c+i*t_step;
        t_k_1 = t_c+(i+1)*t_step;
        t_k_2 = t_c+(i+2)*t_step;

        pos_r = B_spline_trajectory.evaluateDeBoor(t_k)*resolution;
        pos_r_1 = B_spline_trajectory.evaluateDeBoor(t_k_1)*resolution;
        pos_r_2 = B_spline_trajectory.evaluateDeBoor(t_k_2)*resolution;
        x_r(0) = pos_r(0);
        x_r(1) = pos_r(1);
        x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
        v_r_1 = (pos_r_1-pos_r)/t_step;
        v_r_2 = (pos_r_2-pos_r_1)/t_step;
        v_linear_1 = v_r_1.norm();
        v_linear_2 = v_r_2.norm();

        double yaw1 = atan2(v_r_1(1),v_r_1(0));
        double yaw2 = atan2(v_r_2(1),v_r_2(0));
        if(abs(yaw2-yaw1)>PI)
        {
            cout<<"current index : "<<i+1<<endl;
            cout<<"yaw 1 : "<<yaw1<<endl;
            cout<<"yaw 2 : "<<yaw2<<endl;
            if((yaw2-yaw1)<0)
            {
                w = (2*PI+(yaw2-yaw1))/t_step;
            }
            else
            {
                w = ((yaw2-yaw1)-2*PI)/t_step;
            }
        }
        else
        {
            w = (yaw2-yaw1)/t_step;
        }
        //cout<<"reference vel : "<<v_linear_1<<endl;
        //cout<<"reference w : "<<w<<endl;
        u_r(0) = v_linear_1;
        u_r(1) = w;
        X_r.push_back(x_r);
        U_r.push_back(u_r);
    }
//    X_k(0) = 1.98-x_offset;
//    X_k(1) = 0.472-y_offset;
    X_k(0) =68.2612*resolution;
    X_k(1) =96.7822*resolution;
    X_k(2) = X_r[0](2);
    u_k = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);

    MatrixXd A_k = MatrixXd::Zero(3,3);
    MatrixXd I = MatrixXd::Identity(3,3);
    MatrixXd B_k = MatrixXd::Zero(3,2);
    Vector3d X_k_1;
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = 68.2612*resolution+x_offset;
    current_pose.pose.position.y = 96.7822*resolution + y_offset;
    trajectory_predict.poses.push_back(current_pose);
    for(int i=0;i<N;i++)
    {
        A_k(0,2) = -U_r[i](0)*sin(X_r[i](2));
        A_k(1,2) = U_r[i](0)*cos(X_r[i](2));
        B_k(0,0) = cos(X_r[i](2));
        B_k(1,0) = sin(X_r[i](2));
        B_k(2,1) = 1;
        //u_k.col(i)(1)*=-1;
        X_k_1 = (I+t_step*A_k)*X_k+t_step*B_k*u_k.col(i)-t_step*A_k*X_r[i];
        X_k = X_k_1;

        current_pose.pose.position.x = X_k_1(0)+x_offset;
        current_pose.pose.position.y = X_k_1(1)+y_offset;
        trajectory_predict.poses.push_back(current_pose);
    }

    t_c = t + N*t_step;
    predict_end_point.point.x = B_spline_trajectory.evaluateDeBoor(t_c)(0)*resolution+x_offset;
    predict_end_point.point.y = B_spline_trajectory.evaluateDeBoor(t_c)(1)*resolution+y_offset;

    ros::Time t_end = ros::Time::now();
    ROS_INFO("control total time : %5.3f ms",(t_end-t_start).toSec()*1000);

//    for(int i=0;i<N;i++)
//    {
//
//    }

    while(ros::ok())
    {
        trajectory_pub.publish(b_spline_trajectory);
        trajectory_predict_pub.publish(trajectory_predict);
        start_point_pub.publish(start_point);
        predict_end_point_pub.publish(predict_end_point);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}