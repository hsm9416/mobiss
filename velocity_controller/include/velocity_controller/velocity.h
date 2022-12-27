#pragma once
#include <eigen3/Eigen/Dense>


#define PI 3.141592


//wheel distance
double a = 300;
double b = 300;

// each wheel angle
double wheel_theta_fl = 0;
double wheel_theta_rl = 0;
double wheel_theta_rr = 0;
double wheel_theta_fr = 0;

// model position & orientation
double Vx  = 10;
double Vy  = 0;
double rot = 0;

Eigen::Vector3d Pos; 

// joint velocity



// wheel position
double x_w_1 = a;
double x_w_2 = -a;
double x_w_3 = -a;
double x_w_4 = a;

double y_w_1 = b;
double y_w_2 = b;
double y_w_3 = -b;
double y_w_4 = -b;

// wheel position matrix 
Eigen::MatrixXd fl(1,2);
Eigen::MatrixXd rl(1,2);
Eigen::MatrixXd rr(1,2);
Eigen::MatrixXd fr(1,2);

// position matrix 
Eigen::MatrixXd P(3,8);
Eigen::MatrixXd X(4,8);

//trigonometric function
// W1
double s1 = sin(wheel_theta_fl);
double c1 = cos(wheel_theta_fl);
// W2
double s2 = sin(wheel_theta_rl);
double c2 = cos(wheel_theta_rl);
// W3
double s3 = sin(wheel_theta_rr);
double c3 = cos(wheel_theta_rr);
// W4
double s4 = sin(wheel_theta_fr);
double c4 = cos(wheel_theta_fr);


