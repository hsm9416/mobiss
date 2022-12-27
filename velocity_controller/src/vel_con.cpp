#include <ros/ros.h>
// #include "velocity.h"
#include "velocity.h"
#include "inv/wheel.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>




int main(int argc, char** argv){
    ros::init(argc, argv, "set_model_vel");
    ros::NodeHandle nh;

    ros::Publisher pub= nh.advertise<inv::wheel>("/wheel_vel",100); 

    Eigen::MatrixXd X_t = X.transpose();

    Eigen::MatrixXd X_inv =(X_t * X).inverse(); 

    Eigen::MatrixXd X_plus = X_inv * X; 

    Eigen::Vector4f wheel_vel = X_plus * P * Pos;
       

     inv::wheel wheel1; 
     inv::wheel wheel2; 
     inv::wheel wheel3; 
     inv::wheel wheel4; 

     ros::Rate loop_rate(100);

     std_msgs::Float64 wheel1;
     std_msgs::Float64 wheel2;

     std_msgs::Float64 wheel3;
     std_msgs::Float64 wheel4;

    
    while(ros::ok())
        { 
            wheel1.vel1 = wheel_vel[0];
            wheel2.vel2 = wheel_vel[1];
            wheel3.vel3 = wheel_vel[2];
            wheel4.vel4 = wheel_vel[3];

               
            pub.publish(vel1);
            pub.publish(vel2);
            pub.publish(vel3);
            pub.publish(vel4);
        
            ros::spinOnce();
            loop_rate.sleep();     
        }

        return 0;

    }
    
