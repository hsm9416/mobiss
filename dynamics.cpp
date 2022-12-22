// get_inv_dynamics.cpp
#include <ros/ros.h>
#include "hmc_control/dynamics.h"
#include <iostream>
#include <math.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "hmc_msgs/actual.h"
#include "hmc_msgs/impedance_F.h"
#include "hmc_msgs/joint_F.h"
#include "hmc_msgs/jacobi.h"
using namespace Eigen;

void Initial(){
    // set position, velocity, acceration//
    //euler angle(rad)
    //phi = 0, the = 1 psi = 2
    // if (deg_euler[1] == 0.0000){
    //     deg_euler[1] = 0.001;
    // }

    euler << (deg_euler[0]+90)*(PI/180), 
            (deg_euler[1])*(PI/180),
            (deg_euler[2]-90)*(PI/180);
    // ROS_INFO("euler : %f", euler[1]*180/PI);
    euler_d << deg_euler_d[0]*(PI/180),
            deg_euler_d[1]*(PI/180),
            deg_euler_d[2]*(PI/180);
    
    euler_dd << deg_euler_dd[0]*(PI/180),
                deg_euler_dd[1]*(PI/180),
                deg_euler_dd[2]*(PI/180);
    
    

    //platform_dis_X
    x_p << -0.1+x[0], x[1], 0.39+x[2]; 
    //platform_vel_x_1
    x_v << x_d[0], x_d[1], x_d[2]; 
    //platform_acc_X_2
    x_a << x_dd[0], x_dd[1], x_dd[2]; 
    ///////////////////////////////////////


    //platform framevetcor//////////////////////////////////////
    Vector3d p_1, p_2, p_3, p_4, p_5, p_6; 
    p_1 << 0.095*cos(40 * PI/180), 0.095*sin(40 * PI/180), 0;
    p_2 << 0.095*cos(80 * PI/180), 0.095*sin(80 * PI/180), 0;
    p_3 << 0.095*cos(160 * PI/180), 0.095*sin(160 * PI/180), 0;
    p_4 << 0.095*cos(200 * PI/180), 0.095*sin(200 * PI/180), 0;
    p_5 << 0.095*cos(280 * PI/180), 0.095*sin(280 * PI/180), 0;
    p_6 << 0.095*cos(320 * PI/180), 0.095*sin(320 * PI/180), 0;

    // input to MatrixXd p(3,6);
    p.col(0) << p_1;
    p.col(1) << p_2;
    p.col(2) << p_3;
    p.col(3) << p_4;
    p.col(4) << p_5;
    p.col(5) << p_6;

    Vector3d b_1, b_2, b_3, b_4, b_5, b_6; 
    b_1 << 0.120*cos(20 * PI/180), 0.120*sin(20 * PI/180), 0;
    b_2 << 0.120*cos(100 * PI/180), 0.120*sin(100 * PI/180), 0;
    b_3 << 0.120*cos(140 * PI/180), 0.120*sin(140 * PI/180), 0;
    b_4 << 0.120*cos(220 * PI/180), 0.120*sin(220 * PI/180), 0;
    b_5 << 0.120*cos(260 * PI/180), 0.120*sin(260 * PI/180), 0;
    b_6 << 0.120*cos(340 * PI/180), 0.120*sin(340 * PI/180), 0;

    // input to MatrixXd b(3,6);
    b.col(0) << b_1;
    b.col(1) << b_2;
    b.col(2) << b_3;
    b.col(3) << b_4;
    b.col(4) << b_5;
    b.col(5) << b_6;
    //////////////////////////////////////////////////////////

    //rotational matrix///////////////////////////
    //Matrix3d r_x, r_y, r_z;
    Vector3d r_x, r_y, r_z;
    // r_x = AngleAxisd(euler[0], Vector3d::UnitZ());
    // r_y = AngleAxisd(euler[1], Vector3d::UnitY());
    // r_z = AngleAxisd(euler[2], Vector3d::UnitZ());
    r_x << cos(euler[2])*cos(euler[0])-cos(euler[1])*sin(euler[0])*sin(euler[2]) , cos(euler[2])*sin(euler[0])+cos(euler[1])*cos(euler[0])*sin(euler[2]) , sin(euler[2])*sin(euler[1]);
    r_y << -sin(euler[2])*cos(euler[0])-cos(euler[1])*sin(euler[0])*cos(euler[2]), -sin(euler[2])*sin(euler[0])+cos(euler[1])*cos(euler[0])*cos(euler[2]), cos(euler[2])*sin(euler[1]);
    r_z << sin(euler[1])*sin(euler[0]) , -sin(euler[1])*cos(euler[0]) , cos(euler[1]);
    R.col(0) = r_x;
    R.col(1) = r_y;
    R.col(2) = r_z;
    //////////////////////////////////////////////

    //angular velocity//////////////////////////////////////////////////////////////
    Matrix3d w_vel;
    Vector3d w_1, w_2, w_3;

    w_1 << 0,0,1;
    w_2 << cos(euler[0]), sin(euler[0]), 0;
    w_3 << sin(euler[0])*cos(euler[1]), -cos(euler[0])*sin(euler[1]), cos(euler[1]);

    w_vel.col(0) <<w_1;
    w_vel.col(1) <<w_2;
    w_vel.col(2) <<w_3;

    w << w_vel * euler_d;
    ///////////////////////////////////////////////////////////////////////////////

    //angular acceration ////////////////////////////////////////////////////////////////
    Matrix3d a_acc;
    Vector3d alp, a_1, a_2, a_3;

    a_1 << 0,0,0;
    a_2 << -euler_d[0]*sin(euler[0]), euler_d[0]*cos(euler[0]), 0;
    a_3 << euler_d[0]*cos(euler[0])*sin(euler[1])+euler_d[1]*sin(euler[0])*cos(euler[1]),
          euler_d[0]*sin(euler[0])*sin(euler[1])-euler_d[1]*cos(euler[0])*cos(euler[1]),
          -euler_d[1]*sin(euler[1]);

    a_acc.col(0) <<a_1;
    a_acc.col(1) <<a_2;
    a_acc.col(2) <<a_3;

    alp << w_vel * euler_dd + a_acc * euler_d;
    /////////////////////////////////////////////////////////////////////////////////////

    //Gravity term
    G << -9.8, 0.0, 0.0;


    //////////////////////////////////////////////
    c.col(0) = Vector3d::UnitX().cross(Vector3d::UnitY());
    c.col(1) = Vector3d::UnitY().cross(-1 * Vector3d::UnitX());
    c.col(2) = Vector3d::UnitY().cross(-1 * Vector3d::UnitX());
    c.col(3) = Vector3d::UnitY().cross(-1 * Vector3d::UnitX());
    c.col(4) = Vector3d::UnitY().cross(-1 * Vector3d::UnitX());
    c.col(5) = Vector3d::UnitX().cross(Vector3d::UnitY());

    
    Ipp << 7.7193130*pow(10,-2), -3.4624009*pow(10,-7), 1.0079550*pow(10,-6),
           -3.4624009*pow(10,-7), 1.1936907*pow(10,-1), 5.5786021*pow(10,-5),
           1.10079550*pow(10,-6), 5.5786021*pow(10,-5), 7.7138054*pow(10,-2);

    F_joint = VectorXd::Zero(6);
    F_total =  VectorXd::Zero(6);
}

//inverse position kinematics
void I_p_K(){
    Vector3d p_col, L_col;
    MatrixXd a_p_1(3,6);
    // a_p : point of a
    for(int i =0 ; i<6 ; i++){
        p_col = p.col(i);
        a_p.col(i) << x_p + R*p_col; // point_a
        
        
    }
    

    //Vector of link ith 
    for(int i = 0 ; i <6 ; i++){
        L.col(i) = a_p.col(i) - b.col(i);
    }

    //length l of link ith  => scalar_L
    Vector3d l_1;
    for(int i = 0 ; i <6 ; i++){
        L_col = L.col(i);
        l[i] = (L_col.transpose() * L_col);//.cwiseSqrt();
    }
    l = l.cwiseSqrt();
    

    //unit vector prismatic joint n
    for(int i=0 ; i<6 ; i++){
        n.col(i) = L.col(i).array() / l[i];
    }
}

//inverse rate kinematics
void I_r_k(){
    Vector3d a_p_col;
    //velocity of point a
    for(int i=0 ; i <6 ; i++){
        a_p_col = a_p.col(i);
        a_v.col(i) = x_v + w.cross(R*a_p_col);
    }
    //extension_rate_of_linki_li
    for(int i=0 ; i<6 ; i++){
        l_vel[i] = a_v.col(i).adjoint() * n.col(i);
    }
    

}

MatrixXd jacov_1_inv(){
    MatrixXd j_1_inv(6,6);
    MatrixXd j1(3,6), j2(3,6);
    Vector3d j1_col, n_col, p_col;
    
    for(int i=0 ; i<6 ; i++){
        p_col = p.col(i);
        j1.col(i) =  R * p_col;
        n_col = n.col(i);
        j1_col = j1.col(i);
        j2.col(i) = j1_col.cross(n_col);
    }
    
    j_1_inv.topLeftCorner(6,3) = n.transpose();
    j_1_inv.topRightCorner(6,3) = j2.transpose();
    


    return j_1_inv;
}

// MatrixXd jacov_2_inv(){
//     MatrixXd j_2_inv(6,6);
//     Matrix3d jR;

//     jR << 0, cos(euler[0]), sin(euler[0])*sin(euler[1]),
//           0, sin(euler[0]), -cos(euler[0])*sin(euler[1]),
//           1, 0            , cos(euler[1]);

//     j_2_inv.topLeftCorner(3,3).setIdentity();
//     j_2_inv.topRightCorner(3,3).setZero();
//     j_2_inv.bottomLeftCorner(3,3).setZero();
//     j_2_inv.bottomRightCorner(3,3) = jR;
    
//     return j_2_inv;
// }



//inverse acceleration kinematics
void I_a_K(){
    Vector3d a_p_col;
    //a_a: acceleration of point a, n_d: differentiation n
    for(int i=0 ; i<6 ; i++){
        a_p_col = a_p.col(i);
        a_a.col(i) = x_a + alp.cross(R*a_p_col) + w.cross(w.cross(R*a_p_col));
    }

    for(int i=0 ; i<6 ; i++){
        n_d.col(i) = (L.col(i) - l_vel[i] *  n.col(i)).array() / l[i];
    }

}

//N solver
Vector3d cal_N_1(Vector3d& alp_i,Vector3d& unit){
    Vector3d a1__1, a1__2;
    a1__1 = (Iaa1+Iaa2)*(alp_i.dot(unit))*unit;
    a1__2 = ((Inn1+Inn2)*unit).cross(alp_i.cross(unit));

    return a1__1 + a1__2;
}

Vector3d cal_N_2(Vector3d& w_i,Vector3d& unit){
    Vector3d a2__ ;
    a2__ = I * w_i.dot(unit)*unit;
    return a2__.cross(w_i);
}

//inverse dynamics equation      
VectorXd Inv_Dynamics(){
    Vector3d a_p_col, a_v_col, a_a_col, n_col, n_d_col, l_w_col, l_alp_col, c_col, N_col, p_col;//change row component
    Vector3d l_acc2_1;
    VectorXd l_acc1(6), l_acc2(6);
    for(int i=0 ; i<6 ; i++){
        l_acc1[i] = a_a.col(i).dot(n.col(i));
        l_acc2[i] = a_v.col(i).dot(n_d.col(i));
        //l_acc[i] = l_acc1[i] - l_acc2[i];
    }
    l_acc = l_acc1 - l_acc2;
    //std::cout << "l_acc = " << l_acc << std::endl;

    MatrixXd l_w(3,6), l_alp(3,6);

    //link ang_vel(l_w)
    for(int i=0 ; i<6 ; i++){
        n_col = n.col(i);
        a_v_col = a_v.col(i);
        l_w.col(i) = (n_col.cross(a_v_col)).array()/l[i];
    }

    //ang_acc(l_alp)
    for(int i=0 ; i<6 ; i++){
        n_col = n.col(i);
        a_a_col = a_a.col(i);
        l_w_col = l_w.col(i);
        l_alp.col(i) = (n_col.cross(a_a_col).array() - 2 * l_vel[i]*l_w_col.array()).array()/l[i];
        
    }

    //acceleration of the centers of mass of part 1, part2
    MatrixXd ai1(3,6), ai_1(3,6), ai_2(3,6), ai_3(3,6);
    Vector3d ai_2_1, ai_3_1, ai_31, ai_32;
   
    for(int i = 0 ; i <6 ; i++){
        l_w_col = l_w.col(i);
        l_alp_col = l_alp.col(i);
        n_col = n.col(i);
         //ai1 component
        ai_1.col(i) = (l[i] - l1)*(l_w_col.cross(l_w_col).cross(n_col)).array();
        ai_2_1 = (l[i] - l1) * l_alp_col.array();
        ai_2.col(i) = ai_2_1.cross(n_col);
        ai_3_1 = l_vel[i] * n_col.array();
        ai_31 = (2*l_w_col).cross(ai_3_1);
        ai_32 = l_acc[i] * n_col.array();
        ai_3.col(i) = ai_31 + ai_32;
        //complete ai1
        ai1.col(i) = ai_1.col(i) + ai_2.col(i) + ai_3.col(i);
    }
    

    MatrixXd ai2(3,6);
    Vector3d ai_1_col, ai_2_col, ai_3_col, ai2_1, ai2_2, ai2_3;
    for(int i = 0 ; i <6 ; i++){
        l_w_col = l_w.col(i);
        n_col = n.col(i);
        //ai2 component
        ai2_1 = l2*l_w_col.array();
        ai2_2 = l_w_col.cross(n_col);
        ai2_3 = l2*l_alp_col.array();
        //complete ai2
        ai2.col(i) = ai2_1.cross(ai2_2) + ai2_3.cross(n_col);
    }

    Matrix3d Ip = R * Ipp * R.transpose();// moment of inertia of the platform
    
    MatrixXd N_1(3,6), N_2(3,6), N(3,6);
    Vector3d ai1_col, ai2_col, N_1_col, N_2_col, N1, N2, N3, N4;
    N = MatrixXd::Zero(3,6);
    N_1 = MatrixXd::Zero(3,6);
    N_2 = MatrixXd::Zero(3,6);
    for(int i=0 ; i <6 ; i++){
        l_alp_col = l_alp.col(i);
        l_w_col = l_w.col(i);
        n_col = n.col(i);
        ai1_col = ai1.col(i);
        ai2_col = ai2.col(i);
        N_1_col = N_1.col(i);
        N_2_col = N_2.col(i);
        N_1.col(i) = cal_N_1(l_alp_col, n_col);    //(58)
        N_2.col(i) = cal_N_2(l_w_col, n_col);     //(59)
        N1 = -m1 * (l[i] - l1) * (n_col.cross(G)).array();
        N2 = m2 * l2 * n_col.cross(G);
        N3 = m1 * (l[i] - l1) * (n_col.cross(ai1_col)).array();
        N4 = m2 * l2 * n_col.cross(ai2_col);
        
        N.col(i) = N1 - N2 + N_1_col - N_2_col + N3 + N4;
        
    }
    //std::cout << "N :\n" << N << std::endl;
    //std::cout << "ans :\n" << N_2 << std::endl;
    VectorXd m_i(6);  //moment
    for(int i=0 ; i<6 ; i++){
        N_col = N.col(i);
        n_col = n.col(i);
        c_col = c.col(i);
        m_i[i] = N_col.dot(n_col)/c_col.dot(n_col);
        
    }
    //std::cout << "m_i :\n" << m_i << std::endl;
    
    
    MatrixXd F_n(3,6);
    Vector3d m_i_col, F_n_1, F_n_2, F_n_21;
    for(int i=0 ; i<6 ; i++){
        N_col = N.col(i);
        n_col = n.col(i);
        c_col = c.col(i);
        F_n_1 = N_col.cross(n_col);
        F_n_21 = m_i[i]*c_col.array();
        F_n_2= F_n_21.cross(n_col);
        F_n.col(i) = (F_n_1 - F_n_2).array() / l[i];
    }
    
    //r_bar (66)
    Vector3d r_bar = R * Vector3d::Zero();
    
    //x_g (65)
    Vector3d x_g; //w : (4),alp : (5) 
    x_g = x_a + alp.cross(r_bar) + w.cross(w.cross(r_bar));
    
    //sigma term in C
    Vector3d sum_term, sum_F, F_n_col;
    for(int i=0 ; i<6 ; i++){
        p_col = p.col(i);
        F_n_col = F_n.col(i);
        sum_term += (R*p_col).cross(F_n_col);
        sum_F += F_n.col(i);
    }

    //Vector C
    VectorXd C(6); //(69)
    Vector3d C_1, C_2, C_2_11, C_2_1, C_2_2, C_2_3;
    C_1 = mp * G - mp * x_g - sum_F;
    C_2_11 = mp*r_bar.array();
    C_2_1 = (C_2_11).cross(G);
    C_2_2 = mp * (r_bar.cross(x_g)).array();
    C_2_3 = -Ip * alp + (Ip * w).cross(w);
    C_2 = C_2_1 - C_2_2 + C_2_3 - sum_term;

    C.segment(0,3) = C_1;
    C.segment(3,3) = C_2;
    
    //m_1(a_i1-G)*n_i => temp
    VectorXd temp(6), temp_1(3);
    for(int i=0 ; i<6 ; i++){
        ai1_col = ai1.col(i);
        n_col = n.col(i);
        temp_1 = m1*(ai1_col- G);
        temp[i] = temp_1.dot(n_col);
    }
    
    J_I.setZero();
    J_T.setZero();
    MatrixXd J_1_I(6,6), J_1_T(6,6), J_2_I(6,6), Ja(6,6);
    J_1_I = jacov_1_inv();
    // J_2_I = jacov_2_inv();
    J_I = J_1_I;
    Ja = J_I.inverse();
    J_T = Ja.transpose();
    
    
    
    // std::cout << "J_I :\n" << J_I << std::endl;
    J_1_T = (J_1_I.inverse()).transpose();

    VectorXd F(6), T(6); //F_joint(6), T_Cartesian(6);
    F = temp - J_1_T * C ; //joint space
    T = J_I.transpose() * F ; // cartesian space
    std::cout << "T_Cartesian is \n" << T << std::endl;
    return F;
}

void input_callback(const std_msgs::Float32MultiArray &msg){
    for(int i=0 ; i<3 ; i++){
        x[i] = msg.data[i];
    }
    for(int i=3 ; i<6 ; i++){
        x_d[i] = msg.data[i];
    }
    for(int i=6 ; i<9 ; i++){
        x_dd[i] = msg.data[i];
    }

    for(int i=9 ; i<12 ; i++){
        deg_euler[i] = msg.data[i];
    }
    if (deg_euler[1] == 0){
        deg_euler[1] = 0.001;
    }
    for(int i=12; i<15 ; i++){
        deg_euler_d[i] = msg.data[i];
    }
    for(int i=15 ; i<18 ; i++){
        deg_euler_dd[i] = msg.data[i];
    }

    // for(int i=0 ; i<msg.th.size() ; i++){
    //     deg_euler[i] = msg.th[i];
    // }
    // if (deg_euler[1] == 0){
    //     deg_euler[1] = 0.001;
    // }
    // for(int i=0 ; i<msg.th_dot.size() ; i++){
    //     deg_euler_d[i] = msg.th_dot[i];
    // }
    // for(int i=0 ; i<msg.th_ddot.size() ; i++){
    //     deg_euler_dd[i] = msg.th_ddot[i];
    // }
    // for(int i=0 ; i<msg.s.size() ; i++){
    //     x[i] = msg.s[i];
    // }
    // for(int i=0 ; i<msg.s_dot.size() ; i++){
    //     x_d[i] = msg.s_dot[i];
    // }
    // for(int i=0 ; i<msg.s_ddot.size() ; i++){
    //     x_dd[i] = msg.s_ddot[i];
    // }
    
    input_update = true;
    
}

void impedance_callback(const std_msgs::Float32MultiArray &msg_f){
    F_impedance = VectorXd::Zero(6);

    F_impedance[0] = msg_f.data[0];
    F_impedance[1] = msg_f.data[1];
    F_impedance[2] = msg_f.data[2];
    F_impedance[3] = msg_f.data[3];
    F_impedance[4] = msg_f.data[4];
    F_impedance[5] = msg_f.data[5];
    impedance_update = true;
    // std::cout << "F_total is \n" << F_impedance << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "get_inverse_dynamics");
    ros::NodeHandle nh;

    ros::Publisher pub_1 = nh.advertise<hmc_msgs::joint_F>("/to_Joint_Force",1);
    ros::Publisher pub_test = nh.advertise<hmc_msgs::jacobi>("/jocobi",1);

    ros::Subscriber input = nh.subscribe("/Actual", 1, input_callback);
    ros::Subscriber f_im = nh.subscribe("/Cartesian_Force", 1, impedance_callback);

    hmc_msgs::joint_F force;
    hmc_msgs::jacobi jacob;

    ros::Rate loop_rate(100);

    input_update = false;
    impedance_update = true;
    while(ros::ok()){
        if(input_update && impedance_update){ //&& impedance_update
            Initial();
            I_p_K();
            I_r_k();
            I_a_K();
            F_joint = Inv_Dynamics();
            F_total = F_joint + J_T*F_impedance;
            //ROS_INFO("tourque is %f", T[0]);
            std::cout << "F_joint is \n" << F_joint << std::endl;
            std::cout << "F_impedance is \n" << F_impedance << std::endl;
            std::cout << "F_total is \n" << F_total << std::endl;
            std::cout << "Jacobian is \n" << J_T << std::endl;

            jacob.jacobian.clear();
            VectorXd J_I_col(6); 
            for(int i=0 ; i<6 ; i++){
                J_I_col = J_I.col(i);
                for(int j=0 ; j<6 ; j++){
                    jacob.jacobian.push_back(J_I_col[j]);
                }
            }
            // jacob.jacobian1.push_back(J_I.col(0));
            // jacob.jacobian2.push_bach(J_I.col(1));
            // jacob.jacobian3 = J_I.col(2);
            // jacob.jacobian4 = J_I.col(3);
            // jacob.jacobian5 = J_I.col(4);
            // jacob.jacobian6.push_back(J_I.col(5));
            jacob.det = J_I.determinant();

            pub_test.publish(jacob);

            // std::cout << "p : \n" << R*p_col << std::endl;
            force.F_joint.clear();
            force.l_ddot.clear();
            for(int i=0 ; i<6 ; i++){
                force.F_joint.push_back(F_total[i]);
                force.l_ddot.push_back(l_acc[i]);
            }
            pub_1.publish(force);
            
            input_update = false;
            impedance_update = false;
        }
        //ROS_INFO("input updatd : %d", input_update);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}