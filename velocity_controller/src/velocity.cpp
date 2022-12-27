#include "velocity.h"
#include <iostream>
#include <math.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include <eigen3/Eigen/Dense>


void matrix() 
{
// W1
 fl(0,0) = x_w_1;
 fl(0,1) = y_w_1;

// W2
 rl(0,0) = x_w_2;
 rl(0,1) = y_w_2;

// W3
 rr(0,0) = x_w_3;
 rr(0,1) = y_w_3;

// W4
 fr(0,0) = x_w_4;
 fr(0,1) = y_w_4;

 Pos << Vx, Vy, rot;

P << 1,  0, -b,
     0,  1,  a,
     1,  0, -b,
     0,  1, -a,
     1,  0,  b,
     0,  1, -a,
     1,  0,  b,
     0,  1,  a;    
   
X << c1,    0,    0,    0,
     s1,    0,    0,    0, 
      0,    c2,   0,    0,
      0,    s2,   0,    0,
      0,    0,   c3,    0,
      0,    0,   s3,    0,
      0,    0,    0,   c4,
      0,    0,    0,   s4;  

} ;




