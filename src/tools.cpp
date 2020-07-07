#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || ground_truth.size() == 0 )
  {
      std::cout << " Vectors should have the same length" << std::endl;
      return rmse;
  }
  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
    
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero

  // compute the Jacobian matrix
  // prepare reusable constants
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c2*c1;

  if (fabs(c1)<0.0001)
  {
      //std::cout << "[Error]: Can't divide by zero" << std::endl;
      px = py = 0.0001;
      c1 = px*px + py*py;
      c2 = sqrt(c1);
      c3 = c2*c1;
      Hj <<  px/c2  , py/c2 , 0 , 0,
      -py/c1 , px/c1 , 0 , 0, 
      py*(vx*py - vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2  , py/c2 ;
      return Hj;
  }

  Hj <<  px/c2  , py/c2 , 0 , 0,
        -py/c1 , px/c1 , 0 , 0, 
         py*(vx*py - vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2  , py/c2 ; 
 
  return Hj;
}

float Tools::Angle_Norm(float angle)
{
float pi = M_PI;
while (angle > pi)
    angle -= 2 * pi;
   // cout << "- "<<angle << endl;
while (angle < -pi)
    angle += 2 * pi;
    //cout << "+ "<< angle << endl;
return angle;
}