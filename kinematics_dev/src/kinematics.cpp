#include "kinematics.h"

// class constructor
Kinematics::Kinematics(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1> &q_initial) 
    : w_{w}, l_{l}, h_{h}, r_{r}, q_{q_initial}
    {

    }


// update the transforms
Eigen::Matrix4d * Kinematics::updateTransforms(Eigen::Matrix<double, 10, 1>  q)
{

}

// calculate the jacobian
Eigen::Matrix<double, 12, 10> Kinematics::jacobian()
{

}

// navigation kinematics
Eigen::Matrix<double, 10, 1> Kinematics::navigation()
{

}

// actuation kinematics
Eigen::Matrix<double, 10, 1> Kinematics::actuation()
{

}

// calculate V(q)
Eigen::Matrix<double, 10, 10> Kinematics::spatialToCartesian(Eigen::Matrix<double, 10, 1> q)
{

}

// calculate omega
void Kinematics::omega(const Eigen::Vector3d &orientation, Eigen::Matrix3d &omega){
    /*
    This function takes in an orientation 3vector and calculates the omega matrix
    args:
        orientation - 3vector of orientation values: phi, theta, psi
        omega - reference to output omega matrix
    */
    double phi = orientation(0);
    double theta = orientation(1);
    double psi = orientation(2);
    omega << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
             0,            cos(phi),           -sin(phi),
             0, sin(phi)/cos(theta), cos(phi)/cos(theta);
}

// skew symmetric matrix froma vector
void Kinematics::skew(const Eigen::Vector3d &v, Eigen::Matrix3d & skewSym)
{
    /*
    This function takes in a 3vector and modifies a predefined skew symmetric matrix
    args:
        v - 3vector to convert to skew symmetric
        skewSym - reference to skew symmetric matrix
    */
    skewSym <<   0, -v(2),  v(1),
              v(2),     0, -v(0),
             -v(1),  v(0),     0;
}