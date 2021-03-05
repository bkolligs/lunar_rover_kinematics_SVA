#pragma once
#include <eigen3/Eigen/Dense>
#include <math.h>

class Kinematics
{
    private:
        // lateral width from frame
        const double w_=0.1;
        // longitudinal distance from base frame
        const double l_=0.1;
        // vertical distance from base frame
        const double h_=0.1;
        // wheel radius
        const double r_=0.05;
        // number of wheels will always be 4
        const int nWheels_ = 4;

        // current vehicle state
        Eigen::Matrix<double, 10, 1> & q_;
        // current vehicle velocity
        Eigen::Matrix<double, 10, 1> q_dot_;
        // wheel contact point constraints
        Eigen::Matrix<double, 12, 1> contactConstraints_ = Eigen::Matrix<double, 12, 1>::Zero();
        // jacobian matrix
        Eigen::Matrix<double, 12, 10> jacobianMatrix_;
        // list of transformation matrices
        Eigen::Matrix4d * transformList;

    public:
        Kinematics(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1> &q_initial);
        // Kinematics(double w, double l, double h, double r);

        // update the transforms
        void updateTransforms();

        // calculate the jacobian
        void jacobian();

        // navigation kinematics
        void navigation();

        // actuation kinematics
        void actuation();

        // theta is joint angle, pos is translation from parent frame
        Eigen::Matrix4d homogenousTransform(const Eigen::Vector3d &ori, const Eigen::Vector3d &pos);

        // calculate V(q)
        Eigen::Matrix<double, 10, 10> spatialToCartesian(const Eigen::Matrix<double, 10, 1> &q);

        // calculate omega
        void omega(const Eigen::Vector3d &orientation, Eigen::Matrix3d &omega);

        // skew symmetric matrix froma vector
        void skew(const Eigen::Vector3d &v, Eigen::Matrix3d &skewSym);

};



