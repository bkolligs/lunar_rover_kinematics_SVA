#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <iostream>
#include <array>
#include <math.h>

// kinematic direction
enum KinematicDirection {NAVIGATION, ACTUATION};

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
        // rover translation table
        Eigen::Matrix<double, 4, 3> configurationTable;

        // current vehicle state [phi, theta, psi, x, y, z, fl, fr, bl, br]
        Eigen::Matrix<double, 10, 1> q_;
        // current pose orientation from world frame, position in cartesian
        Eigen::Vector3d curWorldOri;
        Eigen::Vector3d curWorldPos;
        // current vehicle velocity
        Eigen::Matrix<double, 10, 1> q_dot_;
        // wheel contact point constraints
        Eigen::Matrix<double, 12, 1> contactConstraints_ = Eigen::Matrix<double, 12, 1>::Zero();
        // jacobian matrix
        Eigen::Matrix<double, 12, 10> jacobianMatrix_ = Eigen::Matrix<double, 12, 10>::Zero();
        // array of transformation matrices
        std::array<Eigen::Matrix4d, 9> transformList;

    public:
        Kinematics(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1> q_initial);
        // Kinematics(double w, double l, double h, double r);

        // update the transforms
        void updateTransforms();

        // calculate the jacobian
        void jacobian();

        // perform the math to predict motion
        void motionPrediction(Eigen::Matrix<double, 10, 1> &q_dot, float deltaT, KinematicDirection direction);

        Eigen::Matrix4d homogenousTransform(const Eigen::Vector3d &ori, const Eigen::Vector3d &pos);

        // calculate V(q)
        Eigen::Matrix<double, 10, 10> spatialToCartesian();

        // for use in V(q)
        void calculateOmega(const Eigen::Vector3d &orientation, Eigen::Matrix3d &omega);

        // included with class so there are less external dependencies
        Eigen::Matrix3d skew(const Eigen::Vector3d &v);

        Eigen::Matrix<double, 10, 1> & getState();

        std::array<Eigen::Matrix4d, 9> getTransforms();

        Eigen::Matrix<double, 12, 10> getJacobian();
};



