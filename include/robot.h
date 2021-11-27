#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

Eigen::MatrixXd arc2x(Eigen::Matrix4d baseFrame, Eigen::MatrixXd kappa, Eigen::MatrixXd length, Eigen::MatrixXd phi, int n);

// Returns the distance between two different points in space. 
double differance(Eigen::Matrix4d p1, Eigen::Matrix4d p2);


//Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop);

//Eigen::Matrix4d matrix_log(Eigen::Matrix4d T);


//Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target, Eigen::Matrix4d T_cur);

