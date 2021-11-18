#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>

//Eigen
#include <Eigen/Dense>


// This function should implement the robot independent mapping (i.e. mapping arc parameters in configuration space to a series of discrete frames in task space)
// Inputs:
// init_frame			4x4 Matrix, specifying the initial frame of the curve
// kappa				m-dimensional vector, storing the curvature of each segment of the curve
// l					m-dimensional vector, storing the length of each segment of the curve
// phi					m-dimensional vector, storing the angle of the bending plane of each segment of the curve
// n					number of requested frames to be returned for each segment (equally distributed along the circular arc)
// bishop				boolean value, specifying whether the material frame of the curve should be maintained or not (meaning a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd		4x4(m*n+1) dimensional matrix storing all of the returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4 block should store the initial frame (init_frame).
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa, std::vector<double> l, std::vector<double> phi, int n, bool bishop);

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie Group) to the corresponding lie algebra se(3)
// Inputs:
// T					4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta, where [S] is the screw axis in matrix form consisting of an angular (skew symmetric) and translational part
//						and theta is the displacement along this screw axis
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T);


// This function should calculate and return a desired twist in the body frame based on a current body frame and a desired body frame (both frames expressed w.r.t. the space frame)
// Inputs:
// T_cur					4x4 Matrix, specifying the current body frame T_sb
// T_target					4x4 Matrix, specifying the desired target body frame T_sd
// Output (return):
//
// Eigen::MatrixXd			6x1 Matrix, expressing the desired body frame twist V_b to move from the current body frame to the desired frame
//							The first three entries should hold the rotational part, while the last thee entries should hold the translational part
Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target, Eigen::Matrix4d T_cur);

// Provides the root mean square difference in two different points
double rmsDiff(Eigen::Matrix4d cur, Eigen::Matrix4d sec);
