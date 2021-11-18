#include <robot_independent.h>
#include <type_traits>

// This function should implement the robot independent mapping (i.e. mapping
// arc parameters in configuration space to a series of discrete frames in task
// space) Inputs:
// init_frame			4x4 Matrix, specifying the initial frame of the
// curve kappa				m-dimensional vector, storing the
// curvature of each segment of the curve l
// m-dimensional vector, storing the length of each segment of the curve phi
// m-dimensional vector, storing the angle of the bending plane of each segment
// of the curve n					number of requested
// frames to be returned for each segment (equally distributed along the
// circular arc) bishop				boolean value, specifying
// whether the material frame of the curve should be maintained or not (meaning
// a constant local rotation around the z-axis)
//
// Output (return):
//
// Eigen::MatrixXd		4x4(m*n+1) dimensional matrix storing all of the
// returned frames along the curve (stacked 4x4 blocks). First, leftmost 4x4
// block should store the initial frame (init_frame).
Eigen::MatrixXd arc_to_x(Eigen::Matrix4d init_frame, std::vector<double> kappa,
                         std::vector<double> length, std::vector<double> phi,
                         int n, bool bishop) {
  // Assumptions
  // n must be 1 or greater

  // Variables
  Eigen::MatrixXd outFrame;
  Eigen::MatrixXd betweenFrame;
  betweenFrame = init_frame;
  outFrame = init_frame; //
  for (int i = 0; i < kappa.size(); i++) {
    for (int j = 0; j < n; j++) {
      double k = kappa.at(i);
      double p = phi.at(i);
      double l = length.at(i) / n;
      if (j >= 1) {
        p = 0;
      }

      Eigen::MatrixXd currentFrame;
      Eigen::Matrix<double, 4, 4> m1;
      Eigen::Matrix<double, 4, 4> m2;
      double r = 1 / k;
      double t = k * l;
      m1 << cos(p), -sin(p), 0, 0, sin(p), cos(p), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

      // Normal use case
      if (k != 0) {
        m2 << cos(t), 0, sin(t), r * (1 - cos(t)), 0, 1, 0, 0, -sin(t), 0,
            cos(t), r * sin(t), 0, 0, 0, 1;
      }
      // Special Case if it is straight
      else {
        m2 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, l, 0, 0, 0, 1;
      }

      if (bishop) // IT IS MAINTAINED AROUND THE Z AXIS
      {
        currentFrame = m1 * m2 * m1.inverse();
      } else {
        currentFrame = m1 * m2;
      }

      betweenFrame = betweenFrame.eval() * currentFrame.eval();
      outFrame.conservativeResize(4, outFrame.cols() + 4);
      outFrame.col(outFrame.cols() - 4) = betweenFrame.col(0);
      outFrame.col(outFrame.cols() - 3) = betweenFrame.col(1);
      outFrame.col(outFrame.cols() - 2) = betweenFrame.col(2);
      outFrame.col(outFrame.cols() - 1) = betweenFrame.col(3);
    }
  }

  return outFrame;
}

// This function should implement mapping from SE(3) (Sepcial Euclidean Lie
// Group) to the corresponding lie algebra se(3) Inputs: T
// 4x4 Matrix, specifying a transformation matrix T = [R p; 0 0 0 1] in SE(3)
// Output (return):
//
// Eigen::Matrix4d		4x4 Matrix, being the product of [S]*theta,
// where [S] is the screw axis in matrix form consisting of an angular (skew
// symmetric) and translational part
//						and theta is the displacement
// along this screw axis
Eigen::Matrix4d matrix_log(Eigen::Matrix4d T) {
  // std::cout << "Function " << T << std::endl;

  // Rotation Position Matrix
  Eigen::Matrix3d rotationMatrix;
  Eigen::MatrixXd position(3, 1);
  rotationMatrix.setZero();
  position.setZero();

  rotationMatrix = T.block<3, 3>(0, 0);
  position = T.block<3, 1>(0, 3);

  // Getting Trace
  //std::cout << "T Function \n" << T << std::endl;
  double trace = rotationMatrix.trace();

  // Creating theta, W, and V
  Eigen::Matrix3d w;
  Eigen::MatrixXd v(3, 1);
  double theta = acos((trace - 1) / 2);

  // Special Case
  double cutoff = 3-(1e-4);
  if (theta <= 1e-4 ) {//trace >= cutoff) {
    //std::cout << "Special Case!! " << std::endl;
    //std::cout << theta << std::endl;
    w.setZero();
    v = position / position.norm();
    theta = position.norm();
  }

  // Normal Case
  else {
    // Calculating Value of G.
    //std::cout << "Normal Case!! " << std::endl;

    Eigen::Matrix3d g;
    g.setZero();
    w.setZero();

    w = (1 / (2 * sin(theta))) * (rotationMatrix - rotationMatrix.transpose());

    Eigen::Matrix3d part1 = (1 / theta) * Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix3d part2 = 0.5*w;
    double part3 = (1.0 / theta) - (0.5*(1/tan(theta / 2)));
    Eigen::Matrix3d part4 = (w * w);

    //std::cout << "Part 1 -> " << part1 << std::endl;
    //std::cout << "Part 2 -> " << part2 << std::endl;
    //std::cout << "Part 3 -> " << part3 << std::endl;
    //std::cout << "Part 4 -> " << part4 << std::endl;
    g = part1 - part2 + (part3*part4);

    // std::cout << "G - > " << g << std::endl;
    // std::cout << "Position Matrix - > " << position << std::endl;
    v = g * position;
  }

  Eigen::Matrix4d matty;
  matty.setZero();
  matty.block(0, 0, 3, 3) << w;
  matty.block(0, 3, 3, 1) << v;
  
  Eigen::Matrix4d matty2;
  matty2.setZero();
  matty2 = matty*theta;
  // std::cout << "Matrix Log -> " << matty << std::endl;;

  return matty2;
}

// This function should calculate and return a desired twist in the body frame
// based on a current body frame and a desired body frame (both frames expressed
// w.r.t. the space frame) Inputs:
// T_cur					4x4 Matrix, specifying the
// current body frame T_sb T_target					4x4
// Matrix, specifying the desired target body frame T_sd Output (return):
//
// Eigen::MatrixXd			6x1 Matrix, expressing the desired body
// frame twist V_b to move from the current body frame to the desired frame
//					.
//.
// 		The first three entries
// should hold the rotational part, while the last thee entries should hold the
// translational part
Eigen::MatrixXd calculate_desired_body_twist(Eigen::Matrix4d T_target,
                                             Eigen::Matrix4d T_cur) {

  int verbose = 2;

  // Determine Tbd
  Eigen::Matrix4d tbd;
  tbd.setZero();
  tbd = T_cur.inverse() * T_target;

  // Matrix Log
  Eigen::Matrix4d logged;
  logged.setZero();
  logged = matrix_log(tbd);

  // W
  double wx = logged(2, 1);
  double wy = logged(0, 2);
  double wz = logged(1, 0);
  // V
  double x = logged(0, 3);
  double y = logged(1, 3);
  double z = logged(2, 3);

  Eigen::MatrixXd body_twist(6, 1);
  body_twist << wx, wy, wz, x, y, z;

  return body_twist;
}


double rmsDiff(Eigen::Matrix4d cur, Eigen::Matrix4d sec){
    double x1 = cur(0,3);
    double y1 = cur(1,3);
    double z1 = cur(2,3);

    double x2 = sec(0,3);
    double y2 = sec(1,3);
    double z2 = sec(2,3);

    double diff = std::sqrt(std::pow((x1-x2),2) + std::pow((y1-y2),2) + std::pow((z1-z2),2));
    return diff;
}
