#define _USE_MATH_DEFINES
#pragma once

#include "robot_independent.h"

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

// Class that implements the kinematic model of a tendon driven continuum robot
class TDCR
{  
	private:
        int m_nseg;
        int m_nq;
        int m_nqps;
        int m_ndisks;
		
        //Member variables defining the parameters of the TDCR
        Eigen::MatrixXd m_bbLen;
        Eigen::MatrixXd m_tDir;
        Eigen::MatrixXd m_q;

        // Matrixes
		Eigen::Matrix4d m_baseFrame;
		Eigen::Matrix4d m_eeFrame;
		Eigen::MatrixXd m_diskFrames;
		
	public:
		TDCR(int ndisks, Eigen::MatrixXd bbLen, Eigen::MatrixXd tDir, Eigen::Matrix4d baseFrame);
        ~TDCR();

        bool s_newQ(Eigen::MatrixXd q);
        Eigen::MatrixXd g_fkine(Eigen::MatrixXd q);
        //Eigen::MatrixXd g_Jacobian(Eigen::MatrixXd q);
        
        Eigen::MatrixXd g_q();
        Eigen::MatrixXd g_baseFrame();
        Eigen::Matrix4d g_eeFrame();
        Eigen::MatrixXd g_diskFrames();
};
