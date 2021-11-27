#define _USE_MATH_DEFINES
#pragma once

#include "robot.h"

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

// A catheter has the functionality of a TDCR but is drawn as if it is a CTCR
class Catheter
{  
	private:
        int m_nseg;
        int m_nq;
        int m_nqps;
        int m_pps;
        double m_rad;
        double m_bbLen;

		
        double m_q1change;
        double m_q2change;
        double m_q3change;

        //Member variables defining the parameters of the TDCR
        Eigen::MatrixXd m_q;
        
        // Matrixes
		Eigen::Matrix4d m_baseFrame;
		Eigen::Matrix4d m_eeFrame;
		Eigen::MatrixXd m_backbone;
		
	public:
        Catheter();
        ~Catheter();
        
        // Setting Functions
        bool s_baseFrame(Eigen::Matrix4d baseFrame);
        bool s_nseg(int nseg);
        bool s_nq(int nq);
        bool s_pps(int pps);
        bool s_rad(double rad);
        bool s_bbLen(double bbLen);

        
        // Doing Functions
        void fkine(Eigen::MatrixXd q);
        
        // Getting Functions
        double g_rad();
        double g_q1change();
        double g_q2change();
        double g_q3change();
        Eigen::MatrixXd g_q();
        Eigen::MatrixXd g_baseFrame();
        Eigen::Matrix4d g_eeFrame();
        Eigen::MatrixXd g_backbone();
};
