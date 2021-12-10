#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

// A catheter has the functionality of a TDCR but is drawn as if it is a CTCR
class Catheter {  
	private:
        int m_nseg;     ///< number of segments
        int m_nq;       ///< number of q values overall
        int m_nqps;     ///< number of q values per segment
        int m_pps;      ///< this is to determine how many points in a segment to be drawn
        double m_rad;
        double m_tendonRad; ///< The radius of the tendons for simulation
        double m_bbLen; ///< The length of the catheter
        double m_bbRad; ///< The radius of the catheter

        double m_q1change;
        double m_q2change;
        double m_q3change;

        //Member variables defining the parameters of the TDCR
        Eigen::MatrixXd m_q;
        
        // Matrixes
		Eigen::Matrix4d m_baseFrame;
		Eigen::Matrix4d m_eeFrame;
		Eigen::MatrixXd m_backbone;

        Eigen::MatrixXd arc2x(Eigen::Matrix4d base, Eigen::MatrixXd kappa, Eigen::MatrixXd length, Eigen::MatrixXd phi, int nSeg);
        double mg_distance(Eigen::Matrix4d point1, Eigen::Matrix4d point2);

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
        double g_distEE();
        double g_rad();
        double g_q1change();
        double g_q2change();
        double g_q3change();
        double g_qChange(int qkind);
        Eigen::MatrixXd g_q();
        Eigen::MatrixXd g_baseFrame();
        Eigen::Matrix4d g_eeFrame();
        Eigen::MatrixXd g_backbone();

};
