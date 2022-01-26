#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

// A catheter has the functionality of a TDCR but is drawn as if it is a CTCR
/**
 * The catheter is a TDCR that enters the Aorta to do surgery. 
 *
 * The robot is made of segments and q values. A TDCR of this style has two tendons per segment. 
 * In order to simplify this every two tendons will be used by 1 motor or in other words one 1 q value. 
 * The pps is to define how many points along the backbone do we want for the visualzier system.
 *
 * The qchange values refer to the simulation that can be run with this system. The change is the accepted amount of change for the simulation. 
 */
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

        std::vector<Eigen::Matrix4d> m_body; ///< The Body of the catheter. 

        //Member variables defining the parameters of the TDCR
        Eigen::MatrixXd m_q;
        
        // Matrixes
		Eigen::Matrix4d m_baseFrame;
		Eigen::Matrix4d m_eeFrame;
		Eigen::MatrixXd m_backbone;

        Eigen::MatrixXd arc2x(Eigen::Matrix4d base, Eigen::MatrixXd kappa, Eigen::MatrixXd length, Eigen::MatrixXd phi, int nSeg);
        double mg_distance(Eigen::Matrix4d point1, Eigen::Matrix4d point2);

	public:
        Catheter();     ///< Constructor
        ~Catheter();    ///< Deconstructor
        
        // Setting Functions
        /**
         *  Sets the base frame from where everything is done from.
         *
         *  @param[in] baseFrame
         *      A 4x4 matrix of doubles holding the orientation and position of the base.
         *  @param[out] Boolean
         *
         *      True if the baseFrame is accepted.
         *      False if it is not.
         */
        bool s_baseFrame(Eigen::Matrix4d baseFrame);
        /**
         *  Sets the number of segments. 
         *
         *  @param[in] nsegs
         *  @param[out] Boolean
         *      
         *      True if the number of segments is accepted
         *      False if not. 
         */
        bool s_nseg(int nseg);
        /**
         * Sets the number of Q values for the robot.
         *
         * @param[in] nq the number of Q values of the whole robot. 
         * @param[out] Boolean
         *  
         *      True if accepetd.
         *      False otherwise.
         */
        bool s_nq(int nq);
        /**
         *  Sets the parts per segment. This is for visualization
         *
         *  @param[in] PPS parts per segment. 
         *  @param[out] Boolean
         *  
         *      True if accepetd.
         *      False otherwise.
         */
        bool s_pps(int pps);
        /**
         *  Sets the radius of the TDCR for visualization
         *
         *  @param[in] rad  the radius of the catheter
         *  @param[out] Boolean
         *
         *      True if accepetd.
         *      False otherwise.
         */
        bool s_rad(double rad);
        /**
         *  Sets the backbone length
         *
         *  @param[in] bblen the length of the backbone
         *  @param[out] Boolean
         *
         *      True if accepetd.
         *      False otherwise.
         */
        bool s_bbLen(double bbLen);

        
        // Doing Functions
        /**
         * Given Q values it runs forward kinematics
         *
         * Using constant curvature model to convert q values to arc values of kappa, phi, length; then into position frame. 
         *
         * @param q a Eigen matrix of doubles the size of (3xnSeg).
         *  
         *      The first row is the bending
         *      The second row is rotation
         *      The third row is translation.
         *      Each column is the Q values for a segment.
         */
        void fkine(Eigen::MatrixXd q);

        
        // Getting Functions

        /**
         *  Returns the distande of the original baseframe to the end effector location after running fkine()
         *  @param[out] double 
         *      
         *      -1 if fkine() has not yet been run
         *      >0 otherwise
         */
        double g_distEE(); 
        /**
         *  Returns the radius of the catheter for visuaziation
         *
         *  @param[out] double
         *      
         *      If the radius has not yet been set then this will return the default otherwise it will return the value set.
         */
        double g_rad();
        double g_q1change(); ///< returns the change of q1
        double g_q2change(); ///< returns the change of q2
        double g_q3change(); ///< return the change of q3
        /**
         *  This function takes returns the q change value based on the q kind. 
         *  
         *  qkind would be an int between 1-3 refering to bending, rotation, bending. 
         *
         *  @param[in] qkind    
         *      The q value we want the change of. 
         *  @param[out] double 
         *      The change of the selected value.
         */
        double g_qChange(int qkind);
        Eigen::MatrixXd g_q();      ///< returns the most recent Q values used in a simulatino
        Eigen::MatrixXd g_baseFrame();  ///< returns the base frame used in the most recent simulation
        Eigen::Matrix4d g_eeFrame();    //< returns the end effector frame from the most recent simulation
        Eigen::MatrixXd g_backbone();  ///< returns the points across the backbone from the most recent simulation.

};
