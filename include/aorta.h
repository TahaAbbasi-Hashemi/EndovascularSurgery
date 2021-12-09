/**
 * @file arota.h
 * TODO
 */

// Include 
#include <robot.h>

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

class Aorta{
    private:
        double m_deadD; ///< idk something dois
        double m_dangerD;
        double m_safeD;
        
        // For Checking
        double m_maxD;
        int m_safety;

        // For Drawing
        Eigen::MatrixXd m_points;

    public:
        // Initalize
        Aorta();
        ~Aorta();

        // Setting Functions
        /**
         * Adds a single point to the point cloud of the Aorta
         * @param[in] point
         * @param[out] True/False 
         *
         *      True if the point does not already exist. 
         *      False if the point does already exist.
         */
        bool s_point(Eigen::Matrix4d point);
        /**
         * Adds a group of points to the point cloud of the Aorta
         * @param[in] points
         * @param[out] True/False
         *
         *      True if all the points did not yet exist. 
         *      False if any of the points did already exist.
         */
        bool s_points(Eigen::MatrixXd points);
        /**
         * Sets the dead distance for the Aorta. The minimum distance something can come to the aorta.
         * @param[in] deadD
         * @param[out] True/False
         *
         *      True if the distance is within the predefined acceptable range.
         *      False if it falls outside the predefined acceptable range.
         */
        bool s_deadD(double deadD);
        bool s_dangerD(double dangerD);
        bool s_safeD(double safeD);

        // Doing Functions
        void clear();
        /** does somthegn
         */
        void checkDistance(Eigen::Matrix4d ee);

        // Getting Functions
        /**
         * Returns the calculated safety value of the current simulation. 
         * 
         * Calculates a safety value from a single point and the every part of the point cloud. The safety value is based on the distance to the cloud
         * @param[out] int; ranging from -1 to 2
         *      
         *      -1 if checkDistance() has not yet been run and s_deadD() and s_dangerD() have both not yet returned ture
         *      0 if the distance is less than the Dead Distance
         *      1 if the distance is greater than Dead Distance but less than Danger Distance
         *      2 if the distance is greater than Danger Distance
         */
        int g_safety();
        double g_maxDist();
        /**
         * Returns the dead distance 
         *
         * @param[out]
         *
         *      Returns -1 if s_deadD() has not yet returned true. 
         *      Returns deadD otherwise
         */
        double g_deadD();
        /**
         * Returns the danger distance
         *
         * @param[out]
         *      
         *      Returns -1 if s_dangerD() has not yet been called
         *      Returns dangerD otherwise
         */
        double g_dangerD();
        /**
         * Returns all points set by s_point() and s_points()
         */
        Eigen::MatrixXd g_points();
};








