#pragma once

//stl
#include <cmath>
#include <iostream>
#include <ctime>

//Eigen
#include <Eigen/Dense>

/**
 * The Aorta class contains a point cloud of the current Aorta Wall and can determine distance from a point to the point cloud. 
 * 
 * Dead means the Catheter/Object has reached a distance to the Aorta wall(point cloud) that can kill the patient. 
 *
 * Danger means the Catheter/Object is apporaching the Aorta Wall (point cloud)
 *
 * Safety classes are -1, 0, 1, 2
 *
 * -1 means something is misconfigured
 * 0 means DEAD
 * 1 means DANGER
 * 2 means SAFE
 */
class Aorta{
    private:
        double m_deadD; ///< This sets the minimum distance an object can get to the Aorta.
        double m_dangerD; ///< This sets the distance at which an object is considered close
        
        // For Checking
        double m_distance; ///< The smallest distance of the object to the vascular wall from the previous simulation
        int m_safety; ///< The safety rating of the previous simulation

        // For Drawing
        Eigen::MatrixXd m_points; ///< The points in the point cloud. 

        /**
         * Returns the distance between two points
         *
         * @param[in] point1
         * @param[in] point2
         * @param[out] double
         */
        double mg_distance(Eigen::Matrix4d point1, Eigen::Matrix4d point2);

    public:
        // Construct and Deconstructor
        Aorta();
        ~Aorta();

        // Setting Functions
        /**
         * Adds a single point to the point cloud of the Aorta
         *
         * @param[in] point
         * @param[out] True/False 
         *
         *      True if the point does not already exist. 
         *      False if the point does already exist.
         */
        bool s_point(Eigen::Matrix4d point);
        /**
         * Adds a group of points to the point cloud of the Aorta
         *
         * @param[in] points
         * @param[out] True/False
         *
         *      True if all the points did not yet exist. 
         *      False if any of the points did already exist.
         */
        bool s_points(Eigen::MatrixXd points);
        /**
         * Sets the dead distance for the Aorta. The minimum distance something can come to the aorta.
         *
         * @param[in] deadD
         * @param[out] True/False
         *
         *      True if the distance is within the predefined acceptable range.
         *      False if it falls outside the predefined acceptable range.
         */
        bool s_deadD(double deadD);
        /**
         * Sets the danger distance for the Aorta. A distance where an object is considered as unsafe or too close. 
         * @param[in] deadD
         * @param[out] True/False
         *
         *      True if the distance is within the predefined acceptable range.
         *      False if it falls outside the predefined acceptable range.
         */
        bool s_dangerD(double dangerD);

        // Doing Functions
        void clear();
        /** 
         * Runs a simulation to determine the safety of a given point and its closest distance to the Aortaic wall
         *
         * @param[in] Eigen::Matrix4d ee
         *
         *      The end effector point that is being tested against.
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
        double g_distance();
        /**
         * Returns the dead distance 
         *
         * @param[out] double
         *
         *      Returns -1 if s_deadD() has not yet returned true. 
         *      Returns deadD otherwise
         */
        double g_deadD();
        /**
         * Returns the danger distance
         *
         * @param[out] double
         *      
         *      Returns -1 if s_dangerD() has not yet been called
         *      Returns dangerD otherwise
         */
        double g_dangerD();
        /**
         * Returns all points set by s_point() and s_points()
         *
         * @param[out] Eigen::MatrixXd
         */
        Eigen::MatrixXd g_points();
};








