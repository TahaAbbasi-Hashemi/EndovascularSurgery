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
        double m_deadD;
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
        bool s_point(Eigen::Matrix4d point);
        bool s_points(Eigen::MatrixXd points);
        bool s_deadD(double deadD);
        bool s_dangerD(double dangerD);
        bool s_safeD(double safeD);

        // Doing Functions
        void clear();
        void checkDistance(Eigen::Matrix4d ee);

        // Getting Functions
        int g_safety(); // -1 for dead, 0 for danger, 1 for safe.
        double g_maxDist();
        Eigen::MatrixXd g_points();
};









