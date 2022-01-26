#pragma once

// stl
#include <cmath>
#include <ctime>
#include <iostream>
#include <random>
#include <vector>
//.h
#include <stdlib.h>

// Eigen
#include <Eigen/Dense>

#include <controller.h>

class Qlearning: public Controller{
    private:
        int m_iterations;
        int m_curState;
        int m_maxState;

        double mg_reward(Eigen::MatrixXd q);
        int mg_randomAction(Eigen::MatrixXd policy);
    public:
        Qlearning(Aorta* aorta, Catheter* catheter):Controller(aorta, catheter){};
        ~Qlearning(){};
        //int g_randAction();// Private
        void d_Q();
};
