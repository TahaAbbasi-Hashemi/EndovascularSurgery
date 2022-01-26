#pragma once

//stl
#include <iostream>
#include <ctime>
#include <random>

// Eigen
#include <Eigen/Dense>

#include <aorta.h>
#include <catheter.h>

class Controller{
    protected:
        Aorta* mp_aorta;
        Catheter* mp_catheter;

        int m_nA;
        int m_nS;
        int m_dof;
        bool engaged;

    public:
        Controller(Aorta* aorta, Catheter* catheter);
        ~Controller();

        /**
         *  Sets the number of actions possible
         */
        bool s_nA(int na);
        /**
         *  Sets the number of states
         */
        bool s_nS(int ns);
};
