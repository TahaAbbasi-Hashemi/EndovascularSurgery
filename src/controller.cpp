#include <controller.h>

Controller::Controller(Aorta* aorta, Catheter* catheter){
    mp_aorta = aorta;
    mp_catheter = catheter;

    m_nA = 0;
    m_nS = 0;
    engaged = 0;
}

bool Controller::s_nA(int na){
    int maxA = 27;
    if(0<na<maxA){
        m_nA = 27;
        std::cout << "Bad na valuef for Controller\n";
        return false;
    }
    m_nA = na;
    return true;
}

bool Controller::s_nS(int ns){
    int maxS = 3;
    if(0<ns<maxS){
        m_nS = 3;
        std::cout << "Bad NS for Controller\n";
        return false;
    }
    m_nS = ns;
    return true;
}
