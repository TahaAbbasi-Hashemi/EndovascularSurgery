#include <catheter.h>

Catheter::Catheter(){
    m_nseg = 0;
    m_nq = 0;
    m_pps = 0;
    m_rad = 0;
    m_q1change = 0.0005;
    m_q2change = M_PI/12;
    m_q3change = 0.5;
}
Catheter::~Catheter()   {}

// Setting Functions
bool Catheter::s_nseg(int nseg){
    if (0 <= nseg < 3){
        m_nseg = nseg;
        return true;
    }else{
        std::cout << "Bad Value for nseg\n";
    }
    return false;
}
bool Catheter::s_nq(int nq){
    if (0 <= nq < 3){
        m_nq = nq;
        return true;
    }else{
        std::cout << "Bad value for nq\n";
    }
    return true;
}
bool Catheter::s_pps(int pps){
    if (0 <= pps < 25){
        m_pps = pps;
        return true;
    }else{
        std::cout << "Bad value for pps\n";
    }
    return false;
}
// Do these Later
bool Catheter::s_baseFrame(Eigen::Matrix4d baseFrame){m_baseFrame = baseFrame; return true;}
bool Catheter::s_rad(double rad){
     if (0 <= rad < 0.1){
        m_rad = rad;
        return true;
    }else{
        m_rad = rad;
        std::cout << "Bad value for catheter radius \n";
    }
    return false;
}
bool Catheter::s_bbLen(double bbLen) {    
    m_bbLen = bbLen;
    return false;
}


// Making Functinos
void Catheter::fkine(Eigen::MatrixXd q){
    Eigen::MatrixXd length(1,m_nseg);
    Eigen::MatrixXd phi(1, m_nseg);
    Eigen::MatrixXd kappa(1, m_nseg);

    for (int i=0; i<m_nseg; i++){
        double bend         = q(0,i);
        double rotation      = q(1,i);
        double tranrlation  = q(2,i); 

        double len = (m_bbLen)*0.75;       // Make this have per segment
        double rad = m_rad;     // switch to tendon radius not cath radius
        double k = -bend/(len*rad);

        length(0,i) = len;
        phi(0,i) = rotation;
        kappa(0,i) = k;
    }

    m_q = q;
    m_backbone = arc2x(m_baseFrame, kappa, length, phi, m_pps);
    m_eeFrame = m_backbone.rightCols(4);
}

// Simple Getting Functions
double Catheter::g_rad()                {return m_rad;}
double Catheter::g_q1change()           {return m_q1change;}
double Catheter::g_q2change()           {return m_q2change;}
double Catheter::g_q3change()           {return m_q3change;}
Eigen::MatrixXd Catheter::g_q()         {return m_q;}
Eigen::MatrixXd Catheter::g_baseFrame() {return m_baseFrame;}
Eigen::Matrix4d Catheter::g_eeFrame()   {return m_eeFrame;}
Eigen::MatrixXd Catheter::g_backbone()  {return m_backbone; }
