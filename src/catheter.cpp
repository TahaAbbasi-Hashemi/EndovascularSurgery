#include <catheter.h>

Catheter::Catheter(){
    m_nseg = 0;
    m_nq = 0;
    m_pps = 0;
    m_rad = 0;
    //m_q1change = 0.0005;
    m_q1change = 0.02;
    m_q2change = M_PI/12;
    m_q3change = 5;
}
Catheter::~Catheter()   {}

// Setting Functions
bool Catheter::s_nseg(int nseg){
    int maxSegments = 3;
    if (0 <= nseg < maxSegments){
        m_nseg = nseg;
        return true;
    }else{
        std::cout << "Bad Value for nseg\n";
    }
    return false;
}
bool Catheter::s_nq(int nq){
    int maxQ = 3;
    if (0 <= nq < maxQ){
        m_nq = nq;
        return true;
    }else{
        std::cout << "Bad value for nq\n";
    }
    return true;
}
bool Catheter::s_pps(int pps){
    int maxPPS = 25;
    if (0 <= pps < maxPPS){
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
    int maxRad = 2;
    if (0 <= rad < maxRad){
        m_rad = rad;
        return true;
    }else{
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
    Eigen::Matrix4d newBase;

    for (int i=0; i<m_nseg; i++){
        double bend         = q(0,i);
        double rotation      = q(1,i);
        double translation  = q(2,i); 

        double len = (m_bbLen)*0.75;       // Make this have per segment
        double rad = m_rad;     // switch to tendon radius not cath radius
        double k = -bend/(len*rad);

        length(0,i) = len;
        phi(0,i) = rotation;
        kappa(0,i) = k;
        newBase = m_baseFrame;
        newBase(2,3) = newBase(2,3) + translation;
    }

    m_q = q;
    m_backbone = arc2x(newBase, kappa, length, phi, m_pps);
    m_eeFrame = m_backbone.rightCols(4);
    std::cout << m_eeFrame << std::endl;
}

// Getting Functions
double Catheter::g_rad()                {return m_rad;}
double Catheter::g_q1change()           {return m_q1change;}
double Catheter::g_q2change()           {return m_q2change;}
double Catheter::g_q3change()           {return m_q3change;}

double Catheter::g_distEE(){
    double dist = differance(g_eeFrame(), g_baseFrame());
    return abs(dist);
}

double Catheter::g_qChange(int qkind){
    if (qkind = 0){
        return g_q1change();
    } else if (qkind = 1){
        return g_q2change();
    }else if (qkind = 2){
        return g_q3change();
    }
    return 0; // If bad value
}

Eigen::MatrixXd Catheter::g_q()         {return m_q;}
Eigen::MatrixXd Catheter::g_baseFrame() {return m_baseFrame;}
Eigen::Matrix4d Catheter::g_eeFrame()   {return m_eeFrame;}
Eigen::MatrixXd Catheter::g_backbone()  {return m_backbone; }
