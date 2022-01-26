#include <catheter.h>

Catheter::Catheter(){
    m_nseg = 0;
    m_nq = 0;
    m_pps = 0;
    m_rad = 0;
 
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
    Eigen::MatrixXd newBB;// = m_baseFrame;
    Eigen::Matrix4d newBase = m_baseFrame;

    // For each segment
    for (int i=0; i<m_nseg; i++){
        double bend         = q(0,i);
        double rotation      = q(1,i);
        double translation  = q(2,i); 

        double length = m_bbLen;       // Make this have per segment
        double kappa = -bend/(length*m_rad);    // Make this per segment

        // Translation is added all at once.
        newBase(2,3) = newBase(2,3) + translation;

        // Making backbone for first point
        if (i==0){ newBB = newBase; }

        // For each pps
        // The rotation and bending is added over segment
        for (int j=0; j<m_pps; j++){
            // Adding Translation
            double lpps = length/m_pps;
            double kpps = kappa;
            double p;// Phi
            if(j>0){ p = 0; }
            else{ p = rotation; }

            double r = 1/kpps;      // The radius
            double t = kpps*lpps;

            // Adding Rotaion
            Eigen::Matrix4d m1;
            m1 << cos(p), -sin(p), 0, 0,
               sin(p), cos(p), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;


            // Adding Bending
            Eigen::Matrix4d m2;
            // If the catheter is straight
            if (kpps == 0){ 
                m2 << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, lpps,
                   0, 0, 0, 1; }
            // Otherwise
            else{ 
                m2 <<cos(t), 0, sin(t), r * (1 - cos(t)), 
                   0, 1, 0, 0,
                   -sin(t), 0, cos(t), r * sin(t),
                   0, 0, 0, 1;}

            Eigen::Matrix4d newPoint = newBase*(m1*m2);//pointFrame;

            newBB.conservativeResize(4, newBB.cols() + 4);
            newBB.col(newBB.cols() - 4) = newPoint.col(0);
            newBB.col(newBB.cols() - 3) = newPoint.col(1);
            newBB.col(newBB.cols() - 2) = newPoint.col(2);
            newBB.col(newBB.cols() - 1) = newPoint.col(3);
            newBase = newBB.rightCols(4);
        }
    }

    m_q = q;
    m_backbone = newBB;
    m_eeFrame = newBase;
}


// Getting Functions
double Catheter::g_rad()                {return m_rad;}
double Catheter::g_q1change()           {return m_q1change;}
double Catheter::g_q2change()           {return m_q2change;}
double Catheter::g_q3change()           {return m_q3change;}

double Catheter::mg_distance(Eigen::Matrix4d point1, Eigen::Matrix4d point2){
    Eigen::MatrixXd pos1 = point1.block<3, 1>(0, 3);
    Eigen::MatrixXd pos2 = point2.block<3, 1>(0, 3);
    Eigen::MatrixXd dist = pos2 - pos1;

    double distNorm = dist.norm();
    return distNorm;
}
double Catheter::g_distEE(){
    double dist = mg_distance(g_eeFrame(), g_baseFrame());
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
