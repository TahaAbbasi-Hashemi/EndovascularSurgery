#include <aorta.h>

Aorta::Aorta(){
    m_deadD = 0;
    m_dangerD = 0;
    m_safeD = 0;
    clear();
    m_safety = 0;
    m_maxD = 0;
}
Aorta::~Aorta(){}

// Setting Functions
bool Aorta::s_point(Eigen::Matrix4d point){
    m_points.conservativeResize(4, m_points.cols()+4);
    m_points.col(m_points.cols() - 4) = point.col(0);
    m_points.col(m_points.cols() - 3) = point.col(1);
    m_points.col(m_points.cols() - 2) = point.col(2);
    m_points.col(m_points.cols() - 1) = point.col(3);
    return true;
}

bool Aorta::s_points(Eigen::MatrixXd points){
    for (int i=0; i<(points.cols()/4); i++){
        m_points.conservativeResize(4, m_points.cols()+4);
        m_points.col(m_points.cols() - 4) = points.col(i*4+0);
        m_points.col(m_points.cols() - 3) = points.col(i*4+1);
        m_points.col(m_points.cols() - 2) = points.col(i*4+2);
        m_points.col(m_points.cols() - 1) = points.col(i*4+3);
    }
    return true;
}

bool Aorta::s_deadD(double deadD){
    if (0 <= deadD <= 1){
        m_deadD = deadD;
        return true;
    }else{
        std::cout << "Bad value for deadD with a value of " << deadD << std::endl;
    }
    return false;
}
bool Aorta::s_dangerD(double dangerD){
    if (m_deadD <= dangerD <= 1){
        m_dangerD = dangerD;
        return true;
    }else{
        std::cout << "Bad value for dangerD with a value of " << dangerD << std::endl;
    }
    return false;
}
bool Aorta::s_safeD(double safeD){
    return false;
}

// Doing Functions
void Aorta::clear(){
    Eigen::Matrix4d dead = Eigen::MatrixXd::Identity(4,4);
    dead(0,3) = -10;
    dead(1,3) = 0;
    dead(2,3) = 0;
    m_points = dead;
}
void Aorta::checkDistance(Eigen::Matrix4d ee){
    // Reset parameters
    m_maxD = 10000;
    m_safety = 1;

    // Find new parameters
    for (int i=0; i<m_points.cols()/4; i++){
        Eigen::Matrix4d currentFrame = m_points.block(0,4*i,4,4);
        double dist = abs(differance(ee, currentFrame));

        // Create the new minium value.
        if (dist < m_maxD){ m_maxD = dist; }
        // In this case we are at the worst position posible no need to continue.
        if (m_safety == -1){ break; }
        if(dist <= m_deadD){
            m_safety = -1;
        }else if (dist <= m_dangerD){
            if(m_safety == 1){ m_safety = 0; }
        }
    }
}


// Getting Functions
Eigen::MatrixXd Aorta::g_points(){return m_points;}
int Aorta::g_safety(){return m_safety;}
double Aorta::g_maxDist(){return m_maxD;}







