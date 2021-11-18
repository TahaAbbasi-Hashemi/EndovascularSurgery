#include <tdcr_model_DVS.h>


TDCR::TDCR(int nseg, int nq; int ndisks, Eigen::MatrixXd bbLen, Eigen::MatrixXd tDir){
    m_nseg = nseg;
    m_nq = nq;
    m_nqps = int(nq/nseg);
    m_ndisks = ndisks;
    m_bbLen = bbLen;
    m_tDir = tDir;
}
TDCR::~TDCR() {}

// Setting Functions
void TDCR::s_baseFrame(Eigen::Matrix4d baseFrame){m_baseFrame = baseFrame;}

// Making Functinos
Eigen::Matrix4d TDCR::fkine(Eigen::MatrixXd q){
   return Eigen::MatrixXd::Identity(4, 4);
}

// Simple Getting Functions
Eigen::MatrixXd TDCR::g_q()         {return m_q;}
Eigen::MatrixXd TDCR::g_baseFrame() {return m_baseFrame;}
Eigen::Matrix4d TDCR::g_eeFrame()   {return m_eeFrame;}
Eigen::MatrixXd TDCR::g_diskFrames(){return m_diskFrames;}
