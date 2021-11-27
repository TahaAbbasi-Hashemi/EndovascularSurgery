#include <robot.h>
#include <type_traits>


Eigen::MatrixXd arc2x(Eigen::Matrix4d baseFrame, Eigen::MatrixXd kappa, Eigen::MatrixXd length, Eigen::MatrixXd phi, int pps){
    
    Eigen::MatrixXd disks = baseFrame;
    
    int nseg = kappa.cols();
    for (int i=0; i<nseg; i++){
            double k = kappa(0, i);
            double p = phi(0, i);
            //baseFrame = baseFrame.eval()*m1.eval():

        for (int j=0; j<pps; j++){
            if(j>0){ p =0;}
            double l = length(0,i)/pps;

            double r = 1/k;
            double t = k*l;
            Eigen::Matrix4d m1;
            m1 << cos(p), -sin(p), 0, 0, sin(p), cos(p), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            

            Eigen::Matrix4d m2;
            if (k == 0){
                m2 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, l, 0, 0, 0, 1;
            }
            else{
                m2 <<cos(t), 0, sin(t), r * (1 - cos(t)), 0, 1, 0, 0, -sin(t), 0, cos(t), r * sin(t), 0, 0, 0, 1;
            }

            Eigen::Matrix4d pointFrame = m1*m2;
            //Eigen::Matrix4d bish = pointFrame*m1.inverse();
            Eigen::Matrix4d eeFrame = baseFrame * pointFrame;

            disks.conservativeResize(4, disks.cols() + 4);
            disks.col(disks.cols() - 4) = eeFrame.col(0);
            disks.col(disks.cols() - 3) = eeFrame.col(1);
            disks.col(disks.cols() - 2) = eeFrame.col(2);
            disks.col(disks.cols() - 1) = eeFrame.col(3);
            baseFrame = disks.rightCols(4);
        }
    }
    return disks;
}

// p1 is ee, p2 is current
double differance(Eigen::Matrix4d p1, Eigen::Matrix4d p2){
    Eigen::MatrixXd pos1 = p1.block<3, 1>(0, 3);
    Eigen::MatrixXd pos2 = p2.block<3, 1>(0, 3);
    Eigen::MatrixXd dist = pos2 - pos1;
    
    double distNorm = dist.norm();
    return distNorm;
}
