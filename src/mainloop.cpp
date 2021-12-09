#include <mainloop.h>

MainLoop::MainLoop(Visualizer vis) {
    mp_vis = vis;

    m_cath.s_baseFrame(Eigen::MatrixXd::Identity(4,4));
    m_cath.s_nseg(1);
    m_cath.s_nq(3);         // bend, rot, trans
    m_cath.s_pps(250);
    m_cath.s_bbLen(100);
    m_cath.s_rad(1);

    m_aorta.s_deadD(1);
    m_aorta.s_dangerD(5);
    int catSeg = 1;
    int catQ = 3;
    Eigen::MatrixXd q(catQ,catSeg);
    for (int i=0; i<catSeg; i++){
        q(0,i) = 0;
        q(1,i) = 0;
        q(2,i) = 0;
    }
    m_q = q;
    
    double r = 10;   // 2mm in diameter
    double h = 100;   // 2mm in Height
    double max = 1000;
    double rot = M_PI/64;
    //double max = 240;
    for (int i=0; i<max; i++){
        Eigen::Matrix4d point = Eigen::MatrixXd::Identity(4,4);
        point(0,3) = r*sin(i*rot);
        point(1,3) = r*cos(i*rot);
        point(2,3) = (h/2) + (h/2)*i/max;
                
        m_aorta.s_point(point);
    }
    m_engaged = false;
    m_cathEn = false;
    m_aortaEn = false;
} 

MainLoop::~MainLoop() {}

void MainLoop::Execute(vtkObject *caller, unsigned long eventId, void *vtkNotUsed(callData)) {
    // Event Loop
    
    if (vtkCommand::TimerEvent == eventId) {
        // Drawing Aorta
        if (m_aortaEn){
            //mp_vis.clear();
            //
            // UPDATE THIS TO USE AORTA FUNCTIONS
            mp_vis.drawAorta(m_aorta.g_points(), 1, 5);
            mp_vis.update();
            m_aortaEn = false;
        }
        // Drawing Catheter
        if(m_cathEn){
            m_cath.fkine(m_q);
            mp_vis.drawCath(m_cath.g_backbone(), 0.89/2);
            mp_vis.update();
            m_cathEn = false;
            //m_engaged = true;
        }
        // The simulation
        if(m_engaged){
            // Before doing anything we check to make sure we are in a safe place
            //m_aorta.checkDistance(m_cath.g_eeFrame());

            // Testing to see all possibilties
            std::vector<int> safetyRatings;
            std::vector<double> distanceRatings;
            std::vector<double> goalRatings;
            for (int i=0; i<3; i++){    // i is each degree of freedom
                for (int j=-1; j<2; j++){   // j is each direction we can go.
                    Eigen::Matrix editQ = m_q;
                    editQ(i,0) = m_q(i,0) + j*m_cath.g_qChange(i);

                    m_cath.fkine(editQ);
                    m_aorta.checkDistance(m_cath.g_eeFrame());
                    safetyRatings.push_back(m_aorta.g_safety());
                    distanceRatings.push_back(m_aorta.g_distance());
                    goalRatings.push_back(m_cath.g_distEE());
                    
                    // Reset back to original State
                    m_cath.fkine(m_q); 
                }
            }

            // Go through each of the possibileities and find the best
            for (int i=0; i<3; i++){    // i is each degree of freedom
                int safestMove=-2;      // if the output is -2 there is a big problen
                double safestRate=1000;
                for (int j=-1; j<2; j++){
                    // Due to the way we ran this... 
                    int currentSpot = i*3+(j+1);

                    // If we ran into a dead spot
                    if(safetyRatings.at(currentSpot) == 0){continue;}
                    
                    // this is a bad way to do this... 
                    int c1 = safetyRatings.at(i*3);
                    int c2 = safetyRatings.at(i*3+1);
                    int c3 = safetyRatings.at(i*3+2);
                    bool best = false;
                    if (safetyRatings.at(currentSpot) >= c1){
                        if (safetyRatings.at(currentSpot) >= c2){
                            if (safetyRatings.at(currentSpot) >= c3){
                                best = true;
                            }
                        }
                    }

                    if (best){
                        if(distanceRatings.at(currentSpot) < safestRate){
                            safestRate = distanceRatings.at(currentSpot);
                            safestMove = j;
                        }
                        // If point 0 if just as good stay at it.. 
                        if(distanceRatings.at(currentSpot) == safestRate){
                            if(j==0){
                                safestRate = distanceRatings.at(currentSpot);
                                safestMove = j;
                            }
                        }
                    }
                }
                std::cout << "Safest Move for Q:"<<i<< "-> " << safestMove << std::endl;
            }

            std::cout << "DONE!!" << std::endl;
            m_engaged = false;
        }
    }


    // Key Presses
    else if (vtkCommand::KeyPressEvent == eventId){
        vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);
        if (*(iren->GetKeySym()) == 'a') {
            mp_vis.clear();
            mp_vis.update();
        }
        if (*(iren->GetKeySym()) == 'c') {
            mp_vis.clear();
            mp_vis.update();
        }
        if (*(iren->GetKeySym()) == 'u') {
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'j') {
            m_q(0,0) = m_q(0,0) + m_cath.g_q1change();
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'l') {
            m_q(0,0) = m_q(0,0) - m_cath.g_q1change();
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'm') {
            m_q(1,0) = m_q(1,0) + m_cath.g_q2change();
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'n') {
            m_q(1,0) = m_q(1,0) - m_cath.g_q2change();
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'k') {
            m_q(2,0) = m_q(2,0) + m_cath.g_q3change();
            m_cathEn = true;
        }
        if (*(iren->GetKeySym()) == 'h') {
            m_q(2,0) = m_q(2,0) - m_cath.g_q3change();
            m_cathEn = true;
        }
        // Test scenario interactions
        if (strcmp((iren->GetKeySym()), "Up") == 0) {}
        if (strcmp((iren->GetKeySym()), "Down") == 0) {}
        if (strcmp((iren->GetKeySym()), "Left") == 0) {}
        if (strcmp((iren->GetKeySym()), "Right") == 0) {}
        if (strcmp((iren->GetKeySym()), "Return") == 0) {
            m_aortaEn = true;
        }
    }
    
}
