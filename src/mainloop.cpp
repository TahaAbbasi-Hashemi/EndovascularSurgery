#include <mainloop.h>

MainLoop::MainLoop(Visualizer vis) {
    mp_vis = vis;

    m_cath.s_baseFrame(Eigen::MatrixXd::Identity(4,4));
    m_cath.s_nseg(1);       // 1 segment
    m_cath.s_nq(3);         // bend, rot, trans
    m_cath.s_pps(250);       // One block each 5mm
    m_cath.s_bbLen(100);         // 1 mm
    m_cath.s_rad(0.89/8);       // 0.89mm

    m_aorta.s_deadD(0.25);
    m_aorta.s_dangerD(0.45);
    m_aorta.s_safeD(0.5);
    int catSeg = 1;
    int catQ = 3;
    Eigen::MatrixXd q(catQ,catSeg);
    for (int i=0; i<catSeg; i++){
        q(0,i) = 0;
        q(1,i) = 2*3.14; // 2pi?? 
        q(2,i) = 0;
    }
    m_q = q;
    
    double r = 0.89;   // 2mm in diameter
    double h = 130;   // 2mm in Height
    double max = 24000;
    for (int i=0; i<max; i++){
        Eigen::Matrix4d point = Eigen::MatrixXd::Identity(4,4);
        point(0,3) = r*sin(i*(M_PI/32));
        point(1,3) = r*cos(i*(M_PI/32));
        point(2,3) = h*i/max;
                
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
            mp_vis.clear();
            mp_vis.drawAorta(m_aorta.g_points(), 0.1, 0.4);
            mp_vis.update();
            m_aortaEn = false;
        }
        // Drawing Catheter
        if(m_cathEn){
            m_cath.fkine(m_q);
            mp_vis.drawCath(m_cath.g_backbone(), 0.89/2);
            mp_vis.update();
            m_cathEn = false;
            m_engaged = true;
        }
        // The simulation
        if(m_engaged){
            m_aorta.checkDistance(m_cath.g_eeFrame());
            int safety = m_aorta.g_safety();
            double maxD = m_aorta.g_maxDist();
            std::cout << "Safety Level -> " << safety << std::endl;

            // Testing all possibilties
            int i=0;
            int j=0;
            int k=0;
            for (int i=-1; i<2; i++){
                Eigen::MatrixXd editQ = m_q;
                editQ(0,0) = m_q(0,0) + i*m_cath.g_q1change();
                        
                // Find if the movement is acceptable or not. 
                m_cath.fkine(editQ);
                m_aorta.checkDistance(m_cath.g_eeFrame());
                safety = m_aorta.g_safety();
                maxD = m_aorta.g_maxDist();
                m_cath.fkine(m_q);
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
