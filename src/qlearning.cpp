#include <qlearning.h>

double Qlearning::mg_reward(Eigen::MatrixXd q){
    Eigen::MatrixXd oQ = mp_catheter->g_q();// Original Q value
    mp_catheter->fkine(q);
    mp_aorta->checkDistance(mp_catheter->g_eeFrame());

    double dist = mp_aorta->g_distance();
    double gdist = mp_catheter->g_distEE();
    double reward = dist*gdist;

    mp_catheter->fkine(oQ);// return to original position

    return reward;
}

/*
Eigen::MatrixXd Qlearning::mg_randomAction(Eigen::MatrixXd policy){
    Eigen::MatrixXd qs = Eigen::MatrixXd::Zeros(m_nS, m_nA);
    std::vector<double> rewards;

    // Order everything
    int col = 0;
    for (int i=-1; i<2; i++){
        for (int j=-1; j<2; j++){
            for (int k=-1; k<2; k++){
                qs(0, col) = i;
                qs(1, col) = j;
                qs(2, col) = k;
                col ++;
            }
        }
    }

    // Sort from greatest to smallest
    std::sort(rewards.begin(), rewards.end(), std::greater<int>());

    // Getting the random value
    double probability = (std::rand() % 100)/100; // Get a value between 0-1
    std::cout << probability << std::endl;

    bool over = false;
    std::vector<int> possibilties; 
    for(int i=0; i<m_nA; i++){
        if(policy(i,0) >= probability){
            over = true;
            possibilities.push_back(i);
        }
    }
    int value = -1;
    if(!over){
        value = std::rand()%m_nA;
    }
    else{
        int app = std::rand() % (possibilities.size());
        value = possibilities.at(app);
    }

    Eigen::MatrixXd selected = qs.cols(value);
    return selected;
}
*/


void Qlearning::d_Q(){
    /*
    Eigen::MatrixXd Q_value(m_nA, m_nS) = Eigen::MatrixXd::Zeros(m_nA, m_nS);
    Eigen::MatrixXd policy(m_nA,m_nS) = Eigen::MatrixXd::Ones(m_nA,m_nS);
    policy = policy.eval()/m_nA;

    double epsilon = 1.0;

    for (int i=1, i<m_iterations, i++){
        epsilon = 1.0/i;
        bool done = false;

        // Reset enviroment

        for(int j=0; j<m_maxStates; j++){

            Eigen::MatriXd action = mg_randomAction(policy)// Get a action.



            // Breaking
            if (newState>5){
                done = true;
            }
            if(done){
                policy = g_greedyPolicy(Qvalue, epsilon);
                break;
            }


        }
    }
    */
}
