#include <mainloop.h>
#include <iostream>
#include <time.h>


MainLoop::MainLoop(Visualizer *vis, TDCRModelDVS *tdcr, double timestep, int area) {
  //Basic
  m_timestep = timestep;
  mp_Vis = vis;
  mp_TDCR = tdcr;
  m_loopCount = 0;
  m_control_loop_active = false;
  
  // Integral stuff
  Eigen::MatrixXd enew(3, 1);
  enew << 0, 0, 0;
  error_integrate = enew;

  // Files
  outfile.open("ground.csv");
  infile.open("pose.txt");
}

MainLoop::~MainLoop() {}


void MainLoop::Execute(vtkObject *caller, unsigned long eventId, void *vtkNotUsed(callData)) {
  if (vtkCommand::TimerEvent == eventId) {
    m_loopCount = m_loopCount + 1;
    Eigen::Matrix4d ee_frame;
    Eigen::MatrixXd disk_frames;
    Eigen::MatrixXd backbone_centerline;
  } 

  if (vtkCommand::KeyPressEvent == eventId) // React on user input for each particular assignment
  {
    vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);
 
    // Start Loop
    if (strcmp((iren->GetKeySym()), "a")) {
        std::cout << "hi";
      m_control_loop_active = true;
      m_loopCount = 0;
    }
  }
}

Eigen::Matrix4d MainLoop::ReadLine(std::string line){
   std::stringstream str_strm(line);
   std::string tmp;
   char delim = ','; // Define the delimiter to split by

   double x,y,z;
   Eigen::Matrix4d frame;
   int iters = 0;

   while (std::getline(str_strm, tmp, delim)) {
       if (iters == 1)
           x = atof(tmp.c_str());
       else if (iters == 2)
           y = atof(tmp.c_str());
       else if (iters == 3)
           z = atof(tmp.c_str());
       iters ++;
   }

   frame = Eigen::Matrix4d::Identity();
   frame(0,3) = x;
   frame(1,3) = y;
   frame(2,3) = z;

   return frame;
}

void MainLoop::MoveMotor1(int position){
  uint8_t dxl_error = 0;
  int32_t dxl_present_position = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, position+4092, &dxl_error);  
}


void MainLoop::MoveMotor2(int position){
  uint8_t dxl_error = 0;
  int32_t dxl_present_position = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, position, &dxl_error);
}

int MainLoop::ConvertQM(double q){
  double rad = (q*1000)/(8.5/2);
  double deg = rad/0.01745;
  int x = deg*(4092/360);
  if(x > 4000){
    x = 4000;
  }else if (x < -4000){
    x = -4000;
  }
  return x;
}
