// stl
#include <array>
#include <cmath>
#include <iostream>

// vtk
#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>

// Eigen
#include "Eigen/Dense"

// includes
#include <ctcr_model.h>
#include <mainloop.h>
#include <tdcr_model.h>
#include <visualizer.h>


int main(int argc, char **argv) {
  
  // Create Visualizer
  Visualizer vis;
  vis.initScene(4);

  // Create TDCR
  std::array<double, 1> length;
  length[0] = 0.1;
  //length[1] = 0.1;
  int n_disks = 8;
  std::array<double, 1> pradius_disks;
  pradius_disks[0] = 0.034;
  //pradius_disks[1] = 0.034;

  Eigen::Matrix4d base_frame = Eigen::Matrix4d::Identity();

  double radius_disks = 0.02;
  double height_disks = 0.01;
  double ro = 0.001;

  TDCRModelDVS tdcr_model(length, n_disks, pradius_disks, base_frame);
 

  Eigen::Matrix<double, 4, 1> q;
  q << 0.001, 0.001, 0.001, 0.001;
  
    Eigen::Matrix4d ee_frame;
    Eigen::MatrixXd disk_frames;


    tdcr_model.forward_kinematics(ee_frame, disk_frames, q);


    // Drawing
    vis.drawTDCR(n_disks, pradius_disks, radius_disks, ro, height_disks);
    vis.updateTDCR(disk_frames);
  

    // Turn off warning messages to prevent them from spamming the terminal
  vtkObject::GlobalWarningDisplayOff();

  // Define required variables to set up the simulation
  double timestep = 0.01;

  // Create Main Loop
  MainLoop eventLoop(&vis, &tdcr_model, timestep, 4);

  // Create Window Interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(vis.getRenderWindow());

  // Set up and start main loop
  renderWindowInteractor->UpdateSize(1200, 700);
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  renderWindowInteractor->Initialize();
  renderWindowInteractor->CreateRepeatingTimer(timestep * 1000.0);
  renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, &eventLoop);
  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, &eventLoop);
  renderWindowInteractor->Start();

  return 1;
}

