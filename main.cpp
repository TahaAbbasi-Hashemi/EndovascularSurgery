// includes
#include <mainloop.h>
#include <visualizer.h>
#include <robot.h>

// stl
#include <array>
#include <cmath>
#include <iostream>

// vtk
#include <vtkCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>

// Eigen
#include <Eigen/Dense>

int main(int argc, char **argv) {
    if (argc == 1) {} // We got not input
   // if (strcmp(scen, "a1") == 0) {}

    Visualizer vis;
    MainLoop eventLoop(vis); // Makes a loop called event loop

    // Turn off warning messages to prevent them from spamming the terminal
    vtkObject::GlobalWarningDisplayOff();
    // Create Window Interactor
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(vis.g_renderWindow());
    // Set up and start main loop
    renderWindowInteractor->UpdateSize(1200, 700);
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    renderWindowInteractor->SetInteractorStyle(style);
    renderWindowInteractor->Initialize();
    renderWindowInteractor->CreateRepeatingTimer(0.01 * 1000.0);
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, &eventLoop);
    renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, &eventLoop);
    renderWindowInteractor->Start();

    return 1;
}
