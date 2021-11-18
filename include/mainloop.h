#pragma once

#define _USE_MATH_DEFINES

#include <visualizer.h>
#include <robot_independent.h>
#include <tdcr_model_DVS.h>
#include <controller.h>

//stl
#include <ctime>
#include <cmath>
#include <fstream>
#include <random>

//vtk
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

//Eigen
#include <Eigen/Dense>

// Class that implements the main simulation loop
class MainLoop : public vtkCommand
{
	private:
        // Base Info
		Visualizer* mp_Vis;
		Controller* mp_Controller;
		TDCRModelDVS* mp_TDCR;
		
        // Loop info
        double m_timestep;
        double m_loopCount;
		bool m_control_loop_active;
	public:
		MainLoop(Visualizer* vis, TDCRModelDVS* tdcr, double timestep, int area);
		~MainLoop();
	
		virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData));
        Eigen::Matrix4d ReadLine(std::string line);       
        void MoveMotor1(int position);
        void MoveMotor2(int position);
        int ConvertQM(double q);
};

