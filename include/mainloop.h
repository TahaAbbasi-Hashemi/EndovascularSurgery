#pragma once

#define _USE_MATH_DEFINES

#include <visualizer.h>
#include <catheter.h>
#include <aorta.h>

//stl
#include <ctime>
#include <cmath>
#include <fstream>

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
		Visualizer mp_vis;
        Catheter m_cath;
        Aorta m_aorta;
        
        Eigen::MatrixXd m_wall;

        bool m_engaged;
        bool m_aortaEn;
        bool m_cathEn;
        Eigen::MatrixXd m_q;
        double m_rotated;
        double m_bended;

		
	public:
		MainLoop(Visualizer vis);
        ~MainLoop();

        // Execution Function 
		virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData));
};
