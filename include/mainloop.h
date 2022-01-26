#pragma once

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
/**
 *  This class contains the main loop of the system. 
 *
 *  This class will contain and control other classes when it comes to ensuring that the system is running smoothly.
 */
class MainLoop : public vtkCommand {
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

	
        void md_safetySim();     ///< safety simulation function.
	public:
		MainLoop(Visualizer vis);   ///< constructor
        ~MainLoop();                ///< destructor

        /**
         *  This runs the simulation function. 
         *
         *  This XXXXXX
         */

        // Execution Function 
        /**
         *  The main loop function.
         *
         *  This takes an input from the system if there is one and sends it to the system. It also runs X amount of times a second.
         */
		virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData));
};
