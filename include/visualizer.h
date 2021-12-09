#pragma once

#define _USE_MATH_DEFINES

//stl
#include <vector>
#include <array>

//vtk
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkCellArray.h>
#include <vtkOpenGLLight.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkCamera.h>
#include <vtkTubeFilter.h>

// Shapes
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkTriangle.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>

#include <vtkNamedColors.h>
#include <vtkRenderWindowInteractor.h>

//Eigen
#include <Eigen/Dense>

// Class that implements the visualizer of the simulator using VTK
class Visualizer{  
	private:
		vtkSmartPointer<vtkRenderer> mp_Ren;
		vtkSmartPointer<vtkRenderWindow> mp_RenWin;
		vtkSmartPointer<vtkAxesActor> mp_target_frame;
        vtkSmartPointer<vtkNamedColors> mp_colors; // This is giving me trouble look into it later
        //std::vector<vtkSmartPointer<vtkActor>> mp_curves;
        std::vector<vtkSmartPointer<vtkActor>> m_curveActors;
        std::vector<vtkSmartPointer<vtkActor>> m_sphereActors;
        std::vector<vtkSmartPointer<vtkActor>> m_frameActors;
        std::vector<vtkSmartPointer<vtkActor>> m_cathActors;
        std::vector<vtkSmartPointer<vtkActor>> m_aortaActors;
		
        // Drawing Functions
        void drawCurves(Eigen::MatrixXd curve, double rad);
		void drawFrames(Eigen::MatrixXd frames);
		void drawPoints(Eigen::MatrixXd points, double rad, char color);
        void drawSphere(Eigen::MatrixXd points, std::vector<vtkSmartPointer<vtkActor>> &actors, double rad, std::vector<double> color, double trans);
		
	public:
        // Init functions
		Visualizer();
		~Visualizer();

        // Set Functions
        
        // Do functions
            // Draw
        void drawCath(Eigen::MatrixXd bb, double rad);
		void drawAorta(Eigen::MatrixXd wall, double dead, double danger);
        void update();
        
            // Clear
        void clearCath();
        void clearAorta();
		void clear();
        
        // Get Functions
		vtkSmartPointer<vtkRenderWindow> g_renderWindow();
};
