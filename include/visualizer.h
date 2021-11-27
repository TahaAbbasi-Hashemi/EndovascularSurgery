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
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkTubeFilter.h>
#include <vtkLineSource.h>
#include <vtkOpenGLLight.h>
#include <vtkCubeSource.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkCylinderSource.h>
#include <vtkMatrix4x4.h>
#include <vtkCamera.h>

#include <vtkTriangle.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>

//Eigen
#include <Eigen/Dense>

// Class that implements the visualizer of the simulator using VTK
class Visualizer{  
	private:
		vtkSmartPointer<vtkRenderer> mp_Ren;
		vtkSmartPointer<vtkRenderWindow> mp_RenWin;
		vtkSmartPointer<vtkAxesActor> mp_target_frame;
        std::vector<vtkSmartPointer<vtkActor>> mp_curves;
		
        // Drawing Functions
        void drawCurves(Eigen::MatrixXd curve, double rad);
		void drawFrames(Eigen::MatrixXd frames);
		void drawPoints(Eigen::MatrixXd points, double rad, char color);
		
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
