#pragma once

//stl
#include <vector>
#include <array>

//vtk
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>   // unused
#include <vtkPolyData.h>    // unused
#include <vtkPolyDataMapper.h>  // unused
#include <vtkProperty.h>
#include <vtkTransform.h>       // unused
#include <vtkCellArray.h>       // unused
#include <vtkOpenGLLight.h>    
#include <vtkUnstructuredGrid.h>    //unused
#include <vtkGeometryFilter.h>      // unused
#include <vtkMatrix4x4.h>       // ???
#include <vtkCamera.h>
#include <vtkTubeFilter.h>      // removed need for.

// Shapes
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkTriangle.h>    // unused
#include <vtkCubeSource.h>  // unused
#include <vtkSphereSource.h>    
#include <vtkCylinderSource.h>  // unused

#include <vtkNamedColors.h>     // Can not figure out how to use.
#include <vtkRenderWindowInteractor.h>

//Eigen
#include <Eigen/Dense>

/**
 * This class is all about drawing the simulation 
 *
 * This functions that should only be called from this class are the drawing and clearing functions. 
 */
class Visualizer{  
	private:
		vtkSmartPointer<vtkRenderer> mp_Ren;
		vtkSmartPointer<vtkRenderWindow> mp_RenWin;
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
		Visualizer(); ///< Constructor
		~Visualizer();///< Deconstructor

        // Set Functions
        
        // Do functions
            // Draw
        /**
         * This function draws a catheter given the backbone radius and the backbone points. 
         *
         * @param[in] bb 
         *      This is the points across the backbone of the catheter. 
         * @param[in] rad
         *      This is the radius of the backbone. 
         */
        void drawCath(Eigen::MatrixXd bb, double rad);

        /**
         * This function draws the Aorta using a point cloud. 
         *
         * The aorta is made up of several spheres, and each sphere is drawn multiple times. 
         * The first is the dead zone which is a dark color. The second is a danger zone which is transparent. 
         * @param[in] wall
         *      The points of the aorta cloud. 
         * @param[in] dead
         *      The radius of the dead zone
         * @param[in] danger
         *      The radius of the danger zone
         */
		void drawAorta(Eigen::MatrixXd wall, double dead, double danger);
        void update(); ///< updates the visualizer system.
        
            // Clear
        void clearCath();   ///< Removes the catheter from the visualizer system. 
        void clearAorta();  ///< Removes the Aorta from the visualizer system.
		void clear();       ///< Removes everything from the visualizer system.
        
        // Get Functions
		vtkSmartPointer<vtkRenderWindow> g_renderWindow(); ///< Returns the render window.
};

