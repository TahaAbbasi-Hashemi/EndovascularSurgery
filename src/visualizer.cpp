#include <visualizer.h>


Visualizer::Visualizer() {
	mp_Ren = vtkSmartPointer<vtkRenderer>::New();
	mp_RenWin = vtkSmartPointer<vtkRenderWindow>::New();
	mp_RenWin->AddRenderer(mp_Ren);
    //mp_renWinInt = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    //mp_renWinInt->SetRenderWindow(mp_RenWin);

    // General settings
    mp_Ren->SetBackground(1,1,1);
	mp_RenWin->SetWindowName("Taha Catheter Simulation");

	// Initialize the scene (light, floor coordinate frames etc)
	vtkSmartPointer<vtkOpenGLLight> sceneLight = vtkSmartPointer<vtkOpenGLLight>::New();
    sceneLight->SetDirectionAngle(45,0);
    sceneLight->SetDiffuseColor(0.8,0.8,0.9);
    sceneLight->SetSpecularColor(0.98,0.98,0.98);
    sceneLight->SetIntensity(1);
    mp_Ren->AddLight(sceneLight);
    
	//Camera
    //mp_Ren->GetActiveCamera()->SetPosition(0,0,150);
    //mp_Ren->GetActiveCamera()->SetFocalPoint(0.05,0,0.05);  
    //mp_Ren->GetActiveCamera()->SetViewUp(0,0,-1);     
    //std::array<unsigned char, 3> blk{{0, 0, 0}};
    //mp_colors->SetColor("Black", blk.data());
}

Visualizer::~Visualizer() {}


void Visualizer::update() { mp_RenWin->Render(); }
vtkSmartPointer<vtkRenderWindow> Visualizer::g_renderWindow() { return mp_RenWin; }


void Visualizer::clear() { 
    clearCath();
    clearAorta();
}
    //mp_Ren->RemoveAllViewProps(); }


void Visualizer::drawFrames(Eigen::MatrixXd frames){
	for(int i = 0; i < frames.cols()/4; i++){
		Eigen::Matrix4d cur_frame;
        cur_frame = frames.block(0,4*i,4,4);
		
		vtkSmartPointer<vtkAxesActor> frame = vtkSmartPointer<vtkAxesActor>::New();
		frame->SetXAxisLabelText("");
		frame->SetYAxisLabelText("");
		frame->SetZAxisLabelText("");
		frame->SetShaftTypeToCylinder();
		frame->SetCylinderRadius(0.04);
		frame->SetTotalLength(0.015,0.015,0.015);
		
		vtkSmartPointer<vtkMatrix4x4> ee_frame_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){ ee_frame_vtk->SetElement(i,j,cur_frame(i,j));}
		}
		frame->SetUserMatrix(ee_frame_vtk);
		
		mp_Ren->AddActor(frame);
        //m_frameActors.push_back(frame);
	}
}

void Visualizer::drawPoints(Eigen::MatrixXd points, double rad, char color){
    for(int i = 0; i < points.cols()/4; i++){
        Eigen::Matrix4d curFrame = points.block(0,4*i,4,4);
        
        // Create a sphere
        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

        // Shape Parameters
        sphereSource->SetCenter(curFrame(0,3), curFrame(1,3), curFrame(2,3));
        sphereSource->SetRadius(rad);
        sphereSource->SetPhiResolution(100);
        sphereSource->SetThetaResolution(100);

        // Visual Parameters
        mapper->SetInputConnection(sphereSource->GetOutputPort());
        actor->SetMapper(mapper);
        actor->GetProperty()->SetOpacity(0.1);
        //actor->GetProperty()->SetColor(mp_colors->GetColor3d("Black").GetData());
        
        mp_Ren->AddActor(actor);
    }
}

void Visualizer::drawSphere(Eigen::MatrixXd points, std::vector<vtkSmartPointer<vtkActor>> &actors, double rad, std::vector<double> color, double trans){
    for(int i = 1; i < points.cols()/4; i++){
        vtkSmartPointer<vtkSphereSource> source = vtkSmartPointer<vtkSphereSource>::New();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

        // Shape Parameters
        Eigen::Matrix4d curFrame;
        curFrame = points.block(0,4*i,4,4);
        source->SetCenter(curFrame(0,3), curFrame(1,3), curFrame(2,3));
        source->SetRadius(rad);
        source->SetPhiResolution(25);
        source->SetThetaResolution(25);

        // Visual Parameters
        mapper->SetInputConnection(source->GetOutputPort());
        actor->SetMapper(mapper);
        actor->GetProperty()->SetOpacity(1);
        actor->GetProperty()->SetOpacity(trans);
        actor->GetProperty()->SetColor(color.at(0), color.at(1), color.at(2));
        actor->GetProperty()->SetDiffuse(0.8);
        actor->GetProperty()->SetSpecular(0.3);
        actor->GetProperty()->SetSpecularPower(60);

        // Adding actor
        mp_Ren->AddActor(actor);
        actors.push_back(actor);
    }
}


// The user interacts with these only!!!
void Visualizer::drawCath(Eigen::MatrixXd bb, double rad){
    // Clear
    clearCath();

    // Set Colors
    std::vector<double> cathColor;
    cathColor.push_back(0.5);
    cathColor.push_back(0.5);
    cathColor.push_back(0.5);

    // Draw
    drawSphere(bb, m_cathActors, 0.5, cathColor, 1);
}

void Visualizer::clearCath(){
    for (int i=0; i<m_cathActors.size(); i++){
        mp_Ren->RemoveActor(m_cathActors.at(i));
    }
    m_cathActors.clear();
}

void Visualizer::drawAorta(Eigen::MatrixXd points, double dead, double danger){
    // CLear
    clearAorta();

    // Set Colors
    std::vector<double> wallColor;
    std::vector<double> dangerColor;
    std::vector<double> deadColor;
    
    wallColor.push_back(0.5);
    wallColor.push_back(0.5);
    wallColor.push_back(0.5);
    dangerColor.push_back(0.5);
    dangerColor.push_back(0.1);
    dangerColor.push_back(0.1);
    deadColor.push_back(1);
    deadColor.push_back(0);
    deadColor.push_back(0);
 
    // Take every 5th point instead
    //drawSphere(points, m_aortaActors, 0.5, wallColor, 1);
    drawSphere(points, m_aortaActors, danger, dangerColor, 0.02);
    drawSphere(points, m_aortaActors, dead, deadColor, 1);



/*
    std::string file = "/home/taha/Development/Cpp/EndovascularSurgery/Aorta.stl";
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    
    std::cout << "DID this\n";
    reader->SetFileName(file.c_str());
    reader->Update();

        // Visual Parameters
    mapper->SetInputConnection(reader->GetOutputPort());
    actor->SetMapper(mapper);

    actor->GetProperty()->SetDiffuse(0.8);
    actor->GetProperty()->SetSpecular(0.3);
    actor->GetProperty()->SetSpecularPower(60);
    //Set Position??

    std::cout << "DID this\n";
    
    mp_Ren->AddActor(actor);
    //actors.push_back(actor);
    */
}


void Visualizer::clearAorta(){
    for (int i=0; i<m_aortaActors.size(); i++){
        mp_Ren->RemoveActor(m_aortaActors.at(i));
    }
    m_aortaActors.clear();
}
