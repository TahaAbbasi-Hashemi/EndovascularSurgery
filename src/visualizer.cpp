#include <visualizer.h>


Visualizer::Visualizer() {
	mp_Ren = vtkSmartPointer<vtkRenderer>::New();
	mp_RenWin = vtkSmartPointer<vtkRenderWindow>::New();
	mp_RenWin->AddRenderer(mp_Ren);

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
    mp_Ren->GetActiveCamera()->SetPosition(0,0,150);
    mp_Ren->GetActiveCamera()->SetFocalPoint(0.05,0,0.05);  
    mp_Ren->GetActiveCamera()->SetViewUp(0,0,-1);     
}

Visualizer::~Visualizer() {}


void Visualizer::update() { mp_RenWin->Render(); }
vtkSmartPointer<vtkRenderWindow> Visualizer::g_renderWindow() { return mp_RenWin; }


void Visualizer::clear() { 
    mp_Ren->RemoveAllViewProps(); }

void Visualizer::drawCurves(Eigen::MatrixXd curve, double rad){
    for(int i = 1; i < curve.cols()/4; i++){
        Eigen::Matrix4d cur_frame;
        cur_frame = curve.block(0,4*i,4,4);
        Eigen::Matrix4d prev_frame;
        prev_frame = curve.block(0,4*(i-1),4,4);

        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
        line->SetPoint1(prev_frame(0,3),prev_frame(1,3),prev_frame(2,3));
        line->SetPoint2(cur_frame(0,3),cur_frame(1,3),cur_frame(2,3));

        vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
        tubeFilter->SetInputConnection(line->GetOutputPort());
        tubeFilter->SetRadius(rad);                               // RADIUS
        tubeFilter->SetNumberOfSides(50);                           // NUmber of sides

        vtkSmartPointer<vtkPolyDataMapper> curveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        curveMapper->SetInputConnection(tubeFilter->GetOutputPort());
        vtkSmartPointer<vtkActor> curveActor = vtkSmartPointer<vtkActor>::New();
        curveActor->SetMapper(curveMapper);
        curveActor->GetProperty()->SetColor(0.9,0.9,0.9);
        mp_curves.push_back(curveActor);
        mp_Ren->AddActor(curveActor);
    }
}


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
        if (color == 'h'){
            actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
            actor->GetProperty()->SetOpacity(0.9);
        }else if (color == 'g'){
            actor->GetProperty()->SetColor(0.0, 0.5, 0.0);
            actor->GetProperty()->SetOpacity(0.1);
        }else if (color == 'r'){
            actor->GetProperty()->SetColor(0.5, 0.0, 0.0);
            actor->GetProperty()->SetOpacity(0.1);
        }
        mp_Ren->AddActor(actor);
    }
}


// The user interacts with these only!!!
void Visualizer::drawCath(Eigen::MatrixXd bb, double rad){
    // Drawing Curves
    clearCath();
    drawCurves(bb, rad);
    // Drawing start and End Frame
    drawFrames(bb.leftCols(4));
    drawFrames(bb.rightCols(4));
}
void Visualizer::clearCath(){
    for (int i=0;  i<mp_curves.size(); i++){
        mp_Ren->RemoveActor(mp_curves.at(i));
    }
    mp_curves.clear();
}

void Visualizer::drawAorta(Eigen::MatrixXd points, double dead, double danger){
    drawPoints(points, dead, 'h');  // Point/Dead   (0.01mm)
    drawPoints(points, danger, 'r');   // Danger       (0.5mm)
}

void Visualizer::clearAorta(){}
