#define VTK_AOS_DATA_ARRAY_TEMPLATE_INSTANTIATING
#define VTK_GENERIC_DATA_ARRAY_INSTANTIATING

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include "ZweiCFD/core/simulation.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <omp.h>

#include <vtkImageSlice.h>
#include <vtkImageSliceMapper.h>
#include <vtkImageProperty.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkIntArray.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkCamera.h>
#include <vtkObjectFactory.h>

namespace zweicfd {

class CustomTrackballCamera : public vtkInteractorStyleTrackballCamera {
public:
    static CustomTrackballCamera* New();
    vtkTypeMacro(CustomTrackballCamera, vtkInteractorStyleTrackballCamera);
    
    Simulation* sim = nullptr;
    bool isDrawing = false;
    
    void drawAtMouse() {
        int* pos = this->Interactor->GetEventPosition();
        
        this->FindPokedRenderer(pos[0], pos[1]);
        vtkRenderer* ren = this->CurrentRenderer;
        if (!ren) {
            return;
        }
        
        double displayCoords[3] = { (double)pos[0], (double)pos[1], 0.0 };
        ren->SetDisplayPoint(displayCoords);
        ren->DisplayToWorld();
        double worldPt1[4];
        ren->GetWorldPoint(worldPt1);
        
        displayCoords[2] = 1.0;
        ren->SetDisplayPoint(displayCoords);
        ren->DisplayToWorld();
        double worldPt2[4];
        ren->GetWorldPoint(worldPt2);
        
        for (int i = 0; i < 3; ++i) {
            worldPt1[i] /= worldPt1[3];
            worldPt2[i] /= worldPt2[3];
        }
        
        double ray[3] = {worldPt2[0] - worldPt1[0], worldPt2[1] - worldPt1[1], worldPt2[2] - worldPt1[2]};
        
        if (std::abs(ray[2]) < 1e-6) return;
        
        double t = -worldPt1[2] / ray[2];
        double pX = worldPt1[0] + t * ray[0];
        double pY = worldPt1[1] + t * ray[1];
        
        sim->addDrawnObstacle(pX, pY, sim->brushSize);
    }
    
    void OnLeftButtonDown() override {
        if (sim && sim->drawMode) {
            isDrawing = true;
            drawAtMouse();
        } else {
            vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        }
    }
    
    void OnLeftButtonUp() override {
        isDrawing = false;
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    }
    
    void OnMouseMove() override {
        if (sim && sim->drawMode && isDrawing) {
            drawAtMouse();
        } else {
            vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
    }
    
    void OnMouseWheelBackward() override {
        if (this->CurrentRenderer) {
            vtkCamera* cam = this->CurrentRenderer->GetActiveCamera();
            if (cam->GetDistance() > maxZoomDistance) {
                return; 
            }
        }
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
    double maxZoomDistance = 200.0; 
};
vtkStandardNewMacro(CustomTrackballCamera);



Simulation::Simulation(int argc, char *argv[]) {
  std::cout << "==== ZweiCFD v0.4   Streamlines & Viscous Flow ====\n";

  std::string filename = (argc >= 2) ? argv[1] : "naca2412.dat";
  if (!foil.loadFromFile(filename)) {
    std::cerr << "Failed to load Airfoil Data!.\n";
  } else {
    std::cout << "Loaded: " << foil.getName() << "\n";
  }

  config = ConfigLoader::load("assets/config.json");

  flow.alpha = config.alpha;
  flow.V_inf = config.v_inf;
  flow.kinematic_viscosity = config.kinematicViscosity;
  flow.windDirection = 0;

  current_sim = 1; 

  rebuildSolverWithRotation();
}

Simulation::~Simulation() {
}

void Simulation::rebuildSolverWithRotation() {
  rotatedFoil = foil;
  rotatedFoil.rotateCoordinates(-flow.alpha);
  solver = std::make_unique<zweicfd::Solver>(rotatedFoil);

  zweicfd::Flowconditions solverFlow = flow;
  solverFlow.alpha = 0.0;

  if (current_sim == 0) {
    results = solver->runSimulation(solverFlow);
  } else if (current_sim == 1) {
    
    lbmSolver = std::make_unique<zweicfd::LBMSolver>(config.lbmGridNX, config.lbmGridNY, config.lbmGridNZ);
    auto &grid = lbmSolver->getGridModifiable();
    const auto &panels = rotatedFoil.getPanels();
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto &panel : panels) {
      minX = std::min({minX, panel.p1.x, panel.p2.x});
      maxX = std::max({maxX, panel.p1.x, panel.p2.x});
      minY = std::min({minY, panel.p1.y, panel.p2.y});
      maxY = std::max({maxY, panel.p1.y, panel.p2.y});
    }
    cowWidth = std::max(0.1, maxX - minX);
    cowHeight = std::max(0.1, maxY - minY);
    
    float lbmScale = 32.0f; 
    cachedLbmScale = lbmScale;
    
    std::cout << "[SIM-LBM] Initializing Volumetric LBM Solver..." << std::endl;
    std::cout << "[SIM-LBM] Grid Size: " << config.lbmGridNX << "x" << config.lbmGridNY << "x" << config.lbmGridNZ << std::endl;
    std::cout << "[SIM-LBM] Computed Grid Scale: " << cachedLbmScale << std::endl;
    
    std::vector<double> sdf2D(grid.NX * grid.NY);
    
#pragma omp parallel for collapse(2)
    for (int y = 0; y < grid.NY; ++y) {
      for (int x = 0; x < grid.NX; ++x) {
        
        float offsetX = -0.15f; 
        double physX = (x - grid.NX / 2.0) / lbmScale - offsetX;
        double physY = (y - grid.NY / 2.0) / lbmScale;
        zweicfd::Point2D p{physX, physY};
        bool inside = false;
        double minDist = 1e9;
        for (const auto &panel : panels) {
          
          if (((panel.p1.y > p.y) != (panel.p2.y > p.y)) &&
              (p.x < (panel.p2.x - panel.p1.x) * (p.y - panel.p1.y) /
                             (panel.p2.y - panel.p1.y) +
                         panel.p1.x)) {
            inside = !inside;
          }

          
          double l2 = (panel.p2.x - panel.p1.x) * (panel.p2.x - panel.p1.x) +
                      (panel.p2.y - panel.p1.y) * (panel.p2.y - panel.p1.y);
          if (l2 == 0.0) {
            double d = std::sqrt((p.x - panel.p1.x) * (p.x - panel.p1.x) +
                                 (p.y - panel.p1.y) * (p.y - panel.p1.y));
            minDist = std::min(minDist, d);
            continue;
          }

          double t = std::max(
              0.0,
              std::min(1.0, ((p.x - panel.p1.x) * (panel.p2.x - panel.p1.x) +
                             (p.y - panel.p1.y) * (panel.p2.y - panel.p1.y)) /
                                l2));
          double projX = panel.p1.x + t * (panel.p2.x - panel.p1.x);
          double projY = panel.p1.y + t * (panel.p2.y - panel.p1.y);

          double d = std::sqrt((p.x - projX) * (p.x - projX) +
                               (p.y - projY) * (p.y - projY));
          minDist = std::min(minDist, d);
        }
        int idx = y * grid.NX + x;
        sdf2D[idx] = inside ? -minDist : minDist;
      }
    }

    int internalNodes = 0;
#pragma omp parallel for collapse(3) reduction(+:internalNodes)
    for (int z = 0; z < grid.NZ; ++z) {
      for (int y = 0; y < grid.NY; ++y) {
        for (int x = 0; x < grid.NX; ++x) {
          int idx2D = y * grid.NX + x;
          double d2D = sdf2D[idx2D];
          
          double physZ = (z - grid.NZ / 2.0) / lbmScale;
          double spanRadius = 0.5; 
          double dZ = std::abs(physZ) - spanRadius;
          
          double dist3D;
          if (d2D > 0.0 && dZ > 0.0) {
              dist3D = std::sqrt(d2D*d2D + dZ*dZ);
          } else {
              dist3D = std::max(d2D, dZ);
          }
          
          int scalarIdx = grid.getScalarIndex(x, y, z);
          float airfoil_sdf = (float)(dist3D * lbmScale);
          grid.sdf[scalarIdx] = std::min(airfoil_sdf, grid.drawn_sdf[scalarIdx]);
          
          if (grid.sdf[scalarIdx] <= 0.0f) internalNodes++;
        }
      }
    }
    
    std::cout << "[SIM-LBM] SDF Voxelization Complete. Internal Nodes: " << internalNodes << std::endl;

    zweicfd::Flowconditions safeLBM = solverFlow;
    safeLBM.V_inf = solverFlow.V_inf;
    safeLBM.kinematic_viscosity = solverFlow.kinematic_viscosity *
                                  (solverFlow.V_inf / std::max(0.0001, solverFlow.V_inf));

    lbmSolver->getGridModifiable().initialize(safeLBM);
    std::cout << "[SIM-LBM] Starting real-time continuous LBM solver..." << std::endl;
    totalLbmSteps = 0;
    freezeFlow = false;
    results = {0.1, 0.05, -0.02};
  }
}

void Simulation::setupVTKWithWindow(vtkRenderWindow* window) {
    this->renderWindow = window;
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);

    if (auto interactor = renderWindow->GetInteractor()) {
        vtkSmartPointer<CustomTrackballCamera> style = vtkSmartPointer<CustomTrackballCamera>::New();
        style->sim = this;
        interactor->SetInteractorStyle(style);
    }

    updateVTKGeometry();
    
    if (renderer) {
        vtkCamera* cam = renderer->GetActiveCamera();
        if (cam) {
            
            renderer->ResetCamera();
            if (auto interactor = renderWindow->GetInteractor()) {
                if (auto style = CustomTrackballCamera::SafeDownCast(interactor->GetInteractorStyle())) {
                    style->maxZoomDistance = cam->GetDistance() * 1.05; 
                }
            }
        }
    }
    
    
    float renderScale = 40.0f;
    float spacing = renderScale / cachedLbmScale;
    
    velocityField = vtkSmartPointer<vtkImageData>::New();
    velocityField->SetDimensions(config.lbmGridNX, config.lbmGridNY, config.lbmGridNZ);
    velocityField->SetSpacing(spacing, spacing, spacing);
    velocityField->SetOrigin(
        (-config.lbmGridNX / 2.0f) * spacing,
        (-config.lbmGridNY / 2.0f) * spacing,
        (-config.lbmGridNZ / 2.0f) * spacing
    );
    
    velocityArray = vtkSmartPointer<vtkFloatArray>::New();
    velocityArray->SetNumberOfComponents(3);
    velocityArray->SetNumberOfTuples(config.lbmGridNX * config.lbmGridNY * config.lbmGridNZ);
    velocityArray->SetName("Velocity");
    
    speedArray = vtkSmartPointer<vtkFloatArray>::New();
    speedArray->SetNumberOfComponents(1);
    speedArray->SetNumberOfTuples(config.lbmGridNX * config.lbmGridNY * config.lbmGridNZ);
    speedArray->SetName("Speed");
    
    for (int i = 0; i < config.lbmGridNX * config.lbmGridNY * config.lbmGridNZ; ++i) {
        velocityArray->SetTuple3(i, 0.0f, 0.0f, 0.0f);
        speedArray->SetValue(i, 0.0f);
    }
    velocityField->GetPointData()->SetVectors(velocityArray);
    velocityField->GetPointData()->SetScalars(speedArray);
    
    
    float inletX = (-config.lbmGridNX / 2.0f + 2.0f) * spacing;
    float spreadY = (config.lbmGridNY / 2.0f - 1.0f) * spacing;
    float spreadZ = (config.lbmGridNZ / 2.0f - 1.0f) * spacing;
    
    streamRake = vtkSmartPointer<vtkPlaneSource>::New();
    streamRake->SetOrigin(inletX, -spreadY, -spreadZ);
    streamRake->SetPoint1(inletX,  spreadY, -spreadZ);
    streamRake->SetPoint2(inletX, -spreadY,  spreadZ);
    streamRake->SetResolution(10, 10); 

    streamTracer = vtkSmartPointer<vtkStreamTracer>::New();
    streamTracer->SetInputData(velocityField);
    streamTracer->SetSourceConnection(streamRake->GetOutputPort());
    streamTracer->SetIntegrationStepUnit(vtkStreamTracer::CELL_LENGTH_UNIT);
    streamTracer->SetMaximumPropagation(5000.0);
    streamTracer->SetMaximumNumberOfSteps(5000);
    streamTracer->SetInitialIntegrationStep(0.2);
    streamTracer->SetIntegrationDirectionToForward();
    
    lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(256);
    lut->SetHueRange(0.667, 0.0);
    lut->SetSaturationRange(1.0, 1.0);
    lut->SetValueRange(1.0, 1.0);
    lut->SetTableRange(0.0, 0.15);
    lut->Build();

    vtkSmartPointer<vtkPolyDataMapper> streamMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    streamMapper->SetInputConnection(streamTracer->GetOutputPort());
    streamMapper->SetScalarModeToUsePointFieldData();
    streamMapper->SelectColorArray("Speed");
    streamMapper->SetScalarRange(0.0, 0.15);
    streamMapper->SetLookupTable(lut);
    
    streamActor = vtkSmartPointer<vtkActor>::New();
    streamActor->SetMapper(streamMapper);
    streamActor->GetProperty()->SetLineWidth(2.0);
    streamActor->GetProperty()->SetOpacity(0.9);
    
    renderer->AddActor(streamActor);
    
    heatmapMapper = vtkSmartPointer<vtkImageSliceMapper>::New();
    heatmapMapper->SetInputData(velocityField);
    heatmapMapper->SetOrientationToZ();
    heatmapMapper->SetSliceNumber(config.lbmGridNZ / 2);
    
    heatmapSlice = vtkSmartPointer<vtkImageSlice>::New();
    heatmapSlice->SetMapper(heatmapMapper);
    heatmapSlice->GetProperty()->SetLookupTable(lut);
    heatmapSlice->GetProperty()->SetUseLookupTableScalarRange(true);
    heatmapSlice->GetProperty()->SetOpacity(0.9);
    heatmapSlice->SetPosition(0.0, 0.0, -0.5); 
    renderer->AddViewProp(heatmapSlice);
    
    drawnPoints = vtkSmartPointer<vtkPoints>::New();
    drawnPolyData = vtkSmartPointer<vtkPolyData>::New();
    drawnPolyData->SetPoints(drawnPoints);
    
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    drawnPolyData->SetVerts(vertices);
    
    vtkSmartPointer<vtkIntArray> shapeIndex = vtkSmartPointer<vtkIntArray>::New();
    shapeIndex->SetName("ShapeIndex");
    drawnPolyData->GetPointData()->AddArray(shapeIndex);
    drawnPolyData->GetPointData()->SetActiveScalars("ShapeIndex");
    
    
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(1.0); 
    sphere->SetPhiResolution(12);
    sphere->SetThetaResolution(12);
    
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetXLength(2.0); cube->SetYLength(2.0); cube->SetZLength(2.0);
    
    vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New(); 
    cone->SetRadius(1.5); cone->SetHeight(2.0); cone->SetResolution(4); 
    
    drawnGlyph = vtkSmartPointer<vtkGlyph3D>::New();
    drawnGlyph->SetInputData(drawnPolyData);
    drawnGlyph->SetSourceConnection(0, sphere->GetOutputPort());
    drawnGlyph->SetSourceConnection(1, cube->GetOutputPort());
    drawnGlyph->SetSourceConnection(2, cone->GetOutputPort());
    drawnGlyph->SetIndexModeToScalar();
    drawnGlyph->SetRange(0, 2);
    
    
    drawnGlyph->SetScaleModeToDataScalingOff(); 
    
    vtkSmartPointer<vtkPolyDataMapper> drawnMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    drawnMapper->SetInputConnection(drawnGlyph->GetOutputPort());
    
    drawnActor = vtkSmartPointer<vtkActor>::New();
    drawnActor->SetMapper(drawnMapper);
    drawnActor->GetProperty()->SetColor(0.8, 0.8, 0.8); 
    
    renderer->AddActor(drawnActor);
    
    renderer->SetBackground(0.05, 0.05, 0.1);
    renderer->SetBackground2(0.15, 0.15, 0.2);
    renderer->GradientBackgroundOn();
    renderer->ResetCamera();
}

void Simulation::updateVTKGeometry() {
    if (!renderer) return;
    
    if (!airfoilPolyData) {
        airfoilPolyData = vtkSmartPointer<vtkPolyData>::New();
        
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(airfoilPolyData);
        
        airfoilActor = vtkSmartPointer<vtkActor>::New();
        airfoilActor->SetMapper(mapper);
        airfoilActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
        
        renderer->AddActor(airfoilActor);
    }
    
    const auto &panels = rotatedFoil.getPanels();
    if (!panels.empty()) {
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
        
        float renderScale = 40.0f;
        float physicalSpan = 1.0f;
        float zStart = -0.5f * physicalSpan * renderScale;
        float offsetX = -0.15f; 
        
        polygon->GetPointIds()->SetNumberOfIds(panels.size());
        for (size_t i = 0; i < panels.size(); ++i) {
            points->InsertNextPoint((panels[i].p1.x + offsetX) * renderScale, panels[i].p1.y * renderScale, zStart);
            polygon->GetPointIds()->SetId(i, i);
        }
        
        vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
        polys->InsertNextCell(polygon);
        
        vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
        tempPolyData->SetPoints(points);
        tempPolyData->SetPolys(polys);
        
        vtkSmartPointer<vtkLinearExtrusionFilter> extrude = vtkSmartPointer<vtkLinearExtrusionFilter>::New();
        extrude->SetInputData(tempPolyData);
        extrude->SetExtrusionTypeToVectorExtrusion();
        extrude->SetVector(0, 0, 1);
        extrude->SetScaleFactor(physicalSpan * renderScale);
        extrude->Update();
        
        airfoilPolyData->ShallowCopy(extrude->GetOutput());
    }
}

void Simulation::stepSimulation() {
    if (current_sim == 1 && lbmSolver) {
        
        if (!freezeFlow) {
            zweicfd::Flowconditions safeLBM = flow;
            safeLBM.V_inf = flow.V_inf;
            safeLBM.kinematic_viscosity =
                flow.kinematic_viscosity * (flow.V_inf / std::max(0.0001, flow.V_inf)) * cachedLbmScale;
                
            for (int i = 0; i < stepsPerFrame; ++i) {
                lbmSolver->step(safeLBM);
                totalLbmSteps++;
            }
        }
        
        
        if (showParticles && streamActor) {
            streamActor->SetVisibility(true);
            
            
            if (totalLbmSteps % vtkUpdateFrequency == 0 || needsVTKUpdate) {
                const auto& grid = lbmSolver->getGrid();
                
                float* vtkData = velocityArray->WritePointer(0, grid.NX * grid.NY * grid.NZ * 3);
                float* speedData = speedArray->WritePointer(0, grid.NX * grid.NY * grid.NZ);
                
                #pragma omp parallel for
                for(int i = 0; i < grid.NX * grid.NY * grid.NZ; ++i) {
                    float vx = grid.u[i].x;
                    float vy = grid.u[i].y;
                    float vz = grid.u[i].z;
                    
                    vtkData[i*3 + 0] = vx;
                    vtkData[i*3 + 1] = vy;
                    vtkData[i*3 + 2] = vz;
                    
                    speedData[i] = std::sqrt(vx*vx + vy*vy + vz*vz);
                }
                
                speedArray->Modified();
                velocityArray->Modified();
                velocityField->Modified();
                needsVTKUpdate = false;
            }
        } else if (streamActor) {
            streamActor->SetVisibility(false);
        }
        
        if (heatmapSlice) {
            heatmapSlice->SetVisibility(showHeatmap);
        }
        if (drawnActor) {
            drawnPolyData->Modified();
            drawnActor->SetVisibility(true);
        }
    }
}

void Simulation::setStreamlineDensity(int resolution) {
    if (streamRake) {
        int r = std::max(2, (int)std::sqrt(resolution));
        streamRake->SetResolution(r, r);
        streamRake->Modified();
    }
}

void Simulation::setRakePosition(float relY) {
    if (streamRake) {
        float renderScale = 40.0f;
        float spacing = renderScale / cachedLbmScale;
        float inletX = (-config.lbmGridNX / 2.0f + 2.0f) * spacing;
        float spreadY = (config.lbmGridNY / 2.0f - 1.0f) * spacing;
        float spreadZ = (config.lbmGridNZ / 2.0f - 1.0f) * spacing;
        
        
        float tightSpread = spreadY * 0.4f; 
        
        
        float maxCenter = spreadY - tightSpread;
        float center = relY * maxCenter;
        
        streamRake->SetOrigin(inletX, center - tightSpread, -spreadZ);
        streamRake->SetPoint1(inletX, center + tightSpread, -spreadZ);
        streamRake->SetPoint2(inletX, center - tightSpread,  spreadZ);
        streamRake->Modified();
    }
}

void Simulation::resetFlow() {
    if (current_sim == 1 && lbmSolver) {
        zweicfd::Flowconditions safeLBM = flow;
        safeLBM.V_inf = flow.V_inf;
        safeLBM.kinematic_viscosity = flow.kinematic_viscosity * (flow.V_inf / std::max(0.0001, flow.V_inf)) * cachedLbmScale;
        lbmSolver->getGridModifiable().initialize(safeLBM);
        totalLbmSteps = 0;
        needsVTKUpdate = true;
    }
}

void Simulation::setColormap(int type) {
    if (!lut) return;
    
    if (type == 0) { 
        lut->SetHueRange(0.667, 0.0);
        lut->SetSaturationRange(1.0, 1.0);
        lut->SetValueRange(1.0, 1.0);
    } else if (type == 1) { 
        lut->SetHueRange(0.0, 0.0);
        lut->SetSaturationRange(0.0, 0.0);
        lut->SetValueRange(0.2, 1.0);
    } else if (type == 2) { 
        lut->SetHueRange(0.8, 0.2); 
        lut->SetSaturationRange(1.0, 1.0);
        lut->SetValueRange(0.5, 1.0);
    } else if (type == 3) { 
        lut->SetHueRange(0.667, 0.0); 
        lut->SetSaturationRange(0.8, 0.8);
        lut->SetValueRange(0.8, 0.8);
    }
    lut->SetTableRange(0.0, 0.15);
    lut->Build();
    
    if (renderWindow) {
        renderWindow->Render();
    }
}
void Simulation::setVisualRotation(double angleDeg) {
    if (airfoilActor) {
        airfoilActor->SetOrigin(4.0, 0.0, 0.0);
        airfoilActor->SetOrientation(0.0, 0.0, angleDeg);
    }
}

void Simulation::fastUpdateRotation(double alpha) {
    if (!lbmSolver) return;
    rotatedFoil = foil;
    rotatedFoil.rotateCoordinates(-alpha);
    
    auto &grid = lbmSolver->getGridModifiable();
    const auto &panels = rotatedFoil.getPanels();
    
    std::vector<double> sdf2D(grid.NX * grid.NY);
    
#pragma omp parallel for collapse(2)
    for (int y = 0; y < grid.NY; ++y) {
      for (int x = 0; x < grid.NX; ++x) {
        float offsetX = -0.15f; 
        double physX = (x - grid.NX / 2.0) / cachedLbmScale - offsetX;
        double physY = (y - grid.NY / 2.0) / cachedLbmScale;
        zweicfd::Point2D p{physX, physY};
        bool inside = false;
        double minDist = 1e9;
        for (const auto &panel : panels) {
          if (((panel.p1.y > p.y) != (panel.p2.y > p.y)) &&
              (p.x < (panel.p2.x - panel.p1.x) * (p.y - panel.p1.y) / (panel.p2.y - panel.p1.y) + panel.p1.x)) {
            inside = !inside;
          }
          double l2 = (panel.p2.x - panel.p1.x) * (panel.p2.x - panel.p1.x) + (panel.p2.y - panel.p1.y) * (panel.p2.y - panel.p1.y);
          if (l2 == 0.0) {
            double d = std::sqrt((p.x - panel.p1.x) * (p.x - panel.p1.x) + (p.y - panel.p1.y) * (p.y - panel.p1.y));
            minDist = std::min(minDist, d);
            continue;
          }
          double t = std::max(0.0, std::min(1.0, ((p.x - panel.p1.x) * (panel.p2.x - panel.p1.x) + (p.y - panel.p1.y) * (panel.p2.y - panel.p1.y)) / l2));
          double projX = panel.p1.x + t * (panel.p2.x - panel.p1.x);
          double projY = panel.p1.y + t * (panel.p2.y - panel.p1.y);
          double d = std::sqrt((p.x - projX) * (p.x - projX) + (p.y - projY) * (p.y - projY));
          minDist = std::min(minDist, d);
        }
        int idx = y * grid.NX + x;
        sdf2D[idx] = inside ? -minDist : minDist;
      }
    }

#pragma omp parallel for collapse(3)
    for (int z = 0; z < grid.NZ; ++z) {
      for (int y = 0; y < grid.NY; ++y) {
        for (int x = 0; x < grid.NX; ++x) {
          int idx2D = y * grid.NX + x;
          double d2D = sdf2D[idx2D];
          double physZ = (z - grid.NZ / 2.0) / cachedLbmScale;
          double spanRadius = 0.5;
          double dZ = std::abs(physZ) - spanRadius;
          double dist3D = (d2D > 0.0 && dZ > 0.0) ? std::sqrt(d2D*d2D + dZ*dZ) : std::max(d2D, dZ);
          
          int scalarIdx = grid.getScalarIndex(x, y, z);
          float airfoil_sdf = (float)(dist3D * cachedLbmScale);
          grid.sdf[scalarIdx] = std::min(airfoil_sdf, grid.drawn_sdf[scalarIdx]);
          
          if (grid.sdf[scalarIdx] > 0.0) {
              if (grid.rho[scalarIdx] < 0.5f) {
                  
                  grid.rho[scalarIdx] = 1.0f;
                  grid.u[scalarIdx] = {(float)flow.V_inf, 0.0f, 0.0f, 0.0f};
                  for (int q = 0; q < 19; ++q) {
                      float cu = zweicfd::D3Q19::cx[q] * flow.V_inf;
                      float u2 = flow.V_inf * flow.V_inf;
                      float feq = zweicfd::D3Q19::w[q] * (1.0f + 3.0f*cu + 4.5f*cu*cu - 1.5f*u2);
                      int idxQ = grid.getIndex(x, y, z, q);
                      grid.f[idxQ] = feq;
                      grid.f_new[idxQ] = feq;
                  }
              }
          }
        }
      }
    }
    needsVTKUpdate = true;
    updateVTKGeometry();
}

void Simulation::clearDrawing() {
    if (!lbmSolver) return;
    
    
    drawnPoints->Initialize();
    drawnPolyData->GetVerts()->Initialize();
    if (auto arr = drawnPolyData->GetPointData()->GetArray("ShapeIndex")) {
        arr->Initialize();
    }
    drawnPolyData->Modified();
    
    
    auto &grid = lbmSolver->getGridModifiable();
    std::fill(grid.drawn_sdf.begin(), grid.drawn_sdf.end(), 1.0f);
    
    
    rebuildSolverWithRotation();
}

void Simulation::addDrawnObstacle(float pX, float pY, float radius) {
    
    if (!lbmSolver) return;
    
    
    float renderScale = 40.0f;
    vtkIdType pid = drawnPoints->InsertNextPoint(pX, pY, 0.0);
    drawnPolyData->GetVerts()->InsertNextCell(1, &pid);
    
    if (auto arr = vtkIntArray::SafeDownCast(drawnPolyData->GetPointData()->GetArray("ShapeIndex"))) {
        arr->InsertNextValue(brushShape);
    }
    
    
    
    
    
    if (!drawnPolyData->GetPointData()->GetArray("ScaleArray")) {
        vtkSmartPointer<vtkFloatArray> scaleArray = vtkSmartPointer<vtkFloatArray>::New();
        scaleArray->SetName("ScaleArray");
        drawnPolyData->GetPointData()->AddArray(scaleArray);
        drawnGlyph->SetScaleModeToScaleByScalar();
        drawnGlyph->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "ScaleArray");
    }
    if (auto sArr = vtkFloatArray::SafeDownCast(drawnPolyData->GetPointData()->GetArray("ScaleArray"))) {
        sArr->InsertNextValue(radius);
    }
    
    drawnPoints->Modified();
    drawnPolyData->GetVerts()->Modified();
    drawnPolyData->Modified();
    
    
    auto &grid = lbmSolver->getGridModifiable();
    float offsetX = -0.15f; 
    
    float physX = pX / renderScale;
    float physY = pY / renderScale;
    
    int cx = (physX + offsetX) * cachedLbmScale + grid.NX / 2.0;
    int cy = physY * cachedLbmScale + grid.NY / 2.0;
    int radCells = radius * cachedLbmScale / renderScale;
    
    for (int z = 0; z < grid.NZ; ++z) {
        for (int y = std::max(0, cy - radCells); y <= std::min(grid.NY-1, cy + radCells); ++y) {
            for (int x = std::max(0, cx - radCells); x <= std::min(grid.NX-1, cx + radCells); ++x) {
                float dist = 0.0f;
                if (brushShape == 0) { 
                    dist = std::sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
                } else if (brushShape == 1) { 
                    dist = std::max(std::abs(x-cx), std::abs(y-cy));
                } else if (brushShape == 2) { 
                    dist = std::abs(x-cx) + std::abs(y-cy);
                }
                
                if (dist <= radCells) {
                    int s_idx = grid.getScalarIndex(x, y, z);
                    grid.drawn_sdf[s_idx] = -1.0f; 
                    grid.sdf[s_idx] = -1.0f;
                }
            }
        }
    }
    needsVTKUpdate = true;
}

} 
