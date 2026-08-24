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
#include <random>
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
#include <vtkCylinderSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkIntArray.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCamera.h>
#include <vtkIdList.h>
#include <vtkPolyDataNormals.h>

#ifdef emit
#undef emit
#include <vtkImplicitPolyDataDistance.h>
#define emit
#else
#include <vtkImplicitPolyDataDistance.h>
#endif

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
    float lbmScale = 32.0f;
    int internalNodes = 0;

    if (rotatedFoil.is3D() && rotatedFoil.getMesh3D()) {
      double bounds[6];
      rotatedFoil.getMesh3D()->GetBounds(bounds);
      cowWidth = std::max(0.1, bounds[1] - bounds[0]);
      cowHeight = std::max(0.1, bounds[3] - bounds[2]);
      double cowDepth = std::max(0.1, bounds[5] - bounds[4]);
      
      float scaleByWidth = (config.lbmGridNX - 4.0f) / (float)cowWidth;
      float scaleByHeight = (config.lbmGridNY - 4.0f) / (float)cowHeight;
      float scaleByDepth = (config.lbmGridNZ - 4.0f) / (float)cowDepth;
      
      lbmScale = std::min({scaleByWidth, scaleByHeight, scaleByDepth});
      lbmScale = std::min(lbmScale, 200.0f);
      cachedLbmScale = lbmScale;

      float offsetX = -0.15f;
      double margin = 6.0 / (double)lbmScale;
      double minBx = bounds[0] - margin;
      double maxBx = bounds[1] + margin;
      double minBy = bounds[2] - margin;
      double maxBy = bounds[3] + margin;
      double minBz = bounds[4] - margin;
      double maxBz = bounds[5] + margin;

      auto implicitDist = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
      implicitDist->SetInput(rotatedFoil.getMesh3D());

      #pragma omp parallel for collapse(3) reduction(+:internalNodes)
      for (int z = 0; z < grid.NZ; ++z) {
        for (int y = 0; y < grid.NY; ++y) {
          for (int x = 0; x < grid.NX; ++x) {
            double physX = (x - grid.NX / 2.0) / lbmScale - offsetX;
            double physY = (y - grid.NY / 2.0) / lbmScale;
            double physZ = (z - grid.NZ / 2.0) / lbmScale;

            double d;
            if (physX < minBx || physX > maxBx ||
                physY < minBy || physY > maxBy ||
                physZ < minBz || physZ > maxBz) {
              double dx = std::max({bounds[0] - physX, 0.0, physX - bounds[1]});
              double dy = std::max({bounds[2] - physY, 0.0, physY - bounds[3]});
              double dz = std::max({bounds[4] - physZ, 0.0, physZ - bounds[5]});
              d = std::sqrt(dx * dx + dy * dy + dz * dz);
            } else {
              double pt[3] = {physX, physY, physZ};
              #pragma omp critical
              {
                d = implicitDist->EvaluateFunction(pt);
              }
            }

            int scalarIdx = grid.getScalarIndex(x, y, z);
            float mesh_sdf = (float)(d * lbmScale);
            grid.sdf[scalarIdx] = std::min(mesh_sdf, grid.drawn_sdf[scalarIdx]);

            if (grid.sdf[scalarIdx] <= 0.0f) internalNodes++;
          }
        }
      }
    } else {
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
      
      float scaleByWidth = 0.35f * config.lbmGridNX / (float)cowWidth;
      float scaleByHeight = 0.45f * config.lbmGridNY / (float)cowHeight;
      lbmScale = std::min(scaleByWidth, scaleByHeight);
      lbmScale = std::max(8.0f, std::min(lbmScale, 64.0f));
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

  #pragma omp parallel for collapse(3) reduction(+:internalNodes)
      for (int z = 0; z < grid.NZ; ++z) {
        for (int y = 0; y < grid.NY; ++y) {
          for (int x = 0; x < grid.NX; ++x) {
            int idx2D = y * grid.NX + x;
            double d2D = sdf2D[idx2D];
            
            double physZ = (z - grid.NZ / 2.0) / lbmScale;
            double spanRadius = (grid.NZ / 2.0) / lbmScale; 
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
    needsVTKUpdate = true;
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
    
    gpuAdvection = std::make_unique<zweicfd::GPUAdvection>();
    float renderScale = 40.0f;
    float spacing = renderScale / cachedLbmScale;
    float spreadX = (config.lbmGridNX / 2.0f) * spacing;
    float spreadY = (config.lbmGridNY / 2.0f) * spacing;
    float spreadZ = (config.lbmGridNZ / 2.0f) * spacing;
    float inletX = (-config.lbmGridNX / 2.0f + 2.0f) * spacing;
    gpuAdvection->setDomainBounds(-spreadX, -spreadY, -spreadZ, spreadX, spreadY, spreadZ);
    gpuAdvection->setInletParams(inletX, 0.0f, spreadY * 1.9f, spreadZ * 1.9f);
    
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
    
    
    streamSeeds = vtkSmartPointer<vtkPolyData>::New();
    updateStreamlineSeeds();

    streamTracer = vtkSmartPointer<vtkStreamTracer>::New();
    streamTracer->SetInputData(velocityField);
    streamTracer->SetSourceData(streamSeeds);
    streamTracer->SetIntegrationStepUnit(vtkStreamTracer::CELL_LENGTH_UNIT);
    streamTracer->SetMaximumPropagation(300.0);
    streamTracer->SetMaximumNumberOfSteps(300);
    streamTracer->SetInitialIntegrationStep(0.5);
    streamTracer->SetMinimumIntegrationStep(0.1);
    streamTracer->SetMaximumIntegrationStep(1.5);
    streamTracer->SetIntegrationDirectionToForward();
    
    jetLut = vtkSmartPointer<vtkLookupTable>::New();
    jetLut->SetNumberOfTableValues(256);
    jetLut->SetTableRange(0.0, 0.15);
    jetLut->SetHueRange(0.667, 0.0);
    jetLut->SetSaturationRange(1.0, 1.0);
    jetLut->SetValueRange(1.0, 1.0);
    jetLut->SetAlphaRange(1.0, 1.0);
    jetLut->Build();

    windTunnelLut = vtkSmartPointer<vtkLookupTable>::New();
    windTunnelLut->SetNumberOfTableValues(256);
    windTunnelLut->SetTableRange(0.0, 0.20);
    for (int i = 0; i < 256; ++i) {
        double t = (double)i / 255.0;
        double alpha = 0.0;
        if (t > 0.04) {
            double ramp = (t - 0.04) / (1.0 - 0.04);
            alpha = std::min(1.0, std::pow(ramp, 1.2));
        }
        double val = 0.6 + 0.4 * std::min(1.0, t * 2.0);
        windTunnelLut->SetTableValue(i, val, val, val, alpha);
    }

    neonLut = vtkSmartPointer<vtkLookupTable>::New();
    neonLut->SetNumberOfTableValues(256);
    neonLut->SetTableRange(0.0, 0.15);
    neonLut->SetHueRange(0.8, 0.2);
    neonLut->SetSaturationRange(1.0, 1.0);
    neonLut->SetValueRange(0.8, 1.0);
    neonLut->SetAlphaRange(1.0, 1.0);
    neonLut->Build();

    thermalLut = vtkSmartPointer<vtkLookupTable>::New();
    thermalLut->SetNumberOfTableValues(256);
    thermalLut->SetTableRange(0.0, 0.15);
    thermalLut->SetHueRange(0.15, 0.0);
    thermalLut->SetSaturationRange(1.0, 1.0);
    thermalLut->SetValueRange(0.2, 1.0);
    thermalLut->SetAlphaRange(1.0, 1.0);
    thermalLut->Build();

    lut = jetLut;

    streamMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    streamMapper->SetInputData(streamTracer->GetOutput());
    streamMapper->SetScalarModeToUsePointFieldData();
    streamMapper->SelectColorArray("Speed");
    streamMapper->SetScalarRange(0.0, 0.15);
    streamMapper->SetLookupTable(jetLut);
    
    streamActor = vtkSmartPointer<vtkActor>::New();
    streamActor->SetMapper(streamMapper);
    streamActor->GetProperty()->SetLineWidth(1.5);
    streamActor->GetProperty()->SetOpacity(0.75);
    
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
    heatmapSlice->SetPosition(0.0, 0.0, 0.0);
    renderer->AddViewProp(heatmapSlice);

    rakeSource = vtkSmartPointer<vtkLineSource>::New();
    rakeSource->SetPoint1(0, -1, 0);
    rakeSource->SetPoint2(0, 1, 0);

    rakeTube = vtkSmartPointer<vtkTubeFilter>::New();
    rakeTube->SetInputConnection(rakeSource->GetOutputPort());
    rakeTube->SetRadius(0.3);
    rakeTube->SetNumberOfSides(16);

    rakeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    rakeMapper->SetInputConnection(rakeTube->GetOutputPort());

    rakeActor = vtkSmartPointer<vtkActor>::New();
    rakeActor->SetMapper(rakeMapper);
    rakeActor->GetProperty()->SetColor(1.0, 0.3, 0.3);
    rakeActor->GetProperty()->SetOpacity(0.5);
    rakeActor->SetVisibility(false);
    renderer->AddActor(rakeActor);
    
    vtkSmartPointer<vtkCylinderSource> circleSource = vtkSmartPointer<vtkCylinderSource>::New();
    circleSource->SetRadius(1.0);
    circleSource->SetHeight(1.0);
    circleSource->SetResolution(24);
    circleSource->SetCapping(true);
    vtkSmartPointer<vtkTransform> circleTrans = vtkSmartPointer<vtkTransform>::New();
    circleTrans->RotateX(90);
    vtkSmartPointer<vtkTransformPolyDataFilter> circleFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    circleFilter->SetTransform(circleTrans);
    circleFilter->SetInputConnection(circleSource->GetOutputPort());

    vtkSmartPointer<vtkCubeSource> squareSource = vtkSmartPointer<vtkCubeSource>::New();
    squareSource->SetXLength(2.0);
    squareSource->SetYLength(2.0);
    squareSource->SetZLength(1.0);

    vtkSmartPointer<vtkCylinderSource> diamondSource = vtkSmartPointer<vtkCylinderSource>::New();
    diamondSource->SetRadius(1.4142);
    diamondSource->SetHeight(1.0);
    diamondSource->SetResolution(4);
    diamondSource->SetCapping(true);
    vtkSmartPointer<vtkTransform> diamondTrans = vtkSmartPointer<vtkTransform>::New();
    diamondTrans->RotateX(90);
    diamondTrans->RotateZ(45);
    vtkSmartPointer<vtkTransformPolyDataFilter> diamondFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    diamondFilter->SetTransform(diamondTrans);
    diamondFilter->SetInputConnection(diamondSource->GetOutputPort());

    for (int s = 0; s < 3; ++s) {
        drawnPoints[s] = vtkSmartPointer<vtkPoints>::New();
        drawnPolyData[s] = vtkSmartPointer<vtkPolyData>::New();
        drawnPolyData[s]->SetPoints(drawnPoints[s]);

        vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
        drawnPolyData[s]->SetVerts(vertices);

        drawnScaleArray[s] = vtkSmartPointer<vtkFloatArray>::New();
        drawnScaleArray[s]->SetName("ScaleArray");
        drawnPolyData[s]->GetPointData()->AddArray(drawnScaleArray[s]);
        drawnPolyData[s]->GetPointData()->SetActiveScalars("ScaleArray");

        drawnGlyph[s] = vtkSmartPointer<vtkGlyph3D>::New();
        drawnGlyph[s]->SetInputData(drawnPolyData[s]);
        if (s == 0) {
            drawnGlyph[s]->SetSourceConnection(circleFilter->GetOutputPort());
        } else if (s == 1) {
            drawnGlyph[s]->SetSourceConnection(squareSource->GetOutputPort());
        } else {
            drawnGlyph[s]->SetSourceConnection(diamondFilter->GetOutputPort());
        }
        drawnGlyph[s]->SetScaleModeToScaleByScalar();
        drawnGlyph[s]->SetScaleFactor(1.0);

        vtkSmartPointer<vtkPolyDataMapper> drawnMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        drawnMapper->SetInputConnection(drawnGlyph[s]->GetOutputPort());
        drawnMapper->ScalarVisibilityOff();

        drawnActor[s] = vtkSmartPointer<vtkActor>::New();
        drawnActor[s]->SetMapper(drawnMapper);
        drawnActor[s]->GetProperty()->SetColor(0.8, 0.8, 0.8);
        renderer->AddActor(drawnActor[s]);
    }
    
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
        airfoilActor->GetProperty()->SetColor(0.92, 0.94, 0.98);
        airfoilActor->GetProperty()->SetAmbient(0.35);
        airfoilActor->GetProperty()->SetDiffuse(0.80);
        airfoilActor->GetProperty()->SetSpecular(0.60);
        airfoilActor->GetProperty()->SetSpecularPower(30.0);
        
        renderer->AddActor(airfoilActor);
    }
    if (airfoilActor) {
        airfoilActor->SetOrientation(0.0, 0.0, 0.0);
    }
    
    if (rotatedFoil.is3D() && rotatedFoil.getMesh3D()) {
        float renderScale = 40.0f;
        float offsetX = -0.15f;
        auto transform = vtkSmartPointer<vtkTransform>::New();
        transform->Scale(renderScale, renderScale, renderScale);
        transform->Translate(offsetX, 0.0, 0.0);

        auto tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        tf->SetInputData(rotatedFoil.getMesh3D());
        tf->SetTransform(transform);
        tf->Update();

        airfoilPolyData->DeepCopy(tf->GetOutput());
    } else {
        const auto &panels = rotatedFoil.getPanels();
        if (!panels.empty()) {
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
            
            float renderScale = 40.0f;
            float physicalSpan = (float)config.lbmGridNZ / cachedLbmScale;
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
    
    auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals->SetInputData(airfoilPolyData);
    normals->ComputePointNormalsOn();
    normals->ComputeCellNormalsOff();
    normals->SplittingOff();
    normals->Update();
    airfoilPolyData->DeepCopy(normals->GetOutput());

    if (velocityField) {
        float renderScale = 40.0f;
        float spacing = renderScale / cachedLbmScale;
        velocityField->SetSpacing(spacing, spacing, spacing);
        velocityField->SetOrigin(
            (-config.lbmGridNX / 2.0f) * spacing,
            (-config.lbmGridNY / 2.0f) * spacing,
            (-config.lbmGridNZ / 2.0f) * spacing
        );
        velocityField->Modified();
    }
    if (gpuAdvection) {
        float renderScale = 40.0f;
        float spacing = renderScale / cachedLbmScale;
        float spreadX = (config.lbmGridNX / 2.0f) * spacing;
        float spreadY = (config.lbmGridNY / 2.0f) * spacing;
        float spreadZ = (config.lbmGridNZ / 2.0f) * spacing;
        float inletX = (-config.lbmGridNX / 2.0f + 2.0f) * spacing;
        gpuAdvection->setDomainBounds(-spreadX, -spreadY, -spreadZ, spreadX, spreadY, spreadZ);
        gpuAdvection->setInletParams(inletX, 0.0f, spreadY * 1.9f, spreadZ * 1.9f);
    }
    updateStreamlineSeeds();
}

void Simulation::stepSimulation() {
    if (isRebuilding) return;
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
        
        
        if (!freezeFlow && showParticles && streamActor) {
            streamActor->SetVisibility(true);
            
            
            if (totalLbmSteps % vtkUpdateFrequency == 0 || needsVTKUpdate) {
                const auto& grid = lbmSolver->getGrid();
                
                int totalCells = grid.NX * grid.NY * grid.NZ;
                float* vtkData = velocityArray->WritePointer(0, totalCells * 3);
                float* speedData = speedArray->WritePointer(0, totalCells);
                
                #pragma omp parallel for schedule(static)
                for(int i = 0; i < totalCells; ++i) {
                    float vx = grid.u[i].x;
                    float vy = grid.u[i].y;
                    float vz = grid.u[i].z;
                    
                    vtkData[i*3 + 0] = vx;
                    vtkData[i*3 + 1] = vy;
                    vtkData[i*3 + 2] = vz;
                    
                    speedData[i] = sqrtf(vx*vx + vy*vy + vz*vz);
                }
                
                speedArray->Modified();
                velocityArray->Modified();
                velocityField->Modified();
                if (volumeMapper) {
                    volumeMapper->Modified();
                }
                
                streamTracer->Update();
                vtkSmartPointer<vtkPolyData> tracerOutput = streamTracer->GetOutput();
                
                if (currentTheme == 1) {
                    vtkDataArray* velArr = tracerOutput ? tracerOutput->GetPointData()->GetArray("Velocity") : nullptr;
                    vtkDataArray* spdArr = tracerOutput ? tracerOutput->GetPointData()->GetArray("Speed") : nullptr;
                    
                    if (tracerOutput && tracerOutput->GetNumberOfCells() > 0 && (velArr || spdArr)) {
                        vtkSmartPointer<vtkCellArray> filteredLines = vtkSmartPointer<vtkCellArray>::New();
                        vtkSmartPointer<vtkIdList> ptIds = vtkSmartPointer<vtkIdList>::New();
                        vtkIdType numCells = tracerOutput->GetNumberOfCells();
                        
                        vtkSmartPointer<vtkFloatArray> devArr = vtkSmartPointer<vtkFloatArray>::New();
                        devArr->SetName("Deviation");
                        devArr->SetNumberOfTuples(tracerOutput->GetNumberOfPoints());
                        for(vtkIdType i = 0; i < tracerOutput->GetNumberOfPoints(); ++i) {
                            devArr->SetValue(i, 0.0f);
                        }
                        
                        std::vector<float> pert;
                        std::vector<bool> active;
                        std::vector<bool> buf;
                        
                        for (vtkIdType c = 0; c < numCells; ++c) {
                            tracerOutput->GetCellPoints(c, ptIds);
                            vtkIdType npts = ptIds->GetNumberOfIds();
                            if (npts < 2) continue;
                            
                            pert.assign(npts, 0.0f);
                            float maxPert = 0.0f;
                            for (vtkIdType i = 0; i < npts; ++i) {
                                vtkIdType pid = ptIds->GetId(i);
                                float pVal = 0.0f;
                                if (velArr) {
                                    double v[3];
                                    velArr->GetTuple(pid, v);
                                    double speed = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
                                    if (speed > 1e-6) {
                                        double trans = std::sqrt(v[1]*v[1] + v[2]*v[2]);
                                        pVal = (float)(trans / speed);
                                    }
                                }
                                
                                if (pVal < 0.02f) pVal = 0.0f;
                                pert[i] = pVal;
                                devArr->SetValue(pid, pVal);
                                if (pVal > maxPert) maxPert = pVal;
                            }
                            
                            bool linePerturbed = (maxPert >= 0.02f);
                            
                            if (!linePerturbed) {
                                continue;
                            }
                            
                            const float segThr = 0.015f;
                            active.assign(npts, false);
                                for (vtkIdType i = 0; i < npts; ++i) {
                                    if (pert[i] >= segThr) active[i] = true;
                                }
                                buf = active;
                                for (vtkIdType i = 0; i < npts; ++i) {
                                    if (active[i]) {
                                        for (int k = -12; k <= 25; ++k) {
                                            vtkIdType idx = i + k;
                                            if (idx >= 0 && idx < npts) buf[idx] = true;
                                        }
                                    }
                                }
                                vtkSmartPointer<vtkIdList> seg = vtkSmartPointer<vtkIdList>::New();
                                for (vtkIdType i = 0; i < npts; ++i) {
                                    if (buf[i]) {
                                        seg->InsertNextId(ptIds->GetId(i));
                                    } else {
                                        if (seg->GetNumberOfIds() >= 2) {
                                            filteredLines->InsertNextCell(seg);
                                        }
                                        seg = vtkSmartPointer<vtkIdList>::New();
                                    }
                                }
                                if (seg->GetNumberOfIds() >= 2) {
                                    filteredLines->InsertNextCell(seg);
                                }
                            }
                        
                        tracerOutput->GetPointData()->AddArray(devArr);
                        if (currentTheme == 1) {
                            tracerOutput->GetPointData()->SetActiveScalars("Deviation");
                        } else {
                            tracerOutput->GetPointData()->SetActiveScalars("Speed");
                        }
                        
                        vtkSmartPointer<vtkPolyData> filteredData = vtkSmartPointer<vtkPolyData>::New();
                        filteredData->SetPoints(tracerOutput->GetPoints());
                        filteredData->GetPointData()->PassData(tracerOutput->GetPointData());
                        if (currentTheme == 1) {
                            filteredData->GetPointData()->SetActiveScalars("Deviation");
                        } else {
                            filteredData->GetPointData()->SetActiveScalars("Speed");
                        }
                        filteredData->SetLines(filteredLines);
                        filteredData->Modified();
                        
                        streamMapper->SetInputData(filteredData);
                    }
                } else {
                    if (tracerOutput && tracerOutput->GetPointData()) {
                        tracerOutput->GetPointData()->SetActiveScalars("Speed");
                    }
                    streamMapper->SetInputData(tracerOutput);
                }
                streamMapper->Modified();
                
                if (gpuAdvection) {
                    if (!gpuAdvection->isReady()) {
                        gpuAdvection->initialize(65536);
                    }
                    if (gpuAdvection->isReady()) {
                        gpuAdvection->updateVelocityField(vtkData, config.lbmGridNX, config.lbmGridNY, config.lbmGridNZ);
                        gpuAdvection->stepAdvection(0.02f);
                    }
                }
                
                needsVTKUpdate = false;
            }
        } else if (streamActor) {
            streamActor->SetVisibility(false);
        }
        
        if (heatmapSlice) {
            heatmapSlice->SetVisibility(showHeatmap);
        }
        for (int s = 0; s < 3; ++s) {
            if (drawnActor[s]) {
                drawnActor[s]->SetVisibility(true);
            }
        }
    }
}

void Simulation::setStreamlineDensity(int resolution) {
    streamlineDensity = resolution;
    updateStreamlineSeeds();
}

void Simulation::setRakePosition(float relY) {
    rakeRelY = relY;
    updateStreamlineSeeds();
}

void Simulation::updateStreamlineSeeds() {
    // added Poisson-Disk sampling so lines wont travel very close, causing performance gains 
    if (!streamSeeds) return;

    float renderScale = 40.0f;
    float spacing = renderScale / cachedLbmScale;
    float inletX = (-config.lbmGridNX / 2.0f + 2.0f) * spacing;
    float spreadY = (config.lbmGridNY / 2.0f - 1.0f) * spacing;
    float spreadZ = (config.lbmGridNZ / 2.0f - 1.0f) * spacing;
    
    float tightSpreadY = spreadY * 0.95f;
    float maxCenterY = std::max(0.0f, spreadY - tightSpreadY);
    float centerY = rakeRelY * maxCenterY;

    int targetN = std::max(5, streamlineDensity);
    float r_min = std::sqrt(1.0f / (targetN * 1.3f));
    float cellSize = r_min / 1.41421356f;
    int gridW = std::max(1, (int)std::ceil(1.0f / cellSize));
    int gridH = std::max(1, (int)std::ceil(1.0f / cellSize));

    std::vector<int> grid(gridW * gridH, -1);
    std::vector<std::pair<float, float>> samplePoints;
    std::vector<int> activeList;

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    samplePoints.push_back({0.5f, 0.5f});
    activeList.push_back(0);
    int initGx = std::min(gridW - 1, (int)(0.5f / cellSize));
    int initGy = std::min(gridH - 1, (int)(0.5f / cellSize));
    grid[initGy * gridW + initGx] = 0;

    while (!activeList.empty() && (int)samplePoints.size() < targetN) {
        int randIdx = (int)(dist01(rng) * activeList.size());
        int pointIdx = activeList[randIdx];
        auto [px, py] = samplePoints[pointIdx];
        bool found = false;

        for (int attempt = 0; attempt < 30; ++attempt) {
            float angle = dist01(rng) * 6.2831853f;
            float radius = r_min * (1.0f + dist01(rng));
            float cx = px + radius * std::cos(angle);
            float cy = py + radius * std::sin(angle);

            if (cx < 0.0f || cx > 1.0f || cy < 0.0f || cy > 1.0f) continue;

            int cgx = (int)(cx / cellSize);
            int cgy = (int)(cy / cellSize);
            bool valid = true;

            for (int dy = -2; dy <= 2 && valid; ++dy) {
                for (int dx = -2; dx <= 2 && valid; ++dx) {
                    int nx = cgx + dx;
                    int ny = cgy + dy;
                    if (nx >= 0 && nx < gridW && ny >= 0 && ny < gridH) {
                        int neighborIdx = grid[ny * gridW + nx];
                        if (neighborIdx != -1) {
                            float diffX = cx - samplePoints[neighborIdx].first;
                            float diffY = cy - samplePoints[neighborIdx].second;
                            if (diffX * diffX + diffY * diffY < r_min * r_min) {
                                valid = false;
                            }
                        }
                    }
                }
            }

            if (valid) {
                int newIdx = (int)samplePoints.size();
                samplePoints.push_back({cx, cy});
                activeList.push_back(newIdx);
                grid[cgy * gridW + cgx] = newIdx;
                found = true;
                if ((int)samplePoints.size() >= targetN) break;
            }
        }

        if (!found) {
            activeList[randIdx] = activeList.back();
            activeList.pop_back();
        }
    }

    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    pts->SetNumberOfPoints((vtkIdType)samplePoints.size());

    for (size_t i = 0; i < samplePoints.size(); ++i) {
        float u = samplePoints[i].first;
        float v = samplePoints[i].second;

        float uNorm = 2.0f * (u - 0.5f);
        float uWarped = 0.5f + 0.5f * (0.7f * uNorm + 0.3f * uNorm * uNorm * uNorm);

        float px = inletX;
        float py = (centerY - tightSpreadY) + uWarped * (2.0f * tightSpreadY);
        float pz = -spreadZ + v * (2.0f * spreadZ);
        pts->SetPoint((vtkIdType)i, px, py, pz);
    }

    streamSeeds->SetPoints(pts);
    streamSeeds->Modified();

    if (rakeSource) {
        rakeSource->SetPoint1(inletX, centerY - tightSpreadY, -spreadZ);
        rakeSource->SetPoint2(inletX, centerY + tightSpreadY, spreadZ);
        rakeSource->Modified();
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
    currentTheme = type;
    needsVTKUpdate = true;
    
    if (type == 1) { 
        lut = windTunnelLut;
        if (streamMapper) {
            streamMapper->SetLookupTable(windTunnelLut);
            streamMapper->SelectColorArray("Deviation");
            streamMapper->SetScalarRange(0.0, 0.20);
        }
        if (streamActor) {
            streamActor->GetProperty()->SetOpacity(1.0);
        }
    } else if (type == 0) { 
        lut = jetLut;
        if (streamMapper) {
            streamMapper->SetLookupTable(jetLut);
            streamMapper->SelectColorArray("Speed");
            streamMapper->SetScalarRange(0.0, 0.15);
        }
        if (streamActor) {
            streamActor->GetProperty()->SetOpacity(0.75);
        }
    } else if (type == 2) { 
        lut = neonLut;
        if (streamMapper) {
            streamMapper->SetLookupTable(neonLut);
            streamMapper->SelectColorArray("Speed");
            streamMapper->SetScalarRange(0.0, 0.15);
        }
        if (streamActor) {
            streamActor->GetProperty()->SetOpacity(0.80);
        }
    } else if (type == 3) { 
        lut = thermalLut;
        if (streamMapper) {
            streamMapper->SetLookupTable(thermalLut);
            streamMapper->SelectColorArray("Speed");
            streamMapper->SetScalarRange(0.0, 0.15);
        }
        if (streamActor) {
            streamActor->GetProperty()->SetOpacity(0.80);
        }
    }

    if (heatmapSlice) {
        heatmapSlice->GetProperty()->SetLookupTable(lut);
    }
    
    if (renderWindow) {
        renderWindow->Render();
    }
}
void Simulation::setVisualRotation(double rx, double ry, double rz) {
    if (airfoilActor) {
        airfoilActor->SetOrigin(4.0, 0.0, 0.0);
        airfoilActor->SetOrientation(rx, ry, rz);
    }
}

void Simulation::fastUpdateRotation(double alpha) {
    if (!lbmSolver) return;
    
    flow.alpha = alpha;
    setVisualRotation(0.0, 0.0, -alpha);
    
    rotatedFoil = foil;
    rotatedFoil.rotateCoordinates(-alpha);
    
    auto &grid = lbmSolver->getGridModifiable();

    if (rotatedFoil.is3D() && rotatedFoil.getMesh3D()) {
      double bounds[6];
      rotatedFoil.getMesh3D()->GetBounds(bounds);
      float offsetX = -0.15f;
      double margin = 6.0 / (double)cachedLbmScale;
      double minBx = bounds[0] - margin;
      double maxBx = bounds[1] + margin;
      double minBy = bounds[2] - margin;
      double maxBy = bounds[3] + margin;
      double minBz = bounds[4] - margin;
      double maxBz = bounds[5] + margin;

      auto implicitDist = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
      implicitDist->SetInput(rotatedFoil.getMesh3D());

      #pragma omp parallel for collapse(3)
      for (int z = 0; z < grid.NZ; ++z) {
        for (int y = 0; y < grid.NY; ++y) {
          for (int x = 0; x < grid.NX; ++x) {
            double physX = (x - grid.NX / 2.0) / cachedLbmScale - offsetX;
            double physY = (y - grid.NY / 2.0) / cachedLbmScale;
            double physZ = (z - grid.NZ / 2.0) / cachedLbmScale;

            double d;
            if (physX < minBx || physX > maxBx ||
                physY < minBy || physY > maxBy ||
                physZ < minBz || physZ > maxBz) {
              double dx = std::max({bounds[0] - physX, 0.0, physX - bounds[1]});
              double dy = std::max({bounds[2] - physY, 0.0, physY - bounds[3]});
              double dz = std::max({bounds[4] - physZ, 0.0, physZ - bounds[5]});
              d = std::sqrt(dx * dx + dy * dy + dz * dz);
            } else {
              double pt[3] = {physX, physY, physZ};
              #pragma omp critical
              {
                d = implicitDist->EvaluateFunction(pt);
              }
            }

            int scalarIdx = grid.getScalarIndex(x, y, z);
            float mesh_sdf = (float)(d * cachedLbmScale);
            grid.sdf[scalarIdx] = std::min(mesh_sdf, grid.drawn_sdf[scalarIdx]);

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
    } else {
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
            double spanRadius = (grid.NZ / 2.0) / cachedLbmScale;
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
    }
    needsVTKUpdate = true;
    updateVTKGeometry();
}

void Simulation::clearDrawing() {
    if (!lbmSolver) return;
    
    for (int s = 0; s < 3; ++s) {
        if (drawnPoints[s]) drawnPoints[s]->Initialize();
        if (drawnPolyData[s]) {
            if (drawnPolyData[s]->GetVerts()) drawnPolyData[s]->GetVerts()->Initialize();
            if (drawnScaleArray[s]) drawnScaleArray[s]->Initialize();
            drawnPolyData[s]->Modified();
        }
    }
    
    auto &grid = lbmSolver->getGridModifiable();
    std::fill(grid.drawn_sdf.begin(), grid.drawn_sdf.end(), 1.0f);
    
    rebuildSolverWithRotation();
}

void Simulation::addDrawnObstacle(float pX, float pY, float radius) {
    if (!lbmSolver) return;
    
    float renderScale = 40.0f;
    float offsetX = -0.15f; 
    float physX = pX / renderScale;
    float physY = pY / renderScale;
    auto &grid = lbmSolver->getGridModifiable();
    int cx = (physX + offsetX) * cachedLbmScale + grid.NX / 2.0;
    int cy = physY * cachedLbmScale + grid.NY / 2.0;
    int radCells = radius * cachedLbmScale / renderScale;
    
    if (isEraser) {
        for (int z = 0; z < grid.NZ; ++z) {
            for (int y = std::max(0, cy - radCells); y <= std::min(grid.NY-1, cy + radCells); ++y) {
                for (int x = std::max(0, cx - radCells); x <= std::min(grid.NX-1, cx + radCells); ++x) {
                    float dist = std::sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
                    if (dist <= radCells) {
                        int s_idx = grid.getScalarIndex(x, y, z);
                        grid.drawn_sdf[s_idx] = 1.0f;
                    }
                }
            }
        }
        rebuildSolverWithRotation();
        return;
    }
    
    int shape = std::clamp(brushShape, 0, 2);
    if (drawnPoints[shape] && drawnPolyData[shape]) {
        vtkIdType pid = drawnPoints[shape]->InsertNextPoint(pX, pY, 0.0);
        drawnPolyData[shape]->GetVerts()->InsertNextCell(1, &pid);
        if (drawnScaleArray[shape]) {
            drawnScaleArray[shape]->InsertNextValue(radius);
        }
        drawnPoints[shape]->Modified();
        drawnPolyData[shape]->GetVerts()->Modified();
        drawnPolyData[shape]->Modified();
    }
    
    for (int z = 0; z < grid.NZ; ++z) {
        for (int y = std::max(0, cy - radCells); y <= std::min(grid.NY-1, cy + radCells); ++y) {
            for (int x = std::max(0, cx - radCells); x <= std::min(grid.NX-1, cx + radCells); ++x) {
                float dist = 0.0f;
                if (shape == 0) { 
                    dist = std::sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
                } else if (shape == 1) { 
                    dist = std::max(std::abs(x-cx), std::abs(y-cy));
                } else if (shape == 2) { 
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

void Simulation::panCamera(double dx, double dy) {
    if (!renderer) return;
    vtkCamera* cam = renderer->GetActiveCamera();
    if (!cam) return;
    double pos[3], fp[3];
    cam->GetPosition(pos);
    cam->GetFocalPoint(fp);
    pos[0] += dx;
    pos[1] += dy;
    fp[0] += dx;
    fp[1] += dy;
    cam->SetPosition(pos);
    cam->SetFocalPoint(fp);
    if (renderWindow) renderWindow->Render();
}

void Simulation::rotateCamera(double dAzimuth, double dElevation) {
    if (!renderer) return;
    vtkCamera* cam = renderer->GetActiveCamera();
    if (!cam) return;
    cam->Azimuth(dAzimuth);
    cam->Elevation(dElevation);
    cam->OrthogonalizeViewUp();
    renderer->ResetCameraClippingRange();
    if (renderWindow) renderWindow->Render();
}

void Simulation::zoomCamera(double factor) {
    if (!renderer) return;
    vtkCamera* cam = renderer->GetActiveCamera();
    if (!cam) return;
    cam->Dolly(factor);
    renderer->ResetCameraClippingRange();
    if (renderWindow) renderWindow->Render();
}

void Simulation::resetCameraView() {
    if (!renderer) return;
    vtkCamera* cam = renderer->GetActiveCamera();
    if (!cam) return;
    cam->SetFocalPoint(0.0, 0.0, 0.0);
    cam->SetPosition(0.0, 0.0, 1.0);
    cam->SetViewUp(0.0, 1.0, 0.0);
    renderer->ResetCamera();
    if (renderWindow) renderWindow->Render();
}

void Simulation::setLineWidth(float width) {
    if (streamActor) {
        streamActor->GetProperty()->SetLineWidth(std::max(1.0f, width));
        if (renderWindow) renderWindow->Render();
    }
}

float Simulation::getLineWidth() const {
    if (streamActor) {
        return streamActor->GetProperty()->GetLineWidth();
    }
    return 2.0f;
}

} 
