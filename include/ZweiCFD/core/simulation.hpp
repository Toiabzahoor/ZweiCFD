#pragma once

#include <string>
#include <memory>
#include <vector>

#include "ZweiCFD/solver/airfoil.hpp"
#include "ZweiCFD/solver/solver.hpp"
#include "ZweiCFD/solver/lbm_solver.hpp"
#include "ZweiCFD/core/config.hpp"

#include <vtkSmartPointer.h>
#include <vtkGlyph3D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkImageSlice.h>
#include <vtkImageSliceMapper.h>
#include <vtkStreamTracer.h>
#include <vtkTubeFilter.h>
#include <vtkPointSource.h>
#include <vtkPlaneSource.h>
#include <vtkLineSource.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkPolygon.h>

namespace zweicfd {

class Simulation {
public:
  Simulation(int argc, char *argv[]);
  ~Simulation();

  void rebuildSolverWithRotation();
  void setupVTKWithWindow(vtkRenderWindow* window);
  void updateVTKGeometry();
  void setStreamlineDensity(int resolution);
  void setRakePosition(float relY);
  void stepSimulation();
  void resetFlow();
  void setColormap(int type);
  
  void fastUpdateRotation(double alpha);
  void setVisualRotation(double angleDeg);
  void addDrawnObstacle(float x, float y, float radius);
  void clearDrawing();
  
  vtkRenderWindow* getRenderWindow() const { return renderWindow; }

  zweicfd::Config config;
  zweicfd::Airfoil foil;
  zweicfd::Airfoil rotatedFoil;
  zweicfd::Flowconditions flow;

  std::unique_ptr<zweicfd::Solver> solver;
  std::unique_ptr<zweicfd::LBMSolver> lbmSolver;

  int current_sim = 1;
  bool freezeFlow = false;
  float cachedLbmScale = 1.0f;
  float cowWidth = 1.0f;
  float cowHeight = 1.0f;
  
  
  int stepsPerFrame = 2;
  int vtkUpdateFrequency = 5;
  int totalLbmSteps = 0;
  
  bool showParticles = true;
  bool showHeatmap = false;
  bool drawMode = false;
  float brushSize = 3.0f;
  int brushShape = 0; 
  bool flapping = false;
  double flapTimer = 0.0;
  
  zweicfd::Coefficients results;

private:
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;
  
  vtkSmartPointer<vtkPolyData> airfoilPolyData;
  vtkSmartPointer<vtkActor> airfoilActor;
  
  vtkSmartPointer<vtkImageData> velocityField;
  vtkSmartPointer<vtkFloatArray> velocityArray;
  vtkSmartPointer<vtkFloatArray> speedArray;
  vtkSmartPointer<vtkPlaneSource> streamRake;
  vtkSmartPointer<vtkStreamTracer> streamTracer;
  vtkSmartPointer<vtkActor> streamActor;
  
  vtkSmartPointer<vtkImageSliceMapper> heatmapMapper;
  vtkSmartPointer<vtkImageSlice> heatmapSlice;
  
  vtkSmartPointer<vtkPoints> drawnPoints;
  vtkSmartPointer<vtkPolyData> drawnPolyData;
  vtkSmartPointer<vtkActor> drawnActor;
  vtkSmartPointer<vtkGlyph3D> drawnGlyph;
  
  vtkSmartPointer<vtkLookupTable> lut;
  int frameCounter = 0;
  bool needsVTKUpdate = true;
};

} 
