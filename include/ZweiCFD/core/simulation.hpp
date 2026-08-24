#pragma once

#include <string>
#include <memory>
#include <vector>
#include <atomic>

#include "ZweiCFD/solver/airfoil.hpp"
#include "ZweiCFD/solver/solver.hpp"
#include "ZweiCFD/solver/lbm_solver.hpp"
#include "ZweiCFD/core/config.hpp"
#include "ZweiCFD/render/gpu_advection.hpp"

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
#include <vtkImageSliceMapper.h>
#include <vtkImageSlice.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkVolumeProperty.h>
#include <vtkVolume.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
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
  void setVisualRotation(double rx, double ry, double rz);
  void addDrawnObstacle(float x, float y, float radius);
  void clearDrawing();
  void panCamera(double dx, double dy);
  void rotateCamera(double dAzimuth, double dElevation);
  void zoomCamera(double factor);
  void resetCameraView();
  void setLineWidth(float width);
  float getLineWidth() const;
  
  vtkRenderWindow* getRenderWindow() const { return renderWindow; }

  zweicfd::Config config;
  zweicfd::Airfoil foil;
  zweicfd::Airfoil rotatedFoil;
  zweicfd::Flowconditions flow;

  std::unique_ptr<zweicfd::Solver> solver;
  std::unique_ptr<zweicfd::LBMSolver> lbmSolver;
  std::unique_ptr<zweicfd::GPUAdvection> gpuAdvection;

  int current_sim = 1;
  bool freezeFlow = false;
  float cachedLbmScale = 1.0f;
  float cowWidth = 1.0f;
  float cowHeight = 1.0f;
  
  
  int stepsPerFrame = 1;
  int vtkUpdateFrequency = 1;
  int totalLbmSteps = 0;
  
  bool showParticles = true;
  bool showHeatmap = false;
  bool drawMode = false;
  bool isEraser = false;
  float brushSize = 3.0f;
  int brushShape = 0; 
  bool flapping = false;
  double flapTimer = 0.0;
  
  void updateStreamlineSeeds();

  zweicfd::Coefficients results;
  bool needsVTKUpdate = true;
  int currentTheme = 0;
  std::atomic<bool> isRebuilding{false};

private:
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;
  
  vtkSmartPointer<vtkPolyData> airfoilPolyData;
  vtkSmartPointer<vtkActor> airfoilActor;
  
  vtkSmartPointer<vtkImageData> velocityField;
  vtkSmartPointer<vtkFloatArray> velocityArray;
  vtkSmartPointer<vtkFloatArray> speedArray;
  vtkSmartPointer<vtkPolyData> streamSeeds;
  float rakeRelY = 0.0f;
  int streamlineDensity = 100;
  vtkSmartPointer<vtkLineSource> rakeSource;
  vtkSmartPointer<vtkTubeFilter> rakeTube;
  vtkSmartPointer<vtkPolyDataMapper> rakeMapper;
  vtkSmartPointer<vtkActor> rakeActor;

  vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper;
  vtkSmartPointer<vtkVolumeProperty> volumeProperty;
  vtkSmartPointer<vtkVolume> volumeActor;
  
  vtkSmartPointer<vtkImageSliceMapper> heatmapMapper;
  vtkSmartPointer<vtkImageSlice> heatmapSlice;
  vtkSmartPointer<vtkStreamTracer> streamTracer;
  vtkSmartPointer<vtkPolyDataMapper> streamMapper;
  vtkSmartPointer<vtkActor> streamActor;
  
  vtkSmartPointer<vtkPoints> drawnPoints[3];
  vtkSmartPointer<vtkPolyData> drawnPolyData[3];
  vtkSmartPointer<vtkFloatArray> drawnScaleArray[3];
  vtkSmartPointer<vtkActor> drawnActor[3];
  vtkSmartPointer<vtkGlyph3D> drawnGlyph[3];
  
  vtkSmartPointer<vtkLookupTable> lut;
  vtkSmartPointer<vtkLookupTable> jetLut;
  vtkSmartPointer<vtkLookupTable> windTunnelLut;
  vtkSmartPointer<vtkLookupTable> neonLut;
  vtkSmartPointer<vtkLookupTable> thermalLut;
  int frameCounter = 0;
};

} 
