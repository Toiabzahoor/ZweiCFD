#pragma once

#include <string>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

namespace zweicfd {

struct Point2D {
  double x;
  double y;
};
struct Panel {
  Point2D p1; 
  Point2D p2; 
  Point2D cp; 
  double length;
  double theta;
  Point2D normal;
  Point2D tangent;
};
class Airfoil {
public:
  Airfoil();
  
  Airfoil(const Airfoil& other) {
      name = other.name;
      coordinates = other.coordinates;
      panels = other.panels;
      is3DModel = other.is3DModel;
      if (other.mesh3D) {
          mesh3D = vtkSmartPointer<vtkPolyData>::New();
          mesh3D->DeepCopy(other.mesh3D);
      }
      if (other.originalMesh3D) {
          originalMesh3D = vtkSmartPointer<vtkPolyData>::New();
          originalMesh3D->DeepCopy(other.originalMesh3D);
      }
  }
  
  Airfoil& operator=(const Airfoil& other) {
      if (this != &other) {
          name = other.name;
          coordinates = other.coordinates;
          panels = other.panels;
          is3DModel = other.is3DModel;
          if (other.mesh3D) {
              mesh3D = vtkSmartPointer<vtkPolyData>::New();
              mesh3D->DeepCopy(other.mesh3D);
          } else {
              mesh3D = nullptr;
          }
          if (other.originalMesh3D) {
              originalMesh3D = vtkSmartPointer<vtkPolyData>::New();
              originalMesh3D->DeepCopy(other.originalMesh3D);
          } else {
              originalMesh3D = nullptr;
          }
      }
      return *this;
  }

  bool loadFromFile(const std::string &filename);
  bool loadFrom2DFile(const std::string &filename);
  bool loadFrom3DMesh(const std::string &filename);
  void generateNACA(double m, double p, double t, int n = 100);
  void generateCylinder(double radius, int n = 100);
  void generateDiamond(double thickness = 0.10);
  void generateFlatPlate(double thickness = 0.02);

  void generatePanels(); 
  void rotateCoordinates(double angleDeg);

  const std::vector<Point2D> &getCoordinates() const { return coordinates; }
  const std::vector<Panel> &getPanels() const { return panels; }

  std::string getName() const { return name; }

  bool is3D() const { return is3DModel && mesh3D != nullptr; }
  vtkSmartPointer<vtkPolyData> getMesh3D() const { return mesh3D; }
  void setMesh3D(vtkSmartPointer<vtkPolyData> mesh) { mesh3D = mesh; originalMesh3D = vtkSmartPointer<vtkPolyData>::New(); originalMesh3D->DeepCopy(mesh); is3DModel = (mesh != nullptr); }
  void setBaseRotation(double rx, double ry, double rz);

private:
  std::string name;
  std::vector<Point2D> coordinates;
  std::vector<Panel> panels; 
  bool is3DModel = false;
  vtkSmartPointer<vtkPolyData> mesh3D;
  vtkSmartPointer<vtkPolyData> originalMesh3D;
};
} 
