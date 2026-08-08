#pragma once

#include <string>
#include <vector>


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
  

  bool loadFromFile(const std::string &filename);
  void generateNACA(double m, double p, double t, int n = 100);
  void generateCylinder(double radius, int n = 100);

  void generatePanels(); 
  void rotateCoordinates(double angleDeg);

  const std::vector<Point2D> &getCoordinates() const { return coordinates; }
  const std::vector<Panel> &getPanels() const { return panels; }

  std::string getName() const { return name; }

private:
  std::string name;
  std::vector<Point2D> coordinates;
  std::vector<Panel> panels; 
};
} 
