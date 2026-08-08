#pragma once

#include <array>
#include <memory>
#include <vector>

#include "ZweiCFD/solver/flowconditions.hpp"

namespace zweicfd {

struct D3Q19 {
  static constexpr int Q = 19;

  static constexpr std::array<int, Q> cx = {0,  1, -1, 0, 0,  0, 0, 1, -1, 1,
                                            -1, 1, -1, 1, -1, 0, 0, 0, 0};
  static constexpr std::array<int, Q> cy = {0, 0, 0, 1, -1, 0, 0,  1, -1, -1,
                                            1, 0, 0, 0, 0,  1, -1, 1, -1};
  static constexpr std::array<int, Q> cz = {0, 0, 0,  0,  0, 1, -1, 0,  0, 0,
                                            0, 1, -1, -1, 1, 1, -1, -1, 1};

  static constexpr std::array<double, Q> w = {
      1.0 / 3.0,  1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0,
      1.0 / 18.0, 1.0 / 18.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
      1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
      1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};

  static constexpr std::array<int, Q> opposite = {
      0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17};
};


struct Vector3D {
  float x, y, z;
};

struct Vector4D {
  float x, y, z, w;
};

class Grid3D {
public:
  Grid3D(int nx, int ny, int nz);
  ~Grid3D();

  
  Grid3D(const Grid3D&) = delete;
  Grid3D& operator=(const Grid3D&) = delete;
  Grid3D(Grid3D&&) = delete;
  Grid3D& operator=(Grid3D&&) = delete;

  inline int getIndex(int x, int y, int z, int q) const {
    return q * (NZ * NY * NX) + (z * NY * NX + y * NX + x);
  }
  inline int getScalarIndex(int x, int y, int z) const {
    return z * NY * NX + y * NX + x;
  }

  int NX, NY, NZ;
  std::vector<float> f;
  std::vector<float> f_new;
  std::vector<float> sdf; 
  std::vector<float> drawn_sdf;
  std::vector<float> rho;
  std::vector<Vector4D> u; 

  double force_x = 0.0;
  double force_y = 0.0;

  void initialize(const Flowconditions &cond);
  void enforceFreestream(const Flowconditions &cond); 
  void computeStep(double tau); 
  void applyBoundaries(); 
};

class LBMSolver {
public:
  LBMSolver(int nx, int ny, int nz);
  ~LBMSolver();
  
  void step(const Flowconditions &cond);

  const Grid3D &getGrid() const { return grid; }
  Grid3D &getGridModifiable() { return grid; }

private:
  Grid3D grid;
  float tau;
  
  int stepCount = 0;
  bool pingPong = false;
};

} 
