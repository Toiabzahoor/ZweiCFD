#pragma once

#include <array>
#include <memory>
#include <vector>
// adding 3d solver for lbm
namespace zweifoil {

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

struct Flowconditions;

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

  int getIndex(int x, int y, int z, int q) const;
  int getScalarIndex(int x, int y, int z) const;

  int NX, NY, NZ;
  std::vector<float> f;
  std::vector<float> f_new;
  std::vector<int> is_solid; // use int for std430 alignment
  std::vector<float> rho;
  std::vector<Vector4D> u; // vec4 for std430 alignment

  unsigned int ssbo_f;
  unsigned int ssbo_f_new;
  unsigned int ssbo_is_solid;
  unsigned int ssbo_rho;
  unsigned int ssbo_u;

  void initialize(const Flowconditions &cond);
  void enforceFreestream(const Flowconditions &cond); // No longer needed if fully GPU, but keeping interface
  void updateMacroscopic(); // No longer needed
  void collideAndStream(double tau); // No longer needed
  void applyBoundaries(); // No longer needed
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
  unsigned int computeShader;
};

} // namespace zweifoil
