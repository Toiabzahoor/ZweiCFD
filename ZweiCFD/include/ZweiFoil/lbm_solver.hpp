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
  double x, y, z;
};

class Grid3D {
public:
  Grid3D(int nx, int ny, int nz);

  int getIndex(int x, int y, int z, int q) const;
  int getScalarIndex(int x, int y, int z) const;

  int NX, NY, NZ;
  std::vector<double> f;
  std::vector<double> f_new;
  std::vector<bool> is_solid;
  std::vector<double> rho;
  std::vector<Vector3D> u;

  void initialize(const Flowconditions &cond);
  void enforceFreestream(const Flowconditions &cond);
  void updateMacroscopic();
  void collideAndStream(double tau);
  void applyBoundaries();
};

class LBMSolver {
public:
  LBMSolver(int nx, int ny, int nz);
  void step(const Flowconditions &cond);

  const Grid3D &getGrid() const { return grid; }
  Grid3D &getGridModifiable() { return grid; }

private:
  Grid3D grid;
  double tau;
};

} // namespace zweifoil
