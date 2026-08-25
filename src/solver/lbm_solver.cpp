#include "ZweiCFD/solver/lbm_solver.hpp"
#include "ZweiCFD/solver/solver.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace zweicfd {

Grid3D::Grid3D(int nx, int ny, int nz) : NX(nx), NY(ny), NZ(nz) {
  std::cout << "[LBM-Grid] Creating Grid3D: " << nx << "x" << ny << "x" << nz << std::endl;
  int totalNodes = NX * NY * NZ;
  int totalQ = totalNodes * D3Q19::Q;

  f.resize(totalNodes * 19, 0.0f);
  f_new.resize(totalNodes * 19, 0.0f);
  sdf.resize(totalNodes, 1.0f);
  drawn_sdf.resize(totalNodes, 1.0f);
  rho.assign(totalNodes, 1.0f);
  u.assign(totalNodes, {0.0f, 0.0f, 0.0f, 0.0f});
}

Grid3D::~Grid3D() {
}


void Grid3D::initialize(const Flowconditions &cond) {
  float v_scale = 0.05f / std::max(0.0001f, (float)cond.V_inf);
  float vx = 0.0f;
  float vy = 0.0f;

  switch (cond.windDirection) {
  case 0: 
    vx = cond.V_inf * v_scale;
    vy = 0.0f;
    break;
  case 1: 
    vx = -cond.V_inf * v_scale;
    vy = 0.0f;
    break;
  case 2: 
    vx = 0.0f;
    vy = -cond.V_inf * v_scale;
    break;
  case 3: 
    vx = 0.0f;
    vy = cond.V_inf * v_scale;
    break;
  }
  float vz = 0.0f;

  for (int z = 0; z < NZ; ++z) {
    for (int y = 0; y < NY; ++y) {
      for (int x = 0; x < NX; ++x) {
        int s_idx = getScalarIndex(x, y, z);
        rho[s_idx] = 1.0f;
        u[s_idx] = {vx, vy, vz, 0.0f};

        float usq = vx * vx + vy * vy + vz * vz;

        for (int q = 0; q < D3Q19::Q; ++q) {
          float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
          float feq = D3Q19::w[q] * 1.0f *
                      (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
          f[getIndex(x, y, z, q)] = feq;
          f_new[getIndex(x, y, z, q)] = feq;
        }
      }
    }
  }
}

void Grid3D::enforceFreestream(const Flowconditions &cond) {
  float v_scale = 0.05f / std::max(0.0001f, (float)cond.V_inf);
  float vx = 0.0f;
  float vy = 0.0f;
  
  switch (cond.windDirection) {
  case 0: vx = cond.V_inf * v_scale; vy = 0.0f; break;
  case 1: vx = -cond.V_inf * v_scale; vy = 0.0f; break;
  case 2: vx = 0.0f; vy = -cond.V_inf * v_scale; break;
  case 3: vx = 0.0f; vy = cond.V_inf * v_scale; break;
  }
  float vz = 0.0f;
  float usq = vx * vx + vy * vy + vz * vz;
  int totalNodes = NZ * NY * NX;
  int sliceSize = NY * NX;

  if (cond.windDirection == 0) {
#pragma omp parallel for collapse(2) schedule(static)
    for (int z = 0; z < NZ; ++z) {
      for (int y = 0; y < NY; ++y) {
        int s_inlet = z * sliceSize + y * NX + 0;
        int s_outlet = z * sliceSize + y * NX + (NX - 1);
        int s_outlet_prev = z * sliceSize + y * NX + (NX - 2);

        rho[s_inlet] = 1.0f;
        u[s_inlet] = {vx, vy, vz, 0.0f};
        rho[s_outlet] = rho[s_outlet_prev];
        u[s_outlet] = u[s_outlet_prev];

        for (int q = 0; q < D3Q19::Q; ++q) {
          float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
          float feq = (float)D3Q19::w[q] * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
          f_new[q * totalNodes + s_inlet] = feq;

          if (D3Q19::cx[q] > 0) {
              f_new[q * totalNodes + s_outlet] = f[q * totalNodes + s_outlet_prev];
          }
        }
      }
    }
  } else if (cond.windDirection == 1) {
#pragma omp parallel for collapse(2) schedule(static)
    for (int z = 0; z < NZ; ++z) {
      for (int y = 0; y < NY; ++y) {
        int s_inlet = z * sliceSize + y * NX + (NX - 1);
        int s_outlet = z * sliceSize + y * NX + 0;
        int s_outlet_prev = z * sliceSize + y * NX + 1;

        rho[s_inlet] = 1.0f;
        u[s_inlet] = {vx, vy, vz, 0.0f};
        rho[s_outlet] = rho[s_outlet_prev];
        u[s_outlet] = u[s_outlet_prev];

        for (int q = 0; q < D3Q19::Q; ++q) {
          float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
          float feq = (float)D3Q19::w[q] * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
          f_new[q * totalNodes + s_inlet] = feq;

          if (D3Q19::cx[q] < 0) {
              f_new[q * totalNodes + s_outlet] = f[q * totalNodes + s_outlet_prev];
          }
        }
      }
    }
  } else if (cond.windDirection == 2) {
#pragma omp parallel for collapse(2) schedule(static)
    for (int z = 0; z < NZ; ++z) {
      for (int x = 0; x < NX; ++x) {
        int s_inlet = z * sliceSize + (NY - 1) * NX + x;
        int s_outlet = z * sliceSize + 0 * NX + x;
        int s_outlet_prev = z * sliceSize + 1 * NX + x;

        rho[s_inlet] = 1.0f;
        u[s_inlet] = {vx, vy, vz, 0.0f};
        rho[s_outlet] = rho[s_outlet_prev];
        u[s_outlet] = u[s_outlet_prev];

        for (int q = 0; q < D3Q19::Q; ++q) {
          float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
          float feq = (float)D3Q19::w[q] * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
          f_new[q * totalNodes + s_inlet] = feq;

          if (D3Q19::cy[q] < 0) {
              f_new[q * totalNodes + s_outlet] = f[q * totalNodes + s_outlet_prev];
          }
        }
      }
    }
  } else if (cond.windDirection == 3) {
#pragma omp parallel for collapse(2) schedule(static)
    for (int z = 0; z < NZ; ++z) {
      for (int x = 0; x < NX; ++x) {
        int s_inlet = z * sliceSize + 0 * NX + x;
        int s_outlet = z * sliceSize + (NY - 1) * NX + x;
        int s_outlet_prev = z * sliceSize + (NY - 2) * NX + x;

        rho[s_inlet] = 1.0f;
        u[s_inlet] = {vx, vy, vz, 0.0f};
        rho[s_outlet] = rho[s_outlet_prev];
        u[s_outlet] = u[s_outlet_prev];

        for (int q = 0; q < D3Q19::Q; ++q) {
          float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
          float feq = (float)D3Q19::w[q] * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
          f_new[q * totalNodes + s_inlet] = feq;

          if (D3Q19::cy[q] > 0) {
              f_new[q * totalNodes + s_outlet] = f[q * totalNodes + s_outlet_prev];
          }
        }
      }
    }
  }
}

void Grid3D::computeStep(double tau, int windDir) {
  float omega = 1.0f / (float)tau;
  float one_minus_omega = 1.0f - omega;
  int totalNodes = NZ * NY * NX;
  int sliceSize = NY * NX;
  
  double total_fx = 0.0;
  double total_fy = 0.0;
  
#pragma omp parallel for collapse(2) reduction(+:total_fx, total_fy) schedule(static)
  for (int z = 0; z < NZ; ++z) {
    for (int y = 0; y < NY; ++y) {
      int rowOffset = z * sliceSize + y * NX;
      for (int x = 0; x < NX; ++x) {
        int s_idx = rowOffset + x;
        if (sdf[s_idx] <= 0.0f) {
            u[s_idx] = {0.0f, 0.0f, 0.0f, 0.0f};
            rho[s_idx] = 1.0f;
            continue;
        }

        float r = 0.0f;
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        float f_local[19];

        for (int q = 0; q < D3Q19::Q; ++q) {
            int nx = x - D3Q19::cx[q];
            int ny = y - D3Q19::cy[q];
            int nz = z - D3Q19::cz[q];

            if (nz < 0) nz = NZ - 1; else if (nz >= NZ) nz = 0;

            if (windDir == 2 || windDir == 3) {
                if (ny < 0) ny = NY - 1; else if (ny >= NY) ny = 0;
                if (nx < 0 || nx >= NX) {
                    f_local[q] = f[D3Q19::opposite[q] * totalNodes + s_idx];
                } else {
                    int n_sidx = nz * sliceSize + ny * NX + nx;
                    if (sdf[n_sidx] <= 0.0f) {
                        f_local[q] = f[D3Q19::opposite[q] * totalNodes + s_idx];
                        total_fx += 2.0 * f_local[q] * (-D3Q19::cx[q]);
                        total_fy += 2.0 * f_local[q] * (-D3Q19::cy[q]);
                    } else {
                        f_local[q] = f[q * totalNodes + n_sidx];
                    }
                }
            } else {
                if (nx < 0) nx = NX - 1; else if (nx >= NX) nx = 0;
                if (ny < 0 || ny >= NY) {
                    f_local[q] = f[D3Q19::opposite[q] * totalNodes + s_idx];
                } else {
                    int n_sidx = nz * sliceSize + ny * NX + nx;
                    if (sdf[n_sidx] <= 0.0f) {
                        f_local[q] = f[D3Q19::opposite[q] * totalNodes + s_idx];
                        total_fx += 2.0 * f_local[q] * (-D3Q19::cx[q]);
                        total_fy += 2.0 * f_local[q] * (-D3Q19::cy[q]);
                    } else {
                        f_local[q] = f[q * totalNodes + n_sidx];
                    }
                }
            }

            r += f_local[q];
            vx += D3Q19::cx[q] * f_local[q];
            vy += D3Q19::cy[q] * f_local[q];
            vz += D3Q19::cz[q] * f_local[q];
        }

        if (r < 0.1f || r > 3.0f || std::isnan(r)) {
            r = 1.0f;
            vx = 0.0f;
            vy = 0.0f;
            vz = 0.0f;
        } else {
            float inv_r = 1.0f / r;
            vx *= inv_r;
            vy *= inv_r;
            vz *= inv_r;
        }

        float usq = vx * vx + vy * vy + vz * vz;
        if (usq > 0.09f) {
            float scale = 0.3f / std::sqrt(usq);
            vx *= scale;
            vy *= scale;
            vz *= scale;
            usq = 0.09f;
        }

        rho[s_idx] = r;
        u[s_idx] = {vx, vy, vz, 0.0f};

        for (int q = 0; q < D3Q19::Q; ++q) {
            float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
            float feq = (float)D3Q19::w[q] * r * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
            float f_val = f_local[q] * one_minus_omega + feq * omega;
            f_new[q * totalNodes + s_idx] = (f_val > 0.0f) ? f_val : 0.0f;
        }
      }
    }
  }
  
  this->force_x = total_fx;
  this->force_y = total_fy;
}

void Grid3D::applyBoundaries() { }

LBMSolver::LBMSolver(int nx, int ny, int nz) : grid(nx, ny, nz), tau(1.0f) {
}

LBMSolver::~LBMSolver() { }

void LBMSolver::step(const Flowconditions &cond) {
  stepCount++;
  
  this->tau = 3.0f * cond.kinematic_viscosity + 0.5f;
  if (this->tau < 0.505f) this->tau = 0.505f; 

  grid.computeStep(tau, cond.windDirection);
  grid.enforceFreestream(cond);
  grid.f.swap(grid.f_new);
}

} 
