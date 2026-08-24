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

#pragma omp parallel for collapse(2)
  for (int z = 0; z < NZ; ++z) {
    for (int y = 0; y < NY; ++y) {
      for (int q = 0; q < D3Q19::Q; ++q) {
        float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
        float feq = D3Q19::w[q] * 1.0f * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
        
        if (D3Q19::cx[q] > 0) { 
            f_new[getIndex(0, y, z, q)] = feq;
        }
        if (D3Q19::cx[q] < 0) {
            f_new[getIndex(NX-1, y, z, q)] = f[getIndex(NX-2, y, z, q)];
        }
      }
    }
  }
}

void Grid3D::computeStep(double tau) {
  float omega = 1.0f / tau;
  
  double total_fx = 0.0;
  double total_fy = 0.0;
  
#pragma omp parallel for collapse(3) reduction(+:total_fx, total_fy)
  for (int z = 0; z < NZ; ++z) {
    for (int y = 0; y < NY; ++y) {
      for (int x = 0; x < NX; ++x) {
        int s_idx = getScalarIndex(x, y, z);
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
            if (nx < 0) nx = NX - 1; else if (nx >= NX) nx = 0;

            if (ny < 0 || ny >= NY) {
                f_local[q] = f[getIndex(x, y, z, D3Q19::opposite[q])];
            } else if (sdf[getScalarIndex(nx, ny, nz)] <= 0.0f) {
                
                f_local[q] = f[getIndex(x, y, z, D3Q19::opposite[q])];
                total_fx += 2.0 * f_local[q] * (-D3Q19::cx[q]);
                total_fy += 2.0 * f_local[q] * (-D3Q19::cy[q]);
            } else {
                
                f_local[q] = f[getIndex(nx, ny, nz, q)];
            }

            r += f_local[q];
            vx += D3Q19::cx[q] * f_local[q];
            vy += D3Q19::cy[q] * f_local[q];
            vz += D3Q19::cz[q] * f_local[q];
        }

        
        rho[s_idx] = r;
        if (r > 0.0f) {
            float inv_r = 1.0f / r;
            vx *= inv_r;
            vy *= inv_r;
            vz *= inv_r;
        }
        u[s_idx] = {vx, vy, vz, 0.0f};

        
        float usq = vx * vx + vy * vy + vz * vz;
        #pragma omp simd
        for (int q = 0; q < D3Q19::Q; ++q) {
            float cu = D3Q19::cx[q] * vx + D3Q19::cy[q] * vy + D3Q19::cz[q] * vz;
            float feq = D3Q19::w[q] * r * (1.0f + 3.0f * cu + 4.5f * cu * cu - 1.5f * usq);
            f_new[getIndex(x, y, z, q)] = f_local[q] * (1.0f - omega) + feq * omega;
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

  grid.computeStep(tau);
  grid.enforceFreestream(cond);
  grid.f.swap(grid.f_new);
}

} 
