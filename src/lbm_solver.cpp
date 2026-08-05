#include "ZweiFoil/lbm_solver.hpp"
#include "ZweiFoil/solver.hpp"
#include "raylib.h"
#include "rlgl.h"
#include "shaders.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

extern "C" {
extern void (*glad_glMemoryBarrier)(unsigned int barriers);
}
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000

namespace zweifoil {

Grid3D::Grid3D(int nx, int ny, int nz) : NX(nx), NY(ny), NZ(nz) {
  std::cout << "[LBM-Grid] Creating Grid3D: " << nx << "x" << ny << "x" << nz << std::endl;
  int totalNodes = NX * NY * NZ;
  int totalQ = totalNodes * D3Q19::Q;

  f.assign(totalQ, 0.0f);
  f_new.assign(totalQ, 0.0f);
  sdf.assign(totalNodes, 1.0f); // Positive means outside (fluid)
  rho.assign(totalNodes, 1.0f);
  u.assign(totalNodes, {0.0f, 0.0f, 0.0f, 0.0f});

  ssbo_f =
      rlLoadShaderBuffer(totalQ * sizeof(float), f.data(), RL_DYNAMIC_DRAW);
  ssbo_f_new =
      rlLoadShaderBuffer(totalQ * sizeof(float), f_new.data(), RL_DYNAMIC_DRAW);
  ssbo_sdf = rlLoadShaderBuffer(totalNodes * sizeof(float), sdf.data(),
                                     RL_DYNAMIC_DRAW);
  ssbo_rho = rlLoadShaderBuffer(totalNodes * sizeof(float), rho.data(),
                                RL_DYNAMIC_DRAW);
  ssbo_u = rlLoadShaderBuffer(totalNodes * sizeof(Vector4D), u.data(),
                              RL_DYNAMIC_DRAW);
}

Grid3D::~Grid3D() {
  rlUnloadShaderBuffer(ssbo_f);
  rlUnloadShaderBuffer(ssbo_f_new);
  rlUnloadShaderBuffer(ssbo_sdf);
  rlUnloadShaderBuffer(ssbo_rho);
  rlUnloadShaderBuffer(ssbo_u);
}

int Grid3D::getIndex(int x, int y, int z, int q) const {
  return (z * NY * NX + y * NX + x) * D3Q19::Q + q;
}

int Grid3D::getScalarIndex(int x, int y, int z) const {
  return z * NY * NX + y * NX + x;
}

void Grid3D::initialize(const Flowconditions &cond) {
  float vx = cond.V_inf;
  float vy = 0.0f;
  switch (cond.windDirection) {
  case 1:
    vx = -vx;
    vy = -vy;
    break;
  case 2: {
    float t = vx;
    vx = vy;
    vy = -t;
  } break;
  case 3: {
    float t = vx;
    vx = -vy;
    vy = t;
  } break;
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

  rlUpdateShaderBuffer(ssbo_f, f.data(), f.size() * sizeof(float), 0);
  rlUpdateShaderBuffer(ssbo_f_new, f_new.data(), f_new.size() * sizeof(float),
                       0);
  rlUpdateShaderBuffer(ssbo_sdf, sdf.data(),
                       sdf.size() * sizeof(float), 0);
  rlUpdateShaderBuffer(ssbo_rho, rho.data(), rho.size() * sizeof(float), 0);
  rlUpdateShaderBuffer(ssbo_u, u.data(), u.size() * sizeof(Vector4D), 0);
}

void Grid3D::enforceFreestream(const Flowconditions &cond) {}
void Grid3D::updateMacroscopic() {}
void Grid3D::collideAndStream(double tau) {}
void Grid3D::applyBoundaries() {}

LBMSolver::LBMSolver(int nx, int ny, int nz) : grid(nx, ny, nz), tau(1.0f) {
  unsigned int shaderId =
      rlLoadShader(lbm_compute_shader_src, RL_COMPUTE_SHADER);
  computeShader = rlLoadShaderProgramCompute(shaderId);
}

LBMSolver::~LBMSolver() { rlUnloadShaderProgram(computeShader); }



void LBMSolver::step(const Flowconditions &cond) {
  static int stepCount = 0;
  if (stepCount % 60 == 0) {
      ::std::cout << "[LBM-Solver] Step " << stepCount << " | v_inf: " << cond.V_inf << " | nu: " << cond.kinematic_viscosity << ::std::endl;
  }
  
  if (stepCount == 120) {
      ::std::cout << "[LBM-Solver] DUMPING GRID TO FILE at step 120..." << ::std::endl;
      rlReadShaderBuffer(grid.ssbo_u, grid.u.data(), grid.u.size() * sizeof(Vector4D), 0);
      rlReadShaderBuffer(grid.ssbo_rho, grid.rho.data(), grid.rho.size() * sizeof(float), 0);
      rlReadShaderBuffer(grid.ssbo_sdf, grid.sdf.data(), grid.sdf.size() * sizeof(float), 0);
      
      ::std::ofstream dump("lbm_dump_120.txt");
      dump << "LBM Grid Dump at Step 120\n";
      dump << "NX=" << grid.NX << ", NY=" << grid.NY << ", NZ=" << grid.NZ << "\n";
      
      int sliceZ = grid.NZ / 2;
      dump << "--- SLICE Z = " << sliceZ << " ---\n";
      for (int y = 0; y < grid.NY; ++y) {
          for (int x = 0; x < grid.NX; ++x) {
              int idx = grid.getScalarIndex(x, y, sliceZ);
              float d = grid.sdf[idx];
              if (d <= 0.0f) {
                  dump << "X ";
              } else {
                  Vector4D v = grid.u[idx];
                  float speed = ::std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
                  if (::std::isnan(speed)) dump << "N ";
                  else if (speed < 0.001f) dump << ". ";
                  else if (speed < 0.02f) dump << "- ";
                  else if (speed < 0.04f) dump << "~ ";
                  else dump << "= ";
              }
          }
          dump << "\n";
      }
      
      dump << "\n--- VELOCITY & DENSITY AT Z=" << sliceZ << ", Y=" << grid.NY/2 << " ---\n";
      for (int x = 0; x < grid.NX; ++x) {
          int idx = grid.getScalarIndex(x, grid.NY/2, sliceZ);
          Vector4D v = grid.u[idx];
          dump << "x=" << x << " sdf=" << grid.sdf[idx] << " rho=" << grid.rho[idx] 
               << " u=(" << v.x << "," << v.y << "," << v.z << ")\n";
      }
      dump.close();
      ::std::cout << "[LBM-Solver] DUMP COMPLETE." << ::std::endl;
  }
  
  stepCount++;
  
  float tau = 3.0f * cond.kinematic_viscosity + 0.5f;
  if (tau < 0.505f) tau = 0.505f; // Enforce minimum numerical stability

  float vx = cond.V_inf;
  float vy = 0.0f;
  switch (cond.windDirection) {
  case 1:
    vx = -vx;
    vy = -vy;
    break;
  case 2: {
    float t = vx;
    vx = vy;
    vy = -t;
  } break;
  case 3: {
    float t = vx;
    vx = -vy;
    vy = t;
  } break;
  }
  float vz = 0.0f;

  rlEnableShader(computeShader);

  int nxLoc = rlGetLocationUniform(computeShader, "NX");
  int nyLoc = rlGetLocationUniform(computeShader, "NY");
  int nzLoc = rlGetLocationUniform(computeShader, "NZ");
  int tauLoc = rlGetLocationUniform(computeShader, "tau");
  int vxLoc = rlGetLocationUniform(computeShader, "vx_inf");
  int vyLoc = rlGetLocationUniform(computeShader, "vy_inf");
  int vzLoc = rlGetLocationUniform(computeShader, "vz_inf");

  rlSetUniform(nxLoc, &grid.NX, RL_SHADER_UNIFORM_INT, 1);
  rlSetUniform(nyLoc, &grid.NY, RL_SHADER_UNIFORM_INT, 1);
  rlSetUniform(nzLoc, &grid.NZ, RL_SHADER_UNIFORM_INT, 1);
  rlSetUniform(tauLoc, &tau, RL_SHADER_UNIFORM_FLOAT, 1);
  rlSetUniform(vxLoc, &vx, RL_SHADER_UNIFORM_FLOAT, 1);
  rlSetUniform(vyLoc, &vy, RL_SHADER_UNIFORM_FLOAT, 1);
  rlSetUniform(vzLoc, &vz, RL_SHADER_UNIFORM_FLOAT, 1);

  static bool pingPong = false;
  rlBindShaderBuffer(pingPong ? grid.ssbo_f_new : grid.ssbo_f, 0);
  rlBindShaderBuffer(pingPong ? grid.ssbo_f : grid.ssbo_f_new, 1);
  pingPong = !pingPong;

  rlBindShaderBuffer(grid.ssbo_sdf, 2);
  rlBindShaderBuffer(grid.ssbo_rho, 3);
  rlBindShaderBuffer(grid.ssbo_u, 4);

  rlComputeShaderDispatch(grid.NX / 8, grid.NY / 8, grid.NZ / 8);
  glad_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  rlDisableShader();
}

} // namespace zweifoil
