#include "ZweiFoil/lbm_solver.hpp"
#include "ZweiFoil/solver.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace zweifoil {

Grid3D::Grid3D(int nx, int ny, int nz) : NX(nx), NY(ny), NZ(nz) {
    int totalNodes = NX * NY * NZ;
    int totalQ = totalNodes * D3Q19::Q;
    
    f.assign(totalQ, 0.0);
    f_new.assign(totalQ, 0.0);
    is_solid.assign(totalNodes, false);
    rho.assign(totalNodes, 1.0);
    u.assign(totalNodes, {0.0, 0.0, 0.0});
}

int Grid3D::getIndex(int x, int y, int z, int q) const {
    return (z * NY * NX + y * NX + x) * D3Q19::Q + q;
}

int Grid3D::getScalarIndex(int x, int y, int z) const {
    return z * NY * NX + y * NX + x;
}

void Grid3D::initialize(const Flowconditions& cond) {
    double vx = cond.V_inf * std::cos(cond.alpha * M_PI / 180.0);
    double vy = cond.V_inf * std::sin(cond.alpha * M_PI / 180.0);
    switch (cond.windDirection) {
        case 1: vx = -vx; vy = -vy; break;
        case 2: { double t = vx; vx = vy; vy = -t; } break;
        case 3: { double t = vx; vx = -vy; vy = t; } break;
    }
    double vz = 0.0;
    
    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                int s_idx = getScalarIndex(x, y, z);
                rho[s_idx] = 1.0;
                u[s_idx] = {vx, vy, vz};
                
                double ux = u[s_idx].x;
                double uy = u[s_idx].y;
                double uz = u[s_idx].z;
                double usq = ux*ux + uy*uy + uz*uz;
                
                for (int q = 0; q < D3Q19::Q; ++q) {
                    double cu = D3Q19::cx[q]*ux + D3Q19::cy[q]*uy + D3Q19::cz[q]*uz;
                    double feq = D3Q19::w[q] * rho[s_idx] * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
                    f[getIndex(x, y, z, q)] = feq;
                    f_new[getIndex(x, y, z, q)] = feq;
                }
            }
        }
    }
}

void Grid3D::enforceFreestream(const Flowconditions& cond) {
    double vx = cond.V_inf * std::cos(cond.alpha * M_PI / 180.0);
    double vy = cond.V_inf * std::sin(cond.alpha * M_PI / 180.0);
    switch (cond.windDirection) {
        case 1: vx = -vx; vy = -vy; break;
        case 2: { double t = vx; vx = vy; vy = -t; } break;
        case 3: { double t = vx; vx = -vy; vy = t; } break;
    }
    double vz = 0.0;
    double usq = vx*vx + vy*vy + vz*vz;

    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                if (x == 0 || x == NX-1 || y == 0 || y == NY-1 || z == 0 || z == NZ-1) {
                    int s_idx = getScalarIndex(x, y, z);
                    rho[s_idx] = 1.0;
                    u[s_idx] = {vx, vy, vz};
                    for (int q = 0; q < D3Q19::Q; ++q) {
                        double cu = D3Q19::cx[q]*vx + D3Q19::cy[q]*vy + D3Q19::cz[q]*vz;
                        double feq = D3Q19::w[q] * 1.0 * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
                        f[getIndex(x, y, z, q)] = feq;
                    }
                }
            }
        }
    }
}

void Grid3D::updateMacroscopic() {
    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                int s_idx = getScalarIndex(x, y, z);
                
                if (is_solid[s_idx]) {
                    rho[s_idx] = 1.0;
                    u[s_idx] = {0.0, 0.0, 0.0};
                    continue;
                }
                
                double density = 0.0;
                double ux = 0.0;
                double uy = 0.0;
                double uz = 0.0;
                
                for (int q = 0; q < D3Q19::Q; ++q) {
                    double val = f[getIndex(x, y, z, q)];
                    density += val;
                    ux += val * D3Q19::cx[q];
                    uy += val * D3Q19::cy[q];
                    uz += val * D3Q19::cz[q];
                }
                
                rho[s_idx] = density;
                if (density > 1e-9) {
                    u[s_idx].x = ux / density;
                    u[s_idx].y = uy / density;
                    u[s_idx].z = uz / density;
                }
            }
        }
    }
}

void Grid3D::collideAndStream(double tau) {
    double omega = 1.0 / tau;
    
    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                int s_idx = getScalarIndex(x, y, z);
                
                if (is_solid[s_idx]) continue;
                
                double density = rho[s_idx];
                double ux = u[s_idx].x;
                double uy = u[s_idx].y;
                double uz = u[s_idx].z;
                double usq = ux*ux + uy*uy + uz*uz;
                
                for (int q = 0; q < D3Q19::Q; ++q) {
                    double cu = D3Q19::cx[q]*ux + D3Q19::cy[q]*uy + D3Q19::cz[q]*uz;
                    double feq = D3Q19::w[q] * density * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
                    
                    int idx = getIndex(x, y, z, q);
                    double f_post = f[idx] - omega * (f[idx] - feq);
                    
                    int next_x = x + D3Q19::cx[q];
                    int next_y = y + D3Q19::cy[q];
                    int next_z = z + D3Q19::cz[q];
                    
                    if (next_x >= 0 && next_x < NX &&
                        next_y >= 0 && next_y < NY &&
                        next_z >= 0 && next_z < NZ) {
                        f_new[getIndex(next_x, next_y, next_z, q)] = f_post;
                    }
                }
            }
        }
    }
    f = f_new;
}

void Grid3D::applyBoundaries() {
    for (int z = 0; z < NZ; ++z) {
        for (int y = 0; y < NY; ++y) {
            for (int x = 0; x < NX; ++x) {
                if (is_solid[getScalarIndex(x, y, z)]) {
                    for (int q = 0; q < D3Q19::Q; ++q) {
                        int opp = D3Q19::opposite[q];
                        
                        int prev_x = x - D3Q19::cx[q];
                        int prev_y = y - D3Q19::cy[q];
                        int prev_z = z - D3Q19::cz[q];
                        
                        if (prev_x >= 0 && prev_x < NX &&
                            prev_y >= 0 && prev_y < NY &&
                            prev_z >= 0 && prev_z < NZ) {
                            
                            if (!is_solid[getScalarIndex(prev_x, prev_y, prev_z)]) {
                                f[getIndex(prev_x, prev_y, prev_z, opp)] = f_new[getIndex(x, y, z, q)];
                            }
                        }
                    }
                }
            }
        }
    }
}

LBMSolver::LBMSolver(int nx, int ny, int nz) : grid(nx, ny, nz), tau(1.0) {
}

void LBMSolver::step(const Flowconditions& cond) {
    tau = 3.0 * cond.kinematic_viscosity + 0.5;
    
    grid.enforceFreestream(cond);
    grid.collideAndStream(tau);
    grid.applyBoundaries();
    grid.updateMacroscopic();
}

}
