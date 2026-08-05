#include "simulation.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <omp.h>

extern "C" {
extern void (*glad_glMemoryBarrier)(unsigned int barriers);
}
#define GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT 0x00000001
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000

#include "imgui.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"
#include "shaders.hpp"

Simulation::Simulation(int argc, char *argv[]) {
  std::cout << "==== ZweiCFD v0.4   Streamlines & Viscous Flow ====\n";

  std::string filename = (argc >= 2) ? argv[1] : "naca0012.dat";
  if (!foil.loadFromFile(filename)) {
    std::cerr << "Failed to load Airfoil Data!.\n";
  } else {
    std::cout << "Loaded: " << foil.getName() << "\n";
  }

  flow.alpha = 5.0;
  flow.V_inf = 1.0;
  flow.kinematic_viscosity = 1.5e-5;
  flow.windDirection = 0;

  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
  InitWindow(1280, 720, "ZweiCFD Engine");
  SetTargetFPS(60);

  rebuildSolverWithRotation();

  rlImGuiSetup(true);

  camera = {0};
  camera.position = Vector3{0.0f, 0.0f, 250.0f};
  camera.target = Vector3{0.0f, 0.0f, 0.0f};
  camera.up = Vector3{0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  strncpy(fileBuf, filename.c_str(), sizeof(fileBuf));

  current_sim = 0;
  targetParticleCount = 16000;

  for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
    wind.particles[i].isActive = 0;
  }

  particleSpawnTimer = 0.0f;
  displayedVelocity = (float)flow.V_inf;
  windArrowDir = getWindArrowAngle(flow.windDirection);

  unsigned int csId = rlLoadShader(particle_update_comp, RL_COMPUTE_SHADER);
  particleComputeShader = rlLoadShaderProgramCompute(csId);

  particleRenderShader =
      rlLoadShaderProgram(particle_render_vs, particle_render_fs);

  quadMesh = GenMeshPlane(1.0f, 1.0f, 1, 1);
  UploadMesh(&quadMesh, false);
  particleMaterial = LoadMaterialDefault();
  particleMaterial.shader.id = particleRenderShader;

  ssbo_particles =
      rlLoadShaderBuffer(WindSystem::MAX_PARTICLES * sizeof(WindParticle),
                         wind.particles, RL_DYNAMIC_DRAW);
}

Simulation::~Simulation() {
  rlUnloadShaderBuffer(ssbo_particles);
  rlUnloadShaderProgram(particleComputeShader);
  UnloadMaterial(particleMaterial);
  UnloadMesh(quadMesh);
  rlImGuiShutdown();
  CloseWindow();
}

float Simulation::randFloat(float min, float max) {
  return min + static_cast<float>(rand()) /
                   (static_cast<float>(RAND_MAX / (max - min)));
}

Vector2 Simulation::getWindArrowAngle(int windDir) {
  switch (windDir) {
  case 0:
    return Vector2{1.0f, 0.0f};
  case 1:
    return Vector2{-1.0f, 0.0f};
  case 2:
    return Vector2{0.0f, 1.0f};
  case 3:
    return Vector2{0.0f, -1.0f};
  default:
    return Vector2{1.0f, 0.0f};
  }
}

void Simulation::spawnParticle(WindParticle &p, int windDirection,
                               const Vector3 &spawnTL, const Vector3 &spawnBR,
                               bool fillScreen) {
  p.isActive = 1;
  p.age = 0.0f;
  p.speedJitter = 1.0f;                 // No jitter for uniform elegant wind
  p.baseSize = randFloat(0.05f, 0.15f); // Thinner elegant lines
  p.alpha = randFloat(0.4f, 0.9f);

  p.pos.x = randFloat(spawnTL.x, spawnBR.x);
  p.pos.y = randFloat(spawnTL.y, spawnBR.y);
  p.pos.z = randFloat(spawnTL.z, spawnBR.z);

  if (fillScreen) {
    // Allow random placement everywhere for initial burst
  } else {
    // Spawn at the incoming edge
    if (windDirection == 0)
      p.pos.x = spawnTL.x;
    else if (windDirection == 1)
      p.pos.x = spawnBR.x;
    else if (windDirection == 2)
      p.pos.y = spawnBR.y;
    else if (windDirection == 3)
      p.pos.y = spawnTL.y;
  }

  p.prevPos = p.pos;
}

void Simulation::rebuildSolverWithRotation() {
  rotatedFoil = foil;
  rotatedFoil.rotateCoordinates(-flow.alpha);
  solver = std::make_unique<zweifoil::Solver>(rotatedFoil);

  zweifoil::Flowconditions solverFlow = flow;
  solverFlow.alpha = 0.0;

  if (current_sim == 0) {
    results = solver->runSimulation(solverFlow);

    if (solver->getInviscidEngine()) {
      auto &cachedGrid = solver->getInviscidEngine()->cachedGrid;
      std::vector<Vector2> floatGrid(cachedGrid.grid.size());
      for (size_t i = 0; i < cachedGrid.grid.size(); ++i) {
        floatGrid[i].x = (float)cachedGrid.grid[i].x;
        floatGrid[i].y = (float)cachedGrid.grid[i].y;
      }
      if (solver->getInviscidEngine()->ssbo_u == 0) {
        solver->getInviscidEngine()->ssbo_u =
            rlLoadShaderBuffer(floatGrid.size() * sizeof(Vector2),
                               floatGrid.data(), RL_DYNAMIC_DRAW);
      } else {
        rlUpdateShaderBuffer(solver->getInviscidEngine()->ssbo_u,
                             floatGrid.data(),
                             floatGrid.size() * sizeof(Vector2), 0);
      }
    }
  } else if (current_sim == 1) {
    // Increased grid size for better geometry approximation
    lbmSolver = std::make_unique<zweifoil::LBMSolver>(128, 64, 64);
    auto &grid = lbmSolver->getGridModifiable();
    const auto &panels = rotatedFoil.getPanels();
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto &panel : panels) {
      minX = std::min({minX, panel.p1.x, panel.p2.x});
      maxX = std::max({maxX, panel.p1.x, panel.p2.x});
      minY = std::min({minY, panel.p1.y, panel.p2.y});
      maxY = std::max({maxY, panel.p1.y, panel.p2.y});
    }
    float cowWidth = std::max(0.1, maxX - minX);
    float cowHeight = std::max(0.1, maxY - minY);
    // Scale so the object takes up at most 25% of grid width and 30% of grid height
    // This gives the fluid plenty of room to bend around the airfoil!
    float lbmScale =
        std::min((128.0f * 0.25f) / cowWidth, (64.0f * 0.30f) / cowHeight);
    cachedLbmScale = lbmScale;
    
    std::cout << "[SIM-LBM] Initializing Volumetric LBM Solver..." << std::endl;
    std::cout << "[SIM-LBM] Grid Size: 128x64x64" << std::endl;
    std::cout << "[SIM-LBM] Computed Grid Scale: " << cachedLbmScale << std::endl;
    
    std::vector<double> sdf2D(grid.NX * grid.NY);
    
#pragma omp parallel for collapse(2)
    for (int y = 0; y < grid.NY; ++y) {
      for (int x = 0; x < grid.NX; ++x) {
        double physX = (x - grid.NX / 2.0) / lbmScale;
        double physY = (y - grid.NY / 2.0) / lbmScale;
        zweifoil::Point2D p{physX, physY};
        bool inside = false;
        double minDist = 1e9;
        for (const auto &panel : panels) {
          // Ray-casting for inside check
          if (((panel.p1.y > p.y) != (panel.p2.y > p.y)) &&
              (p.x < (panel.p2.x - panel.p1.x) * (p.y - panel.p1.y) /
                             (panel.p2.y - panel.p1.y) +
                         panel.p1.x)) {
            inside = !inside;
          }

          // Shortest distance to panel segment
          double l2 = (panel.p2.x - panel.p1.x) * (panel.p2.x - panel.p1.x) +
                      (panel.p2.y - panel.p1.y) * (panel.p2.y - panel.p1.y);
          if (l2 == 0.0) {
            double d = std::sqrt((p.x - panel.p1.x) * (p.x - panel.p1.x) +
                                 (p.y - panel.p1.y) * (p.y - panel.p1.y));
            minDist = std::min(minDist, d);
            continue;
          }

          double t = std::max(
              0.0,
              std::min(1.0, ((p.x - panel.p1.x) * (panel.p2.x - panel.p1.x) +
                             (p.y - panel.p1.y) * (panel.p2.y - panel.p1.y)) /
                                l2));
          double projX = panel.p1.x + t * (panel.p2.x - panel.p1.x);
          double projY = panel.p1.y + t * (panel.p2.y - panel.p1.y);

          double d = std::sqrt((p.x - projX) * (p.x - projX) +
                               (p.y - projY) * (p.y - projY));
          minDist = std::min(minDist, d);
        }
        int idx = y * grid.NX + x;
        sdf2D[idx] = inside ? -minDist : minDist;
      }
    }

    int internalNodes = 0;
#pragma omp parallel for collapse(3) reduction(+:internalNodes)
    for (int z = 0; z < grid.NZ; ++z) {
      for (int y = 0; y < grid.NY; ++y) {
        for (int x = 0; x < grid.NX; ++x) {
          int idx2D = y * grid.NX + x;
          double d2D = sdf2D[idx2D];
          
          double physZ = (z - grid.NZ / 2.0) / lbmScale;
          double spanRadius = 0.5; // Matches the renderSpan proportion
          double dZ = std::abs(physZ) - spanRadius;
          
          double dist3D;
          if (d2D > 0.0 && dZ > 0.0) {
              dist3D = std::sqrt(d2D*d2D + dZ*dZ);
          } else {
              dist3D = std::max(d2D, dZ);
          }
          
          grid.sdf[grid.getScalarIndex(x, y, z)] = (float)(dist3D * lbmScale);
          if (dist3D <= 0.0) internalNodes++;
        }
      }
    }
    
    std::cout << "[SIM-LBM] SDF Voxelization Complete. Internal Nodes: " << internalNodes << std::endl;

    zweifoil::Flowconditions safeLBM = solverFlow;
    safeLBM.V_inf = 0.05;
    safeLBM.kinematic_viscosity = solverFlow.kinematic_viscosity *
                                  (0.05 / std::max(0.0001, solverFlow.V_inf));

    lbmSolver->getGridModifiable().initialize(safeLBM);
    std::cout << "[SIM-LBM] Queuing LBM solver warmup (2500 steps)..." << std::endl;
    warmup_steps_remaining = 2500;
    results = {0.1, 0.05, -0.02};
  }

  // Initialize particles cleanly on the GPU for the new setup
  for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
    wind.particles[i].isActive = 0;
  }
  rlUpdateShaderBuffer(ssbo_particles, wind.particles,
                       targetParticleCount * sizeof(WindParticle), 0);
}

void Simulation::run() {
  const float renderScale = 40.0f;

  while (!WindowShouldClose()) {
    float dt = GetFrameTime();
    if (dt > 0.05f)
      dt = 0.05f;

    if (!ImGui::GetIO().WantCaptureMouse) {
      Vector3 forward = {camera.target.x - camera.position.x,
                         camera.target.y - camera.position.y,
                         camera.target.z - camera.position.z};
      float dist = sqrtf(forward.x * forward.x + forward.y * forward.y +
                         forward.z * forward.z);
      if (dist < 0.1f)
        dist = 0.1f;
      forward.x /= dist;
      forward.y /= dist;
      forward.z /= dist;

      Vector3 up = {0.0f, 1.0f, 0.0f};
      Vector3 right = {forward.y * up.z - forward.z * up.y,
                       forward.z * up.x - forward.x * up.z,
                       forward.x * up.y - forward.y * up.x};
      float rlen =
          sqrtf(right.x * right.x + right.y * right.y + right.z * right.z);
      if (rlen < 0.001f)
        right = {1.0f, 0.0f, 0.0f};
      else {
        right.x /= rlen;
        right.y /= rlen;
        right.z /= rlen;
      }

      Vector3 camUp = {right.y * forward.z - right.z * forward.y,
                       right.z * forward.x - right.x * forward.z,
                       right.x * forward.y - right.y * forward.x};

      // Zoom
      float zoom = GetMouseWheelMove() * (dist * 0.1f);
      if (zoom != 0.0f) {
        if (dist > 900.0f && zoom < 0.0f) {
          zoom = 0.0f; // Prevent zooming past far plane
        }
        camera.position.x += forward.x * zoom;
        camera.position.y += forward.y * zoom;
        camera.position.z += forward.z * zoom;
      }

      // Mouse drag to PAN
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        float panSpeed = dist * 0.0015f;
        camera.position.x -= (right.x * delta.x - camUp.x * delta.y) * panSpeed;
        camera.position.y -= (right.y * delta.x - camUp.y * delta.y) * panSpeed;
        camera.position.z -= (right.z * delta.x - camUp.z * delta.y) * panSpeed;

        camera.target.x -= (right.x * delta.x - camUp.x * delta.y) * panSpeed;
        camera.target.y -= (right.y * delta.x - camUp.y * delta.y) * panSpeed;
        camera.target.z -= (right.z * delta.x - camUp.z * delta.y) * panSpeed;
      }

      // Arrow keys to ORBIT
      float orbitSpeed = 0.05f;
      float dYaw = 0.0f;
      float dPitch = 0.0f;
      if (current_sim == 1) {
          if (IsKeyDown(KEY_LEFT))
            dYaw = -orbitSpeed;
          if (IsKeyDown(KEY_RIGHT))
            dYaw = orbitSpeed;
          if (IsKeyDown(KEY_UP))
            dPitch = orbitSpeed;
          if (IsKeyDown(KEY_DOWN))
            dPitch = -orbitSpeed;
      }

      if (dYaw != 0.0f || dPitch != 0.0f) {
        float theta = atan2f(camera.position.z - camera.target.z,
                             camera.position.x - camera.target.x);
        float phi = asinf((camera.position.y - camera.target.y) / dist);

        theta += dYaw;
        phi += dPitch;

        if (phi > 1.5f)
          phi = 1.5f;
        if (phi < -1.5f)
          phi = -1.5f;

        camera.position.x = camera.target.x + dist * cosf(phi) * cosf(theta);
        camera.position.y = camera.target.y + dist * sinf(phi);
        camera.position.z = camera.target.z + dist * cosf(phi) * sinf(theta);
      }
    }

    if (current_sim == 1 && lbmSolver) {
      zweifoil::Flowconditions safeLBM = flow;
      safeLBM.alpha = 0.0;
      safeLBM.V_inf = 0.05;
      safeLBM.kinematic_viscosity =
          flow.kinematic_viscosity * (0.05 / std::max(0.0001, flow.V_inf));
          
      if (warmup_steps_remaining > 0) {
          int steps = std::min(warmup_steps_remaining, 20);
          for (int i = 0; i < steps; ++i) {
            lbmSolver->step(safeLBM);
          }
          warmup_steps_remaining -= steps;
          if (warmup_steps_remaining == 0) {
              std::cout << "[SIM-LBM] Warmup complete!" << std::endl;
          }
      } else {
          for (int i = 0; i < 15; ++i) {
            lbmSolver->step(safeLBM);
          }
      }
    }

    Vector3 camFwd = {camera.target.x - camera.position.x,
                      camera.target.y - camera.position.y,
                      camera.target.z - camera.position.z};
    float camDist =
        sqrtf(camFwd.x * camFwd.x + camFwd.y * camFwd.y + camFwd.z * camFwd.z);
    float fovRad = camera.fovy * (3.14159f / 180.0f);
    float viewHeight = 2.0f * camDist * tanf(fovRad / 2.0f);
    float viewWidth =
        viewHeight * ((float)GetScreenWidth() / (float)GetScreenHeight());

    float spawnExtX = viewWidth * 2.0f;
    float spawnExtY = viewHeight * 2.0f;
    float spawnExtZ = (current_sim == 0) ? 0.001f : 20.0f;

    if (focusOnModel) {
      const auto &panels = rotatedFoil.getPanels();
      double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
      for (const auto &panel : panels) {
        minX = std::min({minX, panel.p1.x, panel.p2.x});
        maxX = std::max({maxX, panel.p1.x, panel.p2.x});
        minY = std::min({minY, panel.p1.y, panel.p2.y});
        maxY = std::max({maxY, panel.p1.y, panel.p2.y});
      }
      float cWidth = std::max(0.1, maxX - minX) * renderScale;
      float cHeight = std::max(0.1, maxY - minY) * renderScale;
      spawnExtX = cWidth * windEmissionScale;
      spawnExtY = cHeight * windEmissionScale;
      if (current_sim == 1) spawnExtZ = 20.0f;
    }

    Vector3 spawnTL = {camera.target.x - spawnExtX, camera.target.y - spawnExtY,
                       camera.target.z - spawnExtZ};
    Vector3 spawnBR = {camera.target.x + spawnExtX, camera.target.y + spawnExtY,
                       camera.target.z + spawnExtZ};

    // Kill bounds are slightly larger than spawn bounds so they despawn
    // beautifully exactly at the edge of vision
    Vector3 killTL = {camera.target.x - spawnExtX * 1.1f,
                      camera.target.y - spawnExtY * 1.1f,
                      camera.target.z - spawnExtZ * 1.1f};
    Vector3 killBR = {camera.target.x + spawnExtX * 1.1f,
                      camera.target.y + spawnExtY * 1.1f,
                      camera.target.z + spawnExtZ * 1.1f};

    float baseSpeed = 500.0f;
    float speedScale = std::max(0.1f, (float)flow.V_inf);

    // CPU-side continuous spawning has been REMOVED!

    // CPU-side continuous spawning has been REMOVED!
    // The Compute Shader now beautifully recycles particles natively on the GPU
    // when they hit the kill boundaries!

    rlEnableShader(particleComputeShader);

    int maxPLoc = rlGetLocationUniform(particleComputeShader, "maxParticles");
    int dtLoc = rlGetLocationUniform(particleComputeShader, "dt");
    int rsLoc = rlGetLocationUniform(particleComputeShader, "renderScale");
    int simLoc = rlGetLocationUniform(particleComputeShader, "current_sim");
    int vinfLoc = rlGetLocationUniform(particleComputeShader, "v_inf");
    int alphaLoc = rlGetLocationUniform(particleComputeShader, "alpha_angle");
    int windDirLoc =
        rlGetLocationUniform(particleComputeShader, "windDirection");
    int gnxLoc = rlGetLocationUniform(particleComputeShader, "gridNX");
    int gnyLoc = rlGetLocationUniform(particleComputeShader, "gridNY");
    int gnzLoc = rlGetLocationUniform(particleComputeShader, "gridNZ");
    int gscaleLoc = rlGetLocationUniform(particleComputeShader, "gridScale");
    int stlLoc = rlGetLocationUniform(particleComputeShader, "spawnTL");
    int sbrLoc = rlGetLocationUniform(particleComputeShader, "spawnBR");
    int ktlLoc = rlGetLocationUniform(particleComputeShader, "killTL");
    int kbrLoc = rlGetLocationUniform(particleComputeShader, "killBR");
    int bsLoc = rlGetLocationUniform(particleComputeShader, "baseSpeed");

    float fVinf = (float)flow.V_inf;
    float fAlpha = (float)flow.alpha;
    int gridNX = lbmSolver ? lbmSolver->getGrid().NX : 0;
    int gridNY = lbmSolver ? lbmSolver->getGrid().NY : 0;
    int gridNZ = lbmSolver ? lbmSolver->getGrid().NZ : 0;
    const auto &panels = rotatedFoil.getPanels();
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto &panel : panels) {
      minX = std::min({minX, panel.p1.x, panel.p2.x});
      maxX = std::max({maxX, panel.p1.x, panel.p2.x});
      minY = std::min({minY, panel.p1.y, panel.p2.y});
      maxY = std::max({maxY, panel.p1.y, panel.p2.y});
    }
    float cowWidth = std::max(0.1, maxX - minX);
    float cowHeight = std::max(0.1, maxY - minY);
    float lbmScale = cachedLbmScale;
    if (lbmScale <= 0.0f) {
      lbmScale = std::min((128.0f * 0.4f) / cowWidth, (64.0f * 0.5f) / cowHeight);
    }

    rlSetUniform(maxPLoc, &targetParticleCount, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(dtLoc, &dt, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(rsLoc, &renderScale, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(simLoc, &current_sim, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(vinfLoc, &fVinf, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(alphaLoc, &fAlpha, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(windDirLoc, &flow.windDirection, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(gnxLoc, &gridNX, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(gnyLoc, &gridNY, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(gnzLoc, &gridNZ, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(gscaleLoc, &lbmScale, RL_SHADER_UNIFORM_FLOAT, 1);

    int swLoc = rlGetLocationUniform(particleComputeShader, "screenWidth");
    int shLoc = rlGetLocationUniform(particleComputeShader, "screenHeight");
    float fsw = (float)GetScreenWidth();
    float fsh = (float)GetScreenHeight();
    rlSetUniform(swLoc, &fsw, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(shLoc, &fsh, RL_SHADER_UNIFORM_FLOAT, 1);

    float fSpawnTL[3] = {spawnTL.x, spawnTL.y, spawnTL.z};
    float fSpawnBR[3] = {spawnBR.x, spawnBR.y, spawnBR.z};
    float fKillTL[3] = {killTL.x, killTL.y, killTL.z};
    float fKillBR[3] = {killBR.x, killBR.y, killBR.z};
    rlSetUniform(stlLoc, fSpawnTL, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(sbrLoc, fSpawnBR, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(ktlLoc, fKillTL, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(kbrLoc, fKillBR, RL_SHADER_UNIFORM_VEC3, 1);

    float shaderBaseSpeed = baseSpeed * speedScale;
    rlSetUniform(bsLoc, &shaderBaseSpeed, RL_SHADER_UNIFORM_FLOAT, 1);

    rlBindShaderBuffer(ssbo_particles, 5);

    int hasLvpmLoc = rlGetLocationUniform(particleComputeShader, "has_lvpm");
    int hasLvpm = 0;

    if (current_sim == 1 && lbmSolver) {
      rlBindShaderBuffer(lbmSolver->getGrid().ssbo_u, 4);
      rlBindShaderBuffer(lbmSolver->getGrid().ssbo_sdf, 2);
    } else if (current_sim == 0 && solver && solver->getInviscidEngine() &&
               solver->getInviscidEngine()->ssbo_u != 0) {
      rlBindShaderBuffer(solver->getInviscidEngine()->ssbo_u, 6);
      hasLvpm = 1;
    }

    rlSetUniform(hasLvpmLoc, &hasLvpm, RL_SHADER_UNIFORM_INT, 1);

    int groupsX = (targetParticleCount + 255) / 256;
    rlComputeShaderDispatch(groupsX, 1, 1);
    rlDisableShader();

    if (glad_glMemoryBarrier) {
      glad_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT |
                           GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
    }

    BeginDrawing();
    ClearBackground(BLACK);

    BeginMode3D(camera);

    BeginBlendMode(BLEND_ADDITIVE);

    // Render the car outline
    if (solver) {
      for (const auto &panel : rotatedFoil.getPanels()) {
        Vector3 p1 = {panel.p1.x * renderScale, -panel.p1.y * renderScale,
                      0.0f};
        Vector3 p2 = {panel.p2.x * renderScale, -panel.p2.y * renderScale,
                      0.0f};
        DrawLine3D(p1, p2, (Color){0, 255, 255, 255});
      }
    }

    // Flush Raylib's internal render batch before binding our custom VAO!
    rlDrawRenderBatchActive();

    // Disable backface culling because our screen-space quads might be facing
    // the "wrong" way
    rlDisableBackfaceCulling();

    rlEnableShader(particleRenderShader);

    int vpLoc = rlGetLocationUniform(particleRenderShader, "mvp");
    Matrix matMVP =
        MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    rlSetUniformMatrix(vpLoc, matMVP);

    int texLoc = rlGetLocationUniform(particleRenderShader, "texture0");
    int texUnit = 0;
    rlSetUniform(texLoc, &texUnit, RL_SHADER_UNIFORM_INT, 1);
    rlActiveTextureSlot(0);
    rlEnableTexture(particleMaterial.maps[MATERIAL_MAP_DIFFUSE].texture.id);

    float fVinf_render = (float)flow.V_inf;
    int r_ktlLoc = rlGetLocationUniform(particleRenderShader, "killTL");
    int r_kbrLoc = rlGetLocationUniform(particleRenderShader, "killBR");
    int r_vinfLoc = rlGetLocationUniform(particleRenderShader, "v_inf");

    float r_fKillTL[3] = {killTL.x, killTL.y, killTL.z};
    float r_fKillBR[3] = {killBR.x, killBR.y, killBR.z};
    rlSetUniform(r_ktlLoc, r_fKillTL, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(r_kbrLoc, r_fKillBR, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(r_vinfLoc, &fVinf_render, RL_SHADER_UNIFORM_FLOAT, 1);

    Matrix matMvp =
        MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    int mvpLoc = rlGetLocationUniform(particleRenderShader, "mvp");
    rlSetUniformMatrix(mvpLoc, matMvp);

    rlBindShaderBuffer(ssbo_particles, 5);

    rlEnableVertexArray(quadMesh.vaoId);
    if (quadMesh.indices != nullptr) {
      rlDrawVertexArrayElementsInstanced(0, quadMesh.triangleCount * 3, nullptr,
                                         targetParticleCount);
    } else {
      rlDrawVertexArrayInstanced(0, quadMesh.vertexCount, targetParticleCount);
    }
    rlDisableVertexArray();

    rlDisableShader();

    EndBlendMode();

    if (!panels.empty()) {
      std::vector<Vector3> airfoilPoints;
      int leIndex = 0;
      float minX = 1e9f;
      for (int i = 0; i < panels.size(); ++i) {
        Vector3 p = {(float)(panels[i].p1.x * renderScale),
                     (float)(-panels[i].p1.y * renderScale), 0.0f};
        airfoilPoints.push_back(p);
        if (p.x < minX) {
          minX = p.x;
          leIndex = i;
        }
      }

      float renderSpan = (current_sim == 1) ? (1.0f * renderScale / 2.0f) : 0.0f;
      float zOffsets[2] = {-renderSpan, renderSpan};
      int numOffsets = (current_sim == 1) ? 2 : 1;
      
      for (int zo = 0; zo < numOffsets; ++zo) {
          float zOffset = (current_sim == 1) ? zOffsets[zo] : 0.0f;
          Vector3 le = airfoilPoints[leIndex];
          le.z = zOffset;
          for (size_t i = 0; i < airfoilPoints.size(); ++i) {
            Vector3 p1 = airfoilPoints[i]; p1.z = zOffset;
            Vector3 p2 = airfoilPoints[(i + 1) % airfoilPoints.size()]; p2.z = zOffset;
            DrawTriangle3D(le, p1, p2, (Color){235, 240, 245, 255});
            DrawTriangle3D(le, p2, p1, (Color){235, 240, 245, 255});
          }
      }
      
      if (current_sim == 1) {
          for (size_t i = 0; i < airfoilPoints.size(); ++i) {
              Vector3 p1 = airfoilPoints[i]; 
              Vector3 p2 = airfoilPoints[(i + 1) % airfoilPoints.size()]; 
              Vector3 p1f = {p1.x, p1.y, renderSpan};
              Vector3 p1b = {p1.x, p1.y, -renderSpan};
              Vector3 p2f = {p2.x, p2.y, renderSpan};
              Vector3 p2b = {p2.x, p2.y, -renderSpan};
              DrawTriangle3D(p1f, p1b, p2b, (Color){200, 210, 220, 255});
              DrawTriangle3D(p1f, p2b, p2f, (Color){200, 210, 220, 255});
          }
      }
    }

    for (const auto &panel : rotatedFoil.getPanels()) {
      float renderSpan = (current_sim == 1) ? (1.0f * renderScale / 2.0f) : 0.0f;
      float zOffsets[2] = {-renderSpan, renderSpan};
      int numOffsets = (current_sim == 1) ? 2 : 1;
      for (int zo = 0; zo < numOffsets; ++zo) {
          float zOffset = (current_sim == 1) ? zOffsets[zo] : 0.0f;
          Vector3 p1 = {(float)(panel.p1.x * renderScale),
                        (float)(-panel.p1.y * renderScale), zOffset};
          Vector3 p2 = {(float)(panel.p2.x * renderScale),
                        (float)(-panel.p2.y * renderScale), zOffset};
          DrawLine3D(p1, p2, (Color){50, 80, 120, 255});
      }
    }

    EndMode3D();

    {
      float arrowSize = 35.0f;
      Vector2 origin = {80.0f, 80.0f};
      Vector2 dir = windArrowDir;
      Vector2 tip = {origin.x + dir.x * arrowSize,
                     origin.y + dir.y * arrowSize};

      DrawLineEx(origin, tip, 3.0f, ColorAlpha(RAYWHITE, 0.6f));

      float ang = atan2f(dir.y, dir.x);
      float ha1 = ang + 2.3f, ha2 = ang - 2.3f;
      Vector2 h1 = {tip.x + cosf(ha1) * 10.0f, tip.y + sinf(ha1) * 10.0f};
      Vector2 h2 = {tip.x + cosf(ha2) * 10.0f, tip.y + sinf(ha2) * 10.0f};
      DrawTriangle(tip, h1, h2, ColorAlpha(RAYWHITE, 0.7f));
    }

    if (current_sim == 1 && warmup_steps_remaining > 0) {
        DrawText(TextFormat("Warming up LBM Flow Field... %d steps remaining", warmup_steps_remaining), 
                 GetScreenWidth() / 2 - 250, GetScreenHeight() - 60, 24, RED);
    }

    rlImGuiBegin();
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        ImGui::InputText("##file", fileBuf, 256);
        if (ImGui::MenuItem("Load")) {
          if (foil.loadFromFile(fileBuf)) {
            rebuildSolverWithRotation();
            for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i)
              wind.particles[i].isActive = 0;
            wind.activeCount = 0;
            particleSpawnTimer = 0.0f;
          }
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Simulation Type")) {
        if (ImGui::MenuItem("Viscous 2D Panel Method", "", current_sim == 0)) {
          current_sim = 0;
          rebuildSolverWithRotation();
        }
        if (ImGui::MenuItem("3D Volumetric LBM", "", current_sim == 1)) {
          current_sim = 1;
          rebuildSolverWithRotation();
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Wind Direction")) {
        const char *dirNames[] = {"From Left    ", "From Right   ",
                                  "From Top     ", "From Bottom  "};
        for (int d = 0; d < 4; ++d) {
          if (ImGui::MenuItem(dirNames[d], "", flow.windDirection == d)) {
            flow.windDirection = d;
            windArrowDir = getWindArrowAngle(d);
            rebuildSolverWithRotation();
            for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i)
              wind.particles[i].isActive = 0;
            wind.activeCount = 0;
            particleSpawnTimer = 0.0f;
          }
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }

    ImGui::PushStyleColor(ImGuiCol_WindowBg,
                          ImVec4(0.08f, 0.08f, 0.12f, 0.95f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.9f, 1.0f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.18f, 0.18f, 0.28f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.7f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(0.4f, 0.6f, 0.8f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.12f, 0.12f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive,
                          ImVec4(0.12f, 0.12f, 0.18f, 1.0f));

    ImGui::SetNextWindowPos(ImVec2(10, 35), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380, 420), ImGuiCond_FirstUseEver);
    ImGui::Begin("Workspace");

    ImGui::DragInt("Particle Count", &targetParticleCount, 1000.0f, 1000,
                   WindSystem::MAX_PARTICLES);
    ImGui::Checkbox("Focus Wind on Model", &focusOnModel);
    if (focusOnModel) {
      ImGui::DragFloat("Wind Emission Scale", &windEmissionScale, 0.1f, 0.5f,
                       10.0f);
    }
    ImGui::Separator();

    if (current_sim == 0) {
      float alpha = static_cast<float>(flow.alpha);
      float velocity = static_cast<float>(flow.V_inf);
      float kin_visc = static_cast<float>(flow.kinematic_viscosity);
      bool changed = false;

      if (ImGui::DragFloat("AoA (deg)", &alpha, 0.1f, -20.0f, 20.0f)) {
        flow.alpha = alpha;
        changed = true;
      }
      if (ImGui::DragFloat("Velocity", &velocity, 0.1f, 0.1f, 100.0f)) {
        flow.V_inf = velocity;
        changed = true;
      }
      if (ImGui::InputFloat("Viscosity", &kin_visc, 0.0f, 0.0f, "%.6f")) {
        flow.kinematic_viscosity = kin_visc;
        changed = true;
      }

      if (changed)
        rebuildSolverWithRotation();

      if (ImGui::Button("Compute Flow")) {
        rebuildSolverWithRotation();
        std::cout << "\nResults:\n"
                  << " Cl: " << results.cl << "\n"
                  << " Cd: " << results.cd << "\n"
                  << " Cm: " << results.cm << "\n";
      }
      ImGui::Separator();

      ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
                         "Aerodynamic Coefficients:");
      ImGui::Text("Cl: %.4f", results.cl);
      ImGui::Text("Cd: %.4f", results.cd);
      ImGui::Text("Cm: %.4f", results.cm);
    } else if (current_sim == 1) {
      float alpha = static_cast<float>(flow.alpha);
      float velocity = static_cast<float>(flow.V_inf);
      float kin_visc = static_cast<float>(flow.kinematic_viscosity);
      bool changed = false;

      if (ImGui::DragFloat("AoA (deg)", &alpha, 0.1f, -20.0f, 20.0f)) {
        flow.alpha = alpha;
        changed = true;
      }
      if (ImGui::DragFloat("Velocity", &velocity, 0.1f, 0.1f, 100.0f)) {
        flow.V_inf = velocity;
        changed = true;
      }
      if (ImGui::InputFloat("Viscosity", &kin_visc, 0.0f, 0.0f, "%.6f")) {
        flow.kinematic_viscosity = kin_visc;
        changed = true;
      }

      if (changed)
        rebuildSolverWithRotation();

      ImGui::Separator();
      ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f),
                         "3D Volumetric LBM Active");
      if (lbmSolver) {
        ImGui::Text("Grid Resolution: %d x %d x %d", lbmSolver->getGrid().NX,
                    lbmSolver->getGrid().NY, lbmSolver->getGrid().NZ);
      }
      ImGui::Text("Viscous Flow: Navier-Stokes");

      ImGui::Separator();
      ImGui::TextWrapped(
          "Notice: Particles slowing and converging at the surface is the "
          "physical no-slip boundary condition in viscous flow.");
    }

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Wind Velocity Meter");

    displayedVelocity += ((float)flow.V_inf - displayedVelocity) * 0.1f;
    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 gp = ImGui::GetCursorScreenPos();

    float gR = 70.0f;
    float cx = gp.x + gR + 10.0f, cy = gp.y + gR + 10.0f;
    ImVec2 ctr(cx, cy);

    dl->AddCircleFilled(ctr, gR, IM_COL32(20, 20, 30, 255), 64);
    dl->AddCircle(ctr, gR, IM_COL32(100, 150, 200, 80), 64, 2.0f);

    float sFrac = std::min(1.0f, displayedVelocity / 100.0f);
    float sAng = -PI * 0.75f, eAng = PI * 0.75f, sweep = (eAng - sAng) * sFrac;
    ImU32 arcCol;
    if (sFrac < 0.4f) {
      float t = sFrac / 0.4f;
      arcCol = IM_COL32((int)(100 + t * 100), (int)(150 + t * 50), 255, 255);
    } else {
      float t = (sFrac - 0.4f) / 0.6f;
      arcCol = IM_COL32((int)(200 + t * 55), (int)(200 - t * 50), 255, 255);
    }

    const int segs = 48;
    float aT = 8.0f;
    for (int s = 0; s < segs; ++s) {
      float a1 = sAng + sweep * s / segs, a2 = sAng + sweep * (s + 1) / segs;
      float iR = gR - aT;
      dl->AddQuadFilled({cx + cosf(a1) * iR, cy + sinf(a1) * iR},
                        {cx + cosf(a2) * iR, cy + sinf(a2) * iR},
                        {cx + cosf(a2) * gR, cy + sinf(a2) * gR},
                        {cx + cosf(a1) * gR, cy + sinf(a1) * gR}, arcCol);
    }
    for (int t = 0; t <= 10; ++t) {
      float fr = t / 10.0f, ang = sAng + (eAng - sAng) * fr;
      float ti = gR - (t % 2 == 0 ? 16.0f : 12.0f), to = gR - 4.0f;
      dl->AddLine({cx + cosf(ang) * ti, cy + sinf(ang) * ti},
                  {cx + cosf(ang) * to, cy + sinf(ang) * to},
                  IM_COL32(100, 150, 200, 120), 1.5f);
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f", displayedVelocity);
    ImVec2 ts = ImGui::CalcTextSize(buf);
    dl->AddText({cx - ts.x * 0.5f, cy - ts.y * 0.5f - 5.0f},
                IM_COL32(255, 255, 255, 255), buf);

    ImVec2 us = ImGui::CalcTextSize("m/s");
    dl->AddText({cx - us.x * 0.5f, cy + 10.0f}, IM_COL32(150, 180, 200, 180),
                "m/s");

    const char *fromStr[] = {"From Left", "From Right", "From Top",
                             "From Bottom"};
    ImGui::Dummy(ImVec2(0, gR * 2 + 30.0f));
    ImGui::Text("Direction: %s", fromStr[flow.windDirection]);

    float bw = 300.0f;
    ImVec2 bp = ImGui::GetCursorScreenPos();
    dl->AddRectFilled(bp, {bp.x + bw, bp.y + 8.0f}, IM_COL32(25, 25, 35, 255));
    float fw = bw * sFrac;
    for (int bx = 0; bx < (int)fw; ++bx) {
      float fx = bx / bw;
      ImU32 col;
      if (fx < 0.5f)
        col = IM_COL32(100, (int)(150 + 200 * fx), 255, 255);
      else
        col = IM_COL32((int)(100 + 155 * (fx - 0.5f) * 2), 255, 255, 255);
      dl->AddRectFilled({bp.x + bx, bp.y}, {bp.x + bx + 1, bp.y + 8.0f}, col);
    }
    ImGui::Dummy(ImVec2(0, 12.0f));

    ImGui::End();
    ImGui::PopStyleColor(7);

    rlImGuiEnd();
    EndDrawing();
  }
}