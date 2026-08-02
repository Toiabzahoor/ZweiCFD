#include "simulation.hpp"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <omp.h>

extern "C" {
    extern void (*glad_glMemoryBarrier)(unsigned int barriers);
}
#define GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT 0x00000001
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000

#include "rlImGui.h"
#include "imgui.h"
#include "shaders.hpp"
#include "rlgl.h"
#include "raymath.h"

Simulation::Simulation(int argc, char* argv[]) {
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
    
    // IMPORTANT: OpenGL Context is now ready, we can safely initialize shaders and buffers!
    rebuildSolverWithRotation();
    
    rlImGuiSetup(true);

    camera = { 0 };
    camera.target = Vector2{ 0.0f, 0.0f };
    camera.offset = Vector2{ 1280 / 2.0f, 720 / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    strncpy(fileBuf, filename.c_str(), sizeof(fileBuf));

    current_sim = 0;
    targetParticleCount = 16000;

    for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
        wind.particles[i].isActive = 0;
    }
    
    particleSpawnTimer = 0.0f;
    displayedVelocity = (float)flow.V_inf;
    windArrowDir = getWindArrowAngle(flow.windDirection);

    // Initialize Shaders
    unsigned int csId = rlLoadShader(particle_update_comp, RL_COMPUTE_SHADER);
    particleComputeShader = rlLoadShaderProgramCompute(csId);

    particleRenderShader = rlLoadShaderProgram(particle_render_vs, particle_render_fs);

    quadMesh = GenMeshPlane(1.0f, 1.0f, 1, 1); // Generate a plane quad
    UploadMesh(&quadMesh, false); // Actually upload it to the GPU!
    particleMaterial = LoadMaterialDefault();
    particleMaterial.shader.id = particleRenderShader;

    ssbo_particles = rlLoadShaderBuffer(WindSystem::MAX_PARTICLES * sizeof(WindParticle), wind.particles, RL_DYNAMIC_DRAW);
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
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

Vector2 Simulation::getWindArrowAngle(int windDir) {
    switch (windDir) {
        case 0: return Vector2{1.0f, 0.0f};
        case 1: return Vector2{-1.0f, 0.0f};
        case 2: return Vector2{0.0f, 1.0f};
        case 3: return Vector2{0.0f, -1.0f};
        default: return Vector2{1.0f, 0.0f};
    }
}

void Simulation::spawnParticle(WindParticle& p, int windDirection, const Vector2& spawnTL, const Vector2& spawnBR, bool fillScreen) {
    p.isActive = 1;
    p.age = 0.0f;
    p.speedJitter = randFloat(0.9f, 1.1f);
    p.baseSize = randFloat(0.2f, 0.6f);
    p.alpha = randFloat(0.4f, 0.9f);
    
    p.pos.x = randFloat(spawnTL.x, spawnBR.x);
    p.pos.y = randFloat(spawnTL.y, spawnBR.y);
    
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
            auto& cachedGrid = solver->getInviscidEngine()->cachedGrid;
            std::vector<Vector2> floatGrid(cachedGrid.grid.size());
            for (size_t i = 0; i < cachedGrid.grid.size(); ++i) {
                floatGrid[i].x = (float)cachedGrid.grid[i].x;
                floatGrid[i].y = (float)cachedGrid.grid[i].y;
            }
            if (solver->getInviscidEngine()->ssbo_u == 0) {
                solver->getInviscidEngine()->ssbo_u = rlLoadShaderBuffer(floatGrid.size() * sizeof(Vector2), floatGrid.data(), RL_DYNAMIC_DRAW);
            } else {
                rlUpdateShaderBuffer(solver->getInviscidEngine()->ssbo_u, floatGrid.data(), floatGrid.size() * sizeof(Vector2), 0);
            }
        }
    } else if (current_sim == 1) {
        // Reduced grid size for interactive performance
        lbmSolver = std::make_unique<zweifoil::LBMSolver>(64, 32, 32);
        auto& grid = lbmSolver->getGridModifiable();
        const auto& panels = rotatedFoil.getPanels();
        #pragma omp parallel for collapse(3)
        for (int z = 0; z < grid.NZ; ++z) {
            for (int y = 0; y < grid.NY; ++y) {
                for (int x = 0; x < grid.NX; ++x) {
                    double physX = (x - 32.0) / 20.0;
                    double physY = -(y - 16.0) / 20.0;
                    zweifoil::Point2D p{physX, physY};
                    bool inside = false;
                    for (const auto& panel : panels) {
                        if (((panel.p1.y > p.y) != (panel.p2.y > p.y)) &&
                            (p.x < (panel.p2.x - panel.p1.x) * (p.y - panel.p1.y) / (panel.p2.y - panel.p1.y) + panel.p1.x)) {
                            inside = !inside;
                        }
                    }
                    if (inside) {
                        grid.is_solid[grid.getScalarIndex(x, y, z)] = true;
                    }
                }
            }
        }
        
        zweifoil::Flowconditions safeLBM = solverFlow;
        safeLBM.V_inf = 0.05; 
        safeLBM.kinematic_viscosity = 0.1; 
        
        lbmSolver->getGridModifiable().initialize(safeLBM);
        for (int i = 0; i < 10; ++i) {
            lbmSolver->step(safeLBM);
        }
        results = {0.1, 0.05, -0.02};
    }
}

void Simulation::run() {
    const float renderScale = 400.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 delta = GetMouseDelta();
            delta.x *= -1.0f / camera.zoom;
            delta.y *= -1.0f / camera.zoom;
            camera.target.x += delta.x;
            camera.target.y += delta.y;
        }

        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
            camera.offset = GetMousePosition();
            camera.target = mouseWorldPos;
            camera.zoom += (wheel * 0.125f);
            if (camera.zoom < 0.125f) camera.zoom = 0.125f;
        }

        if (current_sim == 1 && lbmSolver) {
            zweifoil::Flowconditions safeLBM = flow;
            safeLBM.alpha = 0.0;
            safeLBM.V_inf = 0.05;
            safeLBM.kinematic_viscosity = 0.1;
            lbmSolver->step(safeLBM);
        }

        Vector2 tlWorld = GetScreenToWorld2D({0, 0}, camera);
        Vector2 brWorld = GetScreenToWorld2D({(float)GetScreenWidth(), (float)GetScreenHeight()}, camera);
        Vector2 spawnTL = GetScreenToWorld2D({-1000, -1000}, camera);
        Vector2 spawnBR = GetScreenToWorld2D({(float)GetScreenWidth() + 1000, (float)GetScreenHeight() + 1000}, camera);
        Vector2 killTL = GetScreenToWorld2D({-2000, -2000}, camera);
        Vector2 killBR = GetScreenToWorld2D({(float)GetScreenWidth() + 2000, (float)GetScreenHeight() + 2000}, camera);

        float baseSpeed = 300.0f;
        float speedScale = std::max(0.1f, (float)flow.V_inf);
        float spawnRate = targetParticleCount * 0.5f; 
        if (spawnRate < 2000.0f) spawnRate = 2000.0f;
        particleSpawnTimer += spawnRate * dt;

        // Fast Ring-Buffer Spawning (No GPU->CPU Sync Required!)
        static int nextSpawnIndex = 0;
        static bool firstRun = true;
        
        if (firstRun) {
            // Initial burst to fill the screen
            for (int i = 0; i < targetParticleCount; ++i) {
                spawnParticle(wind.particles[i], flow.windDirection, spawnTL, spawnBR, true);
            }
            rlUpdateShaderBuffer(ssbo_particles, wind.particles, targetParticleCount * sizeof(WindParticle), 0);
            firstRun = false;
        } else {
            // Continuous flow
            int spawnCount = (int)particleSpawnTimer;
            if (spawnCount > targetParticleCount) spawnCount = targetParticleCount;
            particleSpawnTimer -= spawnCount;
            
            for (int i = 0; i < spawnCount; ++i) {
                int idx = nextSpawnIndex;
                spawnParticle(wind.particles[idx], flow.windDirection, spawnTL, spawnBR, false);
                rlUpdateShaderBuffer(ssbo_particles, &wind.particles[idx], sizeof(WindParticle), idx * sizeof(WindParticle));
                nextSpawnIndex = (nextSpawnIndex + 1) % targetParticleCount;
            }
        }

        // --- DISPATCH COMPUTE SHADER FOR PARTICLES ---
        rlEnableShader(particleComputeShader);
        
        int maxPLoc = rlGetLocationUniform(particleComputeShader, "maxParticles");
        int dtLoc = rlGetLocationUniform(particleComputeShader, "dt");
        int rsLoc = rlGetLocationUniform(particleComputeShader, "renderScale");
        int simLoc = rlGetLocationUniform(particleComputeShader, "current_sim");
        int vinfLoc = rlGetLocationUniform(particleComputeShader, "v_inf");
        int alphaLoc = rlGetLocationUniform(particleComputeShader, "alpha_angle");
        int windDirLoc = rlGetLocationUniform(particleComputeShader, "windDirection");
        int gnxLoc = rlGetLocationUniform(particleComputeShader, "gridNX");
        int gnyLoc = rlGetLocationUniform(particleComputeShader, "gridNY");
        int stlLoc = rlGetLocationUniform(particleComputeShader, "spawnTL");
        int sbrLoc = rlGetLocationUniform(particleComputeShader, "spawnBR");
        int ktlLoc = rlGetLocationUniform(particleComputeShader, "killTL");
        int kbrLoc = rlGetLocationUniform(particleComputeShader, "killBR");
        
        float fVinf = (float)flow.V_inf;
        float fAlpha = (float)flow.alpha;
        int gridNX = lbmSolver ? lbmSolver->getGrid().NX : 0;
        int gridNY = lbmSolver ? lbmSolver->getGrid().NY : 0;

        rlSetUniform(maxPLoc, &targetParticleCount, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(dtLoc, &dt, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(rsLoc, &renderScale, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(simLoc, &current_sim, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(vinfLoc, &fVinf, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(alphaLoc, &fAlpha, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(windDirLoc, &flow.windDirection, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(gnxLoc, &gridNX, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(gnyLoc, &gridNY, RL_SHADER_UNIFORM_INT, 1);
        
        int swLoc = rlGetLocationUniform(particleComputeShader, "screenWidth");
        int shLoc = rlGetLocationUniform(particleComputeShader, "screenHeight");
        float fsw = (float)GetScreenWidth();
        float fsh = (float)GetScreenHeight();
        rlSetUniform(swLoc, &fsw, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(shLoc, &fsh, RL_SHADER_UNIFORM_FLOAT, 1);
        
        float fSpawnTL[2] = {spawnTL.x, spawnTL.y};
        float fSpawnBR[2] = {spawnBR.x, spawnBR.y};
        float fKillTL[2] = {killTL.x, killTL.y};
        float fKillBR[2] = {killBR.x, killBR.y};
        rlSetUniform(stlLoc, fSpawnTL, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(sbrLoc, fSpawnBR, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(ktlLoc, fKillTL, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(kbrLoc, fKillBR, RL_SHADER_UNIFORM_VEC2, 1);

        rlBindShaderBuffer(ssbo_particles, 5);
        
        if (current_sim == 1 && lbmSolver) {
            rlBindShaderBuffer(lbmSolver->getGrid().ssbo_u, 4);
            rlBindShaderBuffer(lbmSolver->getGrid().ssbo_is_solid, 2);
        } else if (current_sim == 0 && solver && solver->getInviscidEngine() && solver->getInviscidEngine()->ssbo_u != 0) {
            rlBindShaderBuffer(solver->getInviscidEngine()->ssbo_u, 6);
        }
        
        int groupsX = (targetParticleCount + 255) / 256;
        rlComputeShaderDispatch(groupsX, 1, 1);
        rlDisableShader();
        
        // Ensure GPU finishes writing to the SSBO before the vertex shader attempts to read it
        if (glad_glMemoryBarrier) {
            glad_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
        }

        BeginDrawing();
        ClearBackground(Color{235, 240, 245, 255});
        
        BeginMode2D(camera);
        
        BeginBlendMode(BLEND_ALPHA);
        
        // --- RENDER PARTICLES VIA INSTANCING ---
        rlEnableShader(particleRenderShader);
        
        int vpLoc = rlGetLocationUniform(particleRenderShader, "mvp");
        Matrix matMVP = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
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
        
        float r_fKillTL[2] = {killTL.x, killTL.y};
        float r_fKillBR[2] = {killBR.x, killBR.y};
        rlSetUniform(r_ktlLoc, r_fKillTL, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(r_kbrLoc, r_fKillBR, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(r_vinfLoc, &fVinf_render, RL_SHADER_UNIFORM_FLOAT, 1);
        
        rlBindShaderBuffer(ssbo_particles, 5);
        
        rlEnableVertexArray(quadMesh.vaoId);
        if (quadMesh.indices != nullptr) {
            rlDrawVertexArrayElementsInstanced(0, quadMesh.triangleCount * 3, nullptr, targetParticleCount);
        } else {
            rlDrawVertexArrayInstanced(0, quadMesh.vertexCount, targetParticleCount);
        }
        rlDisableVertexArray();
        
        rlDisableShader();
        
        EndBlendMode();

        // Mask out particles inside the airfoil by drawing it filled with the background color
        auto panels = rotatedFoil.getPanels();
        if (!panels.empty()) {
            std::vector<Vector2> airfoilPoints;
            int leIndex = 0;
            float minX = 1e9f;
            for (int i = 0; i < panels.size(); ++i) {
                Vector2 p = { (float)(panels[i].p1.x * renderScale), (float)(-panels[i].p1.y * renderScale) };
                airfoilPoints.push_back(p);
                if (p.x < minX) {
                    minX = p.x;
                    leIndex = i;
                }
            }
            
            // Draw triangles manually from the Leading Edge (LE) to every segment
            // This perfectly fills the airfoil shape without bleeding or folding over itself
            Vector2 le = airfoilPoints[leIndex];
            for (size_t i = 0; i < airfoilPoints.size(); ++i) {
                Vector2 p1 = airfoilPoints[i];
                Vector2 p2 = airfoilPoints[(i + 1) % airfoilPoints.size()];
                DrawTriangle(le, p2, p1, (Color){235, 240, 245, 255}); // Winding order counter-clockwise? Raylib culls backfaces sometimes, so draw both to be safe
                DrawTriangle(le, p1, p2, (Color){235, 240, 245, 255});
            }
        }

        for (const auto& panel : rotatedFoil.getPanels()) {
            Vector2 p1 = { (float)(panel.p1.x * renderScale), (float)(-panel.p1.y * renderScale)};
            Vector2 p2 = { (float)(panel.p2.x * renderScale), (float)(-panel.p2.y * renderScale)};
            DrawLineEx(p1, p2, 6.0f, ColorAlpha((Color){80, 120, 150, 100}, 0.4f));
            DrawLineEx(p1, p2, 2.0f, ColorAlpha((Color){220, 240, 255, 255}, 0.9f));
        }

        {
            float arrowSize = 35.0f;
            Vector2 origin = { tlWorld.x + 45.0f, tlWorld.y + 45.0f };
            Vector2 dir = windArrowDir;
            Vector2 tip = { origin.x + dir.x * arrowSize, origin.y + dir.y * arrowSize };
            
            DrawLineEx(origin, tip, 3.0f, ColorAlpha(RAYWHITE, 0.6f));
            
            float ang = atan2f(dir.y, dir.x);
            float ha1 = ang + 2.3f, ha2 = ang - 2.3f;
            Vector2 h1 = { tip.x + cosf(ha1) * 10.0f, tip.y + sinf(ha1) * 10.0f };
            Vector2 h2 = { tip.x + cosf(ha2) * 10.0f, tip.y + sinf(ha2) * 10.0f };
            DrawTriangle(tip, h1, h2, ColorAlpha(RAYWHITE, 0.7f));
        }

        EndMode2D();

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
                const char* dirNames[] = {"From Left    ", "From Right   ", "From Top     ", "From Bottom  "};
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

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.12f, 0.95f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.9f, 1.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.18f, 0.18f, 0.28f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.7f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.6f, 0.8f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.12f, 0.12f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.12f, 0.12f, 0.18f, 1.0f));

        ImGui::SetNextWindowPos(ImVec2(10, 35), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 420), ImGuiCond_FirstUseEver);
        ImGui::Begin("Workspace");

        ImGui::SliderInt("Particle Count", &targetParticleCount, 1000, WindSystem::MAX_PARTICLES);
        ImGui::Separator();

        if (current_sim == 0) {
            float alpha = static_cast<float>(flow.alpha);
            float velocity = static_cast<float>(flow.V_inf);
            float kin_visc = static_cast<float>(flow.kinematic_viscosity);
            bool changed = false;

            if (ImGui::SliderFloat("AoA (deg)", &alpha, -20.0f, 20.0f)) {
                flow.alpha = alpha; changed = true;
            }
            if (ImGui::SliderFloat("Velocity", &velocity, 0.1f, 100.0f)) {
                flow.V_inf = velocity; changed = true;
            }
            if (ImGui::InputFloat("Viscosity", &kin_visc, 0.0f, 0.0f, "%.6f")) {
                flow.kinematic_viscosity = kin_visc; changed = true;
            }

            if (changed) rebuildSolverWithRotation();

            if (ImGui::Button("Compute Flow")) {
                rebuildSolverWithRotation();
                std::cout << "\nResults:\n"
                          << " Cl: " << results.cl << "\n"
                          << " Cd: " << results.cd << "\n"
                          << " Cm: " << results.cm << "\n";
            }
            ImGui::Separator();
            
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "Aerodynamic Coefficients:");
            ImGui::Text("Cl: %.4f", results.cl);
            ImGui::Text("Cd: %.4f", results.cd);
            ImGui::Text("Cm: %.4f", results.cm);
        } else if (current_sim == 1) {
            float alpha = static_cast<float>(flow.alpha);
            float velocity = static_cast<float>(flow.V_inf);
            float kin_visc = static_cast<float>(flow.kinematic_viscosity);
            bool changed = false;

            if (ImGui::SliderFloat("AoA (deg)", &alpha, -20.0f, 20.0f)) {
                flow.alpha = alpha; changed = true;
            }
            if (ImGui::SliderFloat("Velocity", &velocity, 0.1f, 100.0f)) {
                flow.V_inf = velocity; changed = true;
            }
            if (ImGui::InputFloat("Viscosity", &kin_visc, 0.0f, 0.0f, "%.6f")) {
                flow.kinematic_viscosity = kin_visc; changed = true;
            }

            if (changed) rebuildSolverWithRotation();

            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "3D Volumetric LBM Active");
            ImGui::Text("Grid Resolution: 64 x 32 x 32");
            ImGui::Text("Viscous Flow: Navier-Stokes");
            
            ImGui::Separator();
            ImGui::TextWrapped("Notice: Particles slowing and converging at the surface is the physical no-slip boundary condition in viscous flow.");
        }
            
        ImGui::Separator();
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Wind Velocity Meter");
            
            displayedVelocity += ((float)flow.V_inf - displayedVelocity) * 0.1f;
            ImDrawList* dl = ImGui::GetWindowDrawList();
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
                arcCol = IM_COL32((int)(100+t*100), (int)(150+t*50), 255, 255);
            } else {
                float t = (sFrac-0.4f)/0.6f;
                arcCol = IM_COL32((int)(200+t*55), (int)(200-t*50), 255, 255);
            }
            
            const int segs = 48;
            float aT = 8.0f;
            for (int s = 0; s < segs; ++s) {
                float a1 = sAng + sweep*s/segs, a2 = sAng + sweep*(s+1)/segs;
                float iR = gR - aT;
                dl->AddQuadFilled(
                    {cx+cosf(a1)*iR, cy+sinf(a1)*iR},
                    {cx+cosf(a2)*iR, cy+sinf(a2)*iR},
                    {cx+cosf(a2)*gR, cy+sinf(a2)*gR},
                    {cx+cosf(a1)*gR, cy+sinf(a1)*gR}, arcCol
                );
            }
            for (int t = 0; t <= 10; ++t) {
                float fr = t/10.0f, ang = sAng + (eAng-sAng)*fr;
                float ti = gR - (t%2==0?16.0f:12.0f), to = gR - 4.0f;
                dl->AddLine({cx+cosf(ang)*ti, cy+sinf(ang)*ti},
                            {cx+cosf(ang)*to, cy+sinf(ang)*to},
                            IM_COL32(100,150,200,120), 1.5f);
            }
            
            char buf[32]; snprintf(buf, sizeof(buf), "%.1f", displayedVelocity);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            dl->AddText({cx-ts.x*0.5f, cy-ts.y*0.5f-5.0f}, IM_COL32(255,255,255,255), buf);
            
            ImVec2 us = ImGui::CalcTextSize("m/s");
            dl->AddText({cx-us.x*0.5f, cy+10.0f}, IM_COL32(150,180,200,180), "m/s");
            
            const char* fromStr[] = {"From Left", "From Right", "From Top", "From Bottom"};
            ImGui::Dummy(ImVec2(0, gR*2+30.0f));
            ImGui::Text("Direction: %s", fromStr[flow.windDirection]);
            
            float bw = 300.0f;
            ImVec2 bp = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(bp, {bp.x+bw, bp.y+8.0f}, IM_COL32(25,25,35,255));
            float fw = bw * sFrac;
            for (int bx = 0; bx < (int)fw; ++bx) {
                float fx = bx/bw;
                ImU32 col;
                if (fx < 0.5f) col = IM_COL32(100, (int)(150+200*fx), 255, 255);
                else col = IM_COL32((int)(100+155*(fx-0.5f)*2), 255, 255, 255);
                dl->AddRectFilled({bp.x+bx, bp.y}, {bp.x+bx+1, bp.y+8.0f}, col);
            }
            ImGui::Dummy(ImVec2(0, 12.0f));

        ImGui::End();
        ImGui::PopStyleColor(7);

        rlImGuiEnd();
        EndDrawing();
    }
}