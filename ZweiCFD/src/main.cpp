#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/solver.hpp"
#include "raylib.h"
#include "rlImGui.h"
#include "imgui.h"

struct WindParticle {
    Vector2 pos;
    Vector2 prevPos;
    float baseSize;
    float alpha;
    float speedJitter;
    bool active;
    float age;
};

struct WindSystem {
    static constexpr int MAX_PARTICLES = 8000;
    WindParticle particles[MAX_PARTICLES];
    int activeCount = 0;
};

float randFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

Vector2 getWindArrowAngle(int windDir) {
    switch (windDir) {
        case 0: return Vector2{1.0f, 0.0f};
        case 1: return Vector2{-1.0f, 0.0f};
        case 2: return Vector2{0.0f, 1.0f};
        case 3: return Vector2{0.0f, -1.0f};
        default: return Vector2{1.0f, 0.0f};
    }
}

void spawnParticle(WindParticle& p, int windDirection, const Vector2& spawnTL, const Vector2& spawnBR) {
    p.active = true;
    p.age = 0.0f;
    p.speedJitter = randFloat(0.9f, 1.1f);
    p.baseSize = randFloat(0.5f, 1.5f);
    p.alpha = randFloat(0.02f, 0.12f); 

    float margin = 20.0f;
    switch (windDirection) {
        case 0:
            p.pos.x = spawnTL.x - margin;
            p.pos.y = randFloat(spawnTL.y - margin, spawnBR.y + margin);
            break;
        case 1:
            p.pos.x = spawnBR.x + margin;
            p.pos.y = randFloat(spawnTL.y - margin, spawnBR.y + margin);
            break;
        case 2:
            p.pos.x = randFloat(spawnTL.x - margin, spawnBR.x + margin);
            p.pos.y = spawnTL.y - margin;
            break;
        case 3:
            p.pos.x = randFloat(spawnTL.x - margin, spawnBR.x + margin);
            p.pos.y = spawnBR.y + margin;
            break;
    }
    p.prevPos = p.pos;
}

int main(int argc, char* argv[]) {
    std::cout << "==== ZweiCFD v0.3   Streamlines ====\n";

    zweifoil::Airfoil foil;
    std::string filename = (argc >= 2) ? argv[1] : "dummy.dat";

    if (!foil.loadFromFile(filename)) {
        std::cerr << "Failed to load Airfoil Data!.\n";
    } else {
        std::cout << "Loaded: " << foil.getName() << "\n";
    }

    zweifoil::Flowconditions flow;
    flow.alpha = 5.0;
    flow.V_inf = 1.0;
    flow.windDirection = 0;

    auto solver = std::make_unique<zweifoil::Solver>(foil);
    zweifoil::Coefficients results = solver->runInviscid(flow);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "ZweiCFD Engine");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Camera2D camera = { 0 };
    camera.target = Vector2{ 0.0f, 0.0f };
    camera.offset = Vector2{ 1280 / 2.0f, 720 / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    static char fileBuf[256];
    strncpy(fileBuf, filename.c_str(), sizeof(fileBuf));
    static int current_sim = 0;

    WindSystem wind;
    for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
        wind.particles[i].active = false;
    }

    float particleSpawnTimer = 0.0f;
    const float renderScale = 400.0f;
    float displayedVelocity = (float)flow.V_inf;

    Vector2 windArrowDir = getWindArrowAngle(flow.windDirection);

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

        Vector2 tlWorld = GetScreenToWorld2D({0, 0}, camera);
        Vector2 brWorld = GetScreenToWorld2D({(float)GetScreenWidth(), (float)GetScreenHeight()}, camera);

        Vector2 spawnTL = GetScreenToWorld2D({-150, -150}, camera);
        Vector2 spawnBR = GetScreenToWorld2D({(float)GetScreenWidth() + 150, (float)GetScreenHeight() + 150}, camera);

        Vector2 killTL = GetScreenToWorld2D({-400, -400}, camera);
        Vector2 killBR = GetScreenToWorld2D({(float)GetScreenWidth() + 400, (float)GetScreenHeight() + 400}, camera);

        float baseSpeed = 300.0f;
        float speedScale = std::max(0.1f, (float)flow.V_inf);
        float spawnRate = 1500.0f + speedScale * 300.0f;

        particleSpawnTimer += spawnRate * dt;

        while (particleSpawnTimer >= 1.0f && wind.activeCount < WindSystem::MAX_PARTICLES) {
            particleSpawnTimer -= 1.0f;
            for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
                if (!wind.particles[i].active) {
                    spawnParticle(wind.particles[i], flow.windDirection, spawnTL, spawnBR);
                    wind.activeCount++;
                    break;
                }
            }
        }

        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
            WindParticle& p = wind.particles[i];
            if (!p.active) continue;

            p.prevPos = p.pos;
            p.age += dt;

            float norm_U = windArrowDir.x;
            float norm_W = -windArrowDir.y; 

            float step = baseSpeed * speedScale * p.speedJitter * dt;
            p.pos.x += norm_U * step;
            p.pos.y -= norm_W * step;

            if (p.pos.x < killTL.x || p.pos.x > killBR.x ||
                p.pos.y < killTL.y || p.pos.y > killBR.y) {
                p.active = false;
                wind.activeCount--;
                continue;
            }
        }

        BeginDrawing();
        ClearBackground(Color{10, 10, 16, 255});

        BeginMode2D(camera);

        BeginBlendMode(BLEND_ADDITIVE);
        
        Color baseStreamCol = {200, 230, 255, 0}; 

        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
            const WindParticle& p = wind.particles[i];
            if (!p.active) continue;

            float fadeIn = std::min(1.0f, p.age / 0.2f);
            float edgeFade = 1.0f;
            float dx_kill = std::max({killTL.x - p.pos.x, p.pos.x - killBR.x, 0.0f});
            float dy_kill = std::max({killTL.y - p.pos.y, p.pos.y - killBR.y, 0.0f});
            float distFromKill = std::max(dx_kill, dy_kill);

            if (distFromKill > 20.0f) {
                edgeFade = std::max(0.0f, 1.0f - (distFromKill - 20.0f) / 80.0f);
            }

            float currentAlpha = p.alpha * fadeIn * edgeFade;
            if (currentAlpha < 0.005f) continue;

            baseStreamCol.a = static_cast<unsigned char>(currentAlpha * 255.0f);
            
            Vector2 trailDir = {p.pos.x - p.prevPos.x, p.pos.y - p.prevPos.y};
            float stretchFactor = 12.0f; 
            Vector2 trailEnd = {
                p.pos.x - trailDir.x * stretchFactor,
                p.pos.y - trailDir.y * stretchFactor
            };
            
            DrawLineEx(p.pos, trailEnd, p.baseSize, baseStreamCol);
        }
        
        EndBlendMode();

        for (const auto& panel : foil.getPanels()) {
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
                        solver = std::make_unique<zweifoil::Solver>(foil);
                        results = solver->runInviscid(flow);
                        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i)
                            wind.particles[i].active = false;
                        wind.activeCount = 0;
                        particleSpawnTimer = 0.0f;
                    }
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Simulation Type")) {
                if (ImGui::MenuItem("ZweiFoil (2D Panel Method)", "", current_sim == 0))
                    current_sim = 0;
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Wind Direction")) {
                const char* dirNames[] = {"From Left    ", "From Right   ", "From Top     ", "From Bottom  "};
                for (int d = 0; d < 4; ++d) {
                    if (ImGui::MenuItem(dirNames[d], "", flow.windDirection == d)) {
                        flow.windDirection = d;
                        windArrowDir = getWindArrowAngle(d);
                        results = solver->runInviscid(flow);
                        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i)
                            wind.particles[i].active = false;
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
        ImGui::SetNextWindowSize(ImVec2(380, 400), ImGuiCond_FirstUseEver);
        ImGui::Begin("Workspace");

        if (current_sim == 0) {
            float alpha = static_cast<float>(flow.alpha);
            float velocity = static_cast<float>(flow.V_inf);
            bool changed = false;

            if (ImGui::SliderFloat("AoA (deg)", &alpha, -20.0f, 20.0f)) {
                flow.alpha = alpha; changed = true;
            }
            if (ImGui::SliderFloat("Velocity", &velocity, 0.1f, 100.0f)) {
                flow.V_inf = velocity; changed = true;
            }
            if (changed) results = solver->runInviscid(flow);

            if (ImGui::Button("Compute Flow")) {
                results = solver->runInviscid(flow);
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
        }

        ImGui::End();
        ImGui::PopStyleColor(7);

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();

    return 0;
}