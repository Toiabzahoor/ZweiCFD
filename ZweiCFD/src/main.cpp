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

// ============================================================
// Wind Particle System — dense, vivid, flowing
// ============================================================
struct WindParticle {
    Vector2 pos;
    Vector2 prevPos;
    float baseSize;        // 3.0 – 8.0 px
    float alpha;           // 0.0 – 1.0
    float speedJitter;     // 0.85 – 1.15
    bool active;
    float age;
};

struct WindSystem {
    static constexpr int MAX_PARTICLES = 2500;
    WindParticle particles[MAX_PARTICLES];
    int activeCount = 0;
};

float randFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

Vector2 getWindArrowAngle(int windDir) {
    switch (windDir) {
        case 0: return Vector2{1.0f, 0.0f};    // →
        case 1: return Vector2{-1.0f, 0.0f};   // ←
        case 2: return Vector2{0.0f, 1.0f};    // ↓
        case 3: return Vector2{0.0f, -1.0f};   // ↑
        default: return Vector2{1.0f, 0.0f};
    }
}

void spawnParticle(WindParticle& p, int windDirection, 
                   const Vector2& spawnTL, const Vector2& spawnBR) {
    p.active = true;
    p.age = 0.0f;
    p.speedJitter = randFloat(0.85f, 1.15f);
    p.baseSize = randFloat(3.0f, 8.0f);
    p.alpha = randFloat(0.8f, 1.0f);
    
    float margin = 20.0f;
    switch (windDirection) {
        case 0: // From Left → RHS stream
            p.pos.x = spawnTL.x - margin;
            p.pos.y = randFloat(spawnTL.y - margin, spawnBR.y + margin);
            break;
        case 1: // From Right → LHS stream
            p.pos.x = spawnBR.x + margin;
            p.pos.y = randFloat(spawnTL.y - margin, spawnBR.y + margin);
            break;
        case 2: // From Top → falling stream
            p.pos.x = randFloat(spawnTL.x - margin, spawnBR.x + margin);
            p.pos.y = spawnTL.y - margin;
            break;
        case 3: // From Bottom → rising stream
            p.pos.x = randFloat(spawnTL.x - margin, spawnBR.x + margin);
            p.pos.y = spawnBR.y + margin;
            break;
    }
    p.prevPos = p.pos;
}

bool isInsidePolygon(const Vector2& pt, const std::vector<zweifoil::Panel>& panels, float scale) {
    bool inside = false;
    for (size_t i = 0, j = panels.size() - 1; i < panels.size(); j = i++) {
        Vector2 a = { (float)(panels[i].p1.x * scale), (float)(-panels[i].p1.y * scale) };
        Vector2 b = { (float)(panels[j].p1.x * scale), (float)(-panels[j].p1.y * scale) };
        if (((a.y > pt.y) != (b.y > pt.y)) &&
            (pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

int main(int argc, char* argv[]) { 
    std::cout << "==== ZweiCFD v0.3 — Vivid Wind ====\n"; 
         
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

    // ---- Wind Particle System ---- 
    WindSystem wind;
    for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
        wind.particles[i].active = false;
    }
    
    float particleSpawnTimer = 0.0f;
    const float renderScale = 400.0f;
    float displayedVelocity = (float)flow.V_inf;

    // Pre-compute flow direction arrow
    Vector2 windArrowDir = getWindArrowAngle(flow.windDirection);

    while (!WindowShouldClose()) { 
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        // ---- Camera controls ----
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

        // ---- Compute world bounds ----
        Vector2 tlWorld = GetScreenToWorld2D({0, 0}, camera);
        Vector2 brWorld = GetScreenToWorld2D({(float)GetScreenWidth(), (float)GetScreenHeight()}, camera);
        
        // Spawn zone: slightly outside visible area
        Vector2 spawnTL = GetScreenToWorld2D({-150, -150}, camera);
        Vector2 spawnBR = GetScreenToWorld2D({(float)GetScreenWidth() + 150, (float)GetScreenHeight() + 150}, camera);
        
        // Kill zone: particles die when they leave this
        Vector2 killTL = GetScreenToWorld2D({-400, -400}, camera);
        Vector2 killBR = GetScreenToWorld2D({(float)GetScreenWidth() + 400, (float)GetScreenHeight() + 400}, camera);

        // ---- Update particles ----
        // Speed: faster base, proportional to V_inf
        float baseSpeed = 300.0f;  // world-units per second
        float speedScale = std::max(0.1f, (float)flow.V_inf);
        
        // Spawn rate: high enough to maintain dense field
        // At V_inf=1.0: spawnRate = min(400, 60 + 15) = 75 particles/sec
        float spawnRate = std::min(400.0f, 60.0f + speedScale * 15.0f);
        particleSpawnTimer += spawnRate * dt;
        
        const auto& panels = foil.getPanels();

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

        // ---- Update each particle ----
        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
            WindParticle& p = wind.particles[i];
            if (!p.active) continue;

            p.prevPos = p.pos;
            p.age += dt;

            // Get velocity from CFD solver
            zweifoil::Point2D solverPos = { p.pos.x / renderScale, -p.pos.y / renderScale };
            zweifoil::Point2D vel = solver->getVelocityAt(solverPos, flow);

            float ref_V = (float)((flow.V_inf > 0.01) ? flow.V_inf : 1.0f);
            float norm_U = (float)vel.x / ref_V;
            float norm_W = (float)vel.y / ref_V;
            
            float mag = sqrtf(norm_U * norm_U + norm_W * norm_W);
            if (mag > 5.0f) {
                norm_U = (norm_U / mag) * 5.0f;
                norm_W = (norm_W / mag) * 5.0f;
            }

            // Move
            float step = baseSpeed * speedScale * p.speedJitter * dt;
            p.pos.x += norm_U * step;
            p.pos.y -= norm_W * step;

            // ---- Collision with airfoil ----
            if (isInsidePolygon(p.pos, panels, renderScale)) {
                p.active = false;
                wind.activeCount--;
                continue;
            }

            // ---- Kill zone check ----
            if (p.pos.x < killTL.x || p.pos.x > killBR.x ||
                p.pos.y < killTL.y || p.pos.y > killBR.y) {
                p.active = false;
                wind.activeCount--;
                continue;
            }

            // ---- Alpha: keep it bright! ----
            // Fade-in over first 0.1s
            float fadeIn = std::min(1.0f, p.age / 0.1f);
            
            // Fade-out only at extreme edges near kill zone
            float edgeFade = 1.0f;
            float dx_kill = std::max({killTL.x - p.pos.x, p.pos.x - killBR.x, 0.0f});
            float dy_kill = std::max({killTL.y - p.pos.y, p.pos.y - killBR.y, 0.0f});
            float distFromKill = std::max(dx_kill, dy_kill);
            if (distFromKill > 20.0f) {
                edgeFade = std::max(0.0f, 1.0f - (distFromKill - 20.0f) / 80.0f);
            }
            
            // Keep alpha high! Base * fadeIn * (0.7 + 0.3 * edgeFade)
            // In visible area: edgeFade=1.0 so multiplier = 1.0 → alpha = baseAlpha (0.8-1.0)
            p.alpha = p.alpha * fadeIn * (0.7f + 0.3f * edgeFade);
            p.alpha = std::max(0.15f, p.alpha);  // always somewhat visible
        }

        // ========================================
        // ---- DRAW ----
        // ========================================
        BeginDrawing(); 
        ClearBackground(Color{8, 8, 14, 255}); // deeper dark
                 
        BeginMode2D(camera); 

        // ---- Very faint grid (barely visible) ----
        float spacing = 50.0f;
        int firstX = (int)(tlWorld.x / spacing) - 1;
        int lastX = (int)(brWorld.x / spacing) + 1;
        Color gridCol = Color{20, 20, 30, 60}; // alpha 60/255 = 0.23 — very dim
        for (int i = firstX; i <= lastX; i++) {
            DrawLineV({i * spacing, tlWorld.y}, {i * spacing, brWorld.y}, gridCol);
        }
        int firstY = (int)(tlWorld.y / spacing) - 1;
        int lastY = (int)(brWorld.y / spacing) + 1;
        for (int i = firstY; i <= lastY; i++) {
            DrawLineV({tlWorld.x, i * spacing}, {brWorld.x, i * spacing}, gridCol);
        }

        // ---- Draw wind particles (BRIGHT, BIG, DENSE) ----
        for (int i = 0; i < WindSystem::MAX_PARTICLES; ++i) {
            const WindParticle& p = wind.particles[i];
            if (!p.active || p.alpha < 0.01f) continue;

            float size = p.baseSize;
            float a = std::min(1.0f, p.alpha);
            
            // Vivid cyan/blue color scheme — NOT white!
            Color outerGlow = ColorAlpha((Color){30, 120, 220, 255}, a * 0.25f);
            Color midGlow   = ColorAlpha((Color){60, 190, 255, 255}, a * 0.50f);
            Color core       = ColorAlpha((Color){140, 230, 255, 255}, a * 0.95f);
            
            // Draw glow layers (larger radii for visibility)
            DrawCircleV(p.pos, size * 4.0f, outerGlow);
            DrawCircleV(p.pos, size * 2.0f, midGlow);
            DrawCircleV(p.pos, size * 1.0f, core);
            
            // Draw motion streak ONLY if particle moved enough
            Vector2 trailDir = {p.pos.x - p.prevPos.x, p.pos.y - p.prevPos.y};
            float trailLen = sqrtf(trailDir.x * trailDir.x + trailDir.y * trailDir.y);
            if (trailLen > 0.5f) {
                Vector2 trailEnd = {
                    p.pos.x - trailDir.x * 0.6f,
                    p.pos.y - trailDir.y * 0.6f
                };
                Color trailCol = ColorAlpha((Color){50, 170, 255, 255}, a * 0.25f);
                DrawLineEx(p.pos, trailEnd, size * 0.6f, trailCol);
            }
        }

        // ---- Draw Airfoil ----
        for (const auto& panel : foil.getPanels()) { 
            Vector2 p1 = { (float)(panel.p1.x * renderScale), (float)(-panel.p1.y * renderScale)}; 
            Vector2 p2 = { (float)(panel.p2.x * renderScale), (float)(-panel.p2.y * renderScale)}; 
            DrawLineEx(p1, p2, 6.0f, ColorAlpha((Color){60, 170, 255, 100}, 0.4f));
            DrawLineEx(p1, p2, 2.5f, ColorAlpha((Color){180, 230, 255, 255}, 0.9f));
        }
        
        // ---- Wind direction arrow (top-left corner) ----
        {
            float arrowSize = 35.0f;
            Vector2 origin = { tlWorld.x + 45.0f, tlWorld.y + 45.0f };
            Vector2 dir = windArrowDir;
            Vector2 tip = { origin.x + dir.x * arrowSize, origin.y + dir.y * arrowSize };
            
            DrawLineEx(origin, tip, 3.0f, ColorAlpha(SKYBLUE, 0.8f));
            
            // Arrowhead
            float ang = atan2f(dir.y, dir.x);
            float ha1 = ang + 2.3f, ha2 = ang - 2.3f;
            Vector2 h1 = { tip.x + cosf(ha1) * 10.0f, tip.y + sinf(ha1) * 10.0f };
            Vector2 h2 = { tip.x + cosf(ha2) * 10.0f, tip.y + sinf(ha2) * 10.0f };
            DrawTriangle(tip, h1, h2, ColorAlpha(SKYBLUE, 0.9f));
        }

        EndMode2D(); 

        // ========================================
        // ---- IMGUI ----
        // ========================================
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
                const char* dirNames[] = {"From Left   →", "From Right  ←", "From Top    ↓", "From Bottom ↑"};
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

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.06f, 0.1f, 0.95f));  
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.8f, 1.0f));  
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.25f, 1.0f)); 
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.5f, 1.0f)); 
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.6f, 1.0f)); 
        ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.1f, 0.1f, 0.15f, 1.0f)); 
        ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.1f, 0.1f, 0.15f, 1.0f)); 

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

            // ============ VELOCITY METER ============
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.8f, 1.0f), "Wind Velocity Meter");
            
            displayedVelocity += ((float)flow.V_inf - displayedVelocity) * 0.1f;
            
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 gp = ImGui::GetCursorScreenPos();
            float gR = 70.0f;
            float cx = gp.x + gR + 10.0f, cy = gp.y + gR + 10.0f;
            ImVec2 ctr(cx, cy);
            
            dl->AddCircleFilled(ctr, gR, IM_COL32(15, 15, 25, 255), 64);
            dl->AddCircle(ctr, gR, IM_COL32(0, 200, 150, 80), 64, 2.0f);
            
            float sFrac = std::min(1.0f, displayedVelocity / 100.0f);
            float sAng = -PI * 0.75f, eAng = PI * 0.75f, sweep = (eAng - sAng) * sFrac;
            
            ImU32 arcCol;
            if (sFrac < 0.4f) {
                float t = sFrac / 0.4f;
                arcCol = IM_COL32((int)(50+t*205), (int)(255-t*55), 100, 255);
            } else {
                float t = (sFrac-0.4f)/0.6f;
                arcCol = IM_COL32(255, (int)(200-t*200), (int)(100-t*100), 255);
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
                            IM_COL32(0,200,150,120), 1.5f);
            }
            
            char buf[32]; snprintf(buf, sizeof(buf), "%.1f", displayedVelocity);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            dl->AddText({cx-ts.x*0.5f, cy-ts.y*0.5f-5.0f}, IM_COL32(255,255,255,255), buf);
            ImVec2 us = ImGui::CalcTextSize("m/s");
            dl->AddText({cx-us.x*0.5f, cy+10.0f}, IM_COL32(100,200,180,180), "m/s");
            
            const char* fromStr[] = {"From Left", "From Right", "From Top", "From Bottom"};
            ImGui::Dummy(ImVec2(0, gR*2+30.0f));
            ImGui::Text("Direction: %s", fromStr[flow.windDirection]);
            
            float bw = 300.0f;
            ImVec2 bp = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(bp, {bp.x+bw, bp.y+8.0f}, IM_COL32(20,20,35,255));
            float fw = bw * sFrac;
            for (int bx = 0; bx < (int)fw; ++bx) {
                float fx = bx/bw;
                ImU32 col;
                if (fx < 0.5f) col = IM_COL32(0, (int)(255*fx*2), 150, 255);
                else col = IM_COL32((int)(255*(fx-0.5f)*2), (int)(255*(1-(fx-0.5f)*2)), 100, 255);
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