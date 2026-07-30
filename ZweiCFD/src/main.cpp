#include <iostream>
#include <string>
#include <memory>
#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/solver.hpp"
#include "raylib.h"
#include "rlImGui.h"
#include "imgui.h"

int main(int argc, char* argv[]) {
    std::cout << "==== ZweiCFD v0.1 ===\n";
    
    zweifoil::Airfoil foil;
    std::string filename = (argc >= 2) ? argv[1] : "dummy.dat";
    
    if (!foil.loadFromFile(filename)) {
        std::cerr << "Failed to load Airfoil Data!.\n";
    } else {
        std::cout << "Loaded: " << foil.getName() << "\n";
    }
    
    zweifoil::Flowconditions flow;
    flow.alpha = 5.0; // 5 deg AoA
    flow.V_inf = 1.0;
    
    // running solver
    auto solver = std::make_unique<zweifoil::Solver>(foil);
    zweifoil::Coefficients results = solver->runInviscid(flow);

    //gui 
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

    //loop
    while (!WindowShouldClose()) {
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

        BeginDrawing();
        ClearBackground(Color{15, 15, 20, 255});
        
        BeginMode2D(camera);
        DrawGrid(100, 50.0f);
        
        float renderScale = 400.0f;
        float gammaDisplayScale = 2.5f;

        for (const auto& panel : foil.getPanels()) {
            Vector2 p1 = { (float)(panel.p1.x * renderScale), (float)(-panel.p1.y * renderScale)};
            Vector2 p2 = { (float)(panel.p2.x * renderScale), (float)(-panel.p2.y * renderScale)};
            DrawLineEx(p1, p2, 3.0f, DARKGRAY);
        }

        const Eigen::VectorXd& gamma = solver->getGammaDistribution();
        if (gamma.size() > 0) {
            const auto& panels = foil.getPanels();
            
            for (size_t i = 0; i < panels.size(); ++i) {
                Vector2 nodePos1 = { (float)(panels[i].p1.x * renderScale), (float)(-panels[i].p1.y * renderScale)};
                float gVal1 = static_cast<float>(gamma(i));
                Vector2 gEnd1 = {
                    nodePos1.x + (float)(panels[i].normal.x * gVal1 * gammaDisplayScale),
                    nodePos1.y + (float)(-panels[i].normal.y * gVal1 * gammaDisplayScale)
                };
                
                Vector2 nodePos2 = { (float)(panels[i].p2.x * renderScale), (float)(-panels[i].p2.y * renderScale)};
                float gVal2 = static_cast<float>(gamma(i+1));
                Vector2 gEnd2 = {
                    nodePos2.x + (float)(panels[i].normal.x * gVal2 * gammaDisplayScale),
                    nodePos2.y + (float)(-panels[i].normal.y * gVal2 * gammaDisplayScale)
                };
                
                Color gColor = (gVal1 >= 0.0f) ? Color{255, 50, 100, 255} : Color{50, 150, 255, 255};
                
                DrawLineEx(nodePos1, gEnd1, 1.0f, Fade(gColor, 0.3f)); 
                DrawLineEx(gEnd1, gEnd2, 2.0f, gColor); 
                DrawCircleV(gEnd1, 1.5f, WHITE);

                if (i == panels.size() - 1) {
                    DrawLineEx(nodePos2, gEnd2, 1.0f, Fade(gColor, 0.3f));
                    DrawCircleV(gEnd2, 1.5f, WHITE);
                }
            }
        }
        EndMode2D();
        
        rlImGuiBegin();
        
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.06f, 0.1f, 0.95f)); 
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.8f, 1.0f)); 
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.15f, 0.15f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.5f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.3f, 0.6f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.1f, 0.1f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.1f, 0.1f, 0.15f, 1.0f));

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 280), ImGuiCond_FirstUseEver);
        ImGui::Begin("Workspace");

        ImGui::SetWindowFontScale(1.3f);
        ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.6f, 1.0f), "ZweiCFD Engine");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::Separator();

        ImGui::Text("Geometry Source");
        ImGui::InputText("##file", fileBuf, 256);
        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            if (foil.loadFromFile(fileBuf)) {
                solver = std::make_unique<zweifoil::Solver>(foil);
                results = solver->runInviscid(flow);
            }
        }

        ImGui::Separator();

        const char* simulators[] = { "ZweiFoil (2D Panel Method)" };
        ImGui::Combo("Simulator", &current_sim, simulators, IM_ARRAYSIZE(simulators));

        ImGui::Separator();

        if (current_sim == 0) {
            float alpha = static_cast<float>(flow.alpha);
            float velocity = static_cast<float>(flow.V_inf);
            ImGui::SliderFloat("AoA (deg)", &alpha, -20.0f, 20.0f);
            ImGui::SliderFloat("Velocity", &velocity, 0.1f, 100.0f);
            flow.alpha = alpha;
            flow.V_inf = velocity;
            
            if (ImGui::Button("Compute Flow")) {
                results = solver->runInviscid(flow);
                // output
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