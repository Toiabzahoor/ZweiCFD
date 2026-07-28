#include <iostream>
#include <string>

#include "ZweiFoil/airfoil.hpp"
#include "ZweiFoil/solver.hpp"

#include "raylib.h"
#include "rlImGui.h"
#include "imgui.h"

int main(int argc, char* argv[]) {
    std::cout << "==== ZweiFoil v0.0 ===\n";
    
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
    zweifoil::Solver solver(foil);
    zweifoil::Coefficients results = solver.runInviscid(flow);
    //gui 
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "ZweiFoil");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Camera2D camera = { 0 };
    camera.target = Vector2{ 0.0f, 0.0f };
    camera.offset = Vector2{ 1280 / 2.0f, 720 / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

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
            const float zoomIncrement = 0.125f;
            camera.zoom += (wheel * zoomIncrement);
            if (camera.zoom < 0.125f) camera.zoom = 0.125f;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode2D(camera);
        DrawGrid(100, 50.0f);

        float renderScale = 400.0f;

        for (const auto& panel : foil.getPanels()) {
            Vector2 p1 = { (float)(panel.p1.x * renderScale), (float)(-panel.p1.y * renderScale)};
            Vector2 p2 = { (float)(panel.p2.x * renderScale), (float)(-panel.p2.y * renderScale)};
            Vector2 cp = { (float)(panel.cp.x * renderScale), (float)(-panel.cp.y * renderScale)};

            DrawLineEx(p1, p2, 3.0f, DARKBLUE);
            DrawCircleV(cp, 2.0f, DARKGREEN);
        }

        const Eigen::VectorXd& gamma = solver.getGammaDistribution();
        if (gamma.size() > 0) {
            const auto& panels = foil.getPanels();
            float gammaDisplayScale = 30.0f;

            for (size_t i = 0; i < panels.size(); ++i) {
                Vector2 nodePos = { (float)(panels[i].p1.x * renderScale), (float)(-panels[i].p1.y * renderScale)};

                float gVal = static_cast<float>(gamma(i));
                Vector2 gEnd = {
                    nodePos.x + (float)(panels[i].normal.x * gVal * gammaDisplayScale),
                    nodePos.y + (float)(-panels[i].normal.y * gVal * gammaDisplayScale)
                };

                Color gColor = (gVal >= 0.0f) ? RED : BLUE;
                DrawLineEx(nodePos, gEnd, 2.0f, gColor);
                DrawCircleV(gEnd, 3.0f, gColor);
            }

            size_t lastNode = panels.size();
            Vector2 lastPos = { (float)(panels.back().p2.x * renderScale), (float)(-panels.back().p2.y * renderScale)};
            float gValLast = static_cast<float>(gamma(lastNode));
            Vector2 gEndLast = {
                lastPos.x + (float)(panels.back().normal.x * gValLast * gammaDisplayScale),
                lastPos.y + (float)(-panels.back().normal.y * gValLast * gammaDisplayScale)
            };
            Color gColorLast = (gValLast >= 0.0f) ? RED : BLUE;
            DrawLineEx(lastPos, gEndLast, 2.0f, gColorLast);
            DrawCircleV(gEndLast, 3.0f, gColorLast);
        }

        EndMode2D();
        
        rlImGuiBegin();
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350,200), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls");
        float alpha = static_cast<float>(flow.alpha);
        float velocity = static_cast<float>(flow.V_inf);
        ImGui::SliderFloat("Angle Of Attack", &alpha, -20.0f, 20.0f);
        ImGui::SliderFloat("Velocity", &velocity, 0.1f, 100.0f);
        flow.alpha = alpha;
        flow.V_inf = velocity;
        if (ImGui::Button("Run Solver")) {
            results = solver.runInviscid(flow);
            // output
            std::cout << "\nResults:\n"
                      << " Cl: " << results.cl << "\n"
                      << " Cd: " << results.cd << "\n"
                      << " Cm: " << results.cm << "\n";

        }
        ImGui::Separator();
        ImGui::Text("Cl: %.4f", results.cl);
        ImGui::Text("Cd: %.4f", results.cd);
        ImGui::Text("Cm: %.4f", results.cm);
        ImGui::End();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    
    return 0;
}