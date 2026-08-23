#include "ZweiCFD/solver/airfoil.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace zweicfd {

Airfoil::Airfoil() : name("unknown") {}

static void loadDummyDiamond(std::string& name, std::vector<Point2D>& coordinates) {
    name = "NACA 0012 (Test)";
    coordinates = {
        Point2D{1.0, 0.0},
        Point2D{0.5, 0.1},
        Point2D{0.0, 0.0},
        Point2D{0.5, -0.1},
        Point2D{1.0, 0.0}
    };
}

bool Airfoil::loadFrom2DFile(const std::string& filename) {
    std::cout << "Loading airfoil coords from: " << filename << "...\n";
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "  Could not open '" << filename << "', using dummy airfoil instead.\n";
        loadDummyDiamond(name, coordinates);
        generatePanels();
        return true;
    }

    std::vector<Point2D> parsed;
    std::string line;
    bool firstLine = true;
    std::string parsedName;

    while (std::getline(file, line)) {
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        size_t end = line.find_last_not_of(" \t\r\n");
        std::string trimmed = line.substr(start, end - start + 1);

        std::istringstream iss(trimmed);
        double x, y;
        if (iss >> x >> y) {
            parsed.push_back(Point2D{x, y});
        } else if (firstLine) {
            parsedName = trimmed;
        }
        firstLine = false;
    }

    file.close();

    if (parsed.size() < 3) {
        std::cerr << "  File '" << filename << "' had too few coordinate points ("
                  << parsed.size() << "), using dummy airfoil instead.\n";
        loadDummyDiamond(name, coordinates);
        generatePanels();
        return true;
    }
    
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    for (const auto& p : parsed) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
    }
    double scale = (maxX - minX > 1e-6) ? 1.0 / (maxX - minX) : 1.0;
    for (auto& p : parsed) {
        p.x = (p.x - minX) * scale;
        p.y = p.y * scale;
    }

    if (std::abs(parsed.front().x - parsed.back().x) > 1e-5 || 
        std::abs(parsed.front().y - parsed.back().y) > 1e-5) {
        parsed.push_back(parsed.front());
    }

    auto te_it = std::max_element(parsed.begin(), parsed.end() - 1, 
        [](const Point2D& a, const Point2D& b) { return a.x < b.x; });

    if (te_it != parsed.begin()) {
        std::vector<Point2D> reordered;
        reordered.insert(reordered.end(), te_it, parsed.end() - 1);
        reordered.insert(reordered.end(), parsed.begin(), te_it);
        reordered.push_back(reordered.front());
        parsed = reordered;
    }

    double area = 0.0;
    for (size_t i = 0; i < parsed.size() - 1; ++i) {
        area += (parsed[i].x * parsed[i+1].y - parsed[i+1].x * parsed[i].y);
    }
    
    if (area > 0.0) { 
        std::reverse(parsed.begin(), parsed.end());
    }

    name = parsedName.empty() ? filename : parsedName;
    coordinates = parsed;
    generatePanels();
    return true;
}

bool Airfoil::loadFrom3DMesh(const std::string& filename) {
    vtkSmartPointer<vtkPolyData> polyData;
    std::string ext = "";
    size_t dotPos = filename.find_last_of('.');
    if (dotPos != std::string::npos) {
        ext = filename.substr(dotPos);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    }

    if (ext == ".stl") {
        auto reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(filename.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    } else if (ext == ".obj") {
        auto reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(filename.c_str());
        reader->Update();
        polyData = reader->GetOutput();
    }

    if (!polyData || polyData->GetNumberOfPoints() < 3) return false;

    double bounds[6];
    polyData->GetBounds(bounds);
    double lx = bounds[1] - bounds[0];
    double ly = bounds[3] - bounds[2];
    double lz = bounds[5] - bounds[4];
    double maxL = std::max(lx, std::max(ly, lz));
    if (maxL < 1e-6) return false;

    double scale = 1.0 / maxL;
    double cx = (bounds[0] + bounds[1]) * 0.5;
    double cy = (bounds[2] + bounds[3]) * 0.5;
    double cz = (bounds[4] + bounds[5]) * 0.5;

    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(scale, scale, scale);
    transform->Translate(-cx + 0.25 / scale, -cy, -cz);

    auto tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    tf->SetInputData(polyData);
    tf->SetTransform(transform);
    tf->Update();

    mesh3D = tf->GetOutput();
    originalMesh3D = vtkSmartPointer<vtkPolyData>::New();
    originalMesh3D->DeepCopy(mesh3D);
    is3DModel = true;

    name = filename.substr(filename.find_last_of("/\\") + 1);
    
    coordinates.clear();
    coordinates.push_back({0.0, -0.2});
    coordinates.push_back({1.0, -0.2});
    coordinates.push_back({1.0, 0.2});
    coordinates.push_back({0.0, 0.2});
    coordinates.push_back({0.0, -0.2});
    generatePanels();

    return true;
}

bool Airfoil::loadFromFile(const std::string& raw_filename) {
    size_t start = raw_filename.find_first_not_of(" \t\r\n");
    std::string filename = (start == std::string::npos) ? "" : raw_filename.substr(start, raw_filename.find_last_not_of(" \t\r\n") - start + 1);

    std::cout << "Loading model from: " << filename << "...\n";

    std::string ext = "";
    size_t dotPos = filename.find_last_of('.');
    if (dotPos != std::string::npos) {
        ext = filename.substr(dotPos);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    }

    bool success = false;
    if (ext == ".stl" || ext == ".obj") {
        success = loadFrom3DMesh(filename);
    } else {
        success = loadFrom2DFile(filename);
    }

    if (!success) {
        std::cerr << "  Could not load '" << filename << "', using dummy airfoil instead.\n";
        loadDummyDiamond(name, coordinates);
        generatePanels();
        return true;
    }

    return true;
}

void Airfoil::generatePanels() {
    panels.clear();
    if (coordinates.size() < 2) return;

    for (size_t i = 0; i < coordinates.size() - 1; ++i) {
        Panel p;
        p.p1 = coordinates[i];
        p.p2 = coordinates[i + 1];

        double dx = p.p2.x - p.p1.x;
        double dy = p.p2.y - p.p1.y;
        
        p.cp.x = (p.p1.x + p.p2.x) / 2.0;
        p.cp.y = (p.p1.y + p.p2.y) / 2.0;
        
        p.length = std::sqrt(dx * dx + dy * dy);
        p.theta = std::atan2(dy, dx);
        
        p.tangent.x = std::cos(p.theta);
        p.tangent.y = std::sin(p.theta);
        
        p.normal.x = -std::sin(p.theta);
        p.normal.y = std::cos(p.theta);

        panels.push_back(p);
    }

    std::cout << "Generated " << panels.size() << " panels for airfoil: " << name << "\n";
}

void Airfoil::rotateCoordinates(double angleDeg) {
    if (is3D() && mesh3D) {
        auto transform = vtkSmartPointer<vtkTransform>::New();
        transform->RotateZ(angleDeg);
        auto tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        tf->SetInputData(mesh3D);
        tf->SetTransform(transform);
        tf->Update();
        mesh3D->DeepCopy(tf->GetOutput());
        return;
    }

    double angleRad = angleDeg * M_PI / 180.0;
    double cosA = std::cos(angleRad);
    double sinA = std::sin(angleRad);
    double cx = 0.25, cy = 0.0;
    for (auto& p : coordinates) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        p.x = cx + dx * cosA - dy * sinA;
        p.y = cy + dx * sinA + dy * cosA;
    }
    generatePanels();
}

void Airfoil::setBaseRotation(double rx, double ry, double rz) {
    if (is3D() && originalMesh3D) {
        auto transform = vtkSmartPointer<vtkTransform>::New();
        transform->RotateX(rx);
        transform->RotateY(ry);
        transform->RotateZ(rz);
        
        auto tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        tf->SetInputData(originalMesh3D);
        tf->SetTransform(transform);
        tf->Update();
        mesh3D->DeepCopy(tf->GetOutput());
    }
}

void Airfoil::generateNACA(double m, double p, double t, int n) {
    std::vector<Point2D> xu, xl;
    
    for (int i = 0; i < n; ++i) {
        double beta = M_PI * i / (n - 1);
        double x = 0.5 * (1.0 - std::cos(beta));
        
        double yt = 5.0 * t * (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
        
        double yc = 0.0, dyc_dx = 0.0;
        if (p > 0.0) {
            if (x < p) {
                yc = m / (p * p) * (2 * p * x - x * x);
                dyc_dx = 2 * m / (p * p) * (p - x);
            } else {
                yc = m / ((1 - p) * (1 - p)) * ((1 - 2 * p) + 2 * p * x - x * x);
                dyc_dx = 2 * m / ((1 - p) * (1 - p)) * (p - x);
            }
        }
        
        double theta = std::atan(dyc_dx);
        
        Point2D p_upper = {x - yt * std::sin(theta), yc + yt * std::cos(theta)};
        Point2D p_lower = {x + yt * std::sin(theta), yc - yt * std::cos(theta)};
        
        xu.push_back(p_upper);
        xl.push_back(p_lower);
    }

    coordinates.clear();
    
    for (int i = n - 1; i >= 0; --i) {
        coordinates.push_back(xu[i]);
    }
    
    for (int i = 1; i < n; ++i) {
        coordinates.push_back(xl[i]);
    }
    
    coordinates.push_back(coordinates.front());
    
    int m_digit = std::round(m * 100);
    int p_digit = std::round(p * 10);
    int t_digit = std::round(t * 100);
    name = "NACA " + std::to_string(m_digit) + std::to_string(p_digit) + (t_digit < 10 ? "0" : "") + std::to_string(t_digit);
    
    double area = 0.0;
    for (size_t i = 0; i < coordinates.size() - 1; ++i) {
        area += (coordinates[i].x * coordinates[i+1].y - coordinates[i+1].x * coordinates[i].y);
    }
    if (area > 0.0) { 
        std::reverse(coordinates.begin(), coordinates.end());
    }

    generatePanels();
}

void Airfoil::generateCylinder(double radius, int n) {
    coordinates.clear();
    name = "Cylinder";
    for (int i = 0; i < n; ++i) {
        double theta = -2.0 * M_PI * i / n;
        coordinates.push_back({0.5 + radius * std::cos(theta), radius * std::sin(theta)});
    }
    generatePanels();
}

}
