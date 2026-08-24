#include "ZweiCFD/core/cli.hpp"
#include "ZweiCFD/core/simulation.hpp"
#include "ZweiCFD/solver/lbm_solver.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>
#include <string>
#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#endif

namespace zweicfd {

void printHelp(const char* progName) {
    std::cout << "=================================================================\n";
    std::cout << "                      ZweiCFD CLI & Debug Mode                   \n";
    std::cout << "=================================================================\n";
    std::cout << "Usage:\n";
    std::cout << "  " << progName << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help              Show this help message\n";
    std::cout << "  -i, --interactive, --menu  Open interactive keyboard-driven CLI menu (default for --cli alone)\n";
    std::cout << "  --cli, --headless       Run in headless CLI mode (no Qt GUI)\n";
    std::cout << "  -v, --verbose           Print detailed step-by-step telemetry\n";
    std::cout << "  -a, --alpha <deg>       Angle of attack in degrees (default: 0.0)\n";
    std::cout << "  -c, --camber <val>      NACA airfoil camber (default: 0.02, range: 0.0 - 0.09)\n";
    std::cout << "  -t, --thickness <val>   NACA airfoil thickness (default: 0.12, range: 0.05 - 0.30)\n";
    std::cout << "  -s, --speed <m/s>       Airspeed V_inf (default: 15.0 m/s)\n";
    std::cout << "  -n, --steps <num>       Number of simulation steps to run (default: 200)\n";
    std::cout << "  --warmup <num>          Warmup steps before accumulating stats (default: 50)\n";
    std::cout << "  --shape <idx>           Shape selector (0: NACA, 1: Cylinder, 3: Diamond)\n";
    std::cout << "  -m, --model <path>      Path to 2D/3D mesh file (.stl, .obj, .dat)\n";
    std::cout << "  --preset <name>         Preset name: naca0012, thick, flatplate, cylinder\n";
    std::cout << "  -g, --grid <spec>       Grid resolution (e.g. 256x128x128, 256, or scale factor 2.0)\n";
    std::cout << "  --scale <val>           Scale factor for grid resolution (e.g. 1.5, 2.0)\n";
    std::cout << "  --nx <num>              Set grid NX dimension (default: 128)\n";
    std::cout << "  --ny <num>              Set grid NY dimension (default: 64)\n";
    std::cout << "  --nz <num>              Set grid NZ dimension (default: 64)\n";
    std::cout << "  --lines <num>           Streamline line count (default: 100)\n";
    std::cout << "  --rake-y <val>          Streamline rake Y offset (default: 0.0)\n";
    std::cout << "  --export-vti <path>     Export simulation field to ParaView .vti file\n";
    std::cout << "  --polar                 Run automated angle-of-attack polar sweep\n";
    std::cout << "  --alpha-min <deg>       Polar sweep minimum angle of attack (default: -4.0)\n";
    std::cout << "  --alpha-max <deg>       Polar sweep maximum angle of attack (default: 16.0)\n";
    std::cout << "  --alpha-step <deg>      Polar sweep angle step size (default: 2.0)\n";
    std::cout << "  --polar-steps <num>     Simulation steps per polar angle (default: 200)\n";
    std::cout << "  --polar-warmup <num>    Warmup steps per polar angle (default: 50)\n";
    std::cout << "  --polar-csv <path>      Export polar sweep data to CSV file\n";
    std::cout << "  --polar-no-reset        Do not reset flow field between polar sweep angles\n";
    std::cout << "=================================================================\n";
}

CLIOptions parseCLI(int argc, char* argv[]) {
    CLIOptions opt;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            opt.help = true;
        } else if (arg == "-i" || arg == "--interactive" || arg == "--menu") {
            opt.isCliMode = true;
            opt.headless = true;
            opt.interactiveMenu = true;
        } else if (arg == "--cli" || arg == "--headless") {
            opt.isCliMode = true;
            opt.headless = true;
        } else if (arg == "-v" || arg == "--verbose") {
            opt.verbose = true;
        } else if ((arg == "-a" || arg == "--alpha") && i + 1 < argc) {
            opt.alpha = std::stod(argv[++i]);
            opt.alphaSet = true;
        } else if ((arg == "-c" || arg == "--camber") && i + 1 < argc) {
            opt.camber = std::stod(argv[++i]);
            opt.camberSet = true;
        } else if ((arg == "-t" || arg == "--thickness") && i + 1 < argc) {
            opt.thickness = std::stod(argv[++i]);
            opt.thicknessSet = true;
        } else if ((arg == "-s" || arg == "--speed") && i + 1 < argc) {
            opt.speed = std::stod(argv[++i]);
            opt.speedSet = true;
        } else if ((arg == "-n" || arg == "--steps") && i + 1 < argc) {
            opt.steps = std::stoi(argv[++i]);
            opt.stepsSet = true;
        } else if (arg == "--warmup" && i + 1 < argc) {
            opt.warmup = std::stoi(argv[++i]);
        } else if (arg == "--shape" && i + 1 < argc) {
            opt.shape = std::stoi(argv[++i]);
            opt.shapeSet = true;
        } else if ((arg == "-m" || arg == "--model") && i + 1 < argc) {
            opt.modelFile = argv[++i];
            opt.modelSet = true;
        } else if (arg == "--preset" && i + 1 < argc) {
            opt.preset = argv[++i];
            opt.presetSet = true;
        } else if ((arg == "-g" || arg == "--grid" || arg == "--res") && i + 1 < argc) {
            std::string spec = argv[++i];
            size_t x1 = spec.find('x');
            if (x1 == std::string::npos) x1 = spec.find('X');
            if (x1 != std::string::npos) {
                size_t x2 = spec.find('x', x1 + 1);
                if (x2 == std::string::npos) x2 = spec.find('X', x1 + 1);
                if (x2 != std::string::npos) {
                    opt.gridNX = std::stoi(spec.substr(0, x1));
                    opt.gridNY = std::stoi(spec.substr(x1 + 1, x2 - x1 - 1));
                    opt.gridNZ = std::stoi(spec.substr(x2 + 1));
                    opt.nxSet = opt.nySet = opt.nzSet = true;
                } else {
                    opt.gridNX = std::stoi(spec.substr(0, x1));
                    opt.gridNY = std::stoi(spec.substr(x1 + 1));
                    opt.gridNZ = opt.gridNY;
                    opt.nxSet = opt.nySet = opt.nzSet = true;
                }
            } else {
                try {
                    double val = std::stod(spec);
                    if (val <= 10.0 && val > 0.0) {
                        opt.gridScale = val;
                        opt.gridScaleSet = true;
                    } else {
                        opt.gridNX = static_cast<int>(val);
                        opt.gridNY = std::max(8, opt.gridNX / 2);
                        opt.gridNZ = opt.gridNY;
                        opt.nxSet = opt.nySet = opt.nzSet = true;
                    }
                } catch (...) {}
            }
        } else if ((arg == "--scale" || arg == "--grid-scale") && i + 1 < argc) {
            opt.gridScale = std::stod(argv[++i]);
            opt.gridScaleSet = true;
        } else if (arg == "--nx" && i + 1 < argc) {
            opt.gridNX = std::stoi(argv[++i]);
            opt.nxSet = true;
        } else if (arg == "--ny" && i + 1 < argc) {
            opt.gridNY = std::stoi(argv[++i]);
            opt.nySet = true;
        } else if (arg == "--nz" && i + 1 < argc) {
            opt.gridNZ = std::stoi(argv[++i]);
            opt.nzSet = true;
        } else if (arg == "--lines" && i + 1 < argc) {
            opt.lines = std::stoi(argv[++i]);
            opt.linesSet = true;
        } else if (arg == "--rake-y" && i + 1 < argc) {
            opt.rakeY = std::stod(argv[++i]);
            opt.rakeYSet = true;
        } else if ((arg == "--export-vti" || arg == "--vti" || arg == "-o") && i + 1 < argc) {
            opt.vtiExportFile = argv[++i];
            opt.vtiExportSet = true;
        } else if (arg == "--polar" || arg == "--sweep") {
            opt.polarSweep = true;
            opt.isCliMode = true;
            opt.headless = true;
            opt.polarSweepSet = true;
        } else if (arg == "--alpha-min" && i + 1 < argc) {
            opt.alphaMin = std::stod(argv[++i]);
            opt.alphaMinSet = true;
        } else if (arg == "--alpha-max" && i + 1 < argc) {
            opt.alphaMax = std::stod(argv[++i]);
            opt.alphaMaxSet = true;
        } else if ((arg == "--alpha-step" || arg == "--step") && i + 1 < argc) {
            opt.alphaStep = std::stod(argv[++i]);
            opt.alphaStepSet = true;
        } else if (arg == "--polar-steps" && i + 1 < argc) {
            opt.polarSteps = std::stoi(argv[++i]);
        } else if (arg == "--polar-warmup" && i + 1 < argc) {
            opt.polarWarmup = std::stoi(argv[++i]);
        } else if ((arg == "--polar-csv" || arg == "--csv") && i + 1 < argc) {
            opt.polarCsvFile = argv[++i];
            opt.polarCsvSet = true;
        } else if (arg == "--polar-reset") {
            opt.polarResetFlow = true;
        } else if (arg == "--polar-no-reset" || arg == "--no-reset") {
            opt.polarResetFlow = false;
        }
    }

    if (opt.presetSet) {
        std::string p = opt.preset;
        std::transform(p.begin(), p.end(), p.begin(), ::tolower);
        if (p == "naca0012") {
            opt.shape = 0;
            opt.camber = 0.0;
            opt.thickness = 0.12;
        } else if (p == "thick") {
            opt.shape = 0;
            opt.camber = 0.06;
            opt.thickness = 0.28;
        } else if (p == "flatplate") {
            opt.shape = 0;
            opt.camber = 0.0;
            opt.thickness = 0.05;
        } else if (p == "cylinder") {
            opt.shape = 1;
        }
    }

    bool anySpecificOption = opt.alphaSet || opt.camberSet || opt.thicknessSet || opt.speedSet ||
                             opt.shapeSet || opt.modelSet || opt.presetSet || opt.linesSet ||
                             opt.rakeYSet || opt.stepsSet || opt.nxSet || opt.nySet || opt.nzSet ||
                             opt.gridScaleSet || opt.vtiExportSet || opt.polarSweepSet ||
                             opt.alphaMinSet || opt.alphaMaxSet || opt.alphaStepSet || opt.polarCsvSet;

    if ((opt.isCliMode || opt.headless) && !anySpecificOption) {
        opt.interactiveMenu = true;
    }

    return opt;
}

int runHeadlessCLI(const CLIOptions& opt, const Config& config) {
    auto setupStartTime = std::chrono::high_resolution_clock::now();

    std::cout << "\n=================================================================\n";
    std::cout << "                 ZweiCFD Headless Simulation Run                 \n";
    std::cout << "=================================================================\n";

    Simulation sim(0, nullptr);
    sim.config = config;
    if (opt.nxSet && opt.gridNX > 0) sim.config.lbmGridNX = opt.gridNX;
    if (opt.nySet && opt.gridNY > 0) sim.config.lbmGridNY = opt.gridNY;
    if (opt.nzSet && opt.gridNZ > 0) sim.config.lbmGridNZ = opt.gridNZ;
    if (opt.gridScaleSet && opt.gridScale > 0.0) {
        sim.config.lbmGridNX = std::max(16, static_cast<int>(std::round(sim.config.lbmGridNX * opt.gridScale)));
        sim.config.lbmGridNY = std::max(8, static_cast<int>(std::round(sim.config.lbmGridNY * opt.gridScale)));
        sim.config.lbmGridNZ = std::max(8, static_cast<int>(std::round(sim.config.lbmGridNZ * opt.gridScale)));
    }
    sim.flow.alpha = opt.alpha;
    sim.flow.V_inf = opt.speed;
    
    if (opt.modelSet && !opt.modelFile.empty()) {
        std::cout << "Loading custom geometry from: " << opt.modelFile << "\n";
        sim.foil.loadFromFile(opt.modelFile);
    } else if (opt.shape == 1) {
        std::cout << "Generating Cylinder Geometry...\n";
        sim.foil.generateCylinder(0.25, 100);
    } else if (opt.shape == 2) {
        std::cout << "Generating Flat Plate Geometry...\n";
        sim.foil.generateFlatPlate(opt.thicknessSet ? opt.thickness : 0.04);
    } else if (opt.shape == 3) {
        std::cout << "Generating Diamond Geometry...\n";
        sim.foil.generateDiamond(opt.thicknessSet ? opt.thickness : 0.10);
    } else {
        std::cout << "Generating NACA Airfoil (m=" << opt.camber << ", p=0.4, t=" << opt.thickness << ")...\n";
        sim.foil.generateNACA(opt.camber, 0.4, opt.thickness);
    }

    sim.rebuildSolverWithRotation();

    if (!sim.lbmSolver) {
        std::cerr << "Error: LBM solver failed to initialize.\n";
        return 1;
    }

    const auto& grid = sim.lbmSolver->getGrid();
    int solidCells = 0;
    int totalCells = grid.NX * grid.NY * grid.NZ;
    for (int i = 0; i < totalCells; ++i) {
        if (grid.sdf[i] <= 0.0f) solidCells++;
    }

    double scale = (double)sim.cachedLbmScale;
    double a_ref_lattice = 1.0;
    if (sim.foil.is3D()) {
        a_ref_lattice = std::max(50.0, ((double)sim.cowWidth * scale) * (std::max(0.2, (double)sim.cowHeight) * scale));
    } else {
        a_ref_lattice = std::max(50.0, (1.0 * scale) * (double)sim.config.lbmGridNZ);
    }
    double f_dyn_lattice = 0.00125 * a_ref_lattice;
    double q_inf = 0.5 * 1.225 * sim.flow.V_inf * sim.flow.V_inf;
    double ref_area_m2 = 1.0;

    std::cout << "\n--- Simulation Setup ---\n";
    std::cout << "  Grid Resolution       : " << grid.NX << " x " << grid.NY << " x " << grid.NZ << " (" << totalCells << " cells)\n";
    std::cout << "  Solid Obstacle Cells  : " << solidCells << " (" << std::fixed << std::setprecision(2) << (100.0 * solidCells / totalCells) << "% of domain)\n";
    std::cout << "  Lattice Scale         : " << scale << "\n";
    std::cout << "  Airspeed (V_inf)      : " << sim.flow.V_inf << " m/s\n";
    std::cout << "  Angle of Attack (deg) : " << sim.flow.alpha << "°\n";
    std::cout << "  Dynamic Pressure (q)  : " << q_inf << " Pa\n";
    std::cout << "  Total Steps to Run    : " << opt.steps << " (Warmup: " << opt.warmup << ")\n\n";

    if (opt.verbose) {
        std::cout << std::setw(8) << "Step" 
                  << std::setw(14) << "Force_X" 
                  << std::setw(14) << "Force_Y" 
                  << std::setw(12) << "CL" 
                  << std::setw(12) << "CD" 
                  << std::setw(12) << "L/D" 
                  << std::setw(14) << "Lift(N)" 
                  << std::setw(14) << "Drag(N)" << "\n";
        std::cout << std::string(100, '-') << "\n";
    }

    double sum_cl = 0.0;
    double sum_cd = 0.0;
    double sum_lift_N = 0.0;
    double sum_drag_N = 0.0;
    int sampled_count = 0;

    double last_cl = 0.0;
    double last_cd = 0.0;
    double last_ld = 0.0;
    double last_lift_N = 0.0;
    double last_drag_N = 0.0;

    Flowconditions safeLBM = sim.flow;
    safeLBM.V_inf = sim.flow.V_inf;
    safeLBM.kinematic_viscosity = sim.flow.kinematic_viscosity * (sim.flow.V_inf / std::max(0.0001, sim.flow.V_inf)) * sim.cachedLbmScale;

    auto simStartTime = std::chrono::high_resolution_clock::now();

    for (int step = 1; step <= opt.steps; ++step) {
        sim.lbmSolver->step(safeLBM);

        const auto& currentGrid = sim.lbmSolver->getGrid();
        double drag = (double)currentGrid.force_x;
        double lift = (double)currentGrid.force_y;

        double raw_cd = std::abs(drag) / f_dyn_lattice;
        double raw_cl = lift / f_dyn_lattice;
        double raw_ld = (std::abs(raw_cd) > 1e-6) ? (raw_cl / raw_cd) : 0.0;

        double lift_N = raw_cl * q_inf * ref_area_m2;
        double drag_N = raw_cd * q_inf * ref_area_m2;

        last_cl = raw_cl;
        last_cd = raw_cd;
        last_ld = raw_ld;
        last_lift_N = lift_N;
        last_drag_N = drag_N;

        if (step > opt.warmup) {
            sum_cl += raw_cl;
            sum_cd += raw_cd;
            sum_lift_N += lift_N;
            sum_drag_N += drag_N;
            sampled_count++;
        }

        if (opt.verbose && (step % 10 == 0 || step == opt.steps)) {
            std::cout << std::setw(8) << step
                      << std::setw(14) << std::fixed << std::setprecision(4) << drag
                      << std::setw(14) << std::fixed << std::setprecision(4) << lift
                      << std::setw(12) << std::fixed << std::setprecision(4) << raw_cl
                      << std::setw(12) << std::fixed << std::setprecision(4) << raw_cd
                      << std::setw(12) << std::fixed << std::setprecision(3) << raw_ld
                      << std::setw(14) << std::fixed << std::setprecision(2) << lift_N
                      << std::setw(14) << std::fixed << std::setprecision(2) << drag_N << "\n";
        }
    }

    auto simEndTime = std::chrono::high_resolution_clock::now();
    double setupTimeSec = std::chrono::duration<double>(simStartTime - setupStartTime).count();
    double simTimeSec = std::chrono::duration<double>(simEndTime - simStartTime).count();
    double totalTimeSec = std::chrono::duration<double>(simEndTime - setupStartTime).count();
    double msPerStep = (opt.steps > 0) ? (simTimeSec * 1000.0 / opt.steps) : 0.0;
    double mlups = (opt.steps > 0 && simTimeSec > 0.0) ? ((double)totalCells * opt.steps) / (simTimeSec * 1e6) : 0.0;

    double avg_cl = (sampled_count > 0) ? (sum_cl / sampled_count) : last_cl;
    double avg_cd = (sampled_count > 0) ? (sum_cd / sampled_count) : last_cd;
    double avg_ld = (std::abs(avg_cd) > 1e-6) ? (avg_cl / avg_cd) : 0.0;
    double avg_lift_N = (sampled_count > 0) ? (sum_lift_N / sampled_count) : last_lift_N;
    double avg_drag_N = (sampled_count > 0) ? (sum_drag_N / sampled_count) : last_drag_N;

    std::cout << "\n=================================================================\n";
    std::cout << "                 FINAL AERODYNAMIC TELEMETRY RESULTS             \n";
    std::cout << "=================================================================\n";
    std::cout << "  Instantaneous Values (Step " << opt.steps << "):\n";
    std::cout << "    Lift Coefficient (CL) : " << std::fixed << std::setprecision(4) << last_cl << "\n";
    std::cout << "    Drag Coefficient (CD) : " << std::fixed << std::setprecision(4) << last_cd << "\n";
    std::cout << "    Lift-to-Drag Ratio    : " << std::fixed << std::setprecision(3) << last_ld << "\n";
    std::cout << "    Lift Force            : " << std::fixed << std::setprecision(2) << last_lift_N << " N\n";
    std::cout << "    Drag Force            : " << std::fixed << std::setprecision(2) << last_drag_N << " N\n";
    std::cout << "\n  Time-Averaged Values (over " << sampled_count << " post-warmup steps):\n";
    std::cout << "    Mean CL               : " << std::fixed << std::setprecision(4) << avg_cl << "\n";
    std::cout << "    Mean CD               : " << std::fixed << std::setprecision(4) << avg_cd << "\n";
    std::cout << "    Mean L/D Ratio        : " << std::fixed << std::setprecision(3) << avg_ld << "\n";
    std::cout << "    Mean Lift Force       : " << std::fixed << std::setprecision(2) << avg_lift_N << " N\n";
    std::cout << "    Mean Drag Force       : " << std::fixed << std::setprecision(2) << avg_drag_N << " N\n";
    std::cout << "\n  Performance & Timing:\n";
    std::cout << "    Setup / Voxelization  : " << std::fixed << std::setprecision(3) << setupTimeSec << " s\n";
    std::cout << "    Simulation Steps Time : " << std::fixed << std::setprecision(3) << simTimeSec << " s\n";
    std::cout << "    Total Elapsed Time    : " << std::fixed << std::setprecision(3) << totalTimeSec << " s\n";
    std::cout << "    Time Per Step         : " << std::fixed << std::setprecision(2) << msPerStep << " ms/step\n";
    std::cout << "    LBM Throughput        : " << std::fixed << std::setprecision(2) << mlups << " MLUPS (Million Lattice Updates/s)\n";
    std::cout << "=================================================================\n\n";

    if (opt.vtiExportSet && !opt.vtiExportFile.empty()) {
        std::cout << "Exporting ParaView dataset to: " << opt.vtiExportFile << " ... ";
        if (sim.exportToVTI(opt.vtiExportFile)) {
            std::cout << "[SUCCESS]\n\n";
        } else {
            std::cout << "[FAILED]\n\n";
        }
    }

    return 0;
}

int runPolarSweepCLI(const CLIOptions& opt, const Config& config) {
    auto sweepStartTime = std::chrono::high_resolution_clock::now();

    std::cout << "\n=================================================================\n";
    std::cout << "               ZweiCFD Automated Polar Sweep Run                \n";
    std::cout << "=================================================================\n";

    Simulation sim(0, nullptr);
    sim.config = config;
    if (opt.nxSet && opt.gridNX > 0) sim.config.lbmGridNX = opt.gridNX;
    if (opt.nySet && opt.gridNY > 0) sim.config.lbmGridNY = opt.gridNY;
    if (opt.nzSet && opt.gridNZ > 0) sim.config.lbmGridNZ = opt.gridNZ;
    if (opt.gridScaleSet && opt.gridScale > 0.0) {
        sim.config.lbmGridNX = std::max(16, static_cast<int>(std::round(sim.config.lbmGridNX * opt.gridScale)));
        sim.config.lbmGridNY = std::max(8, static_cast<int>(std::round(sim.config.lbmGridNY * opt.gridScale)));
        sim.config.lbmGridNZ = std::max(8, static_cast<int>(std::round(sim.config.lbmGridNZ * opt.gridScale)));
    }
    sim.flow.alpha = opt.alphaMin;
    sim.flow.V_inf = opt.speed;

    if (opt.modelSet && !opt.modelFile.empty()) {
        std::cout << "Loading custom geometry from: " << opt.modelFile << "\n";
        sim.foil.loadFromFile(opt.modelFile);
    } else if (opt.shape == 1) {
        std::cout << "Generating Cylinder Geometry...\n";
        sim.foil.generateCylinder(0.25, 100);
    } else if (opt.shape == 2) {
        std::cout << "Generating Flat Plate Geometry...\n";
        sim.foil.generateFlatPlate(opt.thicknessSet ? opt.thickness : 0.04);
    } else if (opt.shape == 3) {
        std::cout << "Generating Diamond Geometry...\n";
        sim.foil.generateDiamond(opt.thicknessSet ? opt.thickness : 0.10);
    } else {
        std::cout << "Generating NACA Airfoil (m=" << opt.camber << ", p=0.4, t=" << opt.thickness << ")...\n";
        sim.foil.generateNACA(opt.camber, 0.4, opt.thickness);
    }

    sim.rebuildSolverWithRotation();

    if (!sim.lbmSolver) {
        std::cerr << "Error: LBM solver failed to initialize.\n";
        return 1;
    }

    const auto& grid = sim.lbmSolver->getGrid();
    int totalCells = grid.NX * grid.NY * grid.NZ;

    double scale = (double)sim.cachedLbmScale;
    double a_ref_lattice = 1.0;
    if (sim.foil.is3D()) {
        a_ref_lattice = std::max(50.0, ((double)sim.cowWidth * scale) * (std::max(0.2, (double)sim.cowHeight) * scale));
    } else {
        a_ref_lattice = std::max(50.0, (1.0 * scale) * (double)sim.config.lbmGridNZ);
    }
    double f_dyn_lattice = 0.00125 * a_ref_lattice;
    double q_inf = 0.5 * 1.225 * sim.flow.V_inf * sim.flow.V_inf;
    double ref_area_m2 = 1.0;

    int stepsPerAngle = opt.stepsSet ? opt.steps : opt.polarSteps;
    int warmupPerAngle = (opt.warmup != 50) ? opt.warmup : opt.polarWarmup;
    if (warmupPerAngle >= stepsPerAngle) {
        warmupPerAngle = std::max(0, stepsPerAngle / 2);
    }

    std::vector<double> angles;
    double stepSize = (std::abs(opt.alphaStep) > 1e-4) ? std::abs(opt.alphaStep) : 1.0;
    if (opt.alphaMin <= opt.alphaMax) {
        for (double a = opt.alphaMin; a <= opt.alphaMax + 1e-6; a += stepSize) {
            angles.push_back(a);
        }
    } else {
        for (double a = opt.alphaMin; a >= opt.alphaMax - 1e-6; a -= stepSize) {
            angles.push_back(a);
        }
    }
    if (angles.empty()) angles.push_back(opt.alphaMin);

    std::string csvOutPath = opt.polarCsvFile;
    if (csvOutPath.empty()) csvOutPath = "polar_results.csv";

    std::cout << "\n--- Polar Sweep Configuration ---\n";
    std::cout << "  Grid Resolution       : " << grid.NX << " x " << grid.NY << " x " << grid.NZ << " (" << totalCells << " cells)\n";
    std::cout << "  Airspeed (V_inf)      : " << sim.flow.V_inf << " m/s\n";
    std::cout << "  Dynamic Pressure (q)  : " << q_inf << " Pa\n";
    std::cout << "  Alpha Range           : [" << opt.alphaMin << "°, " << opt.alphaMax << "°] (Step: " << stepSize << "°, Total points: " << angles.size() << ")\n";
    std::cout << "  Steps per Point       : " << stepsPerAngle << " (Warmup: " << warmupPerAngle << ")\n";
    std::cout << "  Flow Reset per Angle  : " << (opt.polarResetFlow ? "Enabled" : "Disabled (Continuous flow evolution)") << "\n";
    std::cout << "  CSV Export Path       : " << csvOutPath << "\n\n";

    std::cout << std::setw(10) << "Alpha(deg)"
              << std::setw(12) << "CL"
              << std::setw(12) << "CD"
              << std::setw(12) << "L/D"
              << std::setw(14) << "Lift(N)"
              << std::setw(14) << "Drag(N)"
              << std::setw(14) << "Force_X"
              << std::setw(14) << "Force_Y"
              << std::setw(12) << "Time(s)" << "\n";
    std::cout << std::string(104, '-') << "\n";

    struct PolarRecord {
        double alpha;
        double cl;
        double cd;
        double ld;
        double lift_N;
        double drag_N;
        double fx_lattice;
        double fy_lattice;
    };
    std::vector<PolarRecord> records;

    Flowconditions safeLBM = sim.flow;
    safeLBM.V_inf = sim.flow.V_inf;
    safeLBM.kinematic_viscosity = sim.flow.kinematic_viscosity * (sim.flow.V_inf / std::max(0.0001, sim.flow.V_inf)) * sim.cachedLbmScale;

    for (size_t pt = 0; pt < angles.size(); ++pt) {
        double currentAlpha = angles[pt];
        auto ptStartTime = std::chrono::high_resolution_clock::now();

        sim.fastUpdateRotation(currentAlpha);
        if (opt.polarResetFlow && pt > 0) {
            sim.resetFlow();
        }

        double sum_cl = 0.0;
        double sum_cd = 0.0;
        double sum_lift_N = 0.0;
        double sum_drag_N = 0.0;
        double sum_fx = 0.0;
        double sum_fy = 0.0;
        int sampled_count = 0;

        double last_cl = 0.0;
        double last_cd = 0.0;
        double last_lift_N = 0.0;
        double last_drag_N = 0.0;
        double last_fx = 0.0;
        double last_fy = 0.0;

        for (int step = 1; step <= stepsPerAngle; ++step) {
            sim.lbmSolver->step(safeLBM);

            const auto& currentGrid = sim.lbmSolver->getGrid();
            double drag = (double)currentGrid.force_x;
            double lift = (double)currentGrid.force_y;

            double raw_cd = std::abs(drag) / f_dyn_lattice;
            double raw_cl = lift / f_dyn_lattice;
            double lift_N = raw_cl * q_inf * ref_area_m2;
            double drag_N = raw_cd * q_inf * ref_area_m2;

            last_cl = raw_cl;
            last_cd = raw_cd;
            last_lift_N = lift_N;
            last_drag_N = drag_N;
            last_fx = drag;
            last_fy = lift;

            if (step > warmupPerAngle) {
                sum_cl += raw_cl;
                sum_cd += raw_cd;
                sum_lift_N += lift_N;
                sum_drag_N += drag_N;
                sum_fx += drag;
                sum_fy += lift;
                sampled_count++;
            }
        }

        auto ptEndTime = std::chrono::high_resolution_clock::now();
        double ptDurationSec = std::chrono::duration<double>(ptEndTime - ptStartTime).count();

        double mean_cl = (sampled_count > 0) ? (sum_cl / sampled_count) : last_cl;
        double mean_cd = (sampled_count > 0) ? (sum_cd / sampled_count) : last_cd;
        double mean_ld = (std::abs(mean_cd) > 1e-6) ? (mean_cl / mean_cd) : 0.0;
        double mean_lift_N = (sampled_count > 0) ? (sum_lift_N / sampled_count) : last_lift_N;
        double mean_drag_N = (sampled_count > 0) ? (sum_drag_N / sampled_count) : last_drag_N;
        double mean_fx = (sampled_count > 0) ? (sum_fx / sampled_count) : last_fx;
        double mean_fy = (sampled_count > 0) ? (sum_fy / sampled_count) : last_fy;

        records.push_back({currentAlpha, mean_cl, mean_cd, mean_ld, mean_lift_N, mean_drag_N, mean_fx, mean_fy});

        std::cout << std::setw(10) << std::fixed << std::setprecision(2) << currentAlpha
                  << std::setw(12) << std::fixed << std::setprecision(4) << mean_cl
                  << std::setw(12) << std::fixed << std::setprecision(4) << mean_cd
                  << std::setw(12) << std::fixed << std::setprecision(3) << mean_ld
                  << std::setw(14) << std::fixed << std::setprecision(2) << mean_lift_N
                  << std::setw(14) << std::fixed << std::setprecision(2) << mean_drag_N
                  << std::setw(14) << std::fixed << std::setprecision(4) << mean_fx
                  << std::setw(14) << std::fixed << std::setprecision(4) << mean_fy
                  << std::setw(12) << std::fixed << std::setprecision(2) << ptDurationSec << "\n";
    }

    auto sweepEndTime = std::chrono::high_resolution_clock::now();
    double totalSweepSec = std::chrono::duration<double>(sweepEndTime - sweepStartTime).count();

    std::ofstream csvFile(csvOutPath);
    if (csvFile.is_open()) {
        csvFile << "alpha_deg,CL,CD,L_D,Lift_N,Drag_N,Force_X_lattice,Force_Y_lattice,V_inf_mps,q_inf_Pa\n";
        for (const auto& r : records) {
            csvFile << std::fixed << std::setprecision(4) << r.alpha << ","
                    << std::setprecision(6) << r.cl << ","
                    << std::setprecision(6) << r.cd << ","
                    << std::setprecision(6) << r.ld << ","
                    << std::setprecision(4) << r.lift_N << ","
                    << std::setprecision(4) << r.drag_N << ","
                    << std::setprecision(6) << r.fx_lattice << ","
                    << std::setprecision(6) << r.fy_lattice << ","
                    << std::setprecision(2) << sim.flow.V_inf << ","
                    << std::setprecision(2) << q_inf << "\n";
        }
        csvFile.close();
        std::cout << "\n[SUCCESS] Polar sweep data successfully exported to: " << csvOutPath << "\n";
    } else {
        std::cerr << "\n[ERROR] Failed to open CSV file for writing: " << csvOutPath << "\n";
    }

    double max_cl = -1e9;
    double alpha_stall = 0.0;
    double min_cd = 1e9;
    double alpha_min_cd = 0.0;
    double max_ld = -1e9;
    double alpha_opt_ld = 0.0;
    bool found_zero_lift = false;
    double alpha_zero_lift = 0.0;

    for (size_t i = 0; i < records.size(); ++i) {
        if (records[i].cl > max_cl) {
            max_cl = records[i].cl;
            alpha_stall = records[i].alpha;
        }
        if (records[i].cd < min_cd) {
            min_cd = records[i].cd;
            alpha_min_cd = records[i].alpha;
        }
        if (records[i].ld > max_ld) {
            max_ld = records[i].ld;
            alpha_opt_ld = records[i].alpha;
        }
        if (i > 0 && !found_zero_lift) {
            if ((records[i-1].cl <= 0.0 && records[i].cl >= 0.0) ||
                (records[i-1].cl >= 0.0 && records[i].cl <= 0.0)) {
                double dCl = records[i].cl - records[i-1].cl;
                if (std::abs(dCl) > 1e-6) {
                    alpha_zero_lift = records[i-1].alpha - records[i-1].cl * (records[i].alpha - records[i-1].alpha) / dCl;
                    found_zero_lift = true;
                }
            }
        }
    }

    double cl_alpha_deg = 0.0;
    double cl_alpha_rad = 0.0;
    double sumA = 0.0, sumCL = 0.0, sumA2 = 0.0, sumACL = 0.0;
    int regCount = 0;
    for (const auto& r : records) {
        if (r.alpha >= 0.0 && r.alpha <= 8.0 && r.alpha <= alpha_stall) {
            sumA += r.alpha;
            sumCL += r.cl;
            sumA2 += r.alpha * r.alpha;
            sumACL += r.alpha * r.cl;
            regCount++;
        }
    }
    if (regCount >= 2) {
        double denom = regCount * sumA2 - sumA * sumA;
        if (std::abs(denom) > 1e-6) {
            cl_alpha_deg = (regCount * sumACL - sumA * sumCL) / denom;
            cl_alpha_rad = cl_alpha_deg * (180.0 / 3.14159265358979323846);
        }
    }

    std::cout << "\n=================================================================\n";
    std::cout << "                 POLAR SWEEP AERODYNAMIC SUMMARY                 \n";
    std::cout << "=================================================================\n";
    std::cout << "  Max Lift Coefficient (CL,max) : " << std::fixed << std::setprecision(4) << max_cl << " (at alpha = " << std::setprecision(1) << alpha_stall << "°)\n";
    std::cout << "  Min Drag Coefficient (CD,min) : " << std::fixed << std::setprecision(4) << min_cd << " (at alpha = " << std::setprecision(1) << alpha_min_cd << "°)\n";
    std::cout << "  Max Lift-to-Drag Ratio (L/D)  : " << std::fixed << std::setprecision(3) << max_ld << " (at alpha = " << std::setprecision(1) << alpha_opt_ld << "°)\n";
    if (found_zero_lift) {
        std::cout << "  Zero-Lift Angle (alpha_0)     : " << std::fixed << std::setprecision(2) << alpha_zero_lift << "°\n";
    }
    if (regCount >= 2) {
        std::cout << "  Lift Curve Slope (dCL/dAlpha) : " << std::fixed << std::setprecision(4) << cl_alpha_deg << " /deg  (" << std::setprecision(3) << cl_alpha_rad << " /rad)\n";
    }
    std::cout << "  Total Sweep Execution Time    : " << std::fixed << std::setprecision(2) << totalSweepSec << " s\n";
    std::cout << "=================================================================\n\n";

    return 0;
}

#ifdef _WIN32
static void enableVirtualTerminalProcessing() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        DWORD dwMode = 0;
        if (GetConsoleMode(hOut, &dwMode)) {
            dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, dwMode);
        }
    }
}

static void clearConsoleScreen() {
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        if (GetConsoleScreenBufferInfo(hOut, &csbi)) {
            DWORD cellCount = csbi.dwSize.X * csbi.dwSize.Y;
            DWORD count;
            COORD homeCoords = { 0, 0 };
            FillConsoleOutputCharacter(hOut, (TCHAR)' ', cellCount, homeCoords, &count);
            FillConsoleOutputAttribute(hOut, csbi.wAttributes, cellCount, homeCoords, &count);
            SetConsoleCursorPosition(hOut, homeCoords);
            return;
        }
    }
    std::cout << "\033[2J\033[H" << std::flush;
}
#else
static void clearConsoleScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}
#endif

static int selectMenuItem(const std::string& title, const std::vector<std::string>& items, int defaultIndex = 0) {
    int current = defaultIndex;
    int count = static_cast<int>(items.size());

    while (true) {
        clearConsoleScreen();

        std::cout << "=================================================================\n";
        std::cout << "  " << title << "\n";
        std::cout << "=================================================================\n";
        std::cout << "  [Up/Down Arrows] Navigate | [1-" << count << "] Select | [Enter] Confirm | [Esc/q] Back\n\n";

        for (int i = 0; i < count; ++i) {
            if (i == current) {
                std::cout << "  \033[1;36m>> [" << (i + 1) << "] " << items[i] << " <<\033[0m\n";
            } else {
                std::cout << "     [" << (i + 1) << "] " << items[i] << "\n";
            }
        }
        std::cout << "\n=================================================================\n";
        std::cout << "  Current Selection: [" << (current + 1) << "] " << items[current] << "\n";

#ifdef _WIN32
        int ch = _getch();
        if (ch == 224 || ch == 0) {
            int arrow = _getch();
            if (arrow == 72) {
                current = (current - 1 + count) % count;
                continue;
            } else if (arrow == 80) {
                current = (current + 1) % count;
                continue;
            }
        } else if (ch == 13 || ch == 10) {
            clearConsoleScreen();
            return current;
        } else if (ch >= '1' && ch < '1' + count) {
            clearConsoleScreen();
            return ch - '1';
        } else if (ch == 27 || ch == 'q' || ch == 'Q') {
            clearConsoleScreen();
            return -1;
        }
#else
        std::string line;
        if (!std::getline(std::cin, line)) return -1;
        if (line.empty()) {
            clearConsoleScreen();
            return current;
        }
        try {
            int val = std::stoi(line);
            if (val >= 1 && val <= count) {
                clearConsoleScreen();
                return val - 1;
            }
        } catch (...) {}
#endif
    }
}

static std::string promptString(const std::string& prompt, const std::string& defaultVal) {
    std::cout << prompt << " [" << defaultVal << "]: ";
    std::string input;
    std::getline(std::cin, input);
    if (input.empty()) return defaultVal;
    return input;
}

static double promptDouble(const std::string& prompt, double defaultVal) {
    std::cout << prompt << " [" << defaultVal << "]: ";
    std::string input;
    std::getline(std::cin, input);
    if (input.empty()) return defaultVal;
    try {
        return std::stod(input);
    } catch (...) {
        return defaultVal;
    }
}

static int promptInt(const std::string& prompt, int defaultVal) {
    std::cout << prompt << " [" << defaultVal << "]: ";
    std::string input;
    std::getline(std::cin, input);
    if (input.empty()) return defaultVal;
    try {
        return std::stoi(input);
    } catch (...) {
        return defaultVal;
    }
}

static bool promptBool(const std::string& prompt, bool defaultVal) {
    std::cout << prompt << " [" << (defaultVal ? "Y/n" : "y/N") << "]: ";
    std::string input;
    std::getline(std::cin, input);
    if (input.empty()) return defaultVal;
    char c = std::tolower(input[0]);
    if (c == 'y') return true;
    if (c == 'n') return false;
    return defaultVal;
}

static std::string getGeometryDescription(const CLIOptions& opt) {
    if (opt.modelSet && !opt.modelFile.empty()) {
        return "Custom File (" + opt.modelFile + ")";
    }
    if (opt.shape == 1) {
        return "Cylinder (Bluff Body, r=0.25)";
    }
    if (opt.shape == 2) {
        return "Flat Plate (t/c=" + std::to_string(opt.thickness).substr(0, 4) + ")";
    }
    if (opt.shape == 3) {
        return "Diamond Airfoil (t/c=" + std::to_string(opt.thickness).substr(0, 4) + ")";
    }
    int m_digit = std::round(opt.camber * 100);
    int p_digit = 4;
    int t_digit = std::round(opt.thickness * 100);
    std::string name = "NACA " + std::to_string(m_digit) + std::to_string(p_digit) + (t_digit < 10 ? "0" : "") + std::to_string(t_digit);
    return name + " (m=" + std::to_string(opt.camber).substr(0, 4) + ", t=" + std::to_string(opt.thickness).substr(0, 4) + ")";
}

static std::string getGridDescription(const CLIOptions& opt, const Config& config) {
    int nx = (opt.nxSet && opt.gridNX > 0) ? opt.gridNX : config.lbmGridNX;
    int ny = (opt.nySet && opt.gridNY > 0) ? opt.gridNY : config.lbmGridNY;
    int nz = (opt.nzSet && opt.gridNZ > 0) ? opt.gridNZ : config.lbmGridNZ;
    if (opt.gridScaleSet && opt.gridScale > 0.0) {
        nx = std::max(16, static_cast<int>(std::round(nx * opt.gridScale)));
        ny = std::max(8, static_cast<int>(std::round(ny * opt.gridScale)));
        nz = std::max(8, static_cast<int>(std::round(nz * opt.gridScale)));
    }
    int totalCells = nx * ny * nz;
    std::string s = std::to_string(nx) + " x " + std::to_string(ny) + " x " + std::to_string(nz);
    if (opt.gridScaleSet && opt.gridScale > 0.0) {
        s += " (" + std::to_string(opt.gridScale).substr(0, 4) + "x scale)";
    }
    s += " [" + std::to_string(totalCells) + " cells]";
    return s;
}

static void configureGrid(CLIOptions& opt) {
    std::vector<std::string> gridItems = {
        "Fast Grid: 128 x 64 x 64 (0.52M cells, Standard)",
        "Medium Grid: 192 x 96 x 96 (1.77M cells, High Accuracy)",
        "High-Res Grid: 256 x 128 x 128 (4.19M cells, Fine Wake)",
        "Scale Factor Multiplier (e.g. 1.25, 1.5, 2.0)",
        "Custom Dimensions (NX x NY x NZ)",
        "Back to Dashboard"
    };
    int c = selectMenuItem("Configure Grid Resolution & Domain", gridItems, 0);
    if (c == 0) {
        opt.gridNX = 128; opt.gridNY = 64; opt.gridNZ = 64; opt.nxSet = opt.nySet = opt.nzSet = true;
        opt.gridScaleSet = false;
    } else if (c == 1) {
        opt.gridNX = 192; opt.gridNY = 96; opt.gridNZ = 96; opt.nxSet = opt.nySet = opt.nzSet = true;
        opt.gridScaleSet = false;
    } else if (c == 2) {
        opt.gridNX = 256; opt.gridNY = 128; opt.gridNZ = 128; opt.nxSet = opt.nySet = opt.nzSet = true;
        opt.gridScaleSet = false;
    } else if (c == 3) {
        opt.gridScale = promptDouble("  Enter Grid Scale Multiplier (e.g. 1.5, 2.0)", 1.5);
        opt.gridScaleSet = true;
    } else if (c == 4) {
        opt.gridNX = promptInt("  Grid NX dimension", 128);
        opt.gridNY = promptInt("  Grid NY dimension", 64);
        opt.gridNZ = promptInt("  Grid NZ dimension", 64);
        opt.nxSet = opt.nySet = opt.nzSet = true;
        opt.gridScaleSet = false;
    }
}

static void configureGeometry(CLIOptions& opt) {
    std::vector<std::string> shapeItems = {
        "NACA 4-Digit (Interactive Camber & Thickness)",
        "NACA 0012 (Symmetric Airfoil)",
        "NACA 2412 (Cambered General Aviation Airfoil)",
        "NACA 4412 (High-Lift Cambered Airfoil)",
        "Thick Airfoil (m=0.06, t=0.28)",
        "Cylinder (Bluff Body, Karman Vortex Street)",
        "Diamond Airfoil (Supersonic Double-Wedge)",
        "Flat Plate (Thin Aerodynamic Section)",
        "Load Custom File (.dat, .stl, .obj)",
        "Back to Dashboard"
    };
    int shapeChoice = selectMenuItem("Select Geometry Profile", shapeItems, 0);
    if (shapeChoice < 0 || shapeChoice == 9) return;

    opt.modelSet = false;
    opt.modelFile = "";

    if (shapeChoice == 0) {
        opt.shape = 0;
        opt.camber = promptDouble("  Enter NACA Camber (0.00 to 0.09)", 0.02);
        opt.camberSet = true;
        opt.thickness = promptDouble("  Enter NACA Thickness (0.05 to 0.30)", 0.12);
        opt.thicknessSet = true;
    } else if (shapeChoice == 1) {
        opt.shape = 0; opt.camber = 0.0; opt.thickness = 0.12; opt.preset = "naca0012";
    } else if (shapeChoice == 2) {
        opt.shape = 0; opt.camber = 0.02; opt.thickness = 0.12; opt.preset = "naca2412";
    } else if (shapeChoice == 3) {
        opt.shape = 0; opt.camber = 0.04; opt.thickness = 0.12; opt.preset = "naca4412";
    } else if (shapeChoice == 4) {
        opt.shape = 0; opt.camber = 0.06; opt.thickness = 0.28; opt.preset = "thick";
    } else if (shapeChoice == 5) {
        opt.shape = 1; opt.preset = "cylinder";
    } else if (shapeChoice == 6) {
        opt.shape = 3;
        opt.thickness = promptDouble("  Diamond Thickness Ratio (t/c)", 0.10);
        opt.thicknessSet = true;
        opt.preset = "diamond";
    } else if (shapeChoice == 7) {
        opt.shape = 2;
        opt.thickness = promptDouble("  Plate Thickness Ratio (t/c)", 0.04);
        opt.thicknessSet = true;
        opt.preset = "flatplate";
    } else if (shapeChoice == 8) {
        std::string file = promptString("  Path to .dat / .stl / .obj file", "assets/naca2412.dat");
        opt.modelFile = file;
        opt.modelSet = true;
    }
    opt.shapeSet = true;
}

static void configurePresets(CLIOptions& opt) {
    std::vector<std::string> presetItems = {
        "NACA 0012 at alpha = 4.0 deg (Symmetric Lift/Drag Baseline)",
        "NACA 2412 at alpha = 4.0 deg (Cambered Lift Generation)",
        "NACA 4412 at alpha = 5.0 deg (High-Lift Profile)",
        "Thick Airfoil at alpha = 5.0 deg (Massive Wake Separation)",
        "Cylinder at alpha = 0.0 deg (Karman Vortex Street Bluff Body)",
        "Flat Plate at alpha = 3.0 deg (Thin Aerodynamic Plate)",
        "Diamond Airfoil at alpha = 4.0 deg (Supersonic Double-Wedge)",
        "Back to Dashboard"
    };
    int qChoice = selectMenuItem("Select Quick Preset", presetItems, 0);
    if (qChoice < 0 || qChoice == 7) return;

    opt.modelSet = false;
    opt.modelFile = "";
    opt.steps = 150;
    opt.warmup = 40;
    opt.speed = 15.0;
    opt.verbose = true;

    if (qChoice == 0) {
        opt.shape = 0; opt.camber = 0.0; opt.thickness = 0.12; opt.alpha = 4.0; opt.preset = "naca0012";
    } else if (qChoice == 1) {
        opt.shape = 0; opt.camber = 0.02; opt.thickness = 0.12; opt.alpha = 4.0; opt.preset = "naca2412";
    } else if (qChoice == 2) {
        opt.shape = 0; opt.camber = 0.04; opt.thickness = 0.12; opt.alpha = 5.0; opt.preset = "naca4412";
    } else if (qChoice == 3) {
        opt.shape = 0; opt.camber = 0.06; opt.thickness = 0.28; opt.alpha = 5.0; opt.preset = "thick";
    } else if (qChoice == 4) {
        opt.shape = 1; opt.alpha = 0.0; opt.preset = "cylinder";
    } else if (qChoice == 5) {
        opt.shape = 2; opt.thickness = 0.04; opt.alpha = 3.0; opt.preset = "flatplate";
    } else if (qChoice == 6) {
        opt.shape = 3; opt.thickness = 0.10; opt.alpha = 4.0; opt.preset = "diamond";
    }
    opt.shapeSet = true; opt.presetSet = true; opt.alphaSet = true; opt.speedSet = true; opt.stepsSet = true;
}

static void runPolarSweepDashboard(CLIOptions& opt, const Config& config) {
    CLIOptions polarOpt = opt;
    polarOpt.isCliMode = true;
    polarOpt.headless = true;
    polarOpt.polarSweep = true;
    polarOpt.polarSweepSet = true;

    int currentSelection = 0;

    while (true) {
        std::vector<std::string> items = {
            "Geometry Profile  : " + getGeometryDescription(polarOpt),
            "Min Angle (a_min) : " + std::to_string(polarOpt.alphaMin).substr(0, 5) + " deg",
            "Max Angle (a_max) : " + std::to_string(polarOpt.alphaMax).substr(0, 5) + " deg",
            "Angle Step (da)   : " + std::to_string(polarOpt.alphaStep).substr(0, 4) + " deg",
            "Airspeed (V_inf)  : " + std::to_string(polarOpt.speed).substr(0, 5) + " m/s",
            "Steps per Angle   : " + std::to_string(polarOpt.polarSteps) + " steps (Warmup: " + std::to_string(polarOpt.polarWarmup) + ")",
            "Grid Resolution   : " + getGridDescription(polarOpt, config),
            "Flow Reset Mode   : " + std::string(polarOpt.polarResetFlow ? "Reset Flow Field Every Angle" : "Continuous Field Evolution"),
            "CSV Output File   : " + polarOpt.polarCsvFile,
            ">>> RUN POLAR SWEEP NOW <<<",
            "Back to Main Dashboard"
        };

        int choice = selectMenuItem("ZweiCFD Automated Polar Sweep Configuration", items, currentSelection);
        currentSelection = (choice >= 0) ? choice : 0;

        if (choice == 0) {
            configureGeometry(polarOpt);
        } else if (choice == 1) {
            polarOpt.alphaMin = promptDouble("  Minimum Angle of Attack (deg)", polarOpt.alphaMin);
            polarOpt.alphaMinSet = true;
        } else if (choice == 2) {
            polarOpt.alphaMax = promptDouble("  Maximum Angle of Attack (deg)", polarOpt.alphaMax);
            polarOpt.alphaMaxSet = true;
        } else if (choice == 3) {
            polarOpt.alphaStep = promptDouble("  Angle Step Size (deg)", polarOpt.alphaStep);
            polarOpt.alphaStepSet = true;
        } else if (choice == 4) {
            polarOpt.speed = promptDouble("  Airspeed V_inf (m/s)", polarOpt.speed);
            polarOpt.speedSet = true;
        } else if (choice == 5) {
            polarOpt.polarSteps = promptInt("  Simulation Steps per Angle", polarOpt.polarSteps);
            polarOpt.polarWarmup = promptInt("  Warmup Steps per Angle", polarOpt.polarWarmup);
        } else if (choice == 6) {
            configureGrid(polarOpt);
        } else if (choice == 7) {
            polarOpt.polarResetFlow = promptBool("  Reset flow field between angles", polarOpt.polarResetFlow);
        } else if (choice == 8) {
            polarOpt.polarCsvFile = promptString("  Output CSV Filepath", polarOpt.polarCsvFile);
            polarOpt.polarCsvSet = true;
        } else if (choice == 9) {
            runPolarSweepCLI(polarOpt, config);
            std::cout << "\n[Press Enter to return to Polar Sweep menu...]\n";
            std::string dummy;
            std::getline(std::cin, dummy);
        } else {
            break;
        }
    }
}

int runInteractiveCLIMenu(CLIOptions& baseOpt, const Config& config) {
#ifdef _WIN32
    enableVirtualTerminalProcessing();
#endif

    CLIOptions opt = baseOpt;
    opt.isCliMode = true;
    opt.headless = true;
    if (!opt.shapeSet && !opt.presetSet && !opt.modelSet) {
        opt.shape = 0;
        opt.camber = 0.02;
        opt.thickness = 0.12;
        opt.shapeSet = true;
    }
    if (!opt.alphaSet) {
        opt.alpha = 4.0;
        opt.alphaSet = true;
    }
    if (!opt.speedSet) {
        opt.speed = 15.0;
        opt.speedSet = true;
    }
    if (!opt.stepsSet) {
        opt.steps = 200;
        opt.stepsSet = true;
    }

    int currentSelection = 0;

    while (true) {
        std::vector<std::string> dashboardItems = {
            "Geometry Profile  : " + getGeometryDescription(opt),
            "Angle of Attack   : " + std::to_string(opt.alpha).substr(0, 5) + " deg",
            "Airspeed (V_inf)  : " + std::to_string(opt.speed).substr(0, 5) + " m/s",
            "Grid & Domain     : " + getGridDescription(opt, config),
            "Simulation Steps  : " + std::to_string(opt.steps) + " steps (Warmup: " + std::to_string(opt.warmup) + ")",
            "Streamlines/Rake  : " + std::to_string(opt.lines) + " lines (Rake Y: " + std::to_string(opt.rakeY).substr(0, 4) + ")",
            "ParaView VTI Out  : " + (opt.vtiExportSet ? opt.vtiExportFile : "[Disabled]"),
            "Telemetry Output  : " + std::string(opt.verbose ? "Verbose Live Step Table (ON)" : "Summary Only (OFF)"),
            "Quick Presets     : (NACA 0012, 2412, 4412, Thick, Cylinder, Plate, Diamond)",
            "Automated Polar   : (Angle-of-Attack Sweep & CSV Export Mode)",
            ">>> RUN SIMULATION NOW <<<",
            "Exit CLI"
        };

        int choice = selectMenuItem("ZweiCFD Interactive Simulation Dashboard", dashboardItems, currentSelection);
        currentSelection = (choice >= 0) ? choice : 0;

        if (choice == 0) {
            configureGeometry(opt);
        } else if (choice == 1) {
            opt.alpha = promptDouble("  Angle of Attack in degrees", opt.alpha);
            opt.alphaSet = true;
        } else if (choice == 2) {
            opt.speed = promptDouble("  Airspeed V_inf (m/s)", opt.speed);
            opt.speedSet = true;
        } else if (choice == 3) {
            configureGrid(opt);
        } else if (choice == 4) {
            opt.steps = promptInt("  Simulation Steps to run", opt.steps);
            opt.stepsSet = true;
            opt.warmup = promptInt("  Warmup Steps before force averaging", opt.warmup);
        } else if (choice == 5) {
            opt.lines = promptInt("  Streamline Line Count", opt.lines);
            opt.linesSet = true;
            opt.rakeY = promptDouble("  Streamline Rake Y Offset", opt.rakeY);
            opt.rakeYSet = true;
        } else if (choice == 6) {
            std::string vti = promptString("  ParaView VTI Output File (leave empty to disable)", opt.vtiExportFile);
            if (vti.empty()) {
                opt.vtiExportSet = false;
                opt.vtiExportFile = "";
            } else {
                opt.vtiExportSet = true;
                opt.vtiExportFile = vti;
            }
        } else if (choice == 7) {
            opt.verbose = !opt.verbose;
        } else if (choice == 8) {
            configurePresets(opt);
        } else if (choice == 9) {
            runPolarSweepDashboard(opt, config);
        } else if (choice == 10) {
            runHeadlessCLI(opt, config);
            std::cout << "\n[Press Enter to return to Dashboard...]\n";
            std::string dummy;
            std::getline(std::cin, dummy);
        } else {
            std::cout << "\nExiting ZweiCFD CLI. Goodbye!\n\n";
            break;
        }
    }
    return 0;
}

}
