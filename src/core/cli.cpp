#include "ZweiCFD/core/cli.hpp"
#include "ZweiCFD/core/simulation.hpp"
#include "ZweiCFD/solver/lbm_solver.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <vector>

namespace zweicfd {

void printHelp(const char* progName) {
    std::cout << "=================================================================\n";
    std::cout << "                      ZweiCFD CLI & Debug Mode                   \n";
    std::cout << "=================================================================\n";
    std::cout << "Usage:\n";
    std::cout << "  " << progName << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help              Show this help message\n";
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
    std::cout << "  --lines <num>           Streamline line count (default: 100)\n";
    std::cout << "  --rake-y <val>          Streamline rake Y offset (default: 0.0)\n";
    std::cout << "=================================================================\n";
}

CLIOptions parseCLI(int argc, char* argv[]) {
    CLIOptions opt;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            opt.help = true;
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
        } else if (arg == "--lines" && i + 1 < argc) {
            opt.lines = std::stoi(argv[++i]);
            opt.linesSet = true;
        } else if (arg == "--rake-y" && i + 1 < argc) {
            opt.rakeY = std::stod(argv[++i]);
            opt.rakeYSet = true;
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

    return opt;
}

int runHeadlessCLI(const CLIOptions& opt, const Config& config) {
    std::cout << "\n=================================================================\n";
    std::cout << "                 ZweiCFD Headless Simulation Run                 \n";
    std::cout << "=================================================================\n";

    Simulation sim(0, nullptr);
    sim.config = config;
    sim.flow.alpha = opt.alpha;
    sim.flow.V_inf = opt.speed;
    
    if (opt.modelSet && !opt.modelFile.empty()) {
        std::cout << "Loading custom geometry from: " << opt.modelFile << "\n";
        sim.foil.loadFromFile(opt.modelFile);
    } else if (opt.shape == 1) {
        std::cout << "Generating Cylinder Geometry...\n";
        sim.foil.generateCylinder(0.25, 100);
    } else if (opt.shape == 3) {
        std::cout << "Generating Diamond Geometry...\n";
        sim.foil.generateCylinder(0.001, 3);
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
        a_ref_lattice = std::max(50.0, (1.0 * scale) * (double)config.lbmGridNZ);
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
    std::cout << "=================================================================\n\n";

    return 0;
}

}
