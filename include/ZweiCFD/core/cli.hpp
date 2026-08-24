#pragma once

#include <string>
#include "ZweiCFD/core/config.hpp"

namespace zweicfd {

struct CLIOptions {
    bool isCliMode = false;
    bool headless = false;
    bool verbose = false;
    bool help = false;

    double alpha = 0.0;
    double camber = 0.02;
    double thickness = 0.12;
    double speed = 15.0;
    int shape = 0;
    std::string modelFile = "";
    std::string preset = "";
    int lines = 100;
    double rakeY = 0.0;
    int steps = 200;
    int warmup = 50;
    int gridNX = 0;
    int gridNY = 0;
    int gridNZ = 0;
    double gridScale = 0.0;
    std::string vtiExportFile = "";

    bool polarSweep = false;
    double alphaMin = -4.0;
    double alphaMax = 16.0;
    double alphaStep = 2.0;
    int polarSteps = 200;
    int polarWarmup = 50;
    std::string polarCsvFile = "polar_results.csv";
    bool polarResetFlow = true;

    bool alphaSet = false;
    bool camberSet = false;
    bool thicknessSet = false;
    bool speedSet = false;
    bool shapeSet = false;
    bool modelSet = false;
    bool presetSet = false;
    bool linesSet = false;
    bool rakeYSet = false;
    bool stepsSet = false;
    bool nxSet = false;
    bool nySet = false;
    bool nzSet = false;
    bool gridScaleSet = false;
    bool vtiExportSet = false;
    bool polarSweepSet = false;
    bool alphaMinSet = false;
    bool alphaMaxSet = false;
    bool alphaStepSet = false;
    bool polarCsvSet = false;
    bool interactiveMenu = false;
};

CLIOptions parseCLI(int argc, char* argv[]);
void printHelp(const char* progName);
int runHeadlessCLI(const CLIOptions& options, const Config& config);
int runPolarSweepCLI(const CLIOptions& options, const Config& config);
int runInteractiveCLIMenu(CLIOptions& options, const Config& config);

}
