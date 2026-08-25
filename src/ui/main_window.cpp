#include "ZweiCFD/ui/main_window.hpp"
#include "ZweiCFD/ui/polar_dialog.hpp"
#include "ZweiCFD/ui/cp_dialog.hpp"
#include "ZweiCFD/core/cli.hpp"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QPushButton>
#include <QToolBar>
#include <QMenuBar>
#include <QMenu>
#include <QFileDialog>
#include <QAction>
#include <QActionGroup>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QProgressDialog>
#include <QStatusBar>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>
#include <algorithm>

namespace zweicfd {

MainWindow::MainWindow(const CLIOptions* opt, QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("ZweiCFD Engine - Qt + VTK");
    resize(1400, 800);

    simulation = std::make_unique<Simulation>(0, nullptr);

    auto vtkRenderWidget = new QVTKOpenGLNativeWidget(this);
    setCentralWidget(vtkRenderWidget);
    
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkRenderWidget->setRenderWindow(renderWindow);
    
    simulation->setupVTKWithWindow(renderWindow);

    simTimer = new QTimer(this);
    connect(simTimer, &QTimer::timeout, this, &MainWindow::updateSimulation);
    simTimer->start(22);

    QMenuBar* menuBar = new QMenuBar(this);
    this->setMenuBar(menuBar);
    
    QMenu* fileMenu = menuBar->addMenu("&File");
    QAction* loadAction = fileMenu->addAction("&Load Model/Profile...");
    connect(loadAction, &QAction::triggered, this, [this]() {
        QString fileName = QFileDialog::getOpenFileName(
            this,
            "Open Custom Model or Profile",
            "",
            "All Supported Formats (*.dat *.txt *.csv *.stl *.obj);;2D Airfoils (*.dat *.txt *.csv);;3D Meshes (*.stl *.obj);;All Files (*.*)"
        );
        if (!fileName.isEmpty()) {
            if (simulation->foil.loadFromFile(fileName.toStdString())) {
                simulation->customFoil = simulation->foil;
                simulation->hasCustomFoil = true;
                shapeSelector->blockSignals(true);
                shapeSelector->setCurrentIndex(2);
                shapeSelector->blockSignals(false);
                
                if (alphaSlider) {
                    alphaSlider->blockSignals(true);
                    alphaSlider->setValue(0);
                    alphaSlider->blockSignals(false);
                }
                simulation->flow.alpha = 0.0;
                simulation->setVisualRotation(0, 0, 0);

                if (simulation->foil.is3D()) {
                    simulation->freezeFlow = true;
                    simulation->rotatedFoil = simulation->foil;
                    simulation->updateVTKGeometry();
                    
                    QDialog* dlg = new QDialog(this);
                    dlg->setWindowTitle("Set Initial Model Orientation");
                    dlg->setWindowFlags(dlg->windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
                    dlg->setAttribute(Qt::WA_DeleteOnClose);
                    dlg->setModal(false);
                    QVBoxLayout* layout = new QVBoxLayout(dlg);
                    
                    QLabel* infoLabel = new QLabel("Adjust model orientation to align with freestream (Left to Right):");
                    layout->addWidget(infoLabel);
                    
                    QSlider* pitchSlider = new QSlider(Qt::Horizontal);
                    pitchSlider->setRange(-180, 180);
                    pitchSlider->setValue(0);
                    QSlider* yawSlider = new QSlider(Qt::Horizontal);
                    yawSlider->setRange(-180, 180);
                    yawSlider->setValue(0);
                    QSlider* rollSlider = new QSlider(Qt::Horizontal);
                    rollSlider->setRange(-180, 180);
                    rollSlider->setValue(0);
                    
                    QLabel* pLabel = new QLabel("Pitch (Z / Nose Up-Down): 0°");
                    QLabel* yLabel = new QLabel("Yaw (Y / Turn Left-Right): 0°");
                    QLabel* rLabel = new QLabel("Roll (X / Bank Wings): 0°");
                    
                    auto updateRotation = [this, pitchSlider, yawSlider, rollSlider, pLabel, yLabel, rLabel]() {
                        pLabel->setText(QString("Pitch (Z / Nose Up-Down): %1°").arg(pitchSlider->value()));
                        yLabel->setText(QString("Yaw (Y / Turn Left-Right): %1°").arg(yawSlider->value()));
                        rLabel->setText(QString("Roll (X / Bank Wings): %1°").arg(rollSlider->value()));
                        simulation->foil.setBaseRotation(rollSlider->value(), yawSlider->value(), pitchSlider->value());
                        simulation->customFoil = simulation->foil;
                        simulation->rotatedFoil = simulation->foil;
                        simulation->updateVTKGeometry();
                        if (auto vtkRenderWidget = qobject_cast<QVTKOpenGLNativeWidget*>(centralWidget())) {
                            vtkRenderWidget->renderWindow()->Render();
                        }
                    };
                    
                    connect(pitchSlider, &QSlider::valueChanged, dlg, updateRotation);
                    connect(yawSlider, &QSlider::valueChanged, dlg, updateRotation);
                    connect(rollSlider, &QSlider::valueChanged, dlg, updateRotation);
                    
                    layout->addWidget(pLabel); layout->addWidget(pitchSlider);
                    layout->addWidget(yLabel); layout->addWidget(yawSlider);
                    layout->addWidget(rLabel); layout->addWidget(rollSlider);
                    
                    QHBoxLayout* quickBtns = new QHBoxLayout();
                    QPushButton* p90Btn = new QPushButton("+90° Pitch", dlg);
                    connect(p90Btn, &QPushButton::clicked, [pitchSlider]() {
                        int v = pitchSlider->value() + 90;
                        if (v > 180) v -= 360;
                        pitchSlider->setValue(v);
                    });
                    QPushButton* y90Btn = new QPushButton("+90° Yaw", dlg);
                    connect(y90Btn, &QPushButton::clicked, [yawSlider]() {
                        int v = yawSlider->value() + 90;
                        if (v > 180) v -= 360;
                        yawSlider->setValue(v);
                    });
                    QPushButton* r90Btn = new QPushButton("+90° Roll", dlg);
                    connect(r90Btn, &QPushButton::clicked, [rollSlider]() {
                        int v = rollSlider->value() + 90;
                        if (v > 180) v -= 360;
                        rollSlider->setValue(v);
                    });
                    QPushButton* rstBtn = new QPushButton("Reset", dlg);
                    connect(rstBtn, &QPushButton::clicked, [pitchSlider, yawSlider, rollSlider]() {
                        pitchSlider->setValue(0);
                        yawSlider->setValue(0);
                        rollSlider->setValue(0);
                    });
                    quickBtns->addWidget(p90Btn);
                    quickBtns->addWidget(y90Btn);
                    quickBtns->addWidget(r90Btn);
                    quickBtns->addWidget(rstBtn);
                    layout->addLayout(quickBtns);

                    QPushButton* okBtn = new QPushButton("Apply & Start Simulation", dlg);
                    okBtn->setStyleSheet("QPushButton { font-weight: bold; padding: 6px; }");
                    connect(okBtn, &QPushButton::clicked, dlg, &QDialog::accept);
                    connect(dlg, &QDialog::finished, this, [this]() {
                        updateMorphing();
                        simulation->freezeFlow = false;
                    });
                    layout->addWidget(okBtn);
                    
                    dlg->show();
                    dlg->move(this->x() + 50, this->y() + 100);
                } else {
                    updateMorphing();
                }
            }
        }
    });
    QAction* exportAction = fileMenu->addAction("&Export to ParaView (.vti)...");
    exportAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
    connect(exportAction, &QAction::triggered, this, [this]() {
        QString fileName = QFileDialog::getSaveFileName(
            this,
            "Export Simulation to ParaView VTI",
            "simulation.vti",
            "ParaView Image Data (*.vti);;All Files (*.*)"
        );
        if (!fileName.isEmpty()) {
            if (!fileName.endsWith(".vti", Qt::CaseInsensitive)) {
                fileName += ".vti";
            }
            if (simulation && simulation->exportToVTI(fileName.toStdString())) {
                statusBar()->showMessage(QString("Successfully exported to %1").arg(fileName), 5000);
            } else {
                QMessageBox::warning(this, "Export Failed", "Failed to export simulation grid to .vti file.");
            }
        }
    });
    fileMenu->addSeparator();
    fileMenu->addAction("E&xit", this, &QWidget::close);

    QMenu* presetMenu = menuBar->addMenu("&Presets");
    QAction* karmanAction = presetMenu->addAction("Karman Vortex Street");
    connect(karmanAction, &QAction::triggered, this, [this]() {
        shapeSelector->setCurrentIndex(1);
        speedSlider->setValue(8);
        colormapSelector->setCurrentIndex(2);
        if (simulation) {
            simulation->flow.V_inf = 0.08;
            simulation->flow.kinematic_viscosity = 0.005;
            simulation->resetFlow();
        }
    });

    QAction* bluffAction = presetMenu->addAction("Large Bluff Body");
    connect(bluffAction, &QAction::triggered, this, [this]() {
        if (simulation) {
            simulation->foil.generateCylinder(0.75, 100);
            shapeSelector->blockSignals(true);
            shapeSelector->setCurrentIndex(2);
            shapeSelector->blockSignals(false);
            speedSlider->setValue(4);
            colormapSelector->setCurrentIndex(1);
            simulation->flow.V_inf = 0.04;
            simulation->flow.kinematic_viscosity = 0.01;
            simulation->rebuildSolverWithRotation();
            simulation->freezeFlow = false;
            simulation->updateVTKGeometry();
        }
    });

    QAction* thickAction = presetMenu->addAction("Thick Airfoil");
    connect(thickAction, &QAction::triggered, this, [this]() {
        shapeSelector->setCurrentIndex(0);
        camberSlider->setValue(6);
        thicknessSlider->setValue(28);
        speedSlider->setValue(5);
        colormapSelector->setCurrentIndex(0);
        updateMorphing();
    });

    QMenu* drawMenu = menuBar->addMenu("&Draw");
    drawModeAction = drawMenu->addAction("&Start Drawing Mode");
    drawModeAction->setShortcut(QKeySequence("Ctrl+D"));
    connect(drawModeAction, &QAction::triggered, this, [this]() {
        setDrawingMode(!simulation->drawMode);
    });
    QAction* clearAction = drawMenu->addAction("&Clear Canvas");
    connect(clearAction, &QAction::triggered, this, [this]() {
        if (simulation) simulation->clearDrawing();
    });

    QMenu* windMenu = menuBar->addMenu("&Wind");
    windActionGroup = new QActionGroup(this);
    windActionGroup->setExclusive(true);

    windLToRAction = windMenu->addAction("Left to Right (Headwind)");
    windLToRAction->setCheckable(true);
    windLToRAction->setChecked(true);
    windLToRAction->setData(0);
    windActionGroup->addAction(windLToRAction);

    windRToLAction = windMenu->addAction("Right to Left (Tailwind)");
    windRToLAction->setCheckable(true);
    windRToLAction->setData(1);
    windActionGroup->addAction(windRToLAction);

    windTToBAction = windMenu->addAction("Top to Bottom (Downdraft)");
    windTToBAction->setCheckable(true);
    windTToBAction->setData(2);
    windActionGroup->addAction(windTToBAction);

    windBToTAction = windMenu->addAction("Bottom to Top (Updraft)");
    windBToTAction->setCheckable(true);
    windBToTAction->setData(3);
    windActionGroup->addAction(windBToTAction);

    connect(windActionGroup, &QActionGroup::triggered, this, [this](QAction* act) {
        if (act) setWindDirection(act->data().toInt());
    });

    QMenu* gridMenu = menuBar->addMenu("&Grid");
    gridActionGroup = new QActionGroup(this);
    gridActionGroup->setExclusive(true);

    auto addGridAct = [&](const QString& name, int idx, bool checked = false) {
        QAction* act = gridMenu->addAction(name);
        act->setCheckable(true);
        act->setChecked(checked);
        act->setData(idx);
        gridActionGroup->addAction(act);
        return act;
    };

    addGridAct("64 x 32 x 32 (Fast - 65k cells)", 0);
    addGridAct("96 x 48 x 48 (Balanced - 221k cells)", 1);
    addGridAct("128 x 64 x 64 (Default - 524k cells)", 2, true);
    addGridAct("160 x 80 x 80 (High Res - 1.02M cells)", 3);
    addGridAct("192 x 96 x 96 (Extreme - 1.77M cells)", 4);

    connect(gridActionGroup, &QActionGroup::triggered, this, [this](QAction* act) {
        if (act) setGridPreset(act->data().toInt());
    });

    gridMenu->addSeparator();
    QAction* customGridAct = gridMenu->addAction("&Custom Grid Resolution...");
    connect(customGridAct, &QAction::triggered, this, &MainWindow::openCustomGridDialog);

    QMenu* controlsMenu = menuBar->addMenu("&Controls");
    QAction* configKeysAction = controlsMenu->addAction("&Configure Keybindings...");
    connect(configKeysAction, &QAction::triggered, this, &MainWindow::openControlsDialog);
    controlsMenu->addSeparator();
    QAction* resetCamAction = controlsMenu->addAction("&Reset Camera View");
    connect(resetCamAction, &QAction::triggered, this, [this]() {
        if (simulation) simulation->resetCameraView();
    });
    QAction* resetFlowAction = controlsMenu->addAction("Reset &Flow");
    connect(resetFlowAction, &QAction::triggered, this, [this]() {
        if (simulation) simulation->resetFlow();
    });

    QMenu* toolsMenu = menuBar->addMenu("&Tools");
    QAction* cpAction = toolsMenu->addAction("Surface Pressure &Distribution (Cp Curve)...");
    cpAction->setShortcut(QKeySequence("Ctrl+C"));
    connect(cpAction, &QAction::triggered, this, &MainWindow::openCpDialog);

    surfaceCpAction = toolsMenu->addAction("3D Surface Cp Colormap on &Mesh");
    surfaceCpAction->setCheckable(true);
    surfaceCpAction->setChecked(false);
    connect(surfaceCpAction, &QAction::toggled, this, &MainWindow::toggleSurfaceCp);

    qCritAction = toolsMenu->addAction("3D &Vortex Cores (Q-Criterion)");
    qCritAction->setCheckable(true);
    qCritAction->setChecked(false);
    qCritAction->setShortcut(QKeySequence("Ctrl+Q"));
    connect(qCritAction, &QAction::toggled, this, &MainWindow::toggleQCriterion);

    toolsMenu->addSeparator();

    QAction* polarAction = toolsMenu->addAction("&Run Polar Sweep (Alpha Sweep)...");
    polarAction->setShortcut(QKeySequence("Ctrl+P"));
    connect(polarAction, &QAction::triggered, this, &MainWindow::openPolarSweepDialog);

    QMenu* resultsMenu = menuBar->addMenu("&Results");
    ldAction = resultsMenu->addAction("L/D Ratio: ---");
    clAction = resultsMenu->addAction("Lift Coefficient (CL): ---");
    cdAction = resultsMenu->addAction("Drag Coefficient (CD): ---");
    resultsMenu->addSeparator();
    liftAction = resultsMenu->addAction("Lift Force: ---");
    dragAction = resultsMenu->addAction("Drag Force: ---");
    resultsMenu->addSeparator();
    alphaAction = resultsMenu->addAction("Angle of Attack: 0.0°");
    speedAction = resultsMenu->addAction("Airspeed: 10.0 m/s");
    qCritResultAction = resultsMenu->addAction("Q-Crit Threshold: 2.10e-05 s^-2 (Sensitivity: 20%)");

    setupUi(opt);
    setupShortcuts();
}

MainWindow::~MainWindow() = default;

void MainWindow::updateMorphing() {
    if (!simulation || simulation->isRebuilding) return;
    int shapeIdx = shapeSelector->currentIndex();
    
    bool isNACA = (shapeIdx == 0);
    camberSlider->setEnabled(isNACA);
    thicknessSlider->setEnabled(isNACA);
    flapToggle->setEnabled(isNACA);
    if (!isNACA) {
        flapToggle->setChecked(false);
        simulation->flapping = false;
    }

    if (shapeIdx == 0) {
        double m = camberSlider->value() / 100.0;
        double p = 0.4;
        double t = thicknessSlider->value() / 100.0;
        simulation->foil.generateNACA(m, p, t);
    } else if (shapeIdx == 1) {
        simulation->foil.generateCylinder(0.25, 100);
    } else if (shapeIdx == 2) {
        if (simulation->hasCustomFoil) {
            simulation->foil = simulation->customFoil;
        }
    } else if (shapeIdx == 3) {
        simulation->foil.generateCylinder(0.001, 3);
    }

    if (!this->isVisible()) {
        simulation->isRebuilding = true;
        simulation->rebuildSolverWithRotation();
        simulation->freezeFlow = false;
        simulation->updateVTKGeometry();
        simulation->updateStreamlineSeeds();
        simulation->isRebuilding = false;
        return;
    }

    if (simTimer) simTimer->stop();
    simulation->isRebuilding = true;

    QProgressDialog* progress = new QProgressDialog("Voxelizing Model and Starting Solver...", "", 0, 0, this);
    progress->setWindowModality(Qt::WindowModal);
    progress->setCancelButton(nullptr);
    progress->show();

    QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
    connect(watcher, &QFutureWatcher<void>::finished, [this, progress, watcher]() {
        progress->close();
        progress->deleteLater();
        watcher->deleteLater();
        simulation->freezeFlow = false;
        simulation->updateVTKGeometry();
        simulation->updateStreamlineSeeds();
        simulation->isRebuilding = false;
        if (simTimer) simTimer->start(22);
        if (auto vtkRenderWidget = qobject_cast<QVTKOpenGLNativeWidget*>(centralWidget())) {
            vtkRenderWidget->renderWindow()->Render();
        }
    });

    QFuture<void> future = QtConcurrent::run([this]() {
        simulation->rebuildSolverWithRotation();
    });
    watcher->setFuture(future);
}

void MainWindow::setupUi(const CLIOptions* opt) {
    QToolBar* topBar = new QToolBar("Settings", this);
    topBar->setMovable(false);
    addToolBar(Qt::TopToolBarArea, topBar);

    topBar->addWidget(new QLabel(" Shape: "));
    shapeSelector = new QComboBox(this);
    shapeSelector->addItems({"NACA Airfoil", "Cylinder (Test)", "Custom (Loaded)", "Empty Canvas"});
    topBar->addWidget(shapeSelector);
    
    topBar->addWidget(new QLabel(" Theme: "));
    colormapSelector = new QComboBox(this);
    colormapSelector->addItems({"Jet", "Wind Tunnel", "Neon", "Thermal"});
    topBar->addWidget(colormapSelector);

    topBar->addWidget(new QLabel(" Wind: "));
    windSelector = new QComboBox(this);
    windSelector->addItems({"Left -> Right", "Right -> Left", "Top -> Bottom", "Bottom -> Top"});
    topBar->addWidget(windSelector);

    topBar->addWidget(new QLabel(" Grid: "));
    gridSelector = new QComboBox(this);
    gridSelector->addItems({"64x32 (Fast)", "96x48 (Medium)", "128x64 (Default)", "160x80 (High)", "192x96 (Ultra)"});
    gridSelector->setCurrentIndex(2);
    topBar->addWidget(gridSelector);
    
    resetViewButton = new QPushButton(" ⟳ Reset View ", this);
    resetViewButton->setStyleSheet("QPushButton { font-weight: bold; color: white; background-color: #5cb85c; border-radius: 4px; padding: 4px; } QPushButton:hover { background-color: #449d44; }");
    topBar->addWidget(resetViewButton);

    topBar->addSeparator();

    topBar->addWidget(new QLabel(" Display: "));
    displayModeSelector = new QComboBox(this);
    displayModeSelector->addItems({"Streamlines", "3D Vortex Cores (Q-Crit)", "Streamlines + Vortex Cores"});
    topBar->addWidget(displayModeSelector);

    scoreLabel = new QLabel(" L/D: --- ", this);
    scoreLabel->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; color: #5cb85c; }");
    topBar->addWidget(scoreLabel);

    topBar->addSeparator();
    controlsButton = new QPushButton(" 🎮 Controls ", this);
    controlsButton->setStyleSheet("QPushButton { font-weight: bold; color: white; background-color: #337ab7; border-radius: 4px; padding: 4px 8px; } QPushButton:hover { background-color: #286090; }");
    topBar->addWidget(controlsButton);

    addToolBarBreak();

    toolbar = new QToolBar("Controls", this);
    toolbar->setMovable(false);
    addToolBar(Qt::TopToolBarArea, toolbar);

    toolbar->addWidget(new QLabel(" Alpha: "));
    alphaSlider = new QSlider(Qt::Horizontal);
    alphaSlider->setRange(-45, 45);
    alphaSlider->setValue(0);
    alphaSlider->setFixedWidth(80);
    toolbar->addWidget(alphaSlider);
    
    toolbar->addWidget(new QLabel(" Camber: "));
    camberSlider = new QSlider(Qt::Horizontal);
    camberSlider->setRange(0, 9);
    camberSlider->setValue(2);
    camberSlider->setFixedWidth(80);
    toolbar->addWidget(camberSlider);

    toolbar->addWidget(new QLabel(" Thick: "));
    thicknessSlider = new QSlider(Qt::Horizontal);
    thicknessSlider->setRange(5, 30);
    thicknessSlider->setValue(12);
    thicknessSlider->setFixedWidth(80);
    toolbar->addWidget(thicknessSlider);

    toolbar->addWidget(new QLabel(" Speed: "));
    speedSlider = new QSlider(Qt::Horizontal);
    speedSlider->setRange(1, 10);
    speedSlider->setValue(3);
    speedSlider->setFixedWidth(80);
    toolbar->addWidget(speedSlider);

    toolbar->addWidget(new QLabel(" Rake Y: "));
    rakeYSlider = new QSlider(Qt::Horizontal);
    rakeYSlider->setRange(-100, 100);
    rakeYSlider->setValue(0);
    rakeYSlider->setFixedWidth(80);
    toolbar->addWidget(rakeYSlider);

    toolbar->addWidget(new QLabel(" Lines: "));
    streamlineDensitySlider = new QSlider(Qt::Horizontal);
    streamlineDensitySlider->setRange(10, 1000);
    streamlineDensitySlider->setValue(100);
    streamlineDensitySlider->setFixedWidth(70);
    toolbar->addWidget(streamlineDensitySlider);

    qCritLabel = new QLabel(" Q-Crit: ");
    qCritLabelAction = toolbar->addWidget(qCritLabel);
    qCritSlider = new QSlider(Qt::Horizontal);
    qCritSlider->setRange(1, 100);
    qCritSlider->setValue(20);
    qCritSlider->setFixedWidth(70);
    qCritSlider->setToolTip("Q-Criterion Vortex Core Sensitivity Threshold");
    qCritSliderAction = toolbar->addWidget(qCritSlider);
    if (qCritLabelAction) qCritLabelAction->setVisible(false);
    if (qCritSliderAction) qCritSliderAction->setVisible(false);
    
    toolbar->addSeparator();

    particlesToggle = new QCheckBox(" Show Particles ", this);
    particlesToggle->setChecked(true);
    toolbar->addWidget(particlesToggle);
    
    flapToggle = new QCheckBox(" Flap Airfoil ", this);
    toolbar->addWidget(flapToggle);

    resetFlowButton = new QPushButton("Reset Flow", this);
    toolbar->addWidget(resetFlowButton);

    brushToolbar = new QToolBar("Drawing Mode", this);
    brushToolbar->setMovable(false);
    addToolBar(Qt::TopToolBarArea, brushToolbar);
    
    brushToolbar->addWidget(new QLabel(" 🎨 Draw Shape: "));
    brushShapeSelector = new QComboBox(this);
    brushShapeSelector->addItems({"Circle", "Square", "Diamond"});
    brushToolbar->addWidget(brushShapeSelector);
    
    brushToolbar->addWidget(new QLabel("  Brush Size: "));
    brushSizeSlider = new QSlider(Qt::Horizontal);
    brushSizeSlider->setRange(1, 100);
    brushSizeSlider->setValue(30);
    brushSizeSlider->setFixedWidth(100);
    brushToolbar->addWidget(brushSizeSlider);

    eraserButton = new QPushButton(" 🧹 Eraser ", this);
    eraserButton->setCheckable(true);
    eraserButton->setStyleSheet("QPushButton:checked { background-color: #f0ad4e; color: white; font-weight: bold; }");
    brushToolbar->addWidget(eraserButton);

    QPushButton* clearInBar = new QPushButton(" Clear Canvas ", this);
    brushToolbar->addWidget(clearInBar);
    connect(clearInBar, &QPushButton::clicked, this, [this]() {
        if (simulation) simulation->clearDrawing();
    });

    QWidget* spacer = new QWidget(this);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    brushToolbar->addWidget(spacer);

    doneDrawButton = new QPushButton(" ✔ Done ", this);
    doneDrawButton->setStyleSheet("QPushButton { font-weight: bold; color: white; background-color: #5cb85c; border-radius: 4px; padding: 4px 14px; font-size: 13px; } QPushButton:hover { background-color: #449d44; }");
    brushToolbar->addWidget(doneDrawButton);

    brushToolbar->setVisible(false);

    connect(shapeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateMorphing);

    connect(colormapSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) {
            simulation->setColormap(index);
        }
    });

    connect(windSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::setWindDirection);
    connect(gridSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::setGridPreset);

    connect(resetViewButton, &QPushButton::clicked, [this]() {
        if (simulation) {
            alphaSlider->setValue(0);
            rakeYSlider->setValue(0);
            simulation->resetCameraView();
            updateMorphing();
        }
    });

    connect(controlsButton, &QPushButton::clicked, this, &MainWindow::openControlsDialog);

    connect(alphaSlider, &QSlider::valueChanged, [this](int value) {
        simulation->flow.alpha = value;
        simulation->setVisualRotation(0, 0, -value);
    });
    connect(alphaSlider, &QSlider::sliderReleased, this, &MainWindow::updateMorphing);
    connect(camberSlider, &QSlider::valueChanged, [this](int value) {
        simulation->setVisualRotation(0, 0, 0);
    });
    connect(camberSlider, &QSlider::sliderReleased, this, &MainWindow::updateMorphing);
    connect(thicknessSlider, &QSlider::valueChanged, [this](int value) {
        simulation->setVisualRotation(0, 0, 0);
    });
    connect(thicknessSlider, &QSlider::sliderReleased, this, &MainWindow::updateMorphing);

    connect(speedSlider, &QSlider::valueChanged, [this](int value) {
        simulation->flow.V_inf = static_cast<double>(value) * 5.0;
        simulation->stepsPerFrame = std::clamp(value, 1, 8);
        simulation->needsVTKUpdate = true;
        simulation->resetFlow();
    });
    
    connect(rakeYSlider, &QSlider::valueChanged, [this](int value) {
        simulation->setRakePosition(value / 100.0f);
    });
    
    connect(streamlineDensitySlider, &QSlider::sliderReleased, [this]() {
        if (simulation) {
            simulation->setStreamlineDensity(streamlineDensitySlider->value());
        }
    });

    connect(particlesToggle, &QCheckBox::toggled, this, [this](bool checked) {
        if (simulation) {
            simulation->setShowParticles(checked);
        }
    });
    
    connect(resetFlowButton, &QPushButton::clicked, [this]() {
        if (simulation) {
            simulation->resetFlow();
        }
    });
    
    connect(displayModeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) {
            if (index == 0) {
                simulation->setShowParticles(true);
                simulation->showHeatmap = false;
                toggleQCriterion(false);
                particlesToggle->setChecked(true);
            } else if (index == 1) {
                simulation->setShowParticles(false);
                simulation->showHeatmap = false;
                toggleQCriterion(true);
                particlesToggle->setChecked(false);
            } else if (index == 2) {
                simulation->setShowParticles(true);
                simulation->showHeatmap = false;
                toggleQCriterion(true);
                particlesToggle->setChecked(true);
            }
        }
    });

    connect(qCritSlider, &QSlider::valueChanged, this, [this](int value) {
        if (simulation) {
            double thresh = 1.0e-5 * std::pow(50.0, (double)(value - 1) / 99.0);
            simulation->setQCriterionThreshold(thresh);
            qCritSlider->setToolTip(QString("Q-Criterion Sensitivity: %1% (Threshold: %2 s^-2)")
                .arg(value)
                .arg(thresh, 0, 'e', 2));
            if (qCritResultAction) {
                qCritResultAction->setText(QString("Q-Crit Threshold: %1 s^-2 (Sensitivity: %2%)")
                    .arg(thresh, 0, 'e', 2)
                    .arg(value));
            }
        }
    });
    
    connect(eraserButton, &QPushButton::toggled, [this](bool checked) {
        if (simulation) simulation->isEraser = checked;
    });

    connect(doneDrawButton, &QPushButton::clicked, this, [this]() {
        setDrawingMode(false);
    });
    
    connect(flapToggle, &QCheckBox::toggled, [this](bool checked) {
        if (simulation) {
            simulation->flapping = checked;
            if (!checked) {
                simulation->fastUpdateRotation(simulation->flow.alpha);
            }
        }
    });

    connect(brushSizeSlider, &QSlider::valueChanged, [this](int value) {
        if (simulation) simulation->brushSize = static_cast<float>(value) / 10.0f;
    });

    connect(brushShapeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) simulation->brushShape = index;
    });

    if (opt) {
        if (opt->alphaSet) alphaSlider->setValue(static_cast<int>(std::round(opt->alpha)));
        if (opt->camberSet) camberSlider->setValue(static_cast<int>(std::round(opt->camber * 100.0)));
        if (opt->thicknessSet) thicknessSlider->setValue(static_cast<int>(std::round(opt->thickness * 100.0)));
        if (opt->speedSet) speedSlider->setValue(static_cast<int>(std::clamp(std::round(opt->speed / 5.0), 1.0, 10.0)));
        if (opt->linesSet) streamlineDensitySlider->setValue(opt->lines);
        if (opt->rakeYSet) rakeYSlider->setValue(static_cast<int>(std::round(opt->rakeY * 100.0)));
        if (opt->shapeSet) shapeSelector->setCurrentIndex(opt->shape);
        if (opt->modelSet && !opt->modelFile.empty()) {
            simulation->foil.loadFromFile(opt->modelFile);
            shapeSelector->blockSignals(true);
            shapeSelector->setCurrentIndex(2);
            shapeSelector->blockSignals(false);
        }
    }

    updateMorphing();
}

void MainWindow::openControlsDialog() {
    ControlsDialog dlg(this);
    connect(&dlg, &ControlsDialog::bindingsChanged, this, &MainWindow::applyKeyBindings);
    dlg.exec();
}

void MainWindow::openPolarSweepDialog() {
    PolarDialog dlg(simulation.get(), this);
    dlg.exec();
}

void MainWindow::applyKeyBindings(const QMap<QString, QKeySequence>& bindings) {
    keyBindings = bindings;
    setupShortcuts();
}

void MainWindow::setupShortcuts() {
    qDeleteAll(activeShortcuts);
    activeShortcuts.clear();

    keyBindings = ControlsDialog::loadBindings();
    for (auto it = keyBindings.begin(); it != keyBindings.end(); ++it) {
        if (it.value().isEmpty()) continue;
        QString actionId = it.key();
        QShortcut* shortcut = new QShortcut(it.value(), this, nullptr, nullptr, Qt::ApplicationShortcut);
        connect(shortcut, &QShortcut::activated, this, [this, actionId]() {
            handleKeyAction(actionId);
        });
        activeShortcuts.append(shortcut);
    }
}

void MainWindow::handleKeyAction(const QString& actionId) {
    if (actionId == "pitch_up") {
        alphaSlider->setValue(std::min(alphaSlider->maximum(), alphaSlider->value() + 1));
        updateMorphing();
    } else if (actionId == "pitch_down") {
        alphaSlider->setValue(std::max(alphaSlider->minimum(), alphaSlider->value() - 1));
        updateMorphing();
    } else if (actionId == "camber_up") {
        camberSlider->setValue(std::min(camberSlider->maximum(), camberSlider->value() + 1));
        updateMorphing();
    } else if (actionId == "camber_down") {
        camberSlider->setValue(std::max(camberSlider->minimum(), camberSlider->value() - 1));
        updateMorphing();
    } else if (actionId == "thickness_up") {
        thicknessSlider->setValue(std::min(thicknessSlider->maximum(), thicknessSlider->value() + 1));
        updateMorphing();
    } else if (actionId == "thickness_down") {
        thicknessSlider->setValue(std::max(thicknessSlider->minimum(), thicknessSlider->value() - 1));
        updateMorphing();
    } else    if (actionId == "toggle_flap") {
        if (flapToggle->isEnabled()) {
            flapToggle->setChecked(!flapToggle->isChecked());
        }
    } else if (actionId == "speed_up") {
        speedSlider->setValue(std::min(speedSlider->maximum(), speedSlider->value() + 1));
        if (simulation) simulation->resetFlow();
    } else if (actionId == "speed_down") {
        speedSlider->setValue(std::max(speedSlider->minimum(), speedSlider->value() - 1));
        if (simulation) simulation->resetFlow();
    } else if (actionId == "rake_up") {
        rakeYSlider->setValue(std::min(rakeYSlider->maximum(), rakeYSlider->value() + 5));
    } else if (actionId == "rake_down") {
        rakeYSlider->setValue(std::max(rakeYSlider->minimum(), rakeYSlider->value() - 5));
    } else if (actionId == "lines_inc") {
        streamlineDensitySlider->setValue(std::min(streamlineDensitySlider->maximum(), streamlineDensitySlider->value() + 50));
    } else if (actionId == "lines_dec") {
        streamlineDensitySlider->setValue(std::max(streamlineDensitySlider->minimum(), streamlineDensitySlider->value() - 50));
    } else if (actionId == "turbo_mode") {
        if (simulation) {
            simulation->resetCameraView();
            simulation->resetFlow();
        }
    } else if (actionId == "reset_flow") {
        resetFlowButton->click();
    } else if (actionId == "rotate_up") {
        if (simulation) simulation->rotateCamera(0.0, 5.0);
    } else if (actionId == "rotate_down") {
        if (simulation) simulation->rotateCamera(0.0, -5.0);
    } else if (actionId == "rotate_left") {
        if (simulation) simulation->rotateCamera(5.0, 0.0);
    } else if (actionId == "rotate_right") {
        if (simulation) simulation->rotateCamera(-5.0, 0.0);
    } else if (actionId == "pan_up") {
        if (simulation) simulation->panCamera(0.0, 2.0);
    } else if (actionId == "pan_down") {
        if (simulation) simulation->panCamera(0.0, -2.0);
    } else if (actionId == "pan_left") {
        if (simulation) simulation->panCamera(-2.0, 0.0);
    } else if (actionId == "pan_right") {
        if (simulation) simulation->panCamera(2.0, 0.0);
    } else if (actionId == "zoom_in") {
        if (simulation) simulation->zoomCamera(1.15);
    } else if (actionId == "zoom_out") {
        if (simulation) simulation->zoomCamera(0.85);
    } else if (actionId == "reset_camera") {
        if (simulation) simulation->resetCameraView();
    } else if (actionId == "draw_mode") {
        if (simulation) setDrawingMode(!simulation->drawMode);
    } else if (actionId == "clear_draw") {
        if (simulation) simulation->clearDrawing();
    } else if (actionId == "cycle_theme") {
        colormapSelector->setCurrentIndex((colormapSelector->currentIndex() + 1) % colormapSelector->count());
    } else if (actionId == "cycle_display") {
        displayModeSelector->setCurrentIndex((displayModeSelector->currentIndex() + 1) % displayModeSelector->count());
    } else if (actionId == "toggle_particles") {
        particlesToggle->setChecked(!particlesToggle->isChecked());
    }
}

void MainWindow::updateSimulation() {
    if (!simulation || simulation->isRebuilding) return;

    if (simulation->flapping) {
        simulation->flapTimer += 0.05;
        double flapAngleDeg = 10.0 * std::sin(simulation->flapTimer);
        simulation->fastUpdateRotation(simulation->flow.alpha + flapAngleDeg);
    }
    
    simulation->stepSimulation();
    
    if (simulation->lbmSolver) {
        const auto& grid = simulation->lbmSolver->getGrid();
        double drag = (double)grid.force_x;
        double lift = (double)grid.force_y;
        
        double alphaEMA = 0.01; 
        static double ema_cl = 0.0;
        static double ema_cd = 0.0;
        static double ema_lift = 0.0;
        static double ema_drag = 0.0;
        
        double scale = (double)simulation->cachedLbmScale;
        double a_ref_lattice = 1.0;
        if (simulation->foil.is3D()) {
            a_ref_lattice = std::max(50.0, ((double)simulation->cowWidth * scale) * (std::max(0.2, (double)simulation->cowHeight) * scale));
        } else {
            a_ref_lattice = std::max(50.0, (1.0 * scale) * (double)simulation->config.lbmGridNZ);
        }
        
        double f_dyn_lattice = 0.00125 * a_ref_lattice;
        
        double raw_cd = std::abs(drag) / f_dyn_lattice;
        double raw_cl = lift / f_dyn_lattice;
        
        ema_cd = alphaEMA * raw_cd + (1.0 - alphaEMA) * ema_cd;
        ema_cl = alphaEMA * raw_cl + (1.0 - alphaEMA) * ema_cl;
        
        double q_inf = 0.5 * 1.225 * simulation->flow.V_inf * simulation->flow.V_inf;
        double ref_area_m2 = 1.0;
        double lift_N = ema_cl * q_inf * ref_area_m2;
        double drag_N = ema_cd * q_inf * ref_area_m2;
        
        ema_drag = alphaEMA * drag_N + (1.0 - alphaEMA) * ema_drag;
        ema_lift = alphaEMA * lift_N + (1.0 - alphaEMA) * ema_lift;
        
        ema_ld = (std::abs(ema_cd) > 1e-6) ? (ema_cl / ema_cd) : 0.0;
        
        static int scoreCounter = 0;
        if (++scoreCounter % 6 == 0) {
            scoreLabel->setText(QString(" L/D: %1 ").arg(ema_ld, 0, 'f', 2));
            if (ldAction) ldAction->setText(QString("L/D Ratio: %1").arg(ema_ld, 0, 'f', 2));
            if (clAction) clAction->setText(QString("Lift Coefficient (CL): %1").arg(ema_cl, 0, 'f', 3));
            if (cdAction) cdAction->setText(QString("Drag Coefficient (CD): %1").arg(ema_cd, 0, 'f', 3));
            if (liftAction) liftAction->setText(QString("Lift Force: %1 N").arg(ema_lift, 0, 'f', 2));
            if (dragAction) dragAction->setText(QString("Drag Force: %1 N").arg(ema_drag, 0, 'f', 2));
            if (alphaAction) alphaAction->setText(QString("Angle of Attack: %1°").arg(simulation->flow.alpha, 0, 'f', 1));
            if (speedAction) speedAction->setText(QString("Airspeed: %1 m/s").arg(simulation->flow.V_inf, 0, 'f', 1));
            if (qCritResultAction && qCritSlider) {
                qCritResultAction->setText(QString("Q-Crit Threshold: %1 s^-2 (Sensitivity: %2%)")
                    .arg(simulation->qCritThreshold, 0, 'e', 2)
                    .arg(qCritSlider->value()));
            }
        }
    }

    if (auto vtkRenderWidget = qobject_cast<QVTKOpenGLNativeWidget*>(centralWidget())) {
        vtkRenderWidget->renderWindow()->Render();
    }
}

void MainWindow::setDrawingMode(bool enabled) {
    if (simulation) {
        simulation->drawMode = enabled;
        if (!enabled) {
            simulation->isEraser = false;
            if (eraserButton) eraserButton->setChecked(false);
        }
    }
    if (toolbar) toolbar->setVisible(!enabled);
    if (brushToolbar) brushToolbar->setVisible(enabled);
    if (drawModeAction) {
        drawModeAction->setText(enabled ? "&Exit Drawing Mode" : "&Start Drawing Mode");
    }
}

void MainWindow::openCpDialog() {
    auto* dlg = new CpDialog(simulation.get(), this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->show();
}

void MainWindow::toggleSurfaceCp(bool checked) {
    if (simulation) {
        simulation->setSurfaceCpVisible(checked);
    }
}

void MainWindow::toggleQCriterion(bool checked) {
    if (simulation) {
        simulation->setQCriterionVisible(checked);
    }
    if (qCritAction && qCritAction->isChecked() != checked) {
        qCritAction->blockSignals(true);
        qCritAction->setChecked(checked);
        qCritAction->blockSignals(false);
    }
    if (qCritLabelAction) qCritLabelAction->setVisible(checked);
    if (qCritSliderAction) qCritSliderAction->setVisible(checked);
}

void MainWindow::setWindDirection(int dir) {
    if (simulation) {
        simulation->setWindDirection(dir);
    }
    if (windSelector && windSelector->currentIndex() != dir) {
        windSelector->blockSignals(true);
        windSelector->setCurrentIndex(dir);
        windSelector->blockSignals(false);
    }
    if (windActionGroup) {
        auto actions = windActionGroup->actions();
        if (dir >= 0 && dir < actions.size() && !actions[dir]->isChecked()) {
            actions[dir]->blockSignals(true);
            actions[dir]->setChecked(true);
            actions[dir]->blockSignals(false);
        }
    }
}

void MainWindow::setGridPreset(int index) {
    if (!simulation) return;
    int nx = 128, ny = 64, nz = 64;
    switch (index) {
        case 0: nx = 64;  ny = 32; nz = 32; break;
        case 1: nx = 96;  ny = 48; nz = 48; break;
        case 2: nx = 128; ny = 64; nz = 64; break;
        case 3: nx = 160; ny = 80; nz = 80; break;
        case 4: nx = 192; ny = 96; nz = 96; break;
        default: return;
    }
    simulation->setGridResolution(nx, ny, nz);

    if (gridSelector && gridSelector->currentIndex() != index) {
        gridSelector->blockSignals(true);
        gridSelector->setCurrentIndex(index);
        gridSelector->blockSignals(false);
    }
    if (gridActionGroup) {
        auto acts = gridActionGroup->actions();
        if (index >= 0 && index < acts.size() && !acts[index]->isChecked()) {
            acts[index]->blockSignals(true);
            acts[index]->setChecked(true);
            acts[index]->blockSignals(false);
        }
    }
    updateMorphing();
}

void MainWindow::openCustomGridDialog() {
    if (!simulation) return;
    QDialog dlg(this);
    dlg.setWindowTitle("Custom LBM Grid Resolution");
    dlg.setWindowFlags(dlg.windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    dlg.setMinimumWidth(320);

    auto layout = new QVBoxLayout(&dlg);
    auto form = new QFormLayout();

    auto nxSpin = new QSpinBox(&dlg);
    nxSpin->setRange(16, 512);
    nxSpin->setSingleStep(16);
    nxSpin->setValue(simulation->config.lbmGridNX);

    auto nySpin = new QSpinBox(&dlg);
    nySpin->setRange(16, 256);
    nySpin->setSingleStep(16);
    nySpin->setValue(simulation->config.lbmGridNY);

    auto nzSpin = new QSpinBox(&dlg);
    nzSpin->setRange(16, 256);
    nzSpin->setSingleStep(16);
    nzSpin->setValue(simulation->config.lbmGridNZ);

    form->addRow("Grid NX (Length):", nxSpin);
    form->addRow("Grid NY (Height):", nySpin);
    form->addRow("Grid NZ (Depth):", nzSpin);

    layout->addLayout(form);

    auto btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    layout->addWidget(btnBox);
    connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        int nx = nxSpin->value();
        int ny = nySpin->value();
        int nz = nzSpin->value();
        simulation->setGridResolution(nx, ny, nz);
        updateMorphing();
    }
}

}
