#include "ZweiCFD/ui/main_window.hpp"
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
#include <QDialog>
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
                shapeSelector->blockSignals(true);
                shapeSelector->setCurrentIndex(2);
                shapeSelector->blockSignals(false);
                
                if (simulation->foil.is3D()) {
                    simulation->freezeFlow = true;
                    simulation->rotatedFoil = simulation->foil;
                    simulation->updateVTKGeometry();
                    
                    QDialog* dlg = new QDialog(this);
                    dlg->setWindowTitle("Set Initial Model Orientation");
                    dlg->setAttribute(Qt::WA_DeleteOnClose);
                    dlg->setModal(false);
                    QVBoxLayout* layout = new QVBoxLayout(dlg);
                    
                    QLabel* infoLabel = new QLabel("Adjust initial model orientation:");
                    layout->addWidget(infoLabel);
                    
                    QSlider* pitchSlider = new QSlider(Qt::Horizontal);
                    pitchSlider->setRange(-180, 180);
                    QSlider* yawSlider = new QSlider(Qt::Horizontal);
                    yawSlider->setRange(-180, 180);
                    QSlider* rollSlider = new QSlider(Qt::Horizontal);
                    rollSlider->setRange(-180, 180);
                    
                    QLabel* pLabel = new QLabel("Pitch (X): 0");
                    QLabel* yLabel = new QLabel("Yaw (Y): 0");
                    QLabel* rLabel = new QLabel("Roll (Z): 0");
                    
                    auto updateRotation = [this, pitchSlider, yawSlider, rollSlider, pLabel, yLabel, rLabel]() {
                        pLabel->setText(QString("Pitch (X): %1°").arg(pitchSlider->value()));
                        yLabel->setText(QString("Yaw (Y): %1°").arg(yawSlider->value()));
                        rLabel->setText(QString("Roll (Z): %1°").arg(rollSlider->value()));
                        simulation->foil.setBaseRotation(pitchSlider->value(), yawSlider->value(), rollSlider->value());
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
                    
                    QPushButton* okBtn = new QPushButton("Apply & Close");
                    connect(okBtn, &QPushButton::clicked, dlg, &QDialog::accept);
                    connect(dlg, &QDialog::finished, this, [this]() {
                        updateMorphing();
                        simulation->freezeFlow = false;
                    });
                    layout->addWidget(okBtn);
                    
                    dlg->show();
                    dlg->move(this->x() + 50, this->y() + 100);
                } else {
                    simulation->rebuildSolverWithRotation();
                    simulation->freezeFlow = false;
                    simulation->updateVTKGeometry();
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
    } else if (shapeIdx == 3) {
        simulation->foil.generateCylinder(0.001, 3);
    }

    if (!this->isVisible()) {
        simulation->isRebuilding = true;
        simulation->rebuildSolverWithRotation();
        simulation->freezeFlow = false;
        simulation->updateVTKGeometry();
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
    
    resetViewButton = new QPushButton(" ⟳ Reset View ", this);
    resetViewButton->setStyleSheet("QPushButton { font-weight: bold; color: white; background-color: #5cb85c; border-radius: 4px; padding: 4px; } QPushButton:hover { background-color: #449d44; }");
    topBar->addWidget(resetViewButton);

    topBar->addSeparator();

    topBar->addWidget(new QLabel(" Display: "));
    displayModeSelector = new QComboBox(this);
    displayModeSelector->addItems({"Streamlines", "Velocity Heatmap"});
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

    connect(resetViewButton, &QPushButton::clicked, [this]() {
        if (simulation) {
            alphaSlider->setValue(0);
            rakeYSlider->setValue(0);
            updateMorphing();
            simulation->resetCameraView();
            simulation->resetFlow();
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
    });
    connect(speedSlider, &QSlider::sliderReleased, [this]() {
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
            simulation->showParticles = checked;
        }
    });
    
    connect(resetFlowButton, &QPushButton::clicked, [this]() {
        if (simulation) {
            simulation->resetFlow();
        }
    });
    
    connect(displayModeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) {
            simulation->showParticles = (index == 0);
            simulation->showHeatmap = (index == 1);
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

}
