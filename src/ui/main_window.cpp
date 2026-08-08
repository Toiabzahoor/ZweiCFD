#include "ZweiCFD/ui/main_window.hpp"

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

namespace zweicfd {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
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
    simTimer->start(16);

    QMenuBar* menuBar = new QMenuBar(this);
    this->setMenuBar(menuBar);
    
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

    QMenu* fileMenu = menuBar->addMenu("&File");
    QAction* loadAction = fileMenu->addAction("&Load Profile...");
    connect(loadAction, &QAction::triggered, this, [this]() {
        QString fileName = QFileDialog::getOpenFileName(this, "Open Profile", "", "Data Files (*.dat);;All Files (*.*)");
        if (!fileName.isEmpty()) {
            if (simulation->foil.loadFromFile(fileName.toStdString())) {
                shapeSelector->blockSignals(true);
                shapeSelector->setCurrentIndex(2);
                shapeSelector->blockSignals(false);
                
                simulation->rebuildSolverWithRotation();
                simulation->freezeFlow = false;
                simulation->updateVTKGeometry();
            }
        }
    });
    fileMenu->addSeparator();
    fileMenu->addAction("E&xit", this, &QWidget::close);

    QMenu* drawMenu = menuBar->addMenu("&Draw");
    QAction* clearAction = drawMenu->addAction("&Clear Canvas");
    connect(clearAction, &QAction::triggered, this, [this]() {
        if (simulation) simulation->clearDrawing();
    });

    setupUi();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUi() {
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
    
    turboButton = new QPushButton(" TURBO SPEED ", this);
    turboButton->setStyleSheet("QPushButton { font-weight: bold; color: white; background-color: #d9534f; border-radius: 4px; padding: 4px; } QPushButton:pressed { background-color: #c9302c; }");
    topBar->addWidget(turboButton);

    topBar->addSeparator();

    topBar->addWidget(new QLabel(" Display: "));
    displayModeSelector = new QComboBox(this);
    displayModeSelector->addItems({"Streamlines", "Velocity Heatmap"});
    topBar->addWidget(displayModeSelector);

    scoreLabel = new QLabel(" L/D: --- ", this);
    scoreLabel->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; color: #5cb85c; }");
    topBar->addWidget(scoreLabel);

    addToolBarBreak();

    QToolBar* toolbar = new QToolBar("Controls", this);
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
    speedSlider->setRange(1, 20);
    speedSlider->setValue(5);
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
    streamlineDensitySlider->setFixedWidth(80);
    toolbar->addWidget(streamlineDensitySlider);
    
    toolbar->addSeparator();

    particlesToggle = new QCheckBox(" Show Particles ", this);
    particlesToggle->setChecked(true);
    toolbar->addWidget(particlesToggle);
    
    flapToggle = new QCheckBox(" Flap Airfoil ", this);
    toolbar->addWidget(flapToggle);

    resetFlowButton = new QPushButton("Reset Flow", this);
    toolbar->addWidget(resetFlowButton);

    addToolBarBreak();

    QToolBar* brushToolbar = new QToolBar("Brush Settings", this);
    brushToolbar->setMovable(false);
    addToolBar(Qt::TopToolBarArea, brushToolbar);
    
    drawModeToggle = new QCheckBox(" Draw Mode ", this);
    brushToolbar->addWidget(drawModeToggle);
    
    brushToolbar->addWidget(new QLabel("  Brush Shape: "));
    brushShapeSelector = new QComboBox(this);
    brushShapeSelector->addItems({"Circle", "Square", "Diamond"});
    brushToolbar->addWidget(brushShapeSelector);
    
    brushToolbar->addWidget(new QLabel("  Brush Size: "));
    brushSizeSlider = new QSlider(Qt::Horizontal);
    brushSizeSlider->setRange(1, 100);
    brushSizeSlider->setValue(30);
    brushSizeSlider->setFixedWidth(100);
    brushToolbar->addWidget(brushSizeSlider);

    auto updateMorphing = [this]() {
        int shapeIdx = shapeSelector->currentIndex();
        
        bool isNACA = (shapeIdx == 0);
        camberSlider->setEnabled(isNACA);
        thicknessSlider->setEnabled(isNACA);

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
        simulation->rebuildSolverWithRotation();
        simulation->freezeFlow = false;
        simulation->updateVTKGeometry();
    };

    connect(shapeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), updateMorphing);

    connect(colormapSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) {
            simulation->setColormap(index);
        }
    });

    connect(turboButton, &QPushButton::pressed, [this]() {
        isTurboMode = true;
        if (simulation) {
            simulation->flow.V_inf = 0.3;
            simulation->setStreamlineDensity(1000);
            simulation->setColormap(2);
            simulation->resetFlow();
            colormapSelector->setCurrentIndex(2);
        }
    });
    
    connect(turboButton, &QPushButton::released, [this]() {
        isTurboMode = false;
        if (simulation) {
            simulation->flow.V_inf = speedSlider->value() / 100.0;
            simulation->setStreamlineDensity(streamlineDensitySlider->value());
            simulation->setColormap(colormapSelector->currentIndex());
            simulation->resetFlow();
        }
    });

    connect(alphaSlider, &QSlider::valueChanged, [this](int value) {
        simulation->flow.alpha = value;
    });
    connect(alphaSlider, &QSlider::sliderReleased, updateMorphing);
    connect(camberSlider, &QSlider::sliderReleased, updateMorphing);
    connect(thicknessSlider, &QSlider::sliderReleased, updateMorphing);

    connect(speedSlider, &QSlider::valueChanged, [this](int value) {
        simulation->flow.V_inf = value / 100.0;
    });
    connect(speedSlider, &QSlider::sliderReleased, [this]() {
        simulation->resetFlow();
    });
    
    connect(rakeYSlider, &QSlider::valueChanged, [this](int value) {
        simulation->setRakePosition(value / 100.0f);
    });
    
    connect(streamlineDensitySlider, &QSlider::valueChanged, [this](int value) {
        if (simulation) {
            simulation->setStreamlineDensity(value);
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
    
    connect(drawModeToggle, &QCheckBox::toggled, [this](bool checked) {
        if (simulation) simulation->drawMode = checked;
    });
    
    connect(flapToggle, &QCheckBox::toggled, [this](bool checked) {
        if (simulation) {
            simulation->flapping = checked;
            if (!checked) {
                simulation->setVisualRotation(0.0);
            }
        }
    });

    connect(brushSizeSlider, &QSlider::valueChanged, [this](int value) {
        if (simulation) simulation->brushSize = static_cast<float>(value) / 10.0f;
    });

    connect(brushShapeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (simulation) simulation->brushShape = index;
    });

    updateMorphing();
}

void MainWindow::updateSimulation() {
    if (simulation->flapping) {
        simulation->flapTimer += 0.05;
        double flapAngleDeg = 10.0 * std::sin(simulation->flapTimer);
        simulation->setVisualRotation(flapAngleDeg);
    }
    
    simulation->stepSimulation();
    
    if (simulation->lbmSolver) {
        const auto& grid = simulation->lbmSolver->getGrid();
        double drag = grid.force_x;
        double lift = grid.force_y;
        
        double raw_ld = (std::abs(drag) > 1e-6) ? (lift / drag) : 0.0;
        
        double alphaEMA = 0.05; 
        ema_ld = alphaEMA * raw_ld + (1.0 - alphaEMA) * ema_ld;
        
        scoreLabel->setText(QString(" L/D: %1 ").arg(ema_ld, 0, 'f', 2));
    }

    if (auto vtkRenderWidget = qobject_cast<QVTKOpenGLNativeWidget*>(centralWidget())) {
        vtkRenderWidget->renderWindow()->Render();
    }
}

}
