#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QSlider>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <memory>

#include "ZweiCFD/core/simulation.hpp"

namespace zweicfd {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void updateSimulation();

private:
    void setupUi();

    std::unique_ptr<Simulation> simulation;
    QComboBox* shapeSelector;
    QComboBox* colormapSelector;
    QPushButton* turboButton;
    QLabel* scoreLabel;
    bool isTurboMode = false;
    double ema_ld = 0.0;
    
    QCheckBox* drawModeToggle;
    QCheckBox* flapToggle;
    QComboBox* displayModeSelector;
    QSlider* alphaSlider;
    QSlider* rakeYSlider;
    QSlider* camberSlider;
    QSlider* thicknessSlider;
    QSlider* speedSlider;
    QSlider* streamlineDensitySlider;
    QSlider* brushSizeSlider;
    QComboBox* brushShapeSelector;
    QCheckBox* particlesToggle;
    QPushButton* resetFlowButton;
    QTimer *simTimer;
};

} 
