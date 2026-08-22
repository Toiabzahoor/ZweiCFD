#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QSlider>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <memory>

#include <QShortcut>
#include <QMap>
#include <QVector>
#include "ZweiCFD/core/simulation.hpp"
#include "ZweiCFD/ui/controls_dialog.hpp"

namespace zweicfd {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void updateSimulation();
    void openControlsDialog();
    void applyKeyBindings(const QMap<QString, QKeySequence>& bindings);

private:
    void setupUi();
    void setupShortcuts();
    void handleKeyAction(const QString& actionId);
    void updateMorphing();
    void setDrawingMode(bool enabled);

    std::unique_ptr<Simulation> simulation;
    QComboBox* shapeSelector;
    QComboBox* colormapSelector;
    QPushButton* resetViewButton;
    QLabel* scoreLabel;
    double ema_ld = 0.0;
    
    QToolBar* toolbar;
    QToolBar* brushToolbar;
    QAction* drawModeAction;
    QPushButton* eraserButton;
    QPushButton* doneDrawButton;
    QCheckBox* flapToggle;
    QComboBox* displayModeSelector;
    QSlider* alphaSlider;
    QSlider* rakeYSlider;
    QSlider* camberSlider;
    QSlider* thicknessSlider;
    QSlider* speedSlider;
    QSlider* streamlineDensitySlider;
    QSlider* lineWidthSlider;
    QSlider* brushSizeSlider;
    QComboBox* brushShapeSelector;
    QCheckBox* particlesToggle;
    QPushButton* resetFlowButton;
    QPushButton* controlsButton;
    QTimer *simTimer;

    QMap<QString, QKeySequence> keyBindings;
    QVector<QShortcut*> activeShortcuts;
};

} 
