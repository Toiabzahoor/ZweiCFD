#pragma once

#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QProgressBar>
#include <QTableWidget>
#include <QLabel>
#include <QThread>
#include <vector>
#include <string>
#include "ZweiCFD/core/simulation.hpp"

namespace zweicfd {

struct PolarPointResult {
    double alpha;
    double cl;
    double cd;
    double ld;
    double lift_N;
    double drag_N;
    double fx;
    double fy;
};

class PolarWorker : public QObject {
    Q_OBJECT
public:
    PolarWorker(Simulation* sim, double aMin, double aMax, double aStep, int steps, int warmup, bool resetFlow, const QString& csvPath);

public slots:
    void process();
    void cancel();

signals:
    void progressChanged(int percent, QString message);
    void pointCompleted(PolarPointResult res);
    void finished(QString summary, QString csvPath);
    void errorOccurred(QString message);

private:
    Simulation* simulation;
    double alphaMin;
    double alphaMax;
    double alphaStep;
    int stepsPerPoint;
    int warmupSteps;
    bool resetFlowPerPoint;
    QString csvFilePath;
    std::atomic<bool> isCancelled{false};
};

class PlotWidget;

class PolarDialog : public QDialog {
    Q_OBJECT
public:
    explicit PolarDialog(Simulation* sim, QWidget* parent = nullptr);
    ~PolarDialog() override;

private slots:
    void onBrowseCsv();
    void onStartSweep();
    void onCancelSweep();
    void onPointCompleted(PolarPointResult res);
    void onProgressChanged(int percent, QString message);
    void onSweepFinished(QString summary, QString csvPath);
    void onSweepError(QString message);
    void onPlotModeChanged(int index);
    void onExportPlot();
    void onCopyPlot();

private:
    void setupUi();

    Simulation* simulation;
    QDoubleSpinBox* alphaMinSpin;
    QDoubleSpinBox* alphaMaxSpin;
    QDoubleSpinBox* alphaStepSpin;
    QSpinBox* stepsSpin;
    QSpinBox* warmupSpin;
    QCheckBox* resetFlowCheck;
    QLineEdit* csvPathEdit;
    QPushButton* browseButton;

    QPushButton* startButton;
    QPushButton* cancelButton;
    QPushButton* closeButton;

    QProgressBar* progressBar;
    QLabel* statusLabel;
    QTableWidget* resultsTable;
    QLabel* summaryLabel;

    PlotWidget* plotWidget = nullptr;
    QPushButton* exportPlotButton = nullptr;
    QPushButton* copyPlotButton = nullptr;

    QThread* workerThread = nullptr;
    PolarWorker* worker = nullptr;
};

}
