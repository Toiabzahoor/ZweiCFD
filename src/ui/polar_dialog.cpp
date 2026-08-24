#include "ZweiCFD/ui/polar_dialog.hpp"
#include "ZweiCFD/solver/lbm_solver.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace zweicfd {

PolarWorker::PolarWorker(Simulation* sim, double aMin, double aMax, double aStep, int steps, int warmup, bool resetFlow, const QString& csvPath)
    : simulation(sim), alphaMin(aMin), alphaMax(aMax), alphaStep(aStep),
      stepsPerPoint(steps), warmupSteps(warmup), resetFlowPerPoint(resetFlow),
      csvFilePath(csvPath) {}

void PolarWorker::cancel() {
    isCancelled.store(true);
}

void PolarWorker::process() {
    if (!simulation) {
        emit errorOccurred("Simulation handle is null.");
        return;
    }

    Simulation workerSim(0, nullptr);
    workerSim.config = simulation->config;
    workerSim.foil = simulation->foil;
    workerSim.flow = simulation->flow;
    workerSim.flow.alpha = alphaMin;
    workerSim.cachedLbmScale = simulation->cachedLbmScale;
    workerSim.cowWidth = simulation->cowWidth;
    workerSim.cowHeight = simulation->cowHeight;
    workerSim.rebuildSolverWithRotation();

    if (!workerSim.lbmSolver) {
        emit errorOccurred("Failed to initialize LBM solver for polar sweep.");
        return;
    }

    double scale = (double)workerSim.cachedLbmScale;
    double a_ref_lattice = 1.0;
    if (workerSim.foil.is3D()) {
        a_ref_lattice = std::max(50.0, ((double)workerSim.cowWidth * scale) * (std::max(0.2, (double)workerSim.cowHeight) * scale));
    } else {
        a_ref_lattice = std::max(50.0, (1.0 * scale) * (double)workerSim.config.lbmGridNZ);
    }
    double f_dyn_lattice = 0.00125 * a_ref_lattice;
    double q_inf = 0.5 * 1.225 * workerSim.flow.V_inf * workerSim.flow.V_inf;
    double ref_area_m2 = 1.0;

    int steps = std::max(10, stepsPerPoint);
    int warmup = std::min(warmupSteps, steps - 1);

    std::vector<double> angles;
    double stepSize = (std::abs(alphaStep) > 1e-4) ? std::abs(alphaStep) : 1.0;
    if (alphaMin <= alphaMax) {
        for (double a = alphaMin; a <= alphaMax + 1e-6; a += stepSize) {
            angles.push_back(a);
        }
    } else {
        for (double a = alphaMin; a >= alphaMax - 1e-6; a -= stepSize) {
            angles.push_back(a);
        }
    }
    if (angles.empty()) angles.push_back(alphaMin);

    std::vector<PolarPointResult> results;
    Flowconditions safeLBM = workerSim.flow;
    safeLBM.V_inf = workerSim.flow.V_inf;
    safeLBM.kinematic_viscosity = workerSim.flow.kinematic_viscosity * (workerSim.flow.V_inf / std::max(0.0001, workerSim.flow.V_inf)) * workerSim.cachedLbmScale;

    for (size_t pt = 0; pt < angles.size(); ++pt) {
        if (isCancelled.load()) {
            break;
        }

        double currentAlpha = angles[pt];
        int percent = static_cast<int>((pt * 100.0) / angles.size());
        emit progressChanged(percent, QString("Calculating alpha = %1° (%2/%3)...").arg(currentAlpha, 0, 'f', 1).arg(pt + 1).arg(angles.size()));

        workerSim.fastUpdateRotation(currentAlpha);
        if (resetFlowPerPoint && pt > 0) {
            workerSim.resetFlow();
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

        for (int s = 1; s <= steps; ++s) {
            if (isCancelled.load()) break;

            workerSim.lbmSolver->step(safeLBM);

            const auto& currentGrid = workerSim.lbmSolver->getGrid();
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

            if (s > warmup) {
                sum_cl += raw_cl;
                sum_cd += raw_cd;
                sum_lift_N += lift_N;
                sum_drag_N += drag_N;
                sum_fx += drag;
                sum_fy += lift;
                sampled_count++;
            }
        }

        if (isCancelled.load()) break;

        double mean_cl = (sampled_count > 0) ? (sum_cl / sampled_count) : last_cl;
        double mean_cd = (sampled_count > 0) ? (sum_cd / sampled_count) : last_cd;
        double mean_ld = (std::abs(mean_cd) > 1e-6) ? (mean_cl / mean_cd) : 0.0;
        double mean_lift_N = (sampled_count > 0) ? (sum_lift_N / sampled_count) : last_lift_N;
        double mean_drag_N = (sampled_count > 0) ? (sum_drag_N / sampled_count) : last_drag_N;
        double mean_fx = (sampled_count > 0) ? (sum_fx / sampled_count) : last_fx;
        double mean_fy = (sampled_count > 0) ? (sum_fy / sampled_count) : last_fy;

        PolarPointResult res{currentAlpha, mean_cl, mean_cd, mean_ld, mean_lift_N, mean_drag_N, mean_fx, mean_fy};
        results.push_back(res);
        emit pointCompleted(res);
    }

    if (isCancelled.load()) {
        emit progressChanged(0, "Sweep cancelled.");
        return;
    }

    if (!csvFilePath.isEmpty()) {
        std::ofstream csv(csvFilePath.toStdString());
        if (csv.is_open()) {
            csv << "alpha_deg,CL,CD,L_D,Lift_N,Drag_N,Force_X_lattice,Force_Y_lattice,V_inf_mps,q_inf_Pa\n";
            for (const auto& r : results) {
                csv << std::fixed << std::setprecision(4) << r.alpha << ","
                    << std::setprecision(6) << r.cl << ","
                    << std::setprecision(6) << r.cd << ","
                    << std::setprecision(6) << r.ld << ","
                    << std::setprecision(4) << r.lift_N << ","
                    << std::setprecision(4) << r.drag_N << ","
                    << std::setprecision(6) << r.fx << ","
                    << std::setprecision(6) << r.fy << ","
                    << std::setprecision(2) << workerSim.flow.V_inf << ","
                    << std::setprecision(2) << q_inf << "\n";
            }
            csv.close();
        }
    }

    double max_cl = -1e9;
    double alpha_stall = 0.0;
    double min_cd = 1e9;
    double alpha_min_cd = 0.0;
    double max_ld = -1e9;
    double alpha_opt_ld = 0.0;
    bool found_zero_lift = false;
    double alpha_zero_lift = 0.0;

    for (size_t i = 0; i < results.size(); ++i) {
        if (results[i].cl > max_cl) {
            max_cl = results[i].cl;
            alpha_stall = results[i].alpha;
        }
        if (results[i].cd < min_cd) {
            min_cd = results[i].cd;
            alpha_min_cd = results[i].alpha;
        }
        if (results[i].ld > max_ld) {
            max_ld = results[i].ld;
            alpha_opt_ld = results[i].alpha;
        }
        if (i > 0 && !found_zero_lift) {
            if ((results[i-1].cl <= 0.0 && results[i].cl >= 0.0) ||
                (results[i-1].cl >= 0.0 && results[i].cl <= 0.0)) {
                double dCl = results[i].cl - results[i-1].cl;
                if (std::abs(dCl) > 1e-6) {
                    alpha_zero_lift = results[i-1].alpha - results[i-1].cl * (results[i].alpha - results[i-1].alpha) / dCl;
                    found_zero_lift = true;
                }
            }
        }
    }

    double cl_alpha_deg = 0.0;
    double sumA = 0.0, sumCL = 0.0, sumA2 = 0.0, sumACL = 0.0;
    int regCount = 0;
    for (const auto& r : results) {
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
        }
    }

    QString summary = QString(
        "<b>Polar Summary:</b><br>"
        "• CL,max: <b>%1</b> (at α = %2°)<br>"
        "• CD,min: <b>%3</b> (at α = %4°)<br>"
        "• (L/D)max: <b>%5</b> (at α = %6°)<br>"
        "• α0 (Zero-Lift): <b>%7</b><br>"
        "• dCL/dα: <b>%8 /deg</b> (%9 /rad)"
    ).arg(max_cl, 0, 'f', 4)
     .arg(alpha_stall, 0, 'f', 1)
     .arg(min_cd, 0, 'f', 4)
     .arg(alpha_min_cd, 0, 'f', 1)
     .arg(max_ld, 0, 'f', 2)
     .arg(alpha_opt_ld, 0, 'f', 1)
     .arg(found_zero_lift ? QString("%1°").arg(alpha_zero_lift, 0, 'f', 2) : QString("N/A"))
     .arg(cl_alpha_deg, 0, 'f', 4)
     .arg(cl_alpha_deg * 57.2958, 0, 'f', 3);

    emit progressChanged(100, "Sweep completed successfully.");
    emit finished(summary, csvFilePath);
}

PolarDialog::PolarDialog(Simulation* sim, QWidget* parent)
    : QDialog(parent), simulation(sim) {
    setupUi();
}

PolarDialog::~PolarDialog() {
    if (workerThread && workerThread->isRunning()) {
        if (worker) worker->cancel();
        workerThread->quit();
        workerThread->wait();
    }
}

void PolarDialog::setupUi() {
    setWindowTitle("Automated Polar Sweep (Alpha Sweep)");
    resize(700, 580);
    setStyleSheet("QDialog { background-color: #1e1e24; color: #f0f0f0; }"
                  "QGroupBox { font-weight: bold; border: 1px solid #3a3a4c; border-radius: 6px; margin-top: 10px; padding-top: 12px; color: #e0e0e0; }"
                  "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
                  "QLabel { color: #dcdcdc; }"
                  "QSpinBox, QDoubleSpinBox, QLineEdit { background-color: #2b2b36; border: 1px solid #44445a; border-radius: 4px; padding: 4px; color: #ffffff; }"
                  "QPushButton { background-color: #3b4252; color: #eceff4; border: 1px solid #4c566a; border-radius: 4px; padding: 6px 14px; font-weight: bold; }"
                  "QPushButton:hover { background-color: #434c5e; }"
                  "QPushButton:pressed { background-color: #2e3440; }"
                  "QPushButton#startBtn { background-color: #2e7d32; border: 1px solid #388e3c; color: white; }"
                  "QPushButton#startBtn:hover { background-color: #388e3c; }"
                  "QPushButton#cancelBtn { background-color: #c62828; border: 1px solid #d32f2f; color: white; }"
                  "QPushButton#cancelBtn:hover { background-color: #d32f2f; }"
                  "QTableWidget { background-color: #23232d; alternate-background-color: #2a2a37; border: 1px solid #3a3a4c; color: #ffffff; gridline-color: #3a3a4c; }"
                  "QHeaderView::section { background-color: #1a1a20; color: #88c0d0; padding: 4px; border: 1px solid #3a3a4c; font-weight: bold; }"
                  "QProgressBar { border: 1px solid #44445a; border-radius: 4px; text-align: center; color: white; background-color: #2b2b36; }"
                  "QProgressBar::chunk { background-color: #5e81ac; border-radius: 3px; }");

    auto* mainLayout = new QVBoxLayout(this);

    auto* configBox = new QGroupBox("Sweep Range & Solver Parameters", this);
    auto* gridLayout = new QGridLayout(configBox);

    gridLayout->addWidget(new QLabel("Min Alpha (°):"), 0, 0);
    alphaMinSpin = new QDoubleSpinBox(this);
    alphaMinSpin->setRange(-20.0, 40.0);
    alphaMinSpin->setValue(-4.0);
    alphaMinSpin->setSingleStep(0.5);
    gridLayout->addWidget(alphaMinSpin, 0, 1);

    gridLayout->addWidget(new QLabel("Max Alpha (°):"), 0, 2);
    alphaMaxSpin = new QDoubleSpinBox(this);
    alphaMaxSpin->setRange(-20.0, 40.0);
    alphaMaxSpin->setValue(16.0);
    alphaMaxSpin->setSingleStep(0.5);
    gridLayout->addWidget(alphaMaxSpin, 0, 3);

    gridLayout->addWidget(new QLabel("Alpha Step (°):"), 1, 0);
    alphaStepSpin = new QDoubleSpinBox(this);
    alphaStepSpin->setRange(0.1, 10.0);
    alphaStepSpin->setValue(2.0);
    alphaStepSpin->setSingleStep(0.5);
    gridLayout->addWidget(alphaStepSpin, 1, 1);

    gridLayout->addWidget(new QLabel("Steps per Point:"), 1, 2);
    stepsSpin = new QSpinBox(this);
    stepsSpin->setRange(20, 2000);
    stepsSpin->setValue(150);
    gridLayout->addWidget(stepsSpin, 1, 3);

    gridLayout->addWidget(new QLabel("Warmup Steps:"), 2, 0);
    warmupSpin = new QSpinBox(this);
    warmupSpin->setRange(0, 1000);
    warmupSpin->setValue(40);
    gridLayout->addWidget(warmupSpin, 2, 1);

    resetFlowCheck = new QCheckBox("Reset Flow Between Angles", this);
    resetFlowCheck->setChecked(true);
    gridLayout->addWidget(resetFlowCheck, 2, 2, 1, 2);

    gridLayout->addWidget(new QLabel("CSV Export Path:"), 3, 0);
    csvPathEdit = new QLineEdit("polar_results.csv", this);
    gridLayout->addWidget(csvPathEdit, 3, 1, 1, 2);
    browseButton = new QPushButton("Browse...", this);
    connect(browseButton, &QPushButton::clicked, this, &PolarDialog::onBrowseCsv);
    gridLayout->addWidget(browseButton, 3, 3);

    mainLayout->addWidget(configBox);

    auto* btnLayout = new QHBoxLayout();
    startButton = new QPushButton("▶ Start Polar Sweep", this);
    startButton->setObjectName("startBtn");
    connect(startButton, &QPushButton::clicked, this, &PolarDialog::onStartSweep);
    btnLayout->addWidget(startButton);

    cancelButton = new QPushButton("⏹ Cancel", this);
    cancelButton->setObjectName("cancelBtn");
    cancelButton->setEnabled(false);
    connect(cancelButton, &QPushButton::clicked, this, &PolarDialog::onCancelSweep);
    btnLayout->addWidget(cancelButton);

    closeButton = new QPushButton("Close", this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    btnLayout->addWidget(closeButton);

    mainLayout->addLayout(btnLayout);

    progressBar = new QProgressBar(this);
    progressBar->setRange(0, 100);
    progressBar->setValue(0);
    mainLayout->addWidget(progressBar);

    statusLabel = new QLabel("Ready to run polar sweep.", this);
    statusLabel->setStyleSheet("color: #88c0d0; font-weight: bold;");
    mainLayout->addWidget(statusLabel);

    resultsTable = new QTableWidget(this);
    resultsTable->setColumnCount(6);
    resultsTable->setHorizontalHeaderLabels({"Alpha (°)", "CL", "CD", "L/D", "Lift (N)", "Drag (N)"});
    resultsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    resultsTable->setAlternatingRowColors(true);
    resultsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    mainLayout->addWidget(resultsTable, 1);

    summaryLabel = new QLabel(this);
    summaryLabel->setStyleSheet("background-color: #23232d; border: 1px solid #3a3a4c; border-radius: 4px; padding: 8px; color: #eceff4;");
    summaryLabel->setText("Summary will appear here after sweep completes.");
    mainLayout->addWidget(summaryLabel);
}

void PolarDialog::onBrowseCsv() {
    QString path = QFileDialog::getSaveFileName(this, "Save Polar Sweep CSV", "polar_results.csv", "CSV Files (*.csv);;All Files (*.*)");
    if (!path.isEmpty()) {
        if (!path.endsWith(".csv", Qt::CaseInsensitive)) {
            path += ".csv";
        }
        csvPathEdit->setText(path);
    }
}

void PolarDialog::onStartSweep() {
    resultsTable->setRowCount(0);
    progressBar->setValue(0);
    summaryLabel->setText("Sweeping angles of attack...");
    startButton->setEnabled(false);
    cancelButton->setEnabled(true);
    alphaMinSpin->setEnabled(false);
    alphaMaxSpin->setEnabled(false);
    alphaStepSpin->setEnabled(false);
    stepsSpin->setEnabled(false);
    warmupSpin->setEnabled(false);

    workerThread = new QThread(this);
    worker = new PolarWorker(
        simulation,
        alphaMinSpin->value(),
        alphaMaxSpin->value(),
        alphaStepSpin->value(),
        stepsSpin->value(),
        warmupSpin->value(),
        resetFlowCheck->isChecked(),
        csvPathEdit->text()
    );
    worker->moveToThread(workerThread);

    connect(workerThread, &QThread::started, worker, &PolarWorker::process);
    connect(worker, &PolarWorker::pointCompleted, this, &PolarDialog::onPointCompleted);
    connect(worker, &PolarWorker::progressChanged, this, &PolarDialog::onProgressChanged);
    connect(worker, &PolarWorker::finished, this, &PolarDialog::onSweepFinished);
    connect(worker, &PolarWorker::errorOccurred, this, &PolarDialog::onSweepError);

    connect(worker, &PolarWorker::finished, workerThread, &QThread::quit);
    connect(worker, &PolarWorker::errorOccurred, workerThread, &QThread::quit);
    connect(workerThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);

    workerThread->start();
}

void PolarDialog::onCancelSweep() {
    if (worker) {
        worker->cancel();
    }
    cancelButton->setEnabled(false);
    statusLabel->setText("Cancelling sweep...");
}

void PolarDialog::onPointCompleted(PolarPointResult res) {
    int row = resultsTable->rowCount();
    resultsTable->insertRow(row);

    resultsTable->setItem(row, 0, new QTableWidgetItem(QString::number(res.alpha, 'f', 2)));
    resultsTable->setItem(row, 1, new QTableWidgetItem(QString::number(res.cl, 'f', 4)));
    resultsTable->setItem(row, 2, new QTableWidgetItem(QString::number(res.cd, 'f', 4)));
    resultsTable->setItem(row, 3, new QTableWidgetItem(QString::number(res.ld, 'f', 2)));
    resultsTable->setItem(row, 4, new QTableWidgetItem(QString::number(res.lift_N, 'f', 2)));
    resultsTable->setItem(row, 5, new QTableWidgetItem(QString::number(res.drag_N, 'f', 2)));

    resultsTable->scrollToBottom();
}

void PolarDialog::onProgressChanged(int percent, QString message) {
    progressBar->setValue(percent);
    statusLabel->setText(message);
}

void PolarDialog::onSweepFinished(QString summary, QString csvPath) {
    summaryLabel->setText(summary);
    startButton->setEnabled(true);
    cancelButton->setEnabled(false);
    alphaMinSpin->setEnabled(true);
    alphaMaxSpin->setEnabled(true);
    alphaStepSpin->setEnabled(true);
    stepsSpin->setEnabled(true);
    warmupSpin->setEnabled(true);
    worker = nullptr;
    workerThread = nullptr;
}

void PolarDialog::onSweepError(QString message) {
    QMessageBox::critical(this, "Polar Sweep Error", message);
    startButton->setEnabled(true);
    cancelButton->setEnabled(false);
    alphaMinSpin->setEnabled(true);
    alphaMaxSpin->setEnabled(true);
    alphaStepSpin->setEnabled(true);
    stepsSpin->setEnabled(true);
    warmupSpin->setEnabled(true);
    worker = nullptr;
    workerThread = nullptr;
}

}
