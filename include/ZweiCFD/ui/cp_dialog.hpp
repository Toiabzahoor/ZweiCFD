#pragma once

#include <QDialog>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include "ZweiCFD/core/simulation.hpp"
#include "ZweiCFD/ui/plot.hpp"

namespace zweicfd {

class CpDialog : public QDialog {
    Q_OBJECT
public:
    explicit CpDialog(Simulation* sim, QWidget* parent = nullptr);
    ~CpDialog() override = default;

private slots:
    void refreshCpPlot();
    void onToggle3DColormap(bool checked);
    void onExportImage();
    void onCopyImage();

private:
    void setupUi();

    Simulation* simulation;
    PlotWidget* plotWidget = nullptr;
    QCheckBox* colormapCheck = nullptr;
    QPushButton* exportButton = nullptr;
    QPushButton* copyButton = nullptr;
    QPushButton* closeButton = nullptr;
    QLabel* statsLabel = nullptr;
    QTimer* refreshTimer = nullptr;
};

}
