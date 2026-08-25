#include "ZweiCFD/ui/cp_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>

namespace zweicfd {

CpDialog::CpDialog(Simulation* sim, QWidget* parent)
    : QDialog(parent), simulation(sim) {
    setWindowTitle("Surface Pressure Coefficient (Cp) Distribution");
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    resize(720, 520);
    setMinimumSize(560, 380);
    setupUi();

    refreshTimer = new QTimer(this);
    connect(refreshTimer, &QTimer::timeout, this, &CpDialog::refreshCpPlot);
    refreshTimer->start(100);

    refreshCpPlot();
}

void CpDialog::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(8);

    auto* topLayout = new QHBoxLayout();
    colormapCheck = new QCheckBox("Map 3D Surface Cp Colormap on Mesh", this);
    if (simulation) {
        colormapCheck->setChecked(simulation->isSurfaceCpVisible());
    }
    connect(colormapCheck, &QCheckBox::toggled, this, &CpDialog::onToggle3DColormap);
    topLayout->addWidget(colormapCheck);

    topLayout->addStretch(1);
    mainLayout->addLayout(topLayout);

    plotWidget = new PlotWidget(this);
    plotWidget->setPlotMode(PlotMode::CpDistribution);
    mainLayout->addWidget(plotWidget, 1);

    statsLabel = new QLabel("Stagnation: Cp = 0.00 | Suction Peak: -Cp = 0.00", this);
    statsLabel->setStyleSheet("background-color: #23232d; border: 1px solid #3a3a4c; border-radius: 4px; padding: 6px; color: #eceff4; font-weight: bold;");
    mainLayout->addWidget(statsLabel);

    auto* btnLayout = new QHBoxLayout();
    exportButton = new QPushButton("Save Plot Image...", this);
    connect(exportButton, &QPushButton::clicked, this, &CpDialog::onExportImage);
    btnLayout->addWidget(exportButton);

    copyButton = new QPushButton("Copy to Clipboard", this);
    connect(copyButton, &QPushButton::clicked, this, &CpDialog::onCopyImage);
    btnLayout->addWidget(copyButton);

    btnLayout->addStretch(1);

    closeButton = new QPushButton("Close", this);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    btnLayout->addWidget(closeButton);

    mainLayout->addLayout(btnLayout);
}

void CpDialog::refreshCpPlot() {
    if (!simulation || !plotWidget) return;

    CpDistribution cp = simulation->extractSurfaceCp();
    const auto& coords = simulation->foil.getCoordinates();
    plotWidget->setCpData(cp, coords);

    double suctionPeak = 0.0;
    if (!cp.upper.empty()) {
        for (const auto& pt : cp.upper) {
            suctionPeak = std::max(suctionPeak, -pt.cp);
        }
    }
    statsLabel->setText(QString("Stagnation: Cp = %1 @ x/c = %2 | Max Suction Peak: -Cp = %3")
                            .arg(cp.cp_stagnation, 0, 'f', 3)
                            .arg(cp.xc_stagnation, 0, 'f', 3)
                            .arg(suctionPeak, 0, 'f', 3));
}

void CpDialog::onToggle3DColormap(bool checked) {
    if (simulation) {
        simulation->setSurfaceCpVisible(checked);
    }
}

void CpDialog::onExportImage() {
    if (!plotWidget) return;
    QString path = QFileDialog::getSaveFileName(this, "Save Cp Plot Image", "surface_cp_distribution.png", "PNG Image (*.png);;JPEG Image (*.jpg *.jpeg);;All Files (*.*)");
    if (!path.isEmpty()) {
        if (!path.endsWith(".png", Qt::CaseInsensitive) && !path.endsWith(".jpg", Qt::CaseInsensitive) && !path.endsWith(".jpeg", Qt::CaseInsensitive)) {
            path += ".png";
        }
        if (plotWidget->exportImage(path)) {
            QMessageBox::information(this, "Image Exported", "Plot saved successfully to:\n" + path);
        }
    }
}

void CpDialog::onCopyImage() {
    if (!plotWidget) return;
    plotWidget->copyToClipboard();
    QMessageBox::information(this, "Copied", "Plot image copied to system clipboard.");
}

}
