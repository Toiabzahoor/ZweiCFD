#pragma once

#include <QWidget>
#include <vector>
#include <QString>
#include <QPointF>
#include <QColor>
#include "ZweiCFD/solver/airfoil.hpp"
#include "ZweiCFD/core/simulation.hpp"

namespace zweicfd {

struct PolarPointResult;

enum class PlotMode {
    LiftCurve,
    DragPolar,
    Efficiency,
    Forces,
    CpDistribution
};

class PlotWidget : public QWidget {
    Q_OBJECT
public:
    explicit PlotWidget(QWidget* parent = nullptr);
    ~PlotWidget() override = default;

    void setPlotMode(PlotMode mode);
    PlotMode getPlotMode() const { return currentMode; }

    void addPoint(const PolarPointResult& pt);
    void setPoints(const std::vector<PolarPointResult>& pts);
    void setCpData(const CpDistribution& cp, const std::vector<Point2D>& foilCoords = {});
    void clear();

    bool exportImage(const QString& filepath);
    void copyToClipboard();

    void setShowLinearFit(bool show);
    void setShowStallMarker(bool show);
    void setShowZeroLift(bool show);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    void drawGridAndAxes(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY, const QString& xLabel, const QString& yLabel);
    void drawCurve(QPainter& painter, const QRect& plotRect, const std::vector<QPointF>& points, double minX, double maxX, double minY, double maxY, const QColor& color, const QString& name, bool isDashed = false);
    void drawMarkers(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY);
    void drawTooltip(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY);
    void drawAirfoilThumbnail(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY);

    QPointF dataToScreen(const QPointF& dataPt, const QRect& plotRect, double minX, double maxX, double minY, double maxY) const;
    QPointF screenToData(const QPointF& screenPt, const QRect& plotRect, double minX, double maxX, double minY, double maxY) const;

    PlotMode currentMode = PlotMode::LiftCurve;
    std::vector<PolarPointResult> dataPoints;
    CpDistribution cpData;
    std::vector<Point2D> airfoilCoords;

    bool showLinearFit = true;
    bool showStallMarker = true;
    bool showZeroLift = true;

    bool hasHover = false;
    QPoint mousePos;
};

}
