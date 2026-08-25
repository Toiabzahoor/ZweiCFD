#include "ZweiCFD/ui/plot.hpp"
#include "ZweiCFD/ui/polar_dialog.hpp"
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QGuiApplication>
#include <QClipboard>
#include <QFontMetrics>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace zweicfd {

static void calculateNiceRange(double rawMin, double rawMax, double& niceMin, double& niceMax, double& step) {
    if (std::abs(rawMax - rawMin) < 1e-6) {
        rawMin -= 1.0;
        rawMax += 1.0;
    }
    double range = rawMax - rawMin;
    double roughStep = range / 5.0;
    double exponent = std::floor(std::log10(roughStep));
    double fraction = roughStep / std::pow(10.0, exponent);
    double niceFraction = 1.0;

    if (fraction <= 1.2) niceFraction = 1.0;
    else if (fraction <= 2.5) niceFraction = 2.0;
    else if (fraction <= 6.0) niceFraction = 5.0;
    else niceFraction = 10.0;

    step = niceFraction * std::pow(10.0, exponent);
    if (step < 1e-4) step = 0.5;

    niceMin = std::floor(rawMin / step) * step;
    niceMax = std::ceil(rawMax / step) * step;
    if (std::abs(niceMax - niceMin) < 1e-4) {
        niceMax += step;
    }
}

static QPainterPath buildSmoothPath(const std::vector<QPointF>& screenPts) {
    QPainterPath path;
    if (screenPts.empty()) return path;
    if (screenPts.size() == 1) {
        path.moveTo(screenPts[0]);
        return path;
    }
    if (screenPts.size() == 2) {
        path.moveTo(screenPts[0]);
        path.lineTo(screenPts[1]);
        return path;
    }

    path.moveTo(screenPts[0]);
    for (size_t i = 0; i < screenPts.size() - 1; ++i) {
        QPointF p0 = (i == 0) ? screenPts[0] : screenPts[i - 1];
        QPointF p1 = screenPts[i];
        QPointF p2 = screenPts[i + 1];
        QPointF p3 = (i + 2 < screenPts.size()) ? screenPts[i + 2] : p2;

        QPointF c1 = p1 + (p2 - p0) * 0.18;
        QPointF c2 = p2 - (p3 - p1) * 0.18;
        path.cubicTo(c1, c2, p2);
    }
    return path;
}

PlotWidget::PlotWidget(QWidget* parent)
    : QWidget(parent) {
    setMouseTracking(true);
    setMinimumSize(360, 260);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PlotWidget::setPlotMode(PlotMode mode) {
    currentMode = mode;
    update();
}

void PlotWidget::addPoint(const PolarPointResult& pt) {
    dataPoints.push_back(pt);
    update();
}

void PlotWidget::setPoints(const std::vector<PolarPointResult>& pts) {
    dataPoints = pts;
    update();
}

void PlotWidget::setCpData(const CpDistribution& cp, const std::vector<Point2D>& foilCoords) {
    cpData = cp;
    airfoilCoords = foilCoords;
    update();
}

void PlotWidget::clear() {
    dataPoints.clear();
    cpData.upper.clear();
    cpData.lower.clear();
    update();
}

bool PlotWidget::exportImage(const QString& filepath) {
    QPixmap pixmap(size() * 2);
    pixmap.setDevicePixelRatio(2.0);
    pixmap.fill(QColor("#111118"));
    {
        QPainter painter(&pixmap);
        render(&painter);
    }
    return pixmap.save(filepath);
}

void PlotWidget::copyToClipboard() {
    QPixmap pixmap(size() * 2);
    pixmap.setDevicePixelRatio(2.0);
    pixmap.fill(QColor("#111118"));
    {
        QPainter painter(&pixmap);
        render(&painter);
    }
    QClipboard* clipboard = QGuiApplication::clipboard();
    if (clipboard) {
        clipboard->setPixmap(pixmap);
    }
}

void PlotWidget::setShowLinearFit(bool show) {
    showLinearFit = show;
    update();
}

void PlotWidget::setShowStallMarker(bool show) {
    showStallMarker = show;
    update();
}

void PlotWidget::setShowZeroLift(bool show) {
    showZeroLift = show;
    update();
}

void PlotWidget::mouseMoveEvent(QMouseEvent* event) {
    mousePos = event->pos();
    hasHover = true;
    update();
}

void PlotWidget::leaveEvent(QEvent* event) {
    QWidget::leaveEvent(event);
    hasHover = false;
    update();
}

QPointF PlotWidget::dataToScreen(const QPointF& dataPt, const QRect& plotRect, double minX, double maxX, double minY, double maxY) const {
    double rangeX = (std::abs(maxX - minX) > 1e-6) ? (maxX - minX) : 1.0;
    double rangeY = (std::abs(maxY - minY) > 1e-6) ? (maxY - minY) : 1.0;

    double sx = plotRect.left() + ((dataPt.x() - minX) / rangeX) * plotRect.width();
    double sy = plotRect.bottom() - ((dataPt.y() - minY) / rangeY) * plotRect.height();
    return QPointF(sx, sy);
}

QPointF PlotWidget::screenToData(const QPointF& screenPt, const QRect& plotRect, double minX, double maxX, double minY, double maxY) const {
    double rangeX = (std::abs(maxX - minX) > 1e-6) ? (maxX - minX) : 1.0;
    double rangeY = (std::abs(maxY - minY) > 1e-6) ? (maxY - minY) : 1.0;

    double dx = minX + ((screenPt.x() - plotRect.left()) / plotRect.width()) * rangeX;
    double dy = minY + ((plotRect.bottom() - screenPt.y()) / plotRect.height()) * rangeY;
    return QPointF(dx, dy);
}

void PlotWidget::drawGridAndAxes(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY, const QString& xLabel, const QString& yLabel) {
    QLinearGradient bgGrad(plotRect.topLeft(), plotRect.bottomLeft());
    bgGrad.setColorAt(0.0, QColor("#14141e"));
    bgGrad.setColorAt(1.0, QColor("#0f0f17"));
    painter.setBrush(bgGrad);
    painter.setPen(QPen(QColor("#242436"), 1));
    painter.drawRoundedRect(plotRect, 6, 6);

    double stepX = 2.0;
    double niceMinX = minX, niceMaxX = maxX;
    calculateNiceRange(minX, maxX, niceMinX, niceMaxX, stepX);

    double stepY = 1.0;
    double niceMinY = minY, niceMaxY = maxY;
    calculateNiceRange(minY, maxY, niceMinY, niceMaxY, stepY);

    QPen gridPen(QColor(255, 255, 255, 14), 1, Qt::DashLine);
    painter.setPen(gridPen);

    for (double vx = niceMinX; vx <= niceMaxX + 1e-4; vx += stepX) {
        if (vx < minX - 1e-4 || vx > maxX + 1e-4) continue;
        QPointF pt = dataToScreen(QPointF(vx, minY), plotRect, minX, maxX, minY, maxY);
        painter.setPen(gridPen);
        painter.drawLine(QPointF(pt.x(), plotRect.top()), QPointF(pt.x(), plotRect.bottom()));

        painter.setPen(QColor("#7e8299"));
        QFont font = painter.font();
        font.setPointSize(8);
        font.setBold(false);
        painter.setFont(font);
        QString labelStr = QString::number(vx, 'f', (stepX < 1.0) ? 1 : 0);
        painter.drawText(QRectF(pt.x() - 25, plotRect.bottom() + 5, 50, 16), Qt::AlignCenter, labelStr);
    }

    for (double vy = niceMinY; vy <= niceMaxY + 1e-4; vy += stepY) {
        if (vy < minY - 1e-4 || vy > maxY + 1e-4) continue;
        QPointF pt = dataToScreen(QPointF(minX, vy), plotRect, minX, maxX, minY, maxY);
        painter.setPen(gridPen);
        painter.drawLine(QPointF(plotRect.left(), pt.y()), QPointF(plotRect.right(), pt.y()));

        painter.setPen(QColor("#7e8299"));
        QFont font = painter.font();
        font.setPointSize(8);
        font.setBold(false);
        painter.setFont(font);
        QString labelStr = QString::number(vy, 'f', (stepY < 1.0) ? 1 : 0);
        painter.drawText(QRectF(plotRect.left() - 48, pt.y() - 8, 42, 16), Qt::AlignRight | Qt::AlignVCenter, labelStr);
    }

    if (minY <= 0.0 && maxY >= 0.0) {
        QPointF zeroY = dataToScreen(QPointF(minX, 0.0), plotRect, minX, maxX, minY, maxY);
        painter.setPen(QPen(QColor("#454562"), 1.5, Qt::SolidLine));
        painter.drawLine(QPointF(plotRect.left(), zeroY.y()), QPointF(plotRect.right(), zeroY.y()));
    }
    if (minX <= 0.0 && maxX >= 0.0) {
        QPointF zeroX = dataToScreen(QPointF(0.0, minY), plotRect, minX, maxX, minY, maxY);
        painter.setPen(QPen(QColor("#454562"), 1.5, Qt::SolidLine));
        painter.drawLine(QPointF(zeroX.x(), plotRect.top()), QPointF(zeroX.x(), plotRect.bottom()));
    }

    painter.setPen(QColor("#9292b0"));
    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);

    painter.drawText(QRectF(plotRect.left(), plotRect.bottom() + 22, plotRect.width(), 20), Qt::AlignCenter, xLabel);

    painter.save();
    painter.translate(plotRect.left() - 38, plotRect.center().y());
    painter.rotate(-90);
    painter.drawText(QRectF(-plotRect.height() / 2, -10, plotRect.height(), 20), Qt::AlignCenter, yLabel);
    painter.restore();
}

void PlotWidget::drawCurve(QPainter& painter, const QRect& plotRect, const std::vector<QPointF>& points, double minX, double maxX, double minY, double maxY, const QColor& color, const QString& name, bool isDashed) {
    if (points.empty()) return;

    std::vector<QPointF> screenPts;
    screenPts.reserve(points.size());
    for (const auto& p : points) {
        screenPts.push_back(dataToScreen(p, plotRect, minX, maxX, minY, maxY));
    }

    QPainterPath path = buildSmoothPath(screenPts);

    if (!isDashed && points.size() >= 2) {
        QPainterPath fillPath = path;
        double zeroYVal = std::clamp(0.0, minY, maxY);
        QPointF zeroBottomRight = dataToScreen(QPointF(points.back().x(), zeroYVal), plotRect, minX, maxX, minY, maxY);
        QPointF zeroBottomLeft = dataToScreen(QPointF(points.front().x(), zeroYVal), plotRect, minX, maxX, minY, maxY);
        fillPath.lineTo(zeroBottomRight);
        fillPath.lineTo(zeroBottomLeft);
        fillPath.closeSubpath();

        QLinearGradient areaGrad(plotRect.topLeft(), plotRect.bottomLeft());
        areaGrad.setColorAt(0.0, QColor(color.red(), color.green(), color.blue(), 45));
        areaGrad.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 2));
        painter.setBrush(areaGrad);
        painter.setPen(Qt::NoPen);
        painter.drawPath(fillPath);
    }

    QPen pen(color, isDashed ? 1.5 : 2.5, isDashed ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);

    if (!isDashed) {
        for (size_t i = 0; i < screenPts.size(); ++i) {
            bool isLast = (i == screenPts.size() - 1);
            if (isLast) {
                painter.setBrush(QColor(color.red(), color.green(), color.blue(), 70));
                painter.setPen(Qt::NoPen);
                painter.drawEllipse(screenPts[i], 9, 9);
            }

            painter.setBrush(color);
            painter.setPen(QPen(QColor("#0f0f17"), 1.8));
            painter.drawEllipse(screenPts[i], isLast ? 5.5 : 3.8, isLast ? 5.5 : 3.8);

            painter.setBrush(Qt::white);
            painter.setPen(Qt::NoPen);
            painter.drawEllipse(screenPts[i], isLast ? 2.5 : 1.5, isLast ? 2.5 : 1.5);
        }
    }
}

void PlotWidget::drawMarkers(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY) {
    if (dataPoints.empty()) return;

    if (currentMode == PlotMode::LiftCurve) {
        auto maxIt = std::max_element(dataPoints.begin(), dataPoints.end(), [](const PolarPointResult& a, const PolarPointResult& b) {
            return a.cl < b.cl;
        });
        if (maxIt != dataPoints.end() && showStallMarker) {
            QPointF sPt = dataToScreen(QPointF(maxIt->alpha, maxIt->cl), plotRect, minX, maxX, minY, maxY);
            painter.setPen(QPen(QColor(255, 82, 82, 160), 1, Qt::DashLine));
            painter.drawLine(QPointF(sPt.x(), plotRect.bottom()), sPt);
            painter.drawLine(QPointF(plotRect.left(), sPt.y()), sPt);

            painter.setBrush(QColor("#ff5252"));
            painter.setPen(QPen(Qt::white, 2));
            painter.drawEllipse(sPt, 6, 6);

            QString badgeText = QString("Stall: CL = %1 @ a = %2°").arg(maxIt->cl, 0, 'f', 2).arg(maxIt->alpha, 0, 'f', 1);
            QFont font = painter.font();
            font.setPointSize(8);
            font.setBold(true);
            painter.setFont(font);

            QRect textRect = painter.fontMetrics().boundingRect(badgeText);
            textRect.adjust(-6, -3, 6, 3);
            int badgeX = sPt.x() - textRect.width() / 2;
            int badgeY = sPt.y() - 24;
            if (badgeX + textRect.width() > plotRect.right()) badgeX = plotRect.right() - textRect.width() - 4;
            if (badgeX < plotRect.left()) badgeX = plotRect.left() + 4;
            if (badgeY < plotRect.top()) badgeY = sPt.y() + 12;

            QRect badgeRect(badgeX, badgeY, textRect.width(), textRect.height());
            painter.setBrush(QColor(35, 15, 20, 230));
            painter.setPen(QPen(QColor("#ff5252"), 1));
            painter.drawRoundedRect(badgeRect, 4, 4);

            painter.setPen(QColor("#ff7070"));
            painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
        }

        if (showZeroLift) {
            for (size_t i = 0; i + 1 < dataPoints.size(); ++i) {
                if ((dataPoints[i].cl <= 0.0 && dataPoints[i+1].cl >= 0.0) ||
                    (dataPoints[i].cl >= 0.0 && dataPoints[i+1].cl <= 0.0)) {
                    double t = (0.0 - dataPoints[i].cl) / (dataPoints[i+1].cl - dataPoints[i].cl);
                    double a0 = dataPoints[i].alpha + t * (dataPoints[i+1].alpha - dataPoints[i].alpha);
                    QPointF sPt = dataToScreen(QPointF(a0, 0.0), plotRect, minX, maxX, minY, maxY);

                    painter.setBrush(QColor("#ffd700"));
                    painter.setPen(QPen(Qt::black, 1.5));
                    painter.drawEllipse(sPt, 5, 5);

                    QString badgeText = QString("a0 = %1°").arg(a0, 0, 'f', 1);
                    QFont font = painter.font();
                    font.setPointSize(8);
                    font.setBold(true);
                    painter.setFont(font);

                    QRect textRect = painter.fontMetrics().boundingRect(badgeText);
                    textRect.adjust(-5, -2, 5, 2);
                    QRect badgeRect(sPt.x() - textRect.width() / 2, sPt.y() + 8, textRect.width(), textRect.height());

                    painter.setBrush(QColor(35, 30, 10, 220));
                    painter.setPen(QPen(QColor("#ffd700"), 1));
                    painter.drawRoundedRect(badgeRect, 4, 4);

                    painter.setPen(QColor("#ffd700"));
                    painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
                    break;
                }
            }
        }

        if (showLinearFit && dataPoints.size() >= 2) {
            double sumA = 0.0, sumCL = 0.0, sumA2 = 0.0, sumACL = 0.0;
            int n = 0;
            for (const auto& pt : dataPoints) {
                if (pt.alpha >= -4.0 && pt.alpha <= 10.0) {
                    sumA += pt.alpha;
                    sumCL += pt.cl;
                    sumA2 += pt.alpha * pt.alpha;
                    sumACL += pt.alpha * pt.cl;
                    n++;
                }
            }
            if (n >= 2) {
                double denom = n * sumA2 - sumA * sumA;
                if (std::abs(denom) > 1e-6) {
                    double slope = (n * sumACL - sumA * sumCL) / denom;
                    double intercept = (sumCL - slope * sumA) / n;
                    std::vector<QPointF> fitLine;
                    fitLine.push_back(QPointF(minX, intercept + slope * minX));
                    fitLine.push_back(QPointF(maxX, intercept + slope * maxX));
                    drawCurve(painter, plotRect, fitLine, minX, maxX, minY, maxY, QColor(255, 215, 0, 160), "Linear Fit", true);
                }
            }
        }
    } else if (currentMode == PlotMode::DragPolar) {
        auto optIt = std::max_element(dataPoints.begin(), dataPoints.end(), [](const PolarPointResult& a, const PolarPointResult& b) {
            return a.ld < b.ld;
        });
        if (optIt != dataPoints.end() && optIt->cd > 1e-4) {
            QPointF sPt = dataToScreen(QPointF(optIt->cd, optIt->cl), plotRect, minX, maxX, minY, maxY);
            painter.setBrush(QColor("#00e676"));
            painter.setPen(QPen(Qt::white, 2));
            painter.drawEllipse(sPt, 6, 6);

            QString badgeText = QString("(L/D)max = %1").arg(optIt->ld, 0, 'f', 2);
            QFont font = painter.font();
            font.setPointSize(8);
            font.setBold(true);
            painter.setFont(font);

            QRect textRect = painter.fontMetrics().boundingRect(badgeText);
            textRect.adjust(-6, -3, 6, 3);
            QRect badgeRect(sPt.x() + 8, sPt.y() - 10, textRect.width(), textRect.height());

            painter.setBrush(QColor(10, 35, 20, 230));
            painter.setPen(QPen(QColor("#00e676"), 1));
            painter.drawRoundedRect(badgeRect, 4, 4);

            painter.setPen(QColor("#00e676"));
            painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
        }
    } else if (currentMode == PlotMode::Efficiency) {
        auto maxIt = std::max_element(dataPoints.begin(), dataPoints.end(), [](const PolarPointResult& a, const PolarPointResult& b) {
            return a.ld < b.ld;
        });
        if (maxIt != dataPoints.end()) {
            QPointF sPt = dataToScreen(QPointF(maxIt->alpha, maxIt->ld), plotRect, minX, maxX, minY, maxY);
            painter.setPen(QPen(QColor(118, 255, 3, 160), 1, Qt::DashLine));
            painter.drawLine(QPointF(sPt.x(), plotRect.bottom()), sPt);

            painter.setBrush(QColor("#76ff03"));
            painter.setPen(QPen(Qt::black, 2));
            painter.drawEllipse(sPt, 6, 6);

            QString badgeText = QString("Max L/D = %1 @ a = %2°").arg(maxIt->ld, 0, 'f', 2).arg(maxIt->alpha, 0, 'f', 1);
            QFont font = painter.font();
            font.setPointSize(8);
            font.setBold(true);
            painter.setFont(font);

            QRect textRect = painter.fontMetrics().boundingRect(badgeText);
            textRect.adjust(-6, -3, 6, 3);
            QRect badgeRect(sPt.x() + 8, sPt.y() - 18, textRect.width(), textRect.height());

            painter.setBrush(QColor(20, 35, 10, 230));
            painter.setPen(QPen(QColor("#76ff03"), 1));
            painter.drawRoundedRect(badgeRect, 4, 4);

            painter.setPen(QColor("#76ff03"));
            painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
        }
    } else if (currentMode == PlotMode::CpDistribution) {
        if (!cpData.upper.empty()) {
            auto peakIt = std::min_element(cpData.upper.begin(), cpData.upper.end(), [](const CpPoint& a, const CpPoint& b) {
                return a.cp < b.cp;
            });
            if (peakIt != cpData.upper.end()) {
                QPointF sPt = dataToScreen(QPointF(peakIt->xc, -peakIt->cp), plotRect, minX, maxX, minY, maxY);
                painter.setBrush(QColor("#00e5ff"));
                painter.setPen(QPen(Qt::white, 2));
                painter.drawEllipse(sPt, 6, 6);

                QString badgeText = QString("Suction Peak: -Cp = %1 @ x/c = %2").arg(-peakIt->cp, 0, 'f', 2).arg(peakIt->xc, 0, 'f', 2);
                QFont font = painter.font();
                font.setPointSize(8);
                font.setBold(true);
                painter.setFont(font);

                QRect textRect = painter.fontMetrics().boundingRect(badgeText);
                textRect.adjust(-6, -3, 6, 3);
                int badgeX = sPt.x() + 8;
                int badgeY = sPt.y() - 14;
                if (badgeX + textRect.width() > plotRect.right()) badgeX = plotRect.right() - textRect.width() - 4;
                QRect badgeRect(badgeX, badgeY, textRect.width(), textRect.height());

                painter.setBrush(QColor(10, 30, 40, 230));
                painter.setPen(QPen(QColor("#00e5ff"), 1));
                painter.drawRoundedRect(badgeRect, 4, 4);

                painter.setPen(QColor("#00e5ff"));
                painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
            }
        }
        if (!cpData.lower.empty() || !cpData.upper.empty()) {
            QPointF sPt = dataToScreen(QPointF(cpData.xc_stagnation, -cpData.cp_stagnation), plotRect, minX, maxX, minY, maxY);
            painter.setBrush(QColor("#ff5252"));
            painter.setPen(QPen(Qt::white, 2));
            painter.drawEllipse(sPt, 6, 6);

            QString badgeText = QString("Stagnation: Cp = %1").arg(cpData.cp_stagnation, 0, 'f', 2);
            QFont font = painter.font();
            font.setPointSize(8);
            font.setBold(true);
            painter.setFont(font);

            QRect textRect = painter.fontMetrics().boundingRect(badgeText);
            textRect.adjust(-6, -3, 6, 3);
            QRect badgeRect(sPt.x() + 8, sPt.y() + 6, textRect.width(), textRect.height());

            painter.setBrush(QColor(40, 15, 20, 230));
            painter.setPen(QPen(QColor("#ff5252"), 1));
            painter.drawRoundedRect(badgeRect, 4, 4);

            painter.setPen(QColor("#ff7070"));
            painter.drawText(badgeRect, Qt::AlignCenter, badgeText);
        }
    }
}

void PlotWidget::drawAirfoilThumbnail(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY) {
    if (airfoilCoords.empty()) return;

    double thumbBaseY = plotRect.bottom() - 18.0;
    double chordScale = plotRect.width() / std::max(0.1, (maxX - minX));

    QPainterPath foilPath;
    for (size_t i = 0; i < airfoilCoords.size(); ++i) {
        double xc = std::clamp(airfoilCoords[i].x, 0.0, 1.0);
        double yc = airfoilCoords[i].y;

        double sx = plotRect.left() + ((xc - minX) / (maxX - minX)) * plotRect.width();
        double sy = thumbBaseY - yc * chordScale * 0.40;

        if (i == 0) foilPath.moveTo(sx, sy);
        else foilPath.lineTo(sx, sy);
    }
    foilPath.closeSubpath();

    painter.setBrush(QColor(28, 38, 56, 170));
    painter.setPen(QPen(QColor("#4c78a8"), 1.2));
    painter.drawPath(foilPath);
}

void PlotWidget::drawTooltip(QPainter& painter, const QRect& plotRect, double minX, double maxX, double minY, double maxY) {
    if (!hasHover) return;
    if (!plotRect.contains(mousePos)) return;

    if (currentMode == PlotMode::CpDistribution) {
        const CpPoint* closestPt = nullptr;
        bool isUpper = true;
        double minDistSq = 1e9;

        for (const auto& pt : cpData.upper) {
            QPointF sPt = dataToScreen(QPointF(pt.xc, -pt.cp), plotRect, minX, maxX, minY, maxY);
            double distSq = (sPt.x() - mousePos.x()) * (sPt.x() - mousePos.x()) + (sPt.y() - mousePos.y()) * (sPt.y() - mousePos.y());
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestPt = &pt;
                isUpper = true;
            }
        }
        for (const auto& pt : cpData.lower) {
            QPointF sPt = dataToScreen(QPointF(pt.xc, -pt.cp), plotRect, minX, maxX, minY, maxY);
            double distSq = (sPt.x() - mousePos.x()) * (sPt.x() - mousePos.x()) + (sPt.y() - mousePos.y()) * (sPt.y() - mousePos.y());
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestPt = &pt;
                isUpper = false;
            }
        }

        if (closestPt && minDistSq < 1000.0) {
            QPointF sPt = dataToScreen(QPointF(closestPt->xc, -closestPt->cp), plotRect, minX, maxX, minY, maxY);

            painter.setPen(QPen(QColor(255, 255, 255, 100), 1, Qt::DashLine));
            painter.drawLine(QPointF(sPt.x(), plotRect.top()), QPointF(sPt.x(), plotRect.bottom()));
            painter.drawLine(QPointF(plotRect.left(), sPt.y()), QPointF(plotRect.right(), sPt.y()));

            painter.setBrush(Qt::white);
            painter.setPen(QPen(isUpper ? QColor("#00e5ff") : QColor("#ff7043"), 2.5));
            painter.drawEllipse(sPt, 7, 7);

            QString text = QString("Surface: %1\nChord Station x/c: %2\nPressure Coeff Cp: %3\nSuction -Cp: %4")
                               .arg(isUpper ? "Upper (Suction)" : "Lower (Pressure)")
                               .arg(closestPt->xc, 0, 'f', 3)
                               .arg(closestPt->cp, 0, 'f', 4)
                               .arg(-closestPt->cp, 0, 'f', 4);

            QFont font = painter.font();
            font.setPointSize(8);
            painter.setFont(font);

            QRect textRect = painter.fontMetrics().boundingRect(QRect(0, 0, 200, 160), Qt::AlignLeft, text);
            textRect.adjust(-8, -6, 8, 6);

            int tipX = sPt.x() + 14;
            int tipY = sPt.y() - textRect.height() / 2;
            if (tipX + textRect.width() > plotRect.right()) tipX = sPt.x() - textRect.width() - 14;
            if (tipY < plotRect.top()) tipY = plotRect.top() + 6;
            if (tipY + textRect.height() > plotRect.bottom()) tipY = plotRect.bottom() - textRect.height() - 6;

            QRect boxRect(tipX, tipY, textRect.width(), textRect.height());
            painter.setBrush(QColor(18, 18, 28, 245));
            painter.setPen(QPen(QColor("#3e3e58"), 1.5));
            painter.drawRoundedRect(boxRect, 6, 6);

            painter.setPen(QColor("#f0f0ff"));
            painter.drawText(boxRect.adjusted(8, 6, -8, -6), Qt::AlignLeft, text);
        }
        return;
    }

    if (dataPoints.empty()) return;

    const PolarPointResult* closestPt = nullptr;
    double minDistSq = 1e9;

    for (const auto& pt : dataPoints) {
        QPointF dataPt;
        if (currentMode == PlotMode::LiftCurve || currentMode == PlotMode::Efficiency || currentMode == PlotMode::Forces) {
            dataPt = QPointF(pt.alpha, (currentMode == PlotMode::LiftCurve) ? pt.cl : (currentMode == PlotMode::Efficiency ? pt.ld : pt.lift_N));
        } else {
            dataPt = QPointF(pt.cd, pt.cl);
        }
        QPointF sPt = dataToScreen(dataPt, plotRect, minX, maxX, minY, maxY);
        double distSq = (sPt.x() - mousePos.x()) * (sPt.x() - mousePos.x()) + (sPt.y() - mousePos.y()) * (sPt.y() - mousePos.y());
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPt = &pt;
        }
    }

    if (closestPt && minDistSq < 800.0) {
        QPointF dataPt;
        if (currentMode == PlotMode::DragPolar) {
            dataPt = QPointF(closestPt->cd, closestPt->cl);
        } else {
            dataPt = QPointF(closestPt->alpha, (currentMode == PlotMode::LiftCurve) ? closestPt->cl : (currentMode == PlotMode::Efficiency ? closestPt->ld : closestPt->lift_N));
        }
        QPointF sPt = dataToScreen(dataPt, plotRect, minX, maxX, minY, maxY);

        painter.setPen(QPen(QColor(255, 255, 255, 100), 1, Qt::DashLine));
        painter.drawLine(QPointF(sPt.x(), plotRect.top()), QPointF(sPt.x(), plotRect.bottom()));
        painter.drawLine(QPointF(plotRect.left(), sPt.y()), QPointF(plotRect.right(), sPt.y()));

        painter.setBrush(Qt::white);
        painter.setPen(QPen(QColor("#00e5ff"), 2.5));
        painter.drawEllipse(sPt, 7, 7);

        QString text = QString("Angle of Attack: %1°\nCL (Lift Coeff): %2\nCD (Drag Coeff): %3\nL/D Efficiency: %4\nLift Force: %5 N\nDrag Force: %6 N")
                           .arg(closestPt->alpha, 0, 'f', 1)
                           .arg(closestPt->cl, 0, 'f', 4)
                           .arg(closestPt->cd, 0, 'f', 4)
                           .arg(closestPt->ld, 0, 'f', 2)
                           .arg(closestPt->lift_N, 0, 'f', 2)
                           .arg(closestPt->drag_N, 0, 'f', 2);

        QFont font = painter.font();
        font.setPointSize(8);
        painter.setFont(font);

        QRect textRect = painter.fontMetrics().boundingRect(QRect(0, 0, 240, 200), Qt::AlignLeft, text);
        textRect.adjust(-8, -6, 8, 6);

        int tipX = sPt.x() + 14;
        int tipY = sPt.y() - textRect.height() / 2;
        if (tipX + textRect.width() > plotRect.right()) {
            tipX = sPt.x() - textRect.width() - 14;
        }
        if (tipY < plotRect.top()) tipY = plotRect.top() + 6;
        if (tipY + textRect.height() > plotRect.bottom()) tipY = plotRect.bottom() - textRect.height() - 6;

        QRect boxRect(tipX, tipY, textRect.width(), textRect.height());
        painter.setBrush(QColor(18, 18, 28, 245));
        painter.setPen(QPen(QColor("#3e3e58"), 1.5));
        painter.drawRoundedRect(boxRect, 6, 6);

        painter.setPen(QColor("#f0f0ff"));
        painter.drawText(boxRect.adjusted(8, 6, -8, -6), Qt::AlignLeft, text);
    }
}

void PlotWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    painter.fillRect(rect(), QColor("#111118"));

    int marginLeft = 56;
    int marginRight = 20;
    int marginTop = 30;
    int marginBottom = 46;
    QRect plotRect(marginLeft, marginTop, width() - marginLeft - marginRight, height() - marginTop - marginBottom);

    if (plotRect.width() <= 10 || plotRect.height() <= 10) return;

    double minX = -4.0, maxX = 16.0;
    double minY = 0.0, maxY = 5.0;
    QString xLabel = "Angle of Attack a (deg)";
    QString yLabel = "Force Coefficient";

    if (currentMode == PlotMode::CpDistribution) {
        minX = -0.05;
        maxX = 1.05;
        minY = -1.2;
        maxY = 2.5;

        for (const auto& pt : cpData.upper) {
            minY = std::min(minY, -pt.cp);
            maxY = std::max(maxY, -pt.cp);
        }
        for (const auto& pt : cpData.lower) {
            minY = std::min(minY, -pt.cp);
            maxY = std::max(maxY, -pt.cp);
        }
        double padY = std::max(0.3, (maxY - minY) * 0.08);
        minY -= padY;
        maxY += padY;
        xLabel = "Chord Station x/c";
        yLabel = "-Cp (Suction ^)";
    } else if (!dataPoints.empty()) {
        if (currentMode == PlotMode::LiftCurve) {
            minX = dataPoints.front().alpha;
            maxX = dataPoints.front().alpha;
            minY = std::min(dataPoints.front().cl, dataPoints.front().cd);
            maxY = std::max(dataPoints.front().cl, dataPoints.front().cd);
            for (const auto& pt : dataPoints) {
                minX = std::min(minX, pt.alpha);
                maxX = std::max(maxX, pt.alpha);
                minY = std::min({minY, pt.cl, pt.cd});
                maxY = std::max({maxY, pt.cl, pt.cd});
            }
            double padX = std::max(0.5, (maxX - minX) * 0.05);
            double padY = std::max(0.2, (maxY - minY) * 0.08);
            minX -= padX; maxX += padX;
            minY = std::min(0.0, minY - padY);
            maxY += padY;
            xLabel = "Angle of Attack a (deg)";
            yLabel = "Aerodynamic Coefficients (CL, CD)";
        } else if (currentMode == PlotMode::DragPolar) {
            minX = dataPoints.front().cd;
            maxX = dataPoints.front().cd;
            minY = dataPoints.front().cl;
            maxY = dataPoints.front().cl;
            for (const auto& pt : dataPoints) {
                minX = std::min(minX, pt.cd);
                maxX = std::max(maxX, pt.cd);
                minY = std::min(minY, pt.cl);
                maxY = std::max(maxY, pt.cl);
            }
            double padX = std::max(0.1, (maxX - minX) * 0.08);
            double padY = std::max(0.2, (maxY - minY) * 0.08);
            minX = std::max(0.0, minX - padX); maxX += padX;
            minY -= padY; maxY += padY;
            xLabel = "Drag Coefficient CD";
            yLabel = "Lift Coefficient CL";
        } else if (currentMode == PlotMode::Efficiency) {
            minX = dataPoints.front().alpha;
            maxX = dataPoints.front().alpha;
            minY = dataPoints.front().ld;
            maxY = dataPoints.front().ld;
            for (const auto& pt : dataPoints) {
                minX = std::min(minX, pt.alpha);
                maxX = std::max(maxX, pt.alpha);
                minY = std::min(minY, pt.ld);
                maxY = std::max(maxY, pt.ld);
            }
            double padX = std::max(0.5, (maxX - minX) * 0.05);
            double padY = std::max(0.5, (maxY - minY) * 0.08);
            minX -= padX; maxX += padX;
            minY -= padY; maxY += padY;
            xLabel = "Angle of Attack a (deg)";
            yLabel = "Lift-to-Drag Ratio (L/D)";
        } else if (currentMode == PlotMode::Forces) {
            minX = dataPoints.front().alpha;
            maxX = dataPoints.front().alpha;
            minY = std::min(dataPoints.front().lift_N, dataPoints.front().drag_N);
            maxY = std::max(dataPoints.front().lift_N, dataPoints.front().drag_N);
            for (const auto& pt : dataPoints) {
                minX = std::min(minX, pt.alpha);
                maxX = std::max(maxX, pt.alpha);
                minY = std::min({minY, pt.lift_N, pt.drag_N});
                maxY = std::max({maxY, pt.lift_N, pt.drag_N});
            }
            double padX = std::max(0.5, (maxX - minX) * 0.05);
            double padY = std::max(1.0, (maxY - minY) * 0.08);
            minX -= padX; maxX += padX;
            minY = std::min(0.0, minY - padY);
            maxY += padY;
            xLabel = "Angle of Attack a (deg)";
            yLabel = "Force (Newtons)";
        }
    }

    drawGridAndAxes(painter, plotRect, minX, maxX, minY, maxY, xLabel, yLabel);

    if (currentMode == PlotMode::CpDistribution) {
        drawAirfoilThumbnail(painter, plotRect, minX, maxX, minY, maxY);

        std::vector<QPointF> upperPts, lowerPts;
        for (const auto& pt : cpData.upper) {
            upperPts.push_back(QPointF(pt.xc, -pt.cp));
        }
        for (const auto& pt : cpData.lower) {
            lowerPts.push_back(QPointF(pt.xc, -pt.cp));
        }
        drawCurve(painter, plotRect, lowerPts, minX, maxX, minY, maxY, QColor("#ff7043"), "Lower Surface");
        drawCurve(painter, plotRect, upperPts, minX, maxX, minY, maxY, QColor("#00e5ff"), "Upper Surface");
    } else if (currentMode == PlotMode::LiftCurve) {
        std::vector<QPointF> clPts, cdPts;
        for (const auto& pt : dataPoints) {
            clPts.push_back(QPointF(pt.alpha, pt.cl));
            cdPts.push_back(QPointF(pt.alpha, pt.cd));
        }
        drawCurve(painter, plotRect, cdPts, minX, maxX, minY, maxY, QColor("#ff7043"), "CD");
        drawCurve(painter, plotRect, clPts, minX, maxX, minY, maxY, QColor("#00e5ff"), "CL");
    } else if (currentMode == PlotMode::DragPolar) {
        std::vector<QPointF> polarPts;
        for (const auto& pt : dataPoints) {
            polarPts.push_back(QPointF(pt.cd, pt.cl));
        }
        drawCurve(painter, plotRect, polarPts, minX, maxX, minY, maxY, QColor("#00e676"), "CL vs CD");
    } else if (currentMode == PlotMode::Efficiency) {
        std::vector<QPointF> ldPts;
        for (const auto& pt : dataPoints) {
            ldPts.push_back(QPointF(pt.alpha, pt.ld));
        }
        drawCurve(painter, plotRect, ldPts, minX, maxX, minY, maxY, QColor("#76ff03"), "L/D");
    } else if (currentMode == PlotMode::Forces) {
        std::vector<QPointF> liftPts, dragPts;
        for (const auto& pt : dataPoints) {
            liftPts.push_back(QPointF(pt.alpha, pt.lift_N));
            dragPts.push_back(QPointF(pt.alpha, pt.drag_N));
        }
        drawCurve(painter, plotRect, dragPts, minX, maxX, minY, maxY, QColor("#ff5252"), "Drag (N)");
        drawCurve(painter, plotRect, liftPts, minX, maxX, minY, maxY, QColor("#00e5ff"), "Lift (N)");
    }

    drawMarkers(painter, plotRect, minX, maxX, minY, maxY);

    int legX = plotRect.right() - 130;
    int legY = 6;
    QRect legendBox(legX - 8, legY, 130, 20);
    painter.setBrush(QColor(22, 22, 32, 220));
    painter.setPen(QPen(QColor("#2a2a3e"), 1));
    painter.drawRoundedRect(legendBox, 4, 4);

    QFont legFont = painter.font();
    legFont.setPointSize(8);
    legFont.setBold(true);
    painter.setFont(legFont);

    if (currentMode == PlotMode::CpDistribution) {
        painter.setBrush(QColor("#00e5ff"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 6, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#00e5ff"));
        painter.drawText(legX + 14, legY + 14, "Upper (Suction)");

        painter.setBrush(QColor("#ff7043"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 68, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#ff7043"));
        painter.drawText(legX + 76, legY + 14, "Lower (Pres)");
    } else if (currentMode == PlotMode::LiftCurve) {
        painter.setBrush(QColor("#00e5ff"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 6, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#00e5ff"));
        painter.drawText(legX + 14, legY + 14, "CL (Lift)");

        painter.setBrush(QColor("#ff7043"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 68, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#ff7043"));
        painter.drawText(legX + 76, legY + 14, "CD (Drag)");
    } else if (currentMode == PlotMode::Forces) {
        painter.setBrush(QColor("#00e5ff"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 6, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#00e5ff"));
        painter.drawText(legX + 14, legY + 14, "Lift (N)");

        painter.setBrush(QColor("#ff5252"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 64, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#ff5252"));
        painter.drawText(legX + 72, legY + 14, "Drag (N)");
    } else if (currentMode == PlotMode::DragPolar) {
        painter.setBrush(QColor("#00e676"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 6, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#00e676"));
        painter.drawText(legX + 14, legY + 14, "Drag Polar");
    } else if (currentMode == PlotMode::Efficiency) {
        painter.setBrush(QColor("#76ff03"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(legX + 6, legY + 10), 3.5, 3.5);
        painter.setPen(QColor("#76ff03"));
        painter.drawText(legX + 14, legY + 14, "L/D Efficiency");
    }

    drawTooltip(painter, plotRect, minX, maxX, minY, maxY);
}

}
