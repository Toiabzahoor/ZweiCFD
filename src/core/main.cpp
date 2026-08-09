#include "ZweiCFD/ui/main_window.hpp"
#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <QMessageBox>
#include <QDebug>

int main(int argc, char* argv[]) {

    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication app(argc, argv);

    try {
        zweicfd::MainWindow window;
        window.show();

        return app.exec();
    } catch (const std::exception& e) {
        qCritical() << "ZweiCFD failed to start:" << e.what();
        QMessageBox::critical(nullptr, "ZweiCFD failed to start",
            QString("ZweiCFD ran into a fatal error on startup:\n\n%1").arg(e.what()));
        return 1;
    }
}
