#include "ZweiCFD/ui/main_window.hpp"
#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <QMessageBox>
#include <QDebug>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

int main(int argc, char* argv[]) {

    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QSurfaceFormat format = QVTKOpenGLNativeWidget::defaultFormat();
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

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
