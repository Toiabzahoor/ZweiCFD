#include "ZweiCFD/ui/main_window.hpp"
#include "ZweiCFD/core/cli.hpp"
#include "ZweiCFD/core/config.hpp"
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
    auto cliOpt = zweicfd::parseCLI(argc, argv);

    if (cliOpt.help) {
        zweicfd::printHelp(argv[0]);
        return 0;
    }

    zweicfd::Config config;
    try {
        config = zweicfd::ConfigLoader::load("assets/config.json");
    } catch (...) {
    }

    if (cliOpt.headless || cliOpt.isCliMode) {
        return zweicfd::runHeadlessCLI(cliOpt, config);
    }

    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QSurfaceFormat format = QVTKOpenGLNativeWidget::defaultFormat();
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);

    try {
        zweicfd::MainWindow window(&cliOpt);
        window.show();

        return app.exec();
    } catch (const std::exception& e) {
        qCritical() << "ZweiCFD failed to start:" << e.what();
        QMessageBox::critical(nullptr, "ZweiCFD failed to start",
            QString("ZweiCFD ran into a fatal error on startup:\n\n%1").arg(e.what()));
        return 1;
    }
}
