#include "ZweiCFD/ui/main_window.hpp"
#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char* argv[]) {
    
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication app(argc, argv);
    
    zweicfd::MainWindow window;
    window.show();
    
    return app.exec();
}
