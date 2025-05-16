#include <QCoreApplication>
#include "robotcontrol.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    app.setApplicationName("ESC Balance Robot");
    app.setApplicationVersion("1.0");
    RobotControl robotControl;
    return app.exec();
}
