#include <QApplication>
#include "PointCloudViewer.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    PointCloudViewer viewer;
    viewer.show();
    
    return app.exec();
}
