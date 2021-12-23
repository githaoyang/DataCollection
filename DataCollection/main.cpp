#include "DataCollection.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DataCollection w;
    w.show();
    return a.exec();
}
