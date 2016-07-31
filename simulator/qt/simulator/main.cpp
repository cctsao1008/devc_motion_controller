#include "simulator.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    simulator w;
    w.show();

    return a.exec();
}
