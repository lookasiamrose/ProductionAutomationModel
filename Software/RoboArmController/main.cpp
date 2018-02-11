#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication qapp(argc, argv);

    MainWindow window;
    window.show();

    return qapp.exec();
}
