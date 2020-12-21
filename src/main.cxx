#include "mainwindow.h"
#include <QApplication>
#include <random>
#include <fstream>

int main(int argc, char *argv[]){

    IMATI_STL::ImatiSTL::app_name = "andreaslib";
    IMATI_STL::ImatiSTL::app_year = "2020";
    IMATI_STL::ImatiSTL::app_authors = "Andreas Scalas";
    IMATI_STL::ImatiSTL::app_version = "1.0";
    IMATI_STL::ImatiSTL::init();


    srand(static_cast<uint>(time(nullptr)));
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();

}
