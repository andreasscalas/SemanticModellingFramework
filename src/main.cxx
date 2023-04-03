#include "mainwindow.h"
#include <QApplication>
#include <random>
#include <fstream>
#include <utilities.h>

int main(int argc, char *argv[]){


    IMATI_STL::ImatiSTL::app_name = "andreaslib";
    IMATI_STL::ImatiSTL::app_year = "2020";
    IMATI_STL::ImatiSTL::app_authors = "Andreas Scalas";
    IMATI_STL::ImatiSTL::app_version = "1.0";
    IMATI_STL::ImatiSTL::init();


//    ExtendedTrimesh* mesh = new ExtendedTrimesh();
//    if(argc > 1)
//    {
//        std::cout << argv[1];
//        mesh->load(argv[1]);
////        std::vector<IMATI_STL::Vertex*> vertices;
////        for(IMATI_STL::Node* n = mesh->V.head(); n != nullptr; n = n->next())
////            vertices.push_back(static_cast<IMATI_STL::Vertex*>(n->data));

//        //std::map<IMATI_STL::Edge*, double> weights = Utilities::computeLaplacianWeights(vertices, mesh, Utilities::WeightType::COTANGENT);
//        //Utilities::smoothRegion(vertices, weights, 1000);
////        mesh->smoothMesh(10, ExtendedTrimesh::WeightType::COTANGENT);
////        mesh->save("smoothed.ply");
//    }
    srand(static_cast<uint>(time(nullptr)));
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();

}
