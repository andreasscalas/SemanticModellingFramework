/****************************************************************************
*   AnnotationTransfer                                                      *
*   Authors: Andreas Scalas                                                 *
*   Copyright (C) 2018  IMATI-GE / CNR                                      *
*   All rights reserved.                                                    *
*                                                                           *
*   This program is free software: you can redistribute it and/or modify    *
*   it under the terms of the GNU General Public License as published by    *
*   the Free Software Foundation, either version 3 of the License, or       *
*   (at your option) any later version.                                     *
*                                                                           *
*   This program is distributed in the hope that it will be useful,         *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of          *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
*   GNU General Public License for more details.                            *
*                                                                           *
*   You should have received a copy of the GNU General Public License       *
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
****************************************************************************/

#include <iostream>
#include <vector>
#include <imatistl.h>
#include <drawablemesh.h>
#include <string>
#include <string.h>
#include <chrono>
#include <fstream>
#include <Constraint.h>
#include <Solver.h>
#include <shapeopmeanvaluecoordinates.h>
#include <constraintsolver.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

using namespace std;
using namespace IMATI_STL;

int main(int argc, char *argv[]){

    static const int BUFFER_SIZE = 65536;
    if(argc < 2){
        std::cout<<"Missing template filename"<<endl<<flush;
        exit(1);
    }

    if(argc < 3) {
        std::cout<<"Missing cage filename"<<endl<<flush;
        exit(2);
    }
    if(argc < 4){
        std::cout<<"Missing constraints filename"<<endl<<flush;
        exit(3);
    }
    ImatiSTL::init();
    ExtendedTrimesh* templateMesh = new ExtendedTrimesh();/*
    ExtendedTrimesh* cageMesh = new ExtendedTrimesh();*/
    templateMesh->load(argv[1]);/*
    cageMesh->load(argv[2]);*/
    ShapeOp::MatrixX3 templatePoints;/*
    ShapeOp::MatrixX3 cagePoints;*/
    templatePoints.resize(templateMesh->V.numels(), 3);/*
    cagePoints.resize(cageMesh->V.numels(), 3);*/

    unsigned int counter = 0;
    for(IMATI_STL::Node* n = templateMesh->V.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        ShapeOp::Vector3 point = {v->x, v->y, v->z};
        templatePoints.row(counter++) = point;
    }
    ShapeOp::Matrix3X mpt = templatePoints.transpose();
//    counter = 0;
//    for(IMATI_STL::Node* n = cageMesh->V.head(); n != nullptr; n = n->next())
//    {
//        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
//        ShapeOp::Vector3 point = {v->x, v->y, v->z};
//        cagePoints.row(counter++) = point;
//    }

//    ShapeOpMeanValueCoordinates* coords = new ShapeOpMeanValueCoordinates(templateMesh, cageMesh, &templatePoints, &cagePoints);
//    coords->loadCoordinates("coords.coord");
//    ShapeOp::SparseMatrix* shapeOpBC = coords->getCoordinates();

    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    ShapeOp::Solver* solver = new ShapeOp::Solver();
    solver->setPoints(mpt);
//    ConstraintSolver* cageSolver = new ConstraintSolver();
//    cageSolver->setModelPointsTransposed(mpt);
//    cageSolver->setCagePoints(&cagePoints);
//    cageSolver->setBarycentricCoordinates(shapeOpBC);
    ifstream constraintsFile;
    constraintsFile.open(argv[3]);

    FILE* fp = fopen(argv[3],"r");
    char buffer[BUFFER_SIZE];
    rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

    rapidjson::Document document;
    if(!(document.ParseStream(frs).HasParseError())){
        if(document.HasMember("constraints") && document["constraints"].IsArray()){
            rapidjson::Value& contraintsList = document["constraints"];
            for (rapidjson::SizeType i = 0; i < contraintsList.Size(); i++) {
                rapidjson::Value& jsonConstraint = contraintsList[i];
                if(jsonConstraint.IsObject()){
                    assert(jsonConstraint.HasMember("type"));
                    assert(jsonConstraint["type"].IsString());
                    string type = jsonConstraint["type"].GetString();
                    vector<int> ids;
                    if(jsonConstraint.HasMember("ids")){
                        rapidjson::Value& jsonIDs = jsonConstraint["ids"];
                        assert(jsonIDs.IsArray());
                        for(unsigned int j = 0; j < jsonIDs.Size(); j++){
                            assert(jsonIDs[j].IsInt());
                            int id = jsonIDs[j].GetInt();
                            if(id < templateMesh->V.numels())
                                ids.push_back(id);
                        }
                    }

                    assert(jsonConstraint.HasMember("weight"));
                    assert(jsonConstraint["weight"].IsDouble());
                    double weight = jsonConstraint["weight"].GetDouble();
                    auto c = ShapeOp::Constraint::shapeConstraintFactory(type, ids, weight, mpt);
                    constraints.push_back(c);
                    solver->addConstraint(c);
//                    cageSolver->addConstraint(c);
                }else
                    exit(6);
            }
        }else
            exit(5);
    }else
        exit(4);


//    cageSolver->initialize();
//    std::cout << "Our solver initialised!" << std::endl << std::flush;
//    auto start = std::chrono::high_resolution_clock::now();
//    cageSolver->solve(50);
//    auto end = std::chrono::high_resolution_clock::now();
//    double seconds = 0;
//    long int millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//    seconds = static_cast<double>(static_cast<double>(millis) / 1000.0);
//    std::cout << "Our solution requires " << seconds << " seconds" << std::endl << std::flush;

    solver->initialize();
    std::cout << "Canonical solver initialised!" << std::endl << std::flush;
    auto start = std::chrono::high_resolution_clock::now();
    solver->solve(50);
    auto end = std::chrono::high_resolution_clock::now();
    double seconds = 0;
    long int millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    seconds = static_cast<double>(static_cast<double>(millis) / 1000.0);
    std::cout << "Canonical solution requires " << seconds << " seconds" << std::endl << std::flush;


}


