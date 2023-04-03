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

    ImatiSTL::init();
    DrawableMesh* templateMesh = new DrawableMesh();
    DrawableMesh* cageMesh = new DrawableMesh();
    templateMesh->load(argv[1]);
    cageMesh->load(argv[2]);
    ShapeOp::MatrixX3 templatePoints;
    ShapeOp::MatrixX3 cagePoints;
    templatePoints.resize(templateMesh->V.numels(), 3);
    cagePoints.resize(cageMesh->V.numels(), 3);

    unsigned int counter = 0;
    for(IMATI_STL::Node* n = templateMesh->V.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        ShapeOp::Vector3 point = {v->x, v->y, v->z};
        templatePoints.row(counter++) = point;
    }
    ShapeOp::Matrix3X mpt = templatePoints.transpose();
    counter = 0;
    for(IMATI_STL::Node* n = cageMesh->V.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        ShapeOp::Vector3 point = {v->x, v->y, v->z};
        cagePoints.row(counter++) = point;
    }

    ShapeOpMeanValueCoordinates* coords = new ShapeOpMeanValueCoordinates(templateMesh, cageMesh, &templatePoints, &cagePoints);
    coords->computeCoordinates();
    coords->saveCoordinates("coords.coord");
	exit(0);
}
