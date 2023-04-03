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
#include <annotationfilemanager.h>
#include <constraintsfilemanager.h>
#include <rapidjson/filereadstream.h>

using namespace std;
using namespace IMATI_STL;

int main(int argc, char *argv[]){

    if(argc < 2){
        std::cout<<"Missing template filename"<<endl<<flush;
        exit(1);
    }

    if(argc < 3) {
        std::cout<<"Missing annotation filename"<<endl<<flush;
        exit(2);
    }

    ImatiSTL::init();
    DrawableMesh* templateMesh = new DrawableMesh();
    templateMesh->load(argv[1]);
    AnnotationFileManager manager;
    manager.setMesh(templateMesh);
    if(!manager.readAnnotations(argv[2]))
        std::cout << "Something went wrong during annotation file opening." << std::endl << std::flush;
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    ShapeOp::MatrixX3 templatePoints;
    templatePoints.resize(templateMesh->V.numels(), 3);
    unsigned int counter = 0;
    for(IMATI_STL::Node* n = templateMesh->V.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        ShapeOp::Vector3 point = {v->x, v->y, v->z};
        templatePoints.row(counter++) = point;
    }
    ShapeOp::Matrix3X mpt = templatePoints.transpose();
    for(IMATI_STL::Node* n = templateMesh->V.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        v->info = nullptr;
        std::vector<int> ids = {static_cast<int>(templateMesh->getPointId(v))};
        constraints.push_back(ShapeOp::Constraint::shapeConstraintFactory("Closeness", ids, 1, mpt));
    }
    std::vector<Annotation*> annotations = templateMesh->getAnnotations();

    for(unsigned int i = 0; i < annotations.size(); i++)
    {
        if(annotations[i]->getTag().compare("Planiccia") == 0)
        {
            std::vector<IMATI_STL::Vertex*> annotationVertices = annotations[i]->getInvolvedVertices();
            std::vector<int> ids;
            for(unsigned int j = 0; j < annotationVertices.size(); j++)
                ids.push_back(templateMesh->getPointId(annotationVertices[j]));
            constraints.push_back(ShapeOp::Constraint::shapeConstraintFactory("Plane", ids, 1, mpt));
        }
    }

    ConstraintsFileManager manager2;
    manager2.setMesh(templateMesh);
    manager2.setSoPoints(&templatePoints);
    manager2.setConstraints(constraints);
    manager2.writeConstraints("constraints.cstr");
	exit(0);
}
