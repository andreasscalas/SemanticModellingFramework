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
#include <annotation.h>
#include <annotationfilemanager.h>
#include <extendedtrimesh.h>
#include <annotationutilities.h>
#include <string>
#include <string.h>
#include <chrono>
#include <fstream>

using namespace std;
using namespace IMATI_STL;

int main(int argc, char *argv[]){

    ImatiSTL::init();
    ExtendedTrimesh* source = new ExtendedTrimesh();
    ExtendedTrimesh* target = new ExtendedTrimesh();

    if(argc < 2){
        std::cout<<"Missing source filename"<<endl<<flush;
        exit(1);
    }

    if(argc < 3) {
        std::cout<<"Missing target filename"<<endl<<flush;
        exit(2);
    }
    if(argc < 4){
        std::cout<<"Missing source annotation file"<<endl<<flush;
        exit(3);
    }

    string sourceFilename(argv[1]);
    string targetFilename(argv[2]);
    string sourceAnnotationFilename(argv[3]);
    string targetAnnotationFilename;
    vector<Annotation*> sourceAnnotations;
    vector<Annotation*> targetAnnotations;

    if(argc < 5)
        targetAnnotationFilename = "output.ant";
    else
        targetAnnotationFilename = argv[4];

    source->load(sourceFilename);
    target->load(targetFilename);
    ofstream file;
    file.open("target_points.m");
    if(file.is_open())
    {
        for(IMATI_STL::Node* n = target->V.head(); n != nullptr; n = n->next())
        {
            IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
            file << v->x << " " << v->y << " " << v->z << std::endl;
        }
        file.close();
    }


    AnnotationFileManager manager1;
    manager1.setMesh(source);
    if(!manager1.readAnnotations(sourceAnnotationFilename))
        std::cout<<"Something went wrong during first annotation file opening."<< std::endl<< std::flush;


    sourceAnnotations = source->getAnnotations();
    for(short m = 2; m <= 2; m++){

        for(vector<Annotation*>::iterator it = sourceAnnotations.begin(); it != sourceAnnotations.end(); it++){
            Annotation* annotation = static_cast<Annotation*>(*it);
            long int duration;
            auto start = std::chrono::high_resolution_clock::now();

            //if(source->V.numels() > target->V.numels() / 10)
                targetAnnotations.push_back(annotation->transfer(target, Utilities::EUCLIDEAN_DISTANCE));
            /*else
                targetAnnotations.push_back(annotation->parallelTransfer(target, Utilities::EUCLIDEAN_DISTANCE));*/
            /*double oldArea = a->getArea();
            double newArea = targetAnnotations.back()->getArea();
            double oldPerimeter = a->getPerimeter();
            double newPerimeter = targetAnnotations.back()->getPerimeter();
            double oldP_A = oldPerimeter/oldArea;
            double newP_A = newPerimeter/newArea;
            double oldA_P = oldArea/oldPerimeter;
            double newA_P = newArea/newPerimeter;
            double oldCircularity = pow(oldPerimeter, 2)/oldArea;
            double newCircularity = pow(newPerimeter, 2)/newArea;
            //cout<<"Old area: "<<oldArea<<" New area: "<<newArea<<endl;*/
            auto end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();



            if(strcmp(argv[argc - 1], "-t") == 0){
                int secs = 0;

                if(duration > 1000){
                    secs = static_cast<int>(duration / 1000);
                    duration = duration % 1000;
                }

                cout << "The trasfer of annotation \""<<annotation->getTag()<<"\" took " << secs << "." << duration << " seconds" << endl << flush;
            }

            /*cout<<abs((1 - oldArea/newArea)*100)<<"%, ";
            cout<<abs((1 - oldPerimeter/newPerimeter)*100)<<"%, ";
            cout<<abs((1 - oldP_A/newP_A)*100)<<"%, ";
            cout<<abs((1 - oldA_P/newA_P)*100)<<"%, ";
            cout<<abs((1 - oldCircularity/newCircularity)*100)<<"%";
            if(m != 3)
                cout<<", ";*/
            /*if(oldArea < newArea)
                cout<<"The new annotation is "<<(1 - oldArea/newArea)*100 <<"% larger than the original one\n" ;
            else
                cout<<"The new annotation is "<<(1 - newArea/oldArea)*100 <<"% smaller than the original one\n" ;*/
        }
    }

    target->setAnnotations(targetAnnotations);
    AnnotationFileManager manager2;
    manager2.setMesh(target);
    if(!manager2.writeAnnotations(targetAnnotationFilename))
        std::cout<<"Something went wrong during second annotation file writing."<< std::endl<< std::flush;

}


