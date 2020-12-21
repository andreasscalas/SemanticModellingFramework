/****************************************************************************
* DrawableTrimesh                                                           *
*                                                                           *
* Consiglio Nazionale delle Ricerche                                        *
* Istituto di Matematica Applicata e Tecnologie Informatiche                *
* Sezione di Genova                                                         *
* IMATI-GE / CNR                                                            *
*                                                                           *
* Authors: Andreas Scalas                                                   *
* Copyright(C) 2013: IMATI-GE / CNR                                         *
* All rights reserved.                                                      *
*                                                                           *
* This program is dual-licensed as follows:                                 *
*                                                                           *
* (1) You may use ImatiSTL as free software; you can redistribute it and/or *
* modify it under the terms of the GNU General Public License as published  *
* by the Free Software Foundation; either version 3 of the License, or      *
* (at your option) any later version.                                       *
* In this case the program is distributed in the hope that it will be       *
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of    *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
* (2) You may use DrawableTrimesh as part of a commercial software. In this *
* case a proper agreement must be reached with the Authors and with         *
* IMATI-GE/CNR based on a proper licensing contract.                        *
****************************************************************************/

#include "drawablemesh.h"
using namespace std;
using namespace IMATI_STL;

DrawableMesh::DrawableMesh() : ExtendedTrimesh() {

    this->drawWireframe = false;
    this->drawable = true;
    this->meshModified = false;
    this->annotationsModified = false;
    this->isCage = false;
    this->vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    this->meshPoints = vtkSmartPointer<vtkPoints>::New();
    this->meshTriangles = vtkSmartPointer<vtkCellArray>::New();
    this->annotatedTriangles = vtkSmartPointer<vtkCellArray>::New();
    this->meshVColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->meshTColors= vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->annotationsTColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->meshActor = vtkSmartPointer<vtkActor>::New();
    this->canvas = vtkSmartPointer<vtkAssembly>::New();

}

void DrawableMesh::draw(vtkSmartPointer<vtkAssembly> assembly) const{

    assembly->RemovePart(this->canvas);
    this->canvas = vtkSmartPointer<vtkAssembly>::NewInstance(this->canvas);

    //If the mesh have to be visualized
    if(drawable){

        VTK_CREATE(vtkPolyDataMapper, mapper);
        this->vtkMesh->GetCellData()->SetScalars(meshTColors);
        mapper->SetInputData(this->vtkMesh);
        meshActor = vtkSmartPointer<vtkActor>::NewInstance(meshActor);
        meshActor->SetMapper(mapper);
        if(drawWireframe)
            meshActor->GetProperty()->SetRepresentationToWireframe();

        if(isCage){
            if(!drawWireframe)
                meshActor->GetProperty()->SetOpacity(0.5);
            VTK_CREATE(vtkPolyDataMapper, pointMapper);
            VTK_CREATE(vtkPolyData, pointPolydata);
            VTK_CREATE(vtkActor, pointActor);
            pointPolydata->SetPoints(this->meshPoints);
            pointPolydata->SetPolys(this->vtkMesh->GetPolys());
            pointPolydata->GetPointData()->SetScalars(meshVColors);
            pointMapper->SetInputData(pointPolydata);
            pointActor->SetMapper(pointMapper);
            pointActor->GetProperty()->SetRepresentationToPoints();
            pointActor->GetProperty()->SetPointSize(10.0f);
            canvas->AddPart(pointActor);
        }

        if(annotations.size() > 0){
            VTK_CREATE(vtkPolyData, annotationsPolydata);
            annotationsPolydata->SetPoints(meshPoints);
            annotationsPolydata->SetPolys(this->annotatedTriangles);
            annotationsPolydata->GetCellData()->SetScalars(annotationsTColors);
            VTK_CREATE(vtkPolyDataMapper, annotationMapper);
            VTK_CREATE(vtkActor, annotationActor);
            annotationMapper->SetInputData(annotationsPolydata);
            annotationActor->GetProperty()->SetOpacity(0.5);
            annotationActor->SetMapper(annotationMapper);
            canvas->AddPart(annotationActor);
        }

        canvas->AddPart(meshActor);
        canvas->Modified();
        assembly->AddPart(canvas);
        assembly->Modified();

    }

}

int DrawableMesh::load(const char* filename){

    int loadResult = TriMesh::load(filename);

    if(loadResult == 0){

        this->filename = filename;
        buildVTKStructure();
    }

    return loadResult;

}

void DrawableMesh::update(){

    //If the mesh has beend deformed...
    if(this->meshModified)

        for(Node* n = V.head(); n!=NULL; n=n->next()){
            long int tmp;
            Vertex* v = (Vertex*) n->data;
            tmp = verticesId[v];
            meshPoints->SetPoint(tmp, v->x, v->y, v->z);
        }

    //If the annotations has beend edited...
    if(this->annotationsModified){

        this->annotationsTColors = vtkSmartPointer<vtkUnsignedCharArray>::NewInstance(this->annotationsTColors);
        this->annotationsTColors ->SetNumberOfComponents(3);
        this->annotationsTColors ->SetName ("AnnotationsTColors");
        this->annotatedTriangles = vtkSmartPointer<vtkCellArray>::NewInstance(this->annotatedTriangles);

        /*For each annotation defined on the mesh and For each outline of the annotation selection, we check if the triangle on the left of the first edge of the outline has been
         * inserted in the list of inner triangles. If not, we perform the region growing inside of that outline and  we insert all the
         * inner triangles in the list (this allow for multiple patches for each annotation) */
        for(std::vector<Annotation*>::iterator ait = this->annotations.begin(); ait != this->annotations.end(); ait++){

            Annotation* annotation = (Annotation*) *ait;
            vector<vector<Vertex*> > outlines = annotation->getOutlines();
            std::vector<Triangle*> annotationTriangles;

            for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){

                vector<Vertex*> outline = (vector<Vertex*>) *oit;
                Triangle* as = outline[0]->getEdge(outline[1])->leftTriangle(outline[0]);
                if(as->info == nullptr || *static_cast<int*>(as->info) != INSIDE){

                    vector<Triangle*> selectedTriangles = Utilities::regionGrowing(outlines);
                    for(vector<Triangle*>::iterator tit = selectedTriangles.begin(); tit != selectedTriangles.end(); tit++){
                        Triangle* t = (Triangle*) *tit;
                        t->info = &INSIDE;
                        annotationTriangles.push_back(t);
                    }

                }
            }

            //Update of the data-visualization linking
            for(std::vector<Triangle*>::iterator tit = annotationTriangles.begin(); tit != annotationTriangles.end(); tit++){
                this->annotatedTriangles->InsertNextCell(this->vtkMesh->GetCell(this->trianglesId[*tit]));
                this->annotationsTColors->InsertNextTypedTuple(annotation->getColor());
            }

            for(vector<Triangle*>::iterator tit = annotationTriangles.begin(); tit != annotationTriangles.end(); tit++)
                (*tit)->info = nullptr;

        }


    }

    if(this->colorModified){
        //The colors of the triangles are updated
        this->vtkMesh->GetCellData()->SetScalars(this->meshTColors);
        if(this->isCage)
            this->vtkMesh->GetPointData()->SetScalars(this->meshVColors);
    }

    this->meshModified = false;
    this->colorModified = false;
    this->annotationsModified = false;

}

void DrawableMesh::buildVTKStructure(){

    Vertex *v;
    Triangle* t;
    Node *n;
    meshPoints = vtkSmartPointer<vtkPoints>::New();
    meshTriangles = vtkSmartPointer<vtkCellArray>::New();
    meshVColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    meshVColors->SetNumberOfComponents(3);
    meshVColors->SetName ("Colors");
    meshTColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    meshTColors->SetNumberOfComponents(3);
    meshTColors->SetName ("TColors");
    long int i = 0;

    FOREACHVERTEX(v,n){
        this->idVertices[i] = (long int) v;
        this->verticesId[v] = i++;
        this->meshPoints->InsertNextPoint(v->x, v->y, v->z);
        meshVColors->InsertNextTypedTuple(GRAY);
    }

    i = 0;

    FOREACHTRIANGLE(t,n){
        Vertex *v1, *v2, *v3;
        v1 = t->v1();
        v2 = t->v2();
        v3 = t->v3();
        idTriangles[i] = (long int) t;
        trianglesId[t] = i++;
        VTK_CREATE(vtkTriangle, triangle);
        triangle->GetPointIds()->SetNumberOfIds(3);
        triangle->GetPointIds()->SetId(0, verticesId[v1]);
        triangle->GetPointIds()->SetId(1, verticesId[v2]);
        triangle->GetPointIds()->SetId(2, verticesId[v3]);
        this->meshTriangles->InsertNextCell(triangle);
        meshTColors->InsertNextTypedTuple(GRAY);
    }

    this->vtkMesh->SetPoints(this->meshPoints);
    this->vtkMesh->SetPolys(this->meshTriangles);
}

bool DrawableMesh::getIsCage() const
{
    return this->isCage;
}

void DrawableMesh::setIsCage(bool value)
{
    this->isCage = value;
}

bool DrawableMesh::getDrawWireframe() const
{
    return this->drawWireframe;
}

void DrawableMesh::setDrawWireframe(bool value)
{
    this->drawWireframe = value;
}

bool DrawableMesh::getDrawable() const
{
    return this->drawable;
}

void DrawableMesh::setDrawable(bool value)
{
    this->drawable = value;
}

bool DrawableMesh::getMeshModified() const
{
    return this->meshModified;
}

void DrawableMesh::setMeshModified(bool value)
{
    this->meshModified = value;
}

bool DrawableMesh::getColorModified() const
{
    return this->colorModified;
}

void DrawableMesh::setColorModified(bool value)
{
    this->colorModified = value;
}

bool DrawableMesh::getAnnotationsModified() const
{
    return this->annotationsModified;
}

void DrawableMesh::setAnnotationsModified(bool value)
{
    this->annotationsModified = value;
}

vtkSmartPointer<vtkPoints> DrawableMesh::getPoints(){
    return this->meshPoints;
}

vtkSmartPointer<vtkCellArray> DrawableMesh::getTriangles(){
    return this->meshTriangles;
}

void DrawableMesh::setPointPosition(vtkIdType pid, double* p){
    Vertex* v = idVertices[pid];
    v->setValue(p[0], p[1], p[2]);
    this->meshPoints->SetPoint(pid, p);
    this->meshPoints->Modified();
}

void DrawableMesh::setSelectedPoints(std::map<long, bool> selectedPoints){

    for(long int i = 0; i < V.numels(); i++){

        if(selectedPoints[i])
            this->meshVColors->SetTypedTuple(i, RED);
        else
            this->meshVColors->SetTypedTuple(i, BLUE);

    }

}

void DrawableMesh::setSelectedTriangles(std::map<long, bool> selectedTriangles){

    for(std::map<long, bool>::iterator iit = selectedTriangles.begin(); iit != selectedTriangles.end(); iit++){
        std::pair<long, bool> p = *iit;
        if(p.second)
            this->meshTColors->SetTypedTuple(p.first, RED);
        else
            this->meshTColors->SetTypedTuple(p.first, GRAY);

    }

}

void DrawableMesh::setColor(int r, int g, int b){

    Node* n;
    Triangle* t;

    FOREACHTRIANGLE(t, n){
        unsigned char color[3] = {r,g,b};
        this->meshTColors->SetTypedTuple(trianglesId[t], color);
    }

}

vtkSmartPointer<vtkActor> DrawableMesh::getMeshActor() const{
    return meshActor;
}

vtkSmartPointer<vtkAssembly> DrawableMesh::getCanvas() const
{
    return this->canvas;
}

void DrawableMesh::setCanvas(const vtkSmartPointer<vtkAssembly> &value)
{
    this->canvas = value;
}

string DrawableMesh::getFilename() const
{
    return filename;
}

