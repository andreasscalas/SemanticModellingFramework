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
#include <utilities.h>
#include <drawableannotation.h>
#include <vtkCellData.h>
#include <vtkTriangle.h>
#include <vtkPointData.h>
#include <drawableattribute.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkLine.h>

using namespace std;
using namespace IMATI_STL;

DrawableMesh::DrawableMesh() : ExtendedTrimesh() {
    init();
}

DrawableMesh::DrawableMesh(DrawableMesh *m) : ExtendedTrimesh (m)
{
    init();

    buildVTKStructure();
    meshModified = true;
    annotationsModified = true;
    update();
}

DrawableMesh::DrawableMesh(ExtendedTrimesh *m) : ExtendedTrimesh (m)
{
    init();

    buildVTKStructure();
    meshModified = true;
    annotationsModified = true;
    update();
}

DrawableMesh::~DrawableMesh()
{
}

void DrawableMesh::init()
{
    this->drawSurface = true;
    this->drawWireframe = false;
	this->drawPoints = false;
    this->drawAnnotations = true;
    this->drawable = true;
    this->meshModified = false;
    this->annotationsModified = false;
    this->isCage = false;
    this->meshPoints = vtkSmartPointer<vtkPoints>::New();
    this->meshEdges = vtkSmartPointer<vtkCellArray>::New();
    this->meshTriangles = vtkSmartPointer<vtkCellArray>::New();
    this->meshVColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->meshEColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->meshTColors= vtkSmartPointer<vtkUnsignedCharArray>::New();
    this->meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
	this->meshPointsActor = vtkSmartPointer<vtkActor>::New();
	this->meshWireframeActor = vtkSmartPointer<vtkActor>::New();
    this->canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

void DrawableMesh::draw(vtkSmartPointer<vtkPropAssembly> assembly){

    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();

    //If the mesh have to be visualized
    if(drawable){
        vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkMesh->SetPoints(meshPoints);
        vtkMesh->SetPolys(meshTriangles);
        vtkMesh->GetCellData()->SetScalars(meshTColors);
        mapper->SetInputData(vtkMesh);
        meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
        meshSurfaceActor->SetMapper(mapper);
        meshSurfaceActor->GetProperty()->SetRepresentationToSurface();
        if(isCage)
            meshSurfaceActor->GetProperty()->SetOpacity(0.5);
        if(drawSurface)
            canvas->AddPart(meshSurfaceActor);

        if(drawAnnotations)
            for (unsigned int i = 0; i < annotations.size(); i++)
                dynamic_cast<DrawableAnnotation*>(annotations[i])->draw(canvas);


        vtkSmartPointer<vtkPolyData> wireframePolydata = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyDataMapper> wireframeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        wireframePolydata->SetPoints(this->meshPoints);
        wireframePolydata->SetLines(this->meshEdges);
        wireframePolydata->GetCellData()->SetScalars(meshEColors);
        wireframeMapper->SetInputData(wireframePolydata);
        meshWireframeActor = vtkSmartPointer<vtkActor>::New();
        meshWireframeActor->SetMapper(wireframeMapper);
        meshWireframeActor->GetProperty()->SetRepresentationToWireframe();

        if(isCage){
            meshWireframeActor->GetProperty()->SetColor(1, 1, 1);
            meshWireframeActor->GetProperty()->SetOpacity(0.5);
        } else {
            meshWireframeActor->GetProperty()->SetOpacity(1.0);
            meshWireframeActor->GetProperty()->SetLineWidth(3.0);
        }
        if (drawWireframe)
            canvas->AddPart(meshWireframeActor);


        vtkSmartPointer<vtkPolyData> pointPolydata = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        meshPointsActor = vtkSmartPointer<vtkActor>::New();
        pointPolydata->SetPoints(this->meshPoints);
        pointPolydata->SetPolys(this->meshTriangles);
        pointPolydata->GetPointData()->SetScalars(meshVColors);
        pointMapper->SetInputData(pointPolydata);
        meshPointsActor->SetMapper(pointMapper);
        if(isCage){
            meshPointsActor->GetProperty()->SetPointSize(10.0f);
        } else {
            meshPointsActor->GetProperty()->SetPointSize(5.0f);
        }
        meshPointsActor->GetProperty()->SetRepresentationToPoints();
        if (drawPoints)
            canvas->AddPart(meshPointsActor);

        canvas->Modified();
        assembly->AddPart(canvas);
        assembly->Modified();

    }


}

int DrawableMesh::load(const char* filename){

    int loadResult = ExtendedTrimesh::load(filename);

    if(loadResult == 0){

        buildVTKStructure();
    }

    return loadResult;

}

void DrawableMesh::update(){

    //If the mesh has beend changed...
    if(this->meshModified){
        if(!this->onlyPointsPositionsModified) {
            if(static_cast<unsigned int>(V.numels()) != verticesId.size() || static_cast<unsigned int>(V.numels()) != idVertices.size() ||
               static_cast<unsigned int>(E.numels()) != edgesId.size() || static_cast<unsigned int>(E.numels()) != idEdges.size() ||
               static_cast<unsigned int>(T.numels()) != trianglesId.size() || static_cast<unsigned int>(T.numels()) != idTriangles.size())
                buildInnerStructure();
        }
        for(IMATI_STL::Node* n = V.head(); n != nullptr; n=n->next())
        {
            Vertex* v = static_cast<Vertex*>(n->data);
            meshPoints->SetPoint(static_cast<vtkIdType>(getPointId(v)), v->x, v->y, v->z);
        }
        meshPoints->Modified();
        for(unsigned int i = 0; i < annotations.size(); i++)
            for(unsigned int j = 0; j < annotations[i]->getAttributes().size(); j++)
                dynamic_cast<DrawableAttribute*>(annotations[i]->getAttributes()[j])->update();

    }

    //If the annotations have beend edited...
    if(this->annotationsModified)
        for(unsigned int i = 0; i < annotations.size(); i++)
            dynamic_cast<DrawableAnnotation*>(annotations[i])->update();

    this->meshModified = false;
    this->annotationsModified = false;

}

void DrawableMesh::buildVTKStructure(){

    Vertex *v;
    Edge* e;
    Triangle* t;
    IMATI_STL::Node *n;
    meshPoints =    vtkSmartPointer<vtkPoints>::New();
    meshEdges =     vtkSmartPointer<vtkCellArray>::New();
    meshTriangles = vtkSmartPointer<vtkCellArray>::New();
    meshVColors =   vtkSmartPointer<vtkUnsignedCharArray>::New();
    meshVColors->SetNumberOfComponents(3);
    meshVColors->SetName("PColors");
    meshEColors =   vtkSmartPointer<vtkUnsignedCharArray>::New();
    meshEColors->SetNumberOfComponents(3);
    meshEColors->SetName("LColors");
    meshTColors =   vtkSmartPointer<vtkUnsignedCharArray>::New();
    meshTColors->SetNumberOfComponents(3);
    meshTColors->SetName ("TColors");
    unsigned long i = 0;

    for (n = V.head(); n != nullptr; n = n->next()){
        v = static_cast<Vertex*>(n->data);
        this->idVertices[i] = v;
        this->verticesId[v] = i++;
        this->meshPoints->InsertNextPoint(v->x, v->y, v->z);
        meshVColors->InsertNextTypedTuple(BLUE);
    }

    i = 0;

    for (n = E.head(); n != nullptr; n = n->next()) {
        e = static_cast<Edge*>(n->data);
        idEdges[i] = e;
        edgesId[e] = i++;
        vtkSmartPointer<vtkLine> edge = vtkSmartPointer<vtkLine>::New();
        edge->GetPointIds()->SetNumberOfIds(2);
        edge->GetPointIds()->SetId(0, static_cast<vtkIdType>(verticesId[e->v1]));
        edge->GetPointIds()->SetId(1, static_cast<vtkIdType>(verticesId[e->v2]));
        this->meshEdges->InsertNextCell(edge);
        this->meshEColors->InsertNextTypedTuple(BLACK);
    }

    i = 0;

    for(n = T.head(); n != nullptr; n = n->next()){
        t = static_cast<Triangle*>(n->data);
        Vertex *v1, *v2, *v3;
        v1 = t->v1();
        v2 = t->v2();
        v3 = t->v3();
        idTriangles[i] = t;
        trianglesId[t] = i++;
        vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
        triangle->GetPointIds()->SetNumberOfIds(3);
        triangle->GetPointIds()->SetId(0, static_cast<vtkIdType>(verticesId[v1]));
        triangle->GetPointIds()->SetId(1, static_cast<vtkIdType>(verticesId[v2]));
        triangle->GetPointIds()->SetId(2, static_cast<vtkIdType>(verticesId[v3]));
        this->meshTriangles->InsertNextCell(triangle);
        meshTColors->InsertNextTypedTuple(GRAY);
    }
}

bool DrawableMesh::getDrawWireframe() const
{
    return this->drawWireframe;
}

void DrawableMesh::setDrawWireframe(bool value)
{
    this->drawWireframe = value;
}

bool DrawableMesh::getDrawPoints() const
{
	return drawPoints;
}

void DrawableMesh::setDrawPoints(bool value)
{
	this->drawPoints = value;
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
    Vertex* v = idVertices[static_cast<unsigned long>(pid)];
    v->setValue(p[0], p[1], p[2]);
    this->meshPoints->SetPoint(pid, p);
    this->meshPoints->Modified();
}

void DrawableMesh::setSelectedPoints(std::map<unsigned long, bool> selectedPoints){

    for(unsigned long i = 0; i < static_cast<unsigned long>(V.numels()); i++){

        if(selectedPoints[i])
            this->meshVColors->SetTypedTuple(static_cast<vtkIdType>(i), RED);
        else
            this->meshVColors->SetTypedTuple(static_cast<vtkIdType>(i), BLUE);

    }

}

void DrawableMesh::setSelectedLines(std::map<unsigned long, bool> selectedLines)
{

    for(unsigned long i = 0; i < static_cast<unsigned long>(E.numels()); i++){

        if(selectedLines[i])
            this->meshEColors->SetTypedTuple(static_cast<vtkIdType>(i), RED);
        else
            this->meshEColors->SetTypedTuple(static_cast<vtkIdType>(i), BLACK);

    }

}

void DrawableMesh::setSelectedTriangles(std::map<unsigned long, bool> selectedTriangles){

    for(std::map<unsigned long, bool>::iterator iit = selectedTriangles.begin(); iit != selectedTriangles.end(); iit++){
        std::pair<unsigned long, bool> p = *iit;
        if(p.second)
            this->meshTColors->SetTypedTuple(static_cast<vtkIdType>(p.first), RED);
        else
            this->meshTColors->SetTypedTuple(static_cast<vtkIdType>(p.first), GRAY);

    }

}

void DrawableMesh::setSelectedAnnotations(std::map<unsigned int, bool> selectedAnnotations)
{
    for(std::map<unsigned int, bool>::iterator iit = selectedAnnotations.begin(); iit != selectedAnnotations.end(); iit++)
        dynamic_cast<DrawableAnnotation*>(annotations[iit->first])->setSelected(iit->second);
}

void DrawableMesh::setColor(int r, int g, int b){

    IMATI_STL::Node* n;
    Triangle* t;

    unsigned char red = static_cast<unsigned char>(r);
    unsigned char green = static_cast<unsigned char>(g);
    unsigned char blue = static_cast<unsigned char>(b);
    for(n = T.head(); n != nullptr; n = n->next()){
        t = static_cast<Triangle*>(n->data);
        unsigned char color[3] = {red, green, blue};
        this->meshTColors->SetTypedTuple(static_cast<vtkIdType>(trianglesId[t]), color);
    }

}

vtkSmartPointer<vtkActor> DrawableMesh::getMeshSurfaceActor() const{
    return meshSurfaceActor;
}

unsigned long DrawableMesh::getEdgeId(Edge * e) const
{
    return edgesId.at(e);
}

Edge *DrawableMesh::getEdge(unsigned long id) const
{
    return idEdges.at(id);
}

vtkSmartPointer<vtkActor> DrawableMesh::getMeshWireframeActor() const
{
    return meshWireframeActor;
}

vtkSmartPointer<vtkActor> DrawableMesh::getMeshPointsActor() const
{
    return meshPointsActor;
}

bool DrawableMesh::getDrawAnnotations() const
{
    return drawAnnotations;
}

void DrawableMesh::setDrawAnnotations(bool value)
{
    drawAnnotations = value;
}


double DrawableMesh::getAnnotationsOpacity()
{
    double opacity =  0.0;

    for(unsigned int i = 0; i < annotations.size(); i++)
        opacity += dynamic_cast<DrawableAnnotation*>(annotations[i])->getOpacity();
    opacity /= annotations.size();
    return opacity;
}


void DrawableMesh::setAnnotationsOpacity(double value)
{
    for(unsigned int i = 0; i < annotations.size(); i++)
        dynamic_cast<DrawableAnnotation*>(annotations[i])->setOpacity(value);
}

vtkSmartPointer<vtkPoints> DrawableMesh::getMeshPoints() const
{
    return meshPoints;
}

void DrawableMesh::setMeshPoints(const vtkSmartPointer<vtkPoints> &value)
{
    meshPoints = value;
}

bool DrawableMesh::getOnlyPointsPositionsModified() const
{
    return onlyPointsPositionsModified;
}

void DrawableMesh::setOnlyPointsPositionsModified(bool value)
{
    onlyPointsPositionsModified = value;
}

bool DrawableMesh::getDrawSurface() const
{
    return drawSurface;
}

void DrawableMesh::setDrawSurface(bool value)
{
    drawSurface = value;
}

vtkSmartPointer<vtkPropAssembly> DrawableMesh::getCanvas() const
{
    return this->canvas;
}

void DrawableMesh::setCanvas(const vtkSmartPointer<vtkPropAssembly> &value)
{
    this->canvas = value;
}

