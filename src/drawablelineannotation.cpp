#include "drawablelineannotation.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkLine.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

using namespace std;
using namespace IMATI_STL;

DrawableLineAnnotation::DrawableLineAnnotation() : DrawableAnnotation()
{
    init();
}

DrawableLineAnnotation::DrawableLineAnnotation(LineAnnotation* annotation)
{
    init();
    this->id = annotation->getId();
    this->tag = annotation->getTag();
    this->color = annotation->getColor();
    this->polyLines = annotation->getPolyLines();
    this->attributes = annotation->getAttributes();
    this->mesh = annotation->getMesh();
}

void DrawableLineAnnotation::init()
{
    annotatedLines = vtkSmartPointer<vtkCellArray>::New();
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
}

float DrawableLineAnnotation::getLineWidth() const
{
    return lineWidth;
}

void DrawableLineAnnotation::setLineWidth(float value)
{
    lineWidth = value;
}

Annotation *DrawableLineAnnotation::transfer(ExtendedTrimesh *otherMesh, short metric)
{
    return LineAnnotation::transfer(otherMesh, metric);
}

Annotation *DrawableLineAnnotation::parallelTransfer(ExtendedTrimesh *otherMesh, short metric)
{
    return LineAnnotation::parallelTransfer(otherMesh, metric);
}

std::vector<Vertex *> DrawableLineAnnotation::getInvolvedVertices()
{
    return LineAnnotation::getInvolvedVertices();
}

bool DrawableLineAnnotation::isPointInAnnotation(Vertex *p)
{
    return LineAnnotation::isPointInAnnotation(p);
}

void DrawableLineAnnotation::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkPolyData> annotationsPolydata = vtkSmartPointer<vtkPolyData>::New();
    annotationsPolydata->SetPoints(meshPoints);
    annotationsPolydata->SetLines(annotatedLines);
    annotationsPolydata->GetCellData()->SetScalars(annotationColors);
    vtkSmartPointer<vtkPolyDataMapper> annotationMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    annotationActor = vtkSmartPointer<vtkActor>::NewInstance(annotationActor);
    annotationMapper->SetInputData(annotationsPolydata);
    annotationActor->SetMapper(annotationMapper);
    annotationActor->GetProperty()->SetOpacity(this->opacity);
    annotationActor->GetProperty()->SetLineWidth(lineWidth);
    canvas->AddPart(annotationActor);
    if(selected){
        double sphereRadius = this->mesh->getBoundingBallRadius() / 1000;
        for(unsigned int i = 0; i < polyLines.size(); i++){
            vtkSmartPointer<vtkSphereSource> originSphereSource = vtkSmartPointer<vtkSphereSource>::New();
            vtkSmartPointer<vtkPolyDataMapper> originMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkActor> originActor = vtkSmartPointer<vtkActor>::New();
            vtkSmartPointer<vtkSphereSource> targetSphereSource = vtkSmartPointer<vtkSphereSource>::New();
            vtkSmartPointer<vtkPolyDataMapper> targetMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkActor> targetActor = vtkSmartPointer<vtkActor>::New();
            originSphereSource->SetCenter(polyLines[i][0]->x, polyLines[i][0]->y, polyLines[i][0]->z);
            originSphereSource->SetRadius(sphereRadius);
            targetSphereSource->SetCenter(polyLines[i][polyLines[i].size() - 1]->x, polyLines[i][polyLines[i].size() - 1]->y, polyLines[i][polyLines[i].size() - 1]->z);
            targetSphereSource->SetRadius(sphereRadius);
            originMapper->SetInputConnection(originSphereSource->GetOutputPort());
            targetMapper->SetInputConnection(targetSphereSource->GetOutputPort());
            originActor->SetMapper(originMapper);
            targetActor->SetMapper(targetMapper);
            originActor->GetProperty()->SetColor(1,0,0);
            originActor->GetProperty()->SetOpacity(this->opacity);
            targetActor->GetProperty()->SetColor(1,0,0);
            targetActor->GetProperty()->SetOpacity(this->opacity);
            canvas->AddPart(originActor);
            canvas->AddPart(targetActor);
        }
    }
    canvas->Modified();
    assembly->AddPart(canvas);
    assembly->Modified();
}

void DrawableLineAnnotation::update()
{
    annotatedLines = vtkSmartPointer<vtkCellArray>::NewInstance(annotatedLines);
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::NewInstance(annotationColors);
    annotationColors->SetNumberOfComponents(3);
    annotationColors->SetName("AnnotationsEColors");

    //Update of the data-visualization linking
    for(unsigned int i = 0; i < polyLines.size(); i++)
        for (unsigned j = 1; j < polyLines[i].size(); j++) {
            vtkSmartPointer<vtkLine> segment = vtkSmartPointer<vtkLine>::New();
            segment->GetPointIds()->SetId(0, static_cast<vtkIdType>(this->mesh->getPointId(polyLines[i][j - 1])));
            segment->GetPointIds()->SetId(1, static_cast<vtkIdType>(this->mesh->getPointId(polyLines[i][j])));
            annotatedLines->InsertNextCell(segment);
            annotationColors->InsertNextTypedTuple(color);
        }

}

void DrawableLineAnnotation::clear()
{
    init();
    this->tag = "";
    this->color = nullptr;
    this->polyLines.clear();
    this->mesh = nullptr;
}
