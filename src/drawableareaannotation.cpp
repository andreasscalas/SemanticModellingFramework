#include "drawableareaannotation.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>
#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkProperty.h>
#include <vtkLine.h>
#include <drawableattribute.h>

using namespace std;
using namespace IMATI_STL;

DrawableAreaAnnotation::DrawableAreaAnnotation() : DrawableAnnotation()
{
    init();
}

DrawableAreaAnnotation::DrawableAreaAnnotation(SurfaceAnnotation* annotation)
{
    init();
    this->id = annotation->getId();
    this->tag = annotation->getTag();
    this->color = annotation->getColor();
    this->outlines = annotation->getOutlines();
    this->attributes = annotation->getAttributes();
    this->mesh = annotation->getMesh();
}

void DrawableAreaAnnotation::init()
{
    annotatedTriangles = vtkSmartPointer<vtkCellArray>::New();
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    outlinesActor = vtkSmartPointer<vtkActor>::New();
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

void DrawableAreaAnnotation::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkPolyData> annotationsPolydata = vtkSmartPointer<vtkPolyData>::New();
    annotationsPolydata->SetPoints(meshPoints);
    annotationsPolydata->SetPolys(annotatedTriangles);
    annotationsPolydata->GetCellData()->SetScalars(annotationColors);
    vtkSmartPointer<vtkPolyDataMapper> annotationMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    annotationActor = vtkSmartPointer<vtkActor>::NewInstance(annotationActor);
    annotationMapper->SetInputData(annotationsPolydata);
    annotationActor->SetMapper(annotationMapper);
    annotationActor->GetProperty()->SetOpacity(this->opacity);
    canvas->AddPart(annotationActor);
    if(drawAttributes)
        for(unsigned int i = 0; i < attributes.size(); i++)
            dynamic_cast<DrawableAttribute*>(attributes[i])->draw(canvas);

    if(selected){
        vtkSmartPointer<vtkPolyData> outlinesData = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkCellArray> vtkOutlines = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkPolyDataMapper> outlinesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        outlinesActor = vtkSmartPointer<vtkActor>::NewInstance(outlinesActor);
        for(unsigned int i = 0; i < outlines.size(); i++){
            for(unsigned int j = 1; j < outlines[i].size(); j++){
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                line->GetPointIds()->SetNumberOfIds(2);
                line->GetPointIds()->SetId(0, static_cast<vtkIdType>(mesh->getPointId(outlines[i][j - 1])));
                line->GetPointIds()->SetId(1, static_cast<vtkIdType>(mesh->getPointId(outlines[i][j])));
                vtkOutlines->InsertNextCell(line);
            }
        }
        outlinesData->SetPoints(meshPoints);
        outlinesData->SetLines(vtkOutlines);
        outlinesMapper->SetInputData(outlinesData);
        outlinesActor->SetMapper(outlinesMapper);
        outlinesActor->GetProperty()->SetLineWidth(5.0);
        outlinesActor->GetProperty()->SetColor(1, 0, 0);
        outlinesActor->GetProperty()->SetOpacity(1.0);
        canvas->AddPart(outlinesActor);
    }
    canvas->Modified();
    assembly->AddPart(canvas);
    assembly->Modified();

}

void DrawableAreaAnnotation::update()
{
    vector<Triangle*> annotationTriangles = getTriangles();
    annotatedTriangles = vtkSmartPointer<vtkCellArray>::NewInstance(annotatedTriangles);
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::NewInstance(annotationColors);
    annotationColors->SetNumberOfComponents(3);
    annotationColors->SetName("AnnotationsTColors");

    //Update of the data-visualization linking
    for(std::vector<Triangle*>::iterator tit = annotationTriangles.begin(); tit != annotationTriangles.end(); tit++){
        vtkSmartPointer<vtkTriangle> t = vtkSmartPointer<vtkTriangle>::New();
        t->GetPointIds()->SetNumberOfIds(3);
        t->GetPointIds()->SetId(0, static_cast<vtkIdType>(mesh->getPointId((*tit)->v1())));
        t->GetPointIds()->SetId(1, static_cast<vtkIdType>(mesh->getPointId((*tit)->v2())));
        t->GetPointIds()->SetId(2, static_cast<vtkIdType>(mesh->getPointId((*tit)->v3())));
        annotatedTriangles->InsertNextCell(t);
        annotationColors->InsertNextTypedTuple(color);
    }

}

void DrawableAreaAnnotation::clear()
{
    init();
    this->tag = "";
    this->color = nullptr;
    this->outlines.clear();
    this->mesh = nullptr;

}

Annotation *DrawableAreaAnnotation::transfer(ExtendedTrimesh *otherMesh, short metric)
{
    return SurfaceAnnotation::transfer(otherMesh, metric);
}

Annotation *DrawableAreaAnnotation::parallelTransfer(ExtendedTrimesh *otherMesh, short metric)
{
    return SurfaceAnnotation::parallelTransfer(otherMesh, metric);
}

std::vector<Vertex *> DrawableAreaAnnotation::getInvolvedVertices()
{
    return SurfaceAnnotation::getInvolvedVertices();
}

bool DrawableAreaAnnotation::isPointInAnnotation(Vertex *p)
{
    return SurfaceAnnotation::isPointInAnnotation(p);
}
