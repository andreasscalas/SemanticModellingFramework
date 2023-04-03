#include "drawablepointannotation.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkActor.h>
//#include <vtkCellData.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkSphereSource.h>
#include <vtkOBBTree.h>
#include <vtkLine.h>

DrawablePointAnnotation::DrawablePointAnnotation() : DrawableAnnotation()
{
    init();
}

DrawablePointAnnotation::DrawablePointAnnotation(PointAnnotation *annotation)
{
    init();
    this->id = annotation->getId();
    this->tag = annotation->getTag();
    this->color = annotation->getColor();
    this->points = annotation->getPoints();
    this->attributes = annotation->getAttributes();
    this->mesh = annotation->getMesh();

}

DrawablePointAnnotation::~DrawablePointAnnotation()
{
    this->points.clear();
}

void DrawablePointAnnotation::init()
{
    annotatedPoints = vtkSmartPointer<vtkPoints>::New();
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
}

float DrawablePointAnnotation::getPointSize() const
{
    return pointSize;
}

void DrawablePointAnnotation::setPointSize(float value)
{
    pointSize = value;
}

void DrawablePointAnnotation::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{
    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkPolyData> annotationsPolydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
    annotatedPoints = vtkSmartPointer<vtkPoints>::NewInstance(annotatedPoints);
    for(unsigned int i = 0; i < points.size(); i++)
        annotatedPoints->InsertNextPoint(points[i]->x, points[i]->y, points[i]->z);
    pointsPolydata->SetPoints(annotatedPoints);
    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter->SetInputData(pointsPolydata);
    vertexFilter->Update();
    annotationsPolydata->ShallowCopy(vertexFilter->GetOutput());
    annotationsPolydata->GetPointData()->SetScalars(annotationColors);
    vtkSmartPointer<vtkPolyDataMapper> annotationMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    annotationMapper->SetInputData(annotationsPolydata);
    annotationActor = vtkSmartPointer<vtkActor>::NewInstance(annotationActor);
    annotationActor->SetMapper(annotationMapper);
    annotationActor->GetProperty()->SetOpacity(this->opacity);
    annotationActor->GetProperty()->SetPointSize(pointSize);
    canvas->AddPart(annotationActor);
    if(selected){
        double corner[3], min[3], mid[3], max[3], sizes[3];
        vtkSmartPointer<vtkPoints> OBBPoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolyData> OBBPolyData= vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
        if(points.size() > 1){
            vtkOBBTree::ComputeOBB(annotatedPoints, corner, max, mid, min, sizes);
        } else {
            sphere->SetCenter(points[0]->x, points[0]->y, points[0]->z);
            sphere->SetRadius(this->mesh->getBoundingBallRadius() / 100);
            sphere->Update();
            vtkOBBTree::ComputeOBB(sphere->GetOutput()->GetPoints(), corner, max, mid, min, sizes);
        }
        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        IMATI_STL::Point minAx(min[0], min[1] , min[2]);
        IMATI_STL::Point midAx(mid[0], mid[1] , mid[2]);
        IMATI_STL::Point maxAx(max[0], max[1] , max[2]);
        IMATI_STL::Point d(corner[0], corner[1], corner[2]);
        IMATI_STL::Point c(d + minAx);
        IMATI_STL::Point e(d + midAx);
        IMATI_STL::Point a(d + maxAx);
        IMATI_STL::Point b(a + minAx);
        IMATI_STL::Point f(a + midAx);
        IMATI_STL::Point g(b + midAx);
        IMATI_STL::Point h(c + midAx);
        vtkIdType aid = OBBPoints->InsertNextPoint(a.x, a.y, a.z);
        vtkIdType bid = OBBPoints->InsertNextPoint(b.x, b.y, b.z);
        vtkIdType cid = OBBPoints->InsertNextPoint(c.x, c.y, c.z);
        vtkIdType did = OBBPoints->InsertNextPoint(d.x, d.y, d.z);
        vtkIdType eid = OBBPoints->InsertNextPoint(e.x, e.y, e.z);
        vtkIdType fid = OBBPoints->InsertNextPoint(f.x, f.y, f.z);
        vtkIdType gid = OBBPoints->InsertNextPoint(g.x, g.y, g.z);
        vtkIdType hid = OBBPoints->InsertNextPoint(h.x, h.y, h.z);
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, aid);
        line->GetPointIds()->SetId(1, bid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, bid);
        line->GetPointIds()->SetId(1, cid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, cid);
        line->GetPointIds()->SetId(1, did);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, did);
        line->GetPointIds()->SetId(1, aid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, aid);
        line->GetPointIds()->SetId(1, fid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, bid);
        line->GetPointIds()->SetId(1, gid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, cid);
        line->GetPointIds()->SetId(1, hid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, did);
        line->GetPointIds()->SetId(1, eid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, eid);
        line->GetPointIds()->SetId(1, fid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, fid);
        line->GetPointIds()->SetId(1, gid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, gid);
        line->GetPointIds()->SetId(1, hid);
        lines->InsertNextCell(line);
        line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, hid);
        line->GetPointIds()->SetId(1, eid);
        lines->InsertNextCell(line);
        OBBPolyData->SetPoints(OBBPoints);
        OBBPolyData->SetLines(lines);
        vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
        outlineMapper->SetInputData(OBBPolyData);
        outlineActor->GetProperty()->SetOpacity(1.0);
        outlineActor->GetProperty()->SetLineWidth(5.0);
        outlineActor->GetProperty()->SetColor(1,0,0);
        outlineActor->SetMapper(outlineMapper);
        canvas->AddPart(outlineActor);
    }
    canvas->Modified();
    assembly->AddPart(canvas);
    assembly->Modified();
}

void DrawablePointAnnotation::update()
{
    annotationColors = vtkSmartPointer<vtkUnsignedCharArray>::NewInstance(annotationColors);
    annotationColors->SetNumberOfComponents(3);
    annotationColors->SetName("AnnotationsVColors");

    //Update of the data-visualization linking
    for(unsigned int i = 0; i < points.size(); i++)
        annotationColors->InsertNextTypedTuple(color);

}

void DrawablePointAnnotation::clear()
{
    init();
    this->tag = "";
    this->color = nullptr;
    this->points.clear();
    this->mesh = nullptr;
}

Annotation *DrawablePointAnnotation::transfer(ExtendedTrimesh *otherMesh, short metric)
{
    return PointAnnotation::transfer(otherMesh, metric);
}

Annotation *DrawablePointAnnotation::parallelTransfer(ExtendedTrimesh *otherMesh, short metric)
{
    return PointAnnotation::parallelTransfer(otherMesh, metric);
}

std::vector<IMATI_STL::Vertex *> DrawablePointAnnotation::getInvolvedVertices()
{
    return PointAnnotation::getInvolvedVertices();
}

bool DrawablePointAnnotation::isPointInAnnotation(IMATI_STL::Vertex *p)
{
    return PointAnnotation::isPointInAnnotation(p);
}