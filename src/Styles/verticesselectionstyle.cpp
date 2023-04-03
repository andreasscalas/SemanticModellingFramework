#include <verticesselectionstyle.h>
#include <imatistl.h>

#include <vtkSphereSource.h>
#include <vtkWorldPointPicker.h>
#include <vtkIdFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkActor.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkImplicitFunction.h>
#include <vtkPlanes.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkIdFilter.h>

using namespace std;
using namespace IMATI_STL;

VerticesSelectionStyle::VerticesSelectionStyle() {

    selectionMode = true;
    visiblePointsOnly = true;
    leftPressed = false;
    this->pointPicker = vtkSmartPointer<vtkPointPicker>::New();
    this->sphereAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->annotation = new DrawablePointAnnotation();

}


void VerticesSelectionStyle::OnRightButtonDown() {

	//If the user is trying to pick a point...
	//The click position of the mouse is taken
	int x, y;
    x = this->Interactor->GetEventPosition()[0];
    y = this->Interactor->GetEventPosition()[1];
    this->FindPokedRenderer(x, y);
    vtkSmartPointer<vtkWorldPointPicker> picker = vtkSmartPointer<vtkWorldPointPicker>::New();
    double pickPos[3];
    int picked = picker->Pick(x, y, 0, this->GetCurrentRenderer());
    picker->GetPickPosition(pickPos);

    std::vector<unsigned long> selected;
    IMATI_STL::Point pickedPos(pickPos[0], pickPos[1], pickPos[2]);
    if(picked >= 0)
    {
        selected.push_back(mesh->getPointId(mesh->getClosestPoint(pickedPos)));
        defineSelection(selected);
        modifySelectedPoints();
    }

	vtkInteractorStyleRubberBandPick::OnRightButtonDown();

}

void VerticesSelectionStyle::OnLeftButtonDown() {


    leftPressed = true;
	if (this->Interactor->GetControlKey())
		this->CurrentMode = VTKISRBP_SELECT;

	vtkInteractorStyleRubberBandPick::OnLeftButtonDown();

}

void VerticesSelectionStyle::OnLeftButtonUp() {

	vtkInteractorStyleRubberBandPick::OnLeftButtonUp();
    leftPressed = false;
	if (this->CurrentMode == VTKISRBP_SELECT) {

		this->CurrentMode = VTKISRBP_ORIENT;

		// Forward events

		vtkPlanes* frustum = static_cast<vtkRenderedAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

		vtkSmartPointer<vtkExtractGeometry> extractGeometry = vtkSmartPointer<vtkExtractGeometry>::New();
        extractGeometry->SetImplicitFunction(static_cast<vtkImplicitFunction*>(frustum));

#if VTK_MAJOR_VERSION <= 5
		extractGeometry->SetInput(this->Points);
#else
        extractGeometry->SetInputData(this->Points);
#endif
		extractGeometry->Update();

		vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
		glyphFilter->Update();

        vtkSmartPointer<vtkPolyData> selected;
		if (visiblePointsOnly) {
			vtkSmartPointer<vtkSelectVisiblePoints> selectVisiblePoints = vtkSmartPointer<vtkSelectVisiblePoints>::New();
			selectVisiblePoints->SetInputConnection(glyphFilter->GetOutputPort());
			selectVisiblePoints->SetRenderer(this->GetCurrentRenderer());
			selectVisiblePoints->Update();
			selected = selectVisiblePoints->GetOutput();
		}
		else
			selected = glyphFilter->GetOutput();


		vtkSmartPointer<vtkIdTypeArray> ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalMeshIds"));

		if (ids == nullptr)
			return;

		std::vector<unsigned long> selectedPoints;
		for (vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
			selectedPoints.push_back(static_cast<unsigned long>(ids->GetValue(i)));

		defineSelection(selectedPoints);
		modifySelectedPoints();
	}

}

void VerticesSelectionStyle::OnMouseMove()
{
    vtkInteractorStyleRubberBandPick::OnMouseMove();
}

void VerticesSelectionStyle::resetSelection() {

	this->pointsSelectionStatus->clear();
    this->assembly->RemovePart(sphereAssembly);
    this->sphereAssembly = vtkSmartPointer<vtkPropAssembly>::New();

    for (IMATI_STL::Node* n = mesh->V.head(); n != nullptr; n = n->next()) {
		IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        (*this->pointsSelectionStatus)[mesh->getPointId(v)] = false;
    }

}

std::map<unsigned long, bool> *VerticesSelectionStyle::getPointsSelectionStatus() const
{
    return pointsSelectionStatus;
}

void VerticesSelectionStyle::setPointsSelectionStatus(std::map<unsigned long, bool> *value)
{
    pointsSelectionStatus = value;
}


void VerticesSelectionStyle::defineSelection(std::vector<unsigned long> selected) {
	for (std::vector<unsigned long>::iterator it = selected.begin(); it != selected.end(); it++) 
			pointsSelectionStatus->at(*it) = selectionMode;
}

void VerticesSelectionStyle::modifySelectedPoints() {
    this->selectedPoints.clear();
    for (unsigned long i = 0; i < static_cast<unsigned long>(mesh->V.numels()); i++)
		if ((*pointsSelectionStatus)[i])
        {
            this->selectedPoints.push_back(i);
            vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            IMATI_STL::Point* p = mesh->getPoint(i);
            sphereSource->SetCenter(p->x, p->y, p->z);
            sphereSource->SetRadius(sphereRadius);
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(sphereSource->GetOutputPort());
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->GetProperty()->SetColor(0,0,1);
            actor->SetMapper(mapper);
            assembly->RemovePart(sphereAssembly);
            sphereAssembly->AddPart(actor);
            assembly->AddPart(sphereAssembly);
        }

    this->mesh->setSelectedPoints(*pointsSelectionStatus);
    this->mesh->draw(assembly);
	this->qvtkwidget->update();

}

void VerticesSelectionStyle::finalizeAnnotation(unsigned int id, string tag, unsigned char color[])
{
    vector<IMATI_STL::Vertex*> selectedPoints;
    for(map<unsigned long, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){
        pair<unsigned long, bool> p = *pit;
        if(p.second)
            selectedPoints.push_back(mesh->getPoint(p.first));
    }

    if(selectedPoints.size() > 0){

        this->annotation->setId(id);
        this->annotation->setTag(tag);
        this->annotation->setColor(color);
        this->annotation->setPoints(selectedPoints);
        this->annotation->setMesh(mesh);
        this->mesh->addAnnotation(annotation);
        this->mesh->setAnnotationsModified(true);
        this->mesh->update();
        this->mesh->draw(assembly);
        this->resetSelection();
        this->modifySelectedPoints();
        this->annotation = new DrawablePointAnnotation();
    }
}

vtkSmartPointer<vtkPolyData> VerticesSelectionStyle::getPoints() const
{
    return Points;
}

void VerticesSelectionStyle::setPoints(const vtkSmartPointer<vtkPolyData>& value)
{
    Points = value;
}

std::map<unsigned long, bool>* VerticesSelectionStyle::getSelectedPoints() const
{
	return pointsSelectionStatus;
}

void VerticesSelectionStyle::setSelectedPoints(std::map<unsigned long, bool>* value)
{
	pointsSelectionStatus = value;
}

vtkSmartPointer<vtkPointPicker> VerticesSelectionStyle::getPointPicker() const
{
	return pointPicker;
}

void VerticesSelectionStyle::setPointPicker(const vtkSmartPointer<vtkPointPicker>& value)
{
	pointPicker = value;
}

bool VerticesSelectionStyle::getSelectionMode() const
{
	return selectionMode;
}

void VerticesSelectionStyle::setSelectionMode(bool value)
{
	selectionMode = value;
}

bool VerticesSelectionStyle::getVisiblePointsOnly() const
{
	return visiblePointsOnly;
}

void VerticesSelectionStyle::setVisiblePointsOnly(bool value)
{
	visiblePointsOnly = value;
}

DrawableMesh* VerticesSelectionStyle::getMesh() const
{
    return mesh;
}

void VerticesSelectionStyle::setMesh(DrawableMesh* value)
{
    mesh = value;

    vtkSmartPointer<vtkIdFilter> idFilterMesh = vtkSmartPointer<vtkIdFilter>::New();
    idFilterMesh->SetInputData(mesh->getMeshPointsActor()->GetMapper()->GetInputAsDataSet());
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetPointIdsArrayName("OriginalMeshIds");
    idFilterMesh->Update();
    vtkSmartPointer<vtkPolyData> inputMesh = static_cast<vtkPolyData*>(idFilterMesh->GetOutput());
    this->sphereRadius = this->mesh->getBoundingBallRadius() / RADIUS_RATIO;
}

vtkSmartPointer<vtkPropAssembly> VerticesSelectionStyle::getAssembly() const
{
	return assembly;
}

void VerticesSelectionStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly>& value)
{
	assembly = value;
}

QVTKOpenGLNativeWidget* VerticesSelectionStyle::getQvtkwidget() const
{
	return qvtkwidget;
}

void VerticesSelectionStyle::setQvtkwidget(QVTKOpenGLNativeWidget* value)
{
	qvtkwidget = value;
}
