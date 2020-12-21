#include <verticesselectionstyle.h>

using namespace std;
using namespace IMATI_STL;

VerticesSelectionStyle::VerticesSelectionStyle() {

    selectionMode = true;
    visiblePointsOnly = true;
    this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    this->SelectedActor = vtkSmartPointer<vtkActor>::New();
    this->SelectedActor->SetMapper(SelectedMapper);
	this->pointPicker = vtkSmartPointer<vtkPointPicker>::New();
    this->annotation = new PointAnnotation();

}


void VerticesSelectionStyle::OnRightButtonDown() {

	//If the user is trying to pick a point...
	//The click position of the mouse is taken
	int x, y;
	x = this->Interactor->GetEventPosition()[0];
	y = this->Interactor->GetEventPosition()[1];
	this->FindPokedRenderer(x, y);
	//Some tolerance is set for the picking
	this->pointPicker->SetTolerance(0.01);
	this->pointPicker->Pick(x, y, 0, this->GetCurrentRenderer());
	vtkIdType pointID = this->pointPicker->GetPointId();
	//If some point has been picked...
	std::vector<unsigned long> selected;
    if (pointID >= 0 && pointID < this->mesh->V.numels()) {

		selected.push_back(static_cast<unsigned long>(pointID));
		defineSelection(selected);
		modifySelectedPoints();

	}
	vtkInteractorStyleRubberBandPick::OnRightButtonDown();

}

void VerticesSelectionStyle::OnLeftButtonDown() {

	if (this->Interactor->GetControlKey())
		this->CurrentMode = VTKISRBP_SELECT;

	vtkInteractorStyleRubberBandPick::OnLeftButtonDown();

}

void VerticesSelectionStyle::OnLeftButtonUp() {

	vtkInteractorStyleRubberBandPick::OnLeftButtonUp();
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

#if VTK_MAJOR_VERSION <= 5
		this->SelectedMapper->SetInput(selected);
#else
		this->SelectedMapper->SetInputData(selected);
#endif
		this->SelectedMapper->ScalarVisibilityOff();

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

void VerticesSelectionStyle::resetSelection() {

	this->pointsSelectionStatus->clear();

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
            this->selectedPoints.push_back(i);

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
        this->annotation = new PointAnnotation();
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

vtkSmartPointer<vtkActor> VerticesSelectionStyle::getSelectedActor() const
{
	return SelectedActor;
}

void VerticesSelectionStyle::setSelectedActor(const vtkSmartPointer<vtkActor>& value)
{
	SelectedActor = value;
}

vtkSmartPointer<vtkDataSetMapper> VerticesSelectionStyle::getSelectedMapper() const
{
	return SelectedMapper;
}

void VerticesSelectionStyle::setSelectedMapper(const vtkSmartPointer<vtkDataSetMapper>& value)
{
	SelectedMapper = value;
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
}

vtkSmartPointer<vtkPropAssembly> VerticesSelectionStyle::getAssembly() const
{
	return assembly;
}

void VerticesSelectionStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly>& value)
{
	assembly = value;
}

QVTKWidget* VerticesSelectionStyle::getQvtkwidget() const
{
	return qvtkwidget;
}

void VerticesSelectionStyle::setQvtkwidget(QVTKWidget* value)
{
	qvtkwidget = value;
}
