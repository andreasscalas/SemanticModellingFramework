#include <cageverticesselectionstyle.h>
#include <annotationselectiondialog.h>
#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkPlanes.h>

void CageVerticesSelectionStyle::setAnnotationToCage(std::map<unsigned int, std::vector<unsigned int> > *value)
{
    annotationToCage = value;
}

std::map<unsigned int, std::vector<unsigned int> > *CageVerticesSelectionStyle::getAnnotationToCage() const
{
    return annotationToCage;
}

vtkSmartPointer<vtkRenderer> CageVerticesSelectionStyle::getRenderer() const
{
    return renderer;
}

void CageVerticesSelectionStyle::setRenderer(const vtkSmartPointer<vtkRenderer> &value)
{
    renderer = value;
    this->SetCurrentRenderer(renderer);
}

CageVerticesSelectionStyle::CageVerticesSelectionStyle(){

    selectionMode = true;
    visiblePointsOnly = true;
    this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    this->SelectedActor = vtkSmartPointer<vtkActor>::New();
    this->SelectedActor->SetMapper(SelectedMapper);
    this->modelTrianglePicker = vtkSmartPointer<vtkCellPicker>::New();
    this->cagePointPicker = vtkSmartPointer<vtkPointPicker>::New();

}


void CageVerticesSelectionStyle::OnRightButtonDown(){

    //If the user is trying to pick a point...
    //The click position of the mouse is taken
    int x, y;
    x = this->Interactor->GetEventPosition()[0];
    y = this->Interactor->GetEventPosition()[1];
    this->FindPokedRenderer(x, y);
    //Some tolerance is set for the picking
    this->cagePointPicker->SetTolerance(0.01);
    this->cagePointPicker->Pick(x, y, 0, renderer);
    vtkIdType pointID = this->cagePointPicker->GetPointId();
    //If some point has been picked...
    std::vector<unsigned long> selected;
    if(pointID >= 0 && pointID < this->cage->V.numels()){

        selected.push_back(static_cast<unsigned long>(pointID));
        defineSelection(selected);
        modifySelectedPoints();

    }
    vtkInteractorStyleRubberBandPick::OnRightButtonDown();

}

void CageVerticesSelectionStyle::OnMiddleButtonDown()
{
    if(Interactor->GetControlKey())
    {
        //If the user is trying to pick a point...
        //The click position of the mouse is taken
        int x, y;
        x = this->Interactor->GetEventPosition()[0];
        y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);
        this->modelTrianglePicker->AddPickList(model->getMeshSurfaceActor());
        this->modelTrianglePicker->PickFromListOn();
        this->modelTrianglePicker->Pick(x, y, 0, this->GetCurrentRenderer());

        vtkIdType cellID = this->modelTrianglePicker->GetCellId();
        //If some point has been picked...
        std::vector<unsigned long> selected;
        if(cellID >= 0 && cellID < this->model->T.numels()){

            DrawableAnnotation* selectedAnnotation;
            std::vector<DrawableAnnotation*> selectedAnnotations;
            for(unsigned int i = 0; i < model->getAnnotations().size(); i++)
            {
//                PointAnnotation* pa = dynamic_cast<PointAnnotation*>(model->getAnnotations()[i]);
//                LineAnnotation* la = dynamic_cast<LineAnnotation*>(model->getAnnotations()[i]);
                SurfaceAnnotation* sa = dynamic_cast<SurfaceAnnotation*>(model->getAnnotations()[i]);
//                if(sa != nullptr && sa->isTriangleInAnnotation(model->getTriangle(cellID)))
//                    selectedAnnotations.push_back(dynamic_cast<DrawableAnnotation*>(sa));
//                if(sa != nullptr && sa->isTriangleInAnnotation(model->getTriangle(cellID)))
//                    selectedAnnotations.push_back(dynamic_cast<DrawableAnnotation*>(sa));
                if(sa != nullptr && sa->isTriangleInAnnotation(model->getTriangle(cellID)))
                    selectedAnnotations.push_back(dynamic_cast<DrawableAnnotation*>(sa));
            }
            if(selectedAnnotations.size() != 0)
            {
                if(selectedAnnotations.size() > 1){
                    AnnotationSelectionDialog* dialog = new AnnotationSelectionDialog();
                    dialog->setAnnotationsList(selectedAnnotations);
                    dialog->exec();
                    selectedAnnotation = dialog->getSelectedAnnotation();
                }else
                    selectedAnnotation = selectedAnnotations[0];


                if(selectedAnnotation != nullptr)
                {
                    for(unsigned int j = 0; j < annotationToCage->at(model->getAnnotationId(selectedAnnotation)).size(); j++)
                        selected.push_back(annotationToCage->at(model->getAnnotationId(selectedAnnotation))[j]);
                    defineSelection(selected);
                    modifySelectedPoints();
                }
            }
        }
    } else
        vtkInteractorStyleRubberBandPick::OnMiddleButtonDown();
}

void CageVerticesSelectionStyle::OnLeftButtonDown(){

    if(this->Interactor->GetControlKey())
        this->CurrentMode = VTKISRBP_SELECT;

    vtkInteractorStyleRubberBandPick::OnLeftButtonDown();

}

void CageVerticesSelectionStyle::OnLeftButtonUp(){

    vtkInteractorStyleRubberBandPick::OnLeftButtonUp();
    if(this->CurrentMode==VTKISRBP_SELECT){

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
        if(visiblePointsOnly){
            vtkSmartPointer<vtkSelectVisiblePoints> selectVisiblePoints = vtkSmartPointer<vtkSelectVisiblePoints>::New();
            selectVisiblePoints->SetInputConnection(glyphFilter->GetOutputPort());
            selectVisiblePoints->SetRenderer(this->renderer);
            selectVisiblePoints->Update();
            selected = selectVisiblePoints->GetOutput();
        }else
            selected = glyphFilter->GetOutput();

        #if VTK_MAJOR_VERSION <= 5
            this->SelectedMapper->SetInput(selected);
        #else
            this->SelectedMapper->SetInputData(selected);
        #endif
            this->SelectedMapper->ScalarVisibilityOff();

        vtkSmartPointer<vtkIdTypeArray> ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));

        if(ids == nullptr)
            return;

        std::vector<unsigned long> selectedPoints;
        for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
            selectedPoints.push_back(static_cast<unsigned long>(ids->GetValue(i)));

        defineSelection(selectedPoints);
        modifySelectedPoints();
    }

}

void CageVerticesSelectionStyle::resetSelection(){

    this->pointsSelectionStatus->clear();


    for(IMATI_STL::Node* n = cage->V.head(); n != nullptr; n = n->next()){
        IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
        (*this->pointsSelectionStatus)[cage->getPointId(v)] = false;
    }
    modifySelectedPoints();
}

void CageVerticesSelectionStyle::defineSelection(std::vector<unsigned long> selected){
    for(std::vector<unsigned long>::iterator it = selected.begin(); it != selected.end(); it++){
        if(!pointsSelectionStatus->at(*it) && selectionMode)
            pointsSelectionStatus->at(*it) = true;
        else if(pointsSelectionStatus->at(*it) && !selectionMode )
            pointsSelectionStatus->at(*it) = false;
    }
}

void CageVerticesSelectionStyle::modifySelectedPoints(){

    cage->setSelectedPoints(*pointsSelectionStatus);
    this->cage->draw(assembly);
    this->qvtkwidget->update();

}

vtkSmartPointer<vtkPolyData> CageVerticesSelectionStyle::CageVerticesSelectionStyle::getPoints() const
{
    return Points;
}

void CageVerticesSelectionStyle::setPoints(const vtkSmartPointer<vtkPolyData> &value)
{
    Points = value;
}

vtkSmartPointer<vtkActor> CageVerticesSelectionStyle::getSelectedActor() const
{
return SelectedActor;
}

void CageVerticesSelectionStyle::setSelectedActor(const vtkSmartPointer<vtkActor> &value)
{
SelectedActor = value;
}

vtkSmartPointer<vtkDataSetMapper> CageVerticesSelectionStyle::getSelectedMapper() const
{
return SelectedMapper;
}

void CageVerticesSelectionStyle::setSelectedMapper(const vtkSmartPointer<vtkDataSetMapper> &value)
{
SelectedMapper = value;
}

std::map<unsigned long, bool> *CageVerticesSelectionStyle::getSelectedPoints() const
{
    return pointsSelectionStatus;
}

void CageVerticesSelectionStyle::setSelectedPoints(std::map<unsigned long, bool> *value)
{
    pointsSelectionStatus = value;
}

vtkSmartPointer<vtkPointPicker> CageVerticesSelectionStyle::getCagePointPicker() const
{
    return cagePointPicker;
}

void CageVerticesSelectionStyle::setCagePointPicker(const vtkSmartPointer<vtkPointPicker> &value)
{
    cagePointPicker = value;
}

bool CageVerticesSelectionStyle::getSelectionMode() const
{
    return selectionMode;
}

void CageVerticesSelectionStyle::setSelectionMode(bool value)
{
    selectionMode = value;
}

bool CageVerticesSelectionStyle::getVisiblePointsOnly() const
{
    return visiblePointsOnly;
}

void CageVerticesSelectionStyle::setVisiblePointsOnly(bool value)
{
visiblePointsOnly = value;
}

DrawableMesh *CageVerticesSelectionStyle::getCage() const
{
    return cage;
}

void CageVerticesSelectionStyle::setCage(DrawableMesh *value)
{
    cage = value;
}

vtkSmartPointer<vtkPropAssembly> CageVerticesSelectionStyle::getAssembly() const
{
    return assembly;
}

void CageVerticesSelectionStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly> &value)
{
    assembly = value;
}

QVTKOpenGLNativeWidget *CageVerticesSelectionStyle::getQvtkwidget() const
{
    return qvtkwidget;
}

void CageVerticesSelectionStyle::setQvtkwidget(QVTKOpenGLNativeWidget *value)
{
    qvtkwidget = value;
}

vtkSmartPointer<vtkCellPicker> CageVerticesSelectionStyle::getModelTrianglePicker() const
{
    return modelTrianglePicker;
}

void CageVerticesSelectionStyle::setModelTrianglePicker(const vtkSmartPointer<vtkCellPicker> &value)
{
    modelTrianglePicker = value;
}

DrawableMesh *CageVerticesSelectionStyle::getModel() const
{
    return model;
}

void CageVerticesSelectionStyle::setModel(DrawableMesh *value)
{
    model = value;

}
