#include "annotationselectioninteractorstyle.h"
#include <vtkCellArray.h>
#include <annotationselectiondialog.h>
#include <vtkRenderer.h>

using namespace std;
using namespace IMATI_STL;
AnnotationSelectionInteractorStyle::AnnotationSelectionInteractorStyle()
{
    this->cellPicker = vtkSmartPointer<vtkCellPicker>::New();
}

void AnnotationSelectionInteractorStyle::OnRightButtonDown()
{
}

void AnnotationSelectionInteractorStyle::OnMouseMove()
{
    vtkInteractorStyleRubberBandPick::OnMouseMove();
}

void AnnotationSelectionInteractorStyle::OnLeftButtonDown()
{
    if(this->Interactor->GetControlKey()){

        this->cellPicker->AddPickList(mesh->getMeshSurfaceActor());
        this->cellPicker->PickFromListOn();
        //The click position of the mouse is taken
        int x, y;
        x = this->Interactor->GetEventPosition()[0];
        y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);
        //Some tolerance is set for the picking
        this->cellPicker->Pick(x, y, 0, ren);
        vtkIdType cellID = this->cellPicker->GetCellId();

        //If some point has been picked...
        if(cellID > 0 && cellID < this->mesh->T.numels()){
            Triangle* t = mesh->getTriangle(static_cast<unsigned long>(cellID));
            double wc[3];
            double bestDistance = DBL_MAX;
            this->cellPicker->GetPickPosition(wc);
            Point* p = new Point(wc[0], wc[1], wc[2]);
            Vertex* v_ = t->v1();
            Vertex* v;
            vector<Annotation*> annotations = mesh->getAnnotations();
            for(IMATI_STL::Node* n = mesh->V.head(); n != nullptr; n = n->next())
                static_cast<IMATI_STL::Vertex*>(n->data)->info = nullptr;

            for(unsigned int i = 0; i < 3; i++){
                double actualDistance = ((*v_) - (*p)).length();
                if(actualDistance < bestDistance){
                    bestDistance = actualDistance;
                    v = v_;
                }
                v_ = t->nextVertex(v_);
            }

            vector<DrawableAnnotation*> selected;
            for(unsigned int i = 0; i < annotations.size(); i++)
                if(annotations[i]->isPointInAnnotation(v))
                    selected.push_back(dynamic_cast<DrawableAnnotation*>(annotations[i]));

            if(selected.size() != 0){
                DrawableAnnotation* selectedAnnotation;
                if(selected.size() > 1){
                    AnnotationSelectionDialog* dialog = new AnnotationSelectionDialog();
                    dialog->setAnnotationsList(selected);
                    dialog->exec();
                    selectedAnnotation = dialog->getSelectedAnnotation();
                }else
                    selectedAnnotation = selected[0];
                vector<Annotation*>::iterator ait = std::find(annotations.begin(), annotations.end(), selectedAnnotation);

                if(ait != annotations.end()){
                    DrawableAnnotation* annotation = dynamic_cast<DrawableAnnotation*>(*ait);
                    annotation->setSelected(!annotation->getSelected());
                    Annotation* selectedMeshAnnotation = mesh->getAnnotations()[ait - annotations.begin()];
                    vector<Annotation*>::iterator sait = std::find(selectedAnnotations.begin(), selectedAnnotations.end(), selectedMeshAnnotation);
                    if(sait == selectedAnnotations.end())
                        selectedAnnotations.push_back(selectedMeshAnnotation);
                    else
                        selectedAnnotations.erase(sait);
                }

                modifySelectedAnnotations();
            }

        }
    }else
        vtkInteractorStyleRubberBandPick::OnLeftButtonDown();
}

void AnnotationSelectionInteractorStyle::OnLeftButtonUp()
{
    vtkInteractorStyleRubberBandPick::OnLeftButtonUp();
}

void AnnotationSelectionInteractorStyle::resetSelection()
{
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        dynamic_cast<DrawableAnnotation*>(mesh->getAnnotations()[i])->setSelected(false);
}

void AnnotationSelectionInteractorStyle::modifySelectedAnnotations()
{
    mesh->draw(assembly);
    assembly->Modified();
    qvtkWidget->update();
}

vtkSmartPointer<vtkPropAssembly> AnnotationSelectionInteractorStyle::getAssembly() const
{
    return assembly;
}

void AnnotationSelectionInteractorStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly> &value)
{
    assembly = value;
}

vtkSmartPointer<vtkRenderer> AnnotationSelectionInteractorStyle::getRen() const
{
    return ren;
}

void AnnotationSelectionInteractorStyle::setRen(const vtkSmartPointer<vtkRenderer> &value)
{
    ren = value;
}


double AnnotationSelectionInteractorStyle::getTolerance() const
{
    return tolerance;
}

QVTKOpenGLNativeWidget *AnnotationSelectionInteractorStyle::getQvtkWidget() const
{
    return qvtkWidget;
}

void AnnotationSelectionInteractorStyle::setQvtkWidget(QVTKOpenGLNativeWidget *value)
{
    qvtkWidget = value;
}

std::vector<Annotation *> AnnotationSelectionInteractorStyle::getSelectedAnnotations() const
{
    return selectedAnnotations;
}


DrawableMesh *AnnotationSelectionInteractorStyle::getMesh() const
{
    return mesh;
}

void AnnotationSelectionInteractorStyle::setMesh(DrawableMesh *value)
{
    mesh = value;
    this->tolerance = this->mesh->getMinEdgeLength() * TOLERANCE_RATIO;
}

vtkSmartPointer<vtkCellPicker> AnnotationSelectionInteractorStyle::getCellPicker() const
{
    return cellPicker;
}

void AnnotationSelectionInteractorStyle::setCellPicker(const vtkSmartPointer<vtkCellPicker> &value)
{
    cellPicker = value;
}

