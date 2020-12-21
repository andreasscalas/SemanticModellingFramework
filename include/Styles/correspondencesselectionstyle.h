#ifndef CORRESPONDENCESSELECTIONSTYLE_H
#define CORRESPONDENCESSELECTIONSTYLE_H
#include <vtkSmartPointer.h>
#include <vtkCellPicker.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkLabeledDataMapper.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCamera.h>
#include <vtkIdFilter.h>
#include <QVTKWidget.h>
#include <drawablemesh.h>

class CorrespondencesSelectionStyle : public vtkInteractorStyleTrackballCamera
{
public:
    CorrespondencesSelectionStyle();
    ~CorrespondencesSelectionStyle();
    static CorrespondencesSelectionStyle* New();
    vtkTypeMacro(CorrespondencesSelectionStyle, vtkInteractorStyleTrackballCamera)

    virtual void OnLeftButtonDown() override;

    std::vector<int> getPickedVertices() const;

    void setRen(const vtkSmartPointer<vtkRenderer> &value);

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

    void setCellPicker(const vtkSmartPointer<vtkCellPicker> &value);

    QVTKWidget *getQvtkWidget() const;
    void setQvtkWidget(QVTKWidget *value);

    vtkSmartPointer<vtkPoints> getMeshPoints() const;
    void setMeshPoints(const vtkSmartPointer<vtkPoints> &value);

private:
    DrawableMesh* mesh;
    vtkSmartPointer<vtkPoints> meshPoints;
    vtkSmartPointer<vtkCellPicker> cellPicker;
    vtkSmartPointer<vtkRenderer> ren;
    vtkSmartPointer<vtkCellArray> selectedPoints;
    vtkSmartPointer<vtkPolyData> data;
    vtkSmartPointer<vtkActor> selectedActor;
    vtkSmartPointer<vtkActor2D> labelsActor;
    std::vector<int> pickedVertices;
    QVTKWidget* qvtkWidget;

};

#endif // CORRESPONDENCESSELECTIONSTYLE_H
