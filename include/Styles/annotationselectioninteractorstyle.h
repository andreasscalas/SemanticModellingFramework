#ifndef ANNOTATIONSELECTIONINTERACTORSTYLE_H
#define ANNOTATIONSELECTIONINTERACTORSTYLE_H


#include <drawablemesh.h>
#include <annotation.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkPropAssembly.h>
#include <vtkCellPicker.h>
#include <QVTKOpenGLNativeWidget.h>


class AnnotationSelectionInteractorStyle: public vtkInteractorStyleRubberBandPick
{
public:

    constexpr static double TOLERANCE_RATIO = 2e-1;
    constexpr static double RADIUS_RATIO = 300;
    static AnnotationSelectionInteractorStyle* New();
    AnnotationSelectionInteractorStyle();
    vtkTypeMacro(AnnotationSelectionInteractorStyle, vtkInteractorStyleRubberBandPick)

    void OnRightButtonDown() override;
    void OnMouseMove() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void resetSelection();
    void modifySelectedAnnotations();

    vtkSmartPointer<vtkPropAssembly> getAssembly() const;
    void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);
    vtkSmartPointer<vtkRenderer> getRen() const;
    void setRen(const vtkSmartPointer<vtkRenderer> &value);
    vtkSmartPointer<vtkCellPicker> getCellPicker() const;
    void setCellPicker(const vtkSmartPointer<vtkCellPicker> &value);
    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);
    double getTolerance() const;

    QVTKOpenGLNativeWidget *getQvtkWidget() const;
    void setQvtkWidget(QVTKOpenGLNativeWidget *value);

    std::vector<Annotation *> getSelectedAnnotations() const;

protected:

    vtkSmartPointer<vtkRenderer> ren;
    vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
    vtkSmartPointer<vtkCellPicker> cellPicker;
    std::vector<Annotation*> selectedAnnotations;

    QVTKOpenGLNativeWidget * qvtkWidget;

    DrawableMesh* mesh;
    double tolerance;
    vtkIdType reachedID;
};

#endif // ANNOTATIONSELECTIONINTERACTORSTYLE_H
