#ifndef ANNOTATIONWINDOW_H
#define ANNOTATIONWINDOW_H

#include "annotationdialog.h"
#include "annotationsrelationship.h"

#include <QMainWindow>
#include <annotationselectioninteractorstyle.h>
#include <lineselectionstyle.h>
#include <measurestyle.h>
#include <triangleselectionstyle.h>
#include <verticesselectionstyle.h>
#include <vtkEventQtSlotConnect.h>

namespace Ui {
class AnnotationWindow;
}

class AnnotationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AnnotationWindow(QWidget *parent = nullptr);
    ~AnnotationWindow();

    void updateView();

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

signals:
    void updateMeshView(DrawableMesh*);
    void annotationWindowClosed(DrawableMesh*);
    void constrainRelationship(AnnotationsRelationship*&);

    bool getShowGeometricAttributes() const;
    void setShowGeometricAttributes(bool value);

private slots:
    void slotSaveAnnotation();
    void slotOpenAnnotation();
    void slotClearAnnotation();
    void slotClearSelections();
    void slotResetCamera(bool);
    void slotCamera(bool);
    void slotVisible(bool);
    void slotDeleteMode(bool);
    void slotVerticesSelection(bool);
    void slotLinesSelection(bool);
    void slotTrianglesInRectangle(bool);
    void slotTrianglesInPolygon(bool);
    void slotAnnotate();
    void slotSelectAnnotation(bool);
    void slotBuildRelationshipsGraph();
    void slotShowRelationships();
    void slotFinalization(std::string tag, uchar *color);
    void slotConstrainRelationship(AnnotationsRelationship*&);
    void rulerSelected(bool);
    void tapeSelected(bool);
    void caliperSelected(bool);
    void boundingSelected(bool);
    void editAnnotation(bool);
    void openContextualMenu(vtkObject*, unsigned long, void*, void*);
    void on_addMeasureButton_clicked();
    void on_addPropertyButton_clicked();

private:
    Ui::AnnotationWindow *ui;
    AnnotationDialog* ad;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    vtkSmartPointer<VerticesSelectionStyle> verticesSelectionStyle;
    vtkSmartPointer<LineSelectionStyle> linesSelectionStyle;
    vtkSmartPointer<TriangleSelectionStyle> trianglesSelectionStyle;
    vtkSmartPointer<AnnotationSelectionInteractorStyle> annotationStyle;
    vtkSmartPointer<MeasureStyle> measureStyle;
    vtkSmartPointer<vtkEventQtSlotConnect> connector;
    std::map<unsigned long, bool> pointsSelectionStatus;
    std::map<unsigned long, bool> edgeSelectionStatus;
    std::map<unsigned long, bool> triangleSelectionStatus;
    DrawableMesh* mesh;
    unsigned int reachedId;

    bool isAnnotationBeingModified;
    Annotation* annotationBeingModified;

    vtkSmartPointer<vtkCamera> originalCamera;

    void closeEvent(QCloseEvent *event);
};

#endif // ANNOTATIONWINDOW_H
