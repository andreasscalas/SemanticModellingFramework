#ifndef ANNOTATIONWINDOW_H
#define ANNOTATIONWINDOW_H

#include <annotationconstraintdialog.h>
#include <annotationdialog.h>
#include <annotationsconstraint.h>
#include <annotationsrelationship.h>
#include <QMainWindow>
#include <annotationselectioninteractorstyle.h>
#include <lineselectionstyle.h>
#include <measurestyle.h>
#include <triangleselectionstyle.h>
#include <verticesselectionstyle.h>

namespace Ui {
class AnnotationWindow;
}

class AnnotationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AnnotationWindow(QWidget *parent = nullptr);
    ~AnnotationWindow();

    void annotationsModified();
    void updateView();
    void resetCamera();

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

    ConstraintSolver *getSolver() const;
    void setSolver(ConstraintSolver *value);

    std::vector<AnnotationsConstraint *> getSemanticConstraints() const;
    void setSemanticConstraints(const std::vector<AnnotationsConstraint *> &value);

    bool getIsSystemConstrained() const;
    void setIsSystemConstrained(bool value);

    ShapeOp::MatrixX3 *getSoMeshPoints() const;
    void setSoMeshPoints(ShapeOp::MatrixX3 *value);

    bool areCoordsComputed() const;
    void setCoordsComputed(bool value);

    bool getShowGeometricAttributes() const;
    void setShowGeometricAttributes(bool value);

    bool getIsTemplate() const;
    void setIsTemplate(bool value);

    std::string getCurrentPath() const;
    void setCurrentPath(const std::string &value);

signals:
    void substituteMesh(DrawableMesh* original, DrawableMesh* newMesh, QString filename);
    void updateMeshView(DrawableMesh*);
    void annotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*>);
    void constrainRelationship(AnnotationsRelationship*&);

    void constrainSystem(std::vector<AnnotationsConstraint*>);
    void releaseConstraints();

    void currentPathChanged(std::string);


private slots:
    void on_hideMeasuresButton_toggled(bool checked);

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
    void slotAddAnnotationsConstraint(std::string type, double weight, double minValue, double maxValue, unsigned int measureId1, unsigned int measureId2, bool directed);
    void slotAddAnnotationsRelationship(std::vector<Annotation*>, std::string, double, double, double, unsigned int, unsigned int, bool);
    void slotConstrainRelationship(AnnotationsRelationship*&);
    void slotConstrain();
    void slotReleaseConstraints();
    void slotUpdate();
    void slotUpdateView();
    void slotActionAddConstraint();
    void slotTransfer();
    void rulerSelected(bool);
    void tapeSelected(bool);
    void caliperSelected(bool);
    void boundingSelected(bool);
    void heightSelected(bool);
    void editAnnotation(bool);
    void openContextualMenu(vtkObject*, unsigned long, void*, void*);
    void on_addMeasureButton_clicked();
    void on_addPropertyButton_clicked();
private:
    Ui::AnnotationWindow *ui;
    AnnotationDialog* ad;
    AnnotationConstraintDialog* cd;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    vtkSmartPointer<VerticesSelectionStyle> verticesSelectionStyle;
    vtkSmartPointer<LineSelectionStyle> linesSelectionStyle;
    vtkSmartPointer<TriangleSelectionStyle> trianglesSelectionStyle;
    vtkSmartPointer<AnnotationSelectionInteractorStyle> annotationStyle;
    vtkSmartPointer<MeasureStyle> measureStyle;
    vtkSmartPointer<vtkCamera>    initialCamera;
    vtkSmartPointer<vtkPropAssembly> measureAssembly;
    std::map<unsigned long, bool> pointsSelectionStatus;
    std::map<unsigned long, bool> edgeSelectionStatus;
    std::map<unsigned long, bool> triangleSelectionStatus;
    std::vector<AnnotationsConstraint*> semanticConstraints;
    std::string currentPath;
    ShapeOp::MatrixX3* soMeshPoints;
    ConstraintSolver* solver;
    DrawableMesh* mesh;
    QString meshFilename;
    unsigned int reachedId;
    unsigned int reachedContraintId;

    bool isAnnotationBeingModified, coreGraphExtracted,
         isSystemConstrained, isTemplate, coordsComputed;
    Annotation* annotationBeingModified;

    vtkSmartPointer<vtkCamera> originalCamera;

    void closeEvent(QCloseEvent *event);
};

#endif // ANNOTATIONWINDOW_H
