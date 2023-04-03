#ifndef LINESELECTIONSTYLE_H
#define LINESELECTIONSTYLE_H

#include <drawablemesh.h>
#include <drawablelineannotation.h>
#include <map>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkPoints.h>
#include <vtkParametricSpline.h>
#include <vtkPropAssembly.h>
#include <vtkPolyData.h>
#include <vtkCellPicker.h>
#include <QVTKOpenGLNativeWidget.h>

class LineSelectionStyle : public vtkInteractorStyleRubberBandPick
{
public:
    constexpr static double TOLERANCE_RATIO = 2e-1;
    constexpr static double RADIUS_RATIO = 1000;
    static LineSelectionStyle* New();
    LineSelectionStyle();
    vtkTypeMacro(LineSelectionStyle, vtkInteractorStyleRubberBandPick)

    void OnRightButtonDown() override;
    void OnMouseMove() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void modifySelectedLines();
    void resetSelection();
    void finalizeAnnotation(unsigned int id, std::string tag, unsigned char color[]);

    std::vector<IMATI_STL::Vertex *> getPolyLine() const;
    void setPolyLine(const std::vector<IMATI_STL::Vertex *> &value);
    vtkSmartPointer<vtkActor> getSplineActor() const;
    void setSplineActor(const vtkSmartPointer<vtkActor> &value);
    vtkSmartPointer<vtkPropAssembly> getSphereAssembly() const;
    void setSphereAssembly(const vtkSmartPointer<vtkPropAssembly> &value);
    vtkSmartPointer<vtkPropAssembly> getAssembly() const;
    void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);
    vtkSmartPointer<vtkRenderer> getRen() const;
    void setRen(const vtkSmartPointer<vtkRenderer> &value);
    vtkSmartPointer<vtkParametricSpline> getSpline() const;
    void setSpline(const vtkSmartPointer<vtkParametricSpline> &value);
    vtkSmartPointer<vtkPoints> getPolylinePoints() const;
    void setPolylinePoints(const vtkSmartPointer<vtkPoints> &value);
    std::map<unsigned long, bool> *getEdgesSelectionStatus() const;
    void setEdgesSelectionStatus(std::map<unsigned long, bool> *value);
    std::map<unsigned long, bool> *getPointsSelectionStatus() const;
    void setPointsSelectionStatus(std::map<unsigned long, bool> *value);
    vtkSmartPointer<vtkCellPicker> getCellPicker() const;
    void setCellPicker(const vtkSmartPointer<vtkCellPicker> &value);
    vtkSmartPointer<vtkPolyData> getPoints() const;
    void setPoints(const vtkSmartPointer<vtkPolyData> &value);
    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

    double getSphereRadius() const;
    void setSphereRadius(double value);
    bool getSelectionMode() const;
    void setSelectionMode(bool value);
    bool getVisibleTrianglesOnly() const;
    void setVisibleTrianglesOnly(bool value);
    bool getVisiblePointsOnly() const;
    void setVisiblePointsOnly(bool value);
    bool getShowSelectedPoints() const;
    void setShowSelectedPoints(bool value);
    bool getAlreadyStarted() const;
    void setAlreadyStarted(bool value);
    bool getLassoStarted() const;
    void setLassoStarted(bool value);

    double getTolerance() const;

    QVTKOpenGLNativeWidget *getQvtkwidget() const;
    void setQvtkwidget(QVTKOpenGLNativeWidget *value);

protected:

    std::map<unsigned long, bool>* edgesSelectionStatus;
    std::map<unsigned long, bool>* pointsSelectionStatus;
    vtkSmartPointer<vtkCellArray> polyLineSegments;
    vtkSmartPointer<vtkPoints> polylinePoints;
    vtkSmartPointer<vtkPolyData> points;
    vtkSmartPointer<vtkParametricSpline> spline;
    vtkSmartPointer<vtkRenderer> ren;
    vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
    vtkSmartPointer<vtkPropAssembly> sphereAssembly;    //Assembly of spheres
    vtkSmartPointer<vtkActor> splineActor;
    QVTKOpenGLNativeWidget* qvtkwidget;
    std::vector<IMATI_STL::Vertex*> polyLine;
    vtkSmartPointer<vtkCellPicker> cellPicker;
    IMATI_STL::Vertex* firstVertex;
    IMATI_STL::Vertex* lastVertex;
    DrawableMesh* mesh;
    DrawableLineAnnotation* annotation;
    double sphereRadius;
    double tolerance;
    bool selectionMode;
    bool visiblePointsOnly;
    bool showSelectedPoints;
    bool alreadyStarted;
    bool lassoStarted;

    vtkIdType reachedID;
};

#endif // LINESELECTIONSTYLE_H
