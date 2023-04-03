#ifndef MEASURESTYLE_H
#define MEASURESTYLE_H

#include <drawableattribute.h>
#include <drawablemesh.h>

#include <vtkInteractorStyleTrackballCamera.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCellPicker.h>
#include <vtkPropAssembly.h>

class MeasureStyle : public vtkInteractorStyleTrackballCamera
{
public:
    enum class MeasureType {RULER, TAPE, CALIBER, BOUNDING, HEIGHT};
    const double epsilon = 1e-7;

    static MeasureStyle* New();
    MeasureStyle();
    ~MeasureStyle() override;
    vtkTypeMacro(MeasureStyle,vtkInteractorStyleTrackballCamera)

    void OnMouseMove() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void OnRightButtonDown() override;
    void OnMiddleButtonUp() override;
    void OnMiddleButtonDown() override;
    void OnMouseWheelBackward() override;
    void OnMouseWheelForward() override;

    void manageRulerMovement(IMATI_STL::Vertex* start, IMATI_STL::Vertex* end);
    void manageTapeMovement(IMATI_STL::Vertex* start, IMATI_STL::Vertex* end);
    void manageHeightMovement(IMATI_STL::Vertex* start);
    void manageCaliberMovement();
    void manageBoundingMovement();

    void reset();

    DrawableAttribute* finalizeAttribute(unsigned int id, std::string key);
    //Getters and setters
    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);
    QVTKOpenGLNativeWidget *getQvtkwidget() const;
    void setQvtkwidget(QVTKOpenGLNativeWidget *value);
    vtkSmartPointer<vtkRenderer> getMeshRenderer() const;
    void setMeshRenderer(const vtkSmartPointer<vtkRenderer> &value);
    MeasureType getMeasureType() const;
    void setMeasureType(const MeasureType &value);
    double getMeasure() const;
    std::vector<IMATI_STL::Vertex *> getMeasurePath() const;
    IMATI_STL::Point *getBoundingDirection() const;
    DrawableAttribute *getOnCreationAttribute() const;

    bool getDrawAttributes() const;
    void setDrawAttributes(bool value);

    void updateView();
protected:
    DrawableMesh* mesh;
    QVTKOpenGLNativeWidget* qvtkwidget;
    vtkSmartPointer<vtkCellPicker> cellPicker;
    vtkSmartPointer<vtkRenderer> meshRenderer;
    vtkSmartPointer<vtkPropAssembly> measureAssembly;
    std::vector<IMATI_STL::Vertex*> measurePath;
    MeasureType measureType;
    IMATI_STL::Vertex* last;
    IMATI_STL::Point * boundingBegin, * boundingEnd, * boundingOrigin;
    bool measureStarted, leftPressed, middlePressed, drawAttributes;
    double measure;
    DrawableAttribute* onCreationAttribute;

};

#endif // MEASURESTYLE_H
