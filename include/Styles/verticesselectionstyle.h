#ifndef VERTICESSELECTIONSTYLE_H
#define VERTICESSELECTIONSTYLE_H

#include "imatistl.h"
#include "drawablemesh.h"
#include "utilities.h"
#include <pointannotation.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkCoordinate.h>
#include <vtkCamera.h>
#include <vtkDataSetMapper.h>
#include <vtkAreaPicker.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkPlanes.h>
#include <vtkImplicitFunction.h>
#include <QVTKWidget.h>
#include <vector>
#include <map>
#include <string>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

/**
 * @brief The VerticesSelectionStyle class controls the interaction with the points of a mesh
 */
class VerticesSelectionStyle : public vtkInteractorStyleRubberBandPick {

private:
    vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
    vtkSmartPointer<vtkPolyData> Points;
	vtkSmartPointer<vtkActor> SelectedActor;
	vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
	std::map<unsigned long, bool>* pointsSelectionStatus;
	std::vector<unsigned long> selectedPoints;
	vtkSmartPointer<vtkPointPicker> pointPicker;        //The point picker
	QVTKWidget* qvtkwidget;
	bool selectionMode;
	bool visiblePointsOnly;
    DrawableMesh* mesh;
    PointAnnotation* annotation;

public:
    static VerticesSelectionStyle* New();

    VerticesSelectionStyle();

    vtkTypeMacro(VerticesSelectionStyle, vtkInteractorStyleRubberBandPick)

    virtual void OnRightButtonDown() override;

	virtual void OnLeftButtonDown() override;

	virtual void OnLeftButtonUp() override;

	void resetSelection();

	void defineSelection(std::vector<unsigned long> selected);

    void modifySelectedPoints();

    void finalizeAnnotation(unsigned int id, std::string tag, unsigned char color[]);

    vtkSmartPointer<vtkPolyData> getPoints() const;
    void setPoints(const vtkSmartPointer<vtkPolyData>& value);
	vtkSmartPointer<vtkActor> getSelectedActor() const;
	void setSelectedActor(const vtkSmartPointer<vtkActor>& value);
	vtkSmartPointer<vtkDataSetMapper> getSelectedMapper() const;
	void setSelectedMapper(const vtkSmartPointer<vtkDataSetMapper>& value);
	std::map<unsigned long, bool>* getSelectedPoints() const;
	void setSelectedPoints(std::map<unsigned long, bool>* value);
	vtkSmartPointer<vtkPointPicker> getPointPicker() const;
	void setPointPicker(const vtkSmartPointer<vtkPointPicker>& value);
	bool getSelectionMode() const;
	void setSelectionMode(bool value);
	bool getVisiblePointsOnly() const;
	void setVisiblePointsOnly(bool value);
    DrawableMesh* getMesh() const;
    void setMesh(DrawableMesh* value);
    vtkSmartPointer<vtkPropAssembly> getAssembly() const;
    void setAssembly(const vtkSmartPointer<vtkPropAssembly>& value);
	QVTKWidget* getQvtkwidget() const;
    void setQvtkwidget(QVTKWidget* value);
    std::map<unsigned long, bool> *getPointsSelectionStatus() const;
    void setPointsSelectionStatus(std::map<unsigned long, bool> *value);
};



#endif // VERTICESSELECTIONSTYLE_H
