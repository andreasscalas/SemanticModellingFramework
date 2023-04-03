#ifndef CAGEVERTICESSELECTIONSTYLE_H
#define CAGEVERTICESSELECTIONSTYLE_H

#include <drawablemesh.h>
#include <vector>
#include <map>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkCellPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkDataSetMapper.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkPolyData.h>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

/**
 * @brief The VerticesSelectionStyle class controls the interaction with the points of a mesh
 */
class CageVerticesSelectionStyle : public vtkInteractorStyleRubberBandPick{

    public:

        vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
        vtkSmartPointer<vtkPolyData> Points;
        vtkSmartPointer<vtkActor> SelectedActor;
        vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
        vtkSmartPointer<vtkCellPicker> modelTrianglePicker;
        vtkSmartPointer<vtkPointPicker> cagePointPicker;        //The point picker
        vtkSmartPointer<vtkRenderer> renderer;
        std::map<unsigned long, bool>* pointsSelectionStatus;
        std::map<unsigned int, std::vector<unsigned int> >* annotationToCage;
        QVTKOpenGLNativeWidget* qvtkwidget;
        bool selectionMode;
        bool visiblePointsOnly;
        DrawableMesh* model;
        DrawableMesh* cage;

        static CageVerticesSelectionStyle* New();

        CageVerticesSelectionStyle();

        vtkTypeMacro(CageVerticesSelectionStyle,vtkInteractorStyleRubberBandPick)

        virtual void OnRightButtonDown() override;

        virtual void OnMiddleButtonDown() override;

        virtual void OnLeftButtonDown() override;

        virtual void OnLeftButtonUp() override;

        void resetSelection();

        void defineSelection(std::vector<unsigned long> selected);

        void modifySelectedPoints();

        vtkSmartPointer<vtkPolyData> getPoints() const;

        void setPoints(const vtkSmartPointer<vtkPolyData> &value);

        vtkSmartPointer<vtkActor> getSelectedActor() const;

        void setSelectedActor(const vtkSmartPointer<vtkActor> &value);

        vtkSmartPointer<vtkDataSetMapper> getSelectedMapper() const;

        void setSelectedMapper(const vtkSmartPointer<vtkDataSetMapper> &value);

        std::map<unsigned long, bool> *getSelectedPoints() const;

        void setSelectedPoints(std::map<unsigned long, bool> *value);

        vtkSmartPointer<vtkPointPicker> getCagePointPicker() const;

        void setCagePointPicker(const vtkSmartPointer<vtkPointPicker> &value);

        bool getSelectionMode() const;

        void setSelectionMode(bool value);

        bool getVisiblePointsOnly() const;

        void setVisiblePointsOnly(bool value);

        DrawableMesh *getCage() const;

        void setCage(DrawableMesh *value);

        vtkSmartPointer<vtkPropAssembly> getAssembly() const;

        void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);

        QVTKOpenGLNativeWidget *getQvtkwidget() const;

        void setQvtkwidget(QVTKOpenGLNativeWidget *value);
        vtkSmartPointer<vtkCellPicker> getModelTrianglePicker() const;
        void setModelTrianglePicker(const vtkSmartPointer<vtkCellPicker> &value);
        DrawableMesh *getModel() const;
        void setModel(DrawableMesh *value);
        void setAnnotationToCage(std::map<unsigned int, std::vector<unsigned int> > *value);
        std::map<unsigned int, std::vector<unsigned int> > *getAnnotationToCage() const;
        vtkSmartPointer<vtkRenderer> getRenderer() const;
        void setRenderer(const vtkSmartPointer<vtkRenderer> &value);
};



#endif // CAGEVERTICESSELECTIONSTYLE_H




