#ifndef TRIANGLESELECTIONSTYLE_H
#define TRIANGLESELECTIONSTYLE_H

#include <drawablemesh.h>
#include <drawableareaannotation.h>

#include <vector>
#include <map>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkCellPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkSpline.h>
#include <vtkRenderer.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkPolyData.h>
#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

/**
 * @brief The TriangleSelectionStyle class controls the interaction with the Triangles of a mesh
 */
class TriangleSelectionStyle : public vtkInteractorStyleRubberBandPick{

    public:
        constexpr static unsigned int RECTANGLE_AREA = 0;
        constexpr static unsigned int LASSO_AREA = 1;
        constexpr static unsigned int PAINTED_LINE = 2;
        constexpr static double TOLERANCE = 5e-2;
        constexpr static double RADIUS_RATIO = 100;

        static TriangleSelectionStyle* New();
        TriangleSelectionStyle();
        vtkTypeMacro(TriangleSelectionStyle,vtkInteractorStyleRubberBandPick)

        void OnRightButtonDown() override;
        void OnMouseMove() override;
        void OnLeftButtonDown() override;
        void OnLeftButtonUp() override;
        void modifySelectedTriangles();
        void SetTriangles(vtkSmartPointer<vtkPolyData> triangles);
        void resetSelection();
        void defineSelection(std::vector<unsigned long> selected);
        void finalizeAnnotation(unsigned int id, std::string tag, unsigned char color[]);

        Annotation* getAnnotation() const;
        DrawableMesh* getMesh() const;
        void setMesh(DrawableMesh *value);
        std::vector<IMATI_STL::Vertex*> getPolygonContour() const;
        void setPolygonContour(const std::vector<IMATI_STL::Vertex*> &value);
        IMATI_STL::Vertex* getInnerVertex() const;
        void setInnerVertex(IMATI_STL::Vertex* value);
        unsigned int getSelectionType() const;
        void setSelectionType(unsigned int value);
        bool getShowSelectedTriangles() const;
        void setShowSelectedTriangles(bool value);
        bool getVisibleTrianglesOnly() const;
        void setVisibleTrianglesOnly(bool value);
        bool getSelectionMode() const;
        void setSelectionMode(bool value);
        vtkSmartPointer<vtkRenderer> getRen() const;
        void setRen(const vtkSmartPointer<vtkRenderer> &value);
        vtkSmartPointer<vtkPropAssembly> getAssembly() const;
        void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);
        std::map<unsigned long, bool> *getTrianglesSelectionStatus() const;
        void setTrianglesSelectionStatus(std::map<unsigned long, bool> *value);


        QVTKOpenGLNativeWidget *getQvtkWidget() const;
        void setQvtkWidget(QVTKOpenGLNativeWidget *value);

private:
        vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
        vtkSmartPointer<vtkPropAssembly> sphereAssembly;    //Assembly of spheres
        vtkSmartPointer<vtkPolyData> Triangles;
        vtkSmartPointer<vtkActor> SplineActor;
        std::map<unsigned long, bool>* pointsSelectionStatus;
        std::map<unsigned long, bool>* trianglesSelectionStatus;
        vtkSmartPointer<vtkCellPicker> cellPicker;      //The cell picker
        vtkSmartPointer<vtkPoints> splinePoints;
        vtkSmartPointer<vtkSpline> spline;
        vtkSmartPointer<vtkRenderer> ren;
        QVTKOpenGLNativeWidget* qvtkWidget;
        double sphereRadius;
        bool selectionMode;
        bool visibleTrianglesOnly;
        bool showSelectedTriangles;
        bool alreadyStarted;
        bool lasso_started;
        unsigned int selectionType;
        DrawableAreaAnnotation* annotation;
        DrawableMesh* mesh;
        IMATI_STL::Vertex* firstVertex;
        IMATI_STL::Vertex* lastVertex;
        IMATI_STL::Vertex* innerVertex;
        std::vector<IMATI_STL::Vertex*> polygonContour;

};

#endif // TRIANGLESELECTIONSTYLE_H
