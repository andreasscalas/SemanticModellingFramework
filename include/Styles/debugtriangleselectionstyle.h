#ifndef DEBUGTRIANGLESELECTIONSTYLE_H
#define DEBUGTRIANGLESELECTIONSTYLE_H

#include <vtkSmartPointer.h>
#include <vtkCellPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkCamera.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkCellData.h>
#include <vtkFrustumSource.h>
#include <vtkParametricFunctionSource.h>
#include <vtkImplicitFunction.h>
#include <vtkPlanes.h>
#include <vector>
#include <map>
#include <string>
#include "drawablemesh.h"
#include "utilities.h"
#include "imatistl.h"
#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

/**
 * @brief The TriangleSelectionStyle class controls the interaction with the Triangles of a mesh
 */
class DebugTriangleSelectionStyle : public vtkInteractorStyleRubberBandPick{

    public:
        constexpr static unsigned int RECTANGLE_AREA = 0;
        constexpr static unsigned int LASSO_AREA = 1;
        constexpr static unsigned int PAINTED_LINE = 2;
        constexpr static double TOLERANCE = 5e-2;
        constexpr static double RADIUS_RATIO = 100;

        static DebugTriangleSelectionStyle* New();
        DebugTriangleSelectionStyle();
        vtkTypeMacro(DebugTriangleSelectionStyle,vtkInteractorStyleRubberBandPick)

        void OnRightButtonDown() override;
        void OnMouseMove() override;
        void OnLeftButtonDown() override;
        void OnLeftButtonUp() override;
        void modifySelectedTriangles();
        void modifySelectedTrianglesId();
        void SetTriangles(vtkSmartPointer<vtkPolyData> triangles);
        void resetSelection();
        void defineSelection(std::vector<unsigned long> selected);

        DrawableMesh* getMesh() const;
        void setMesh(DrawableMesh *value);
        bool getShowSelectedTriangles() const;
        void setShowSelectedTriangles(bool value);
        vtkSmartPointer<vtkRenderer> getRen() const;
        void setRen(const vtkSmartPointer<vtkRenderer> &value);
        vtkSmartPointer<vtkPropAssembly> getAssembly() const;
        void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);
        std::map<unsigned long, bool> *getTrianglesSelectionStatus() const;
        void setTrianglesSelectionStatus(std::map<unsigned long, bool> *value);


        bool getSelectionMode() const;
        void setSelectionMode(bool value);

private:
        vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
        vtkSmartPointer<vtkPolyData> Triangles;
        std::map<unsigned long, bool>* trianglesSelectionStatus;
        vtkSmartPointer<vtkCellPicker> cellPicker;
        vtkSmartPointer<vtkRenderer> ren;
        bool selectionMode;
        bool showSelectedTriangles;
        DrawableMesh* mesh;

};

#endif // DEBUGTRIANGLESELECTIONSTYLE_H
