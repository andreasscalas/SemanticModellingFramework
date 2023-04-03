#ifndef MESHDEFORMATIONSTYLE_H
#define MESHDEFORMATIONSTYLE_H

#include <infopalette.h>
#include <drawablemesh.h>
//#include <barycentriccoordinates.h>
#include <shapeopbarycentriccoordinates.h>
#include <constraintsolver.h>
#include <annotationsconstraint.h>
#include <map>

#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <QVTKOpenGLNativeWidget.h>

#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

/**
 * @brief The MeshDeformationStyle class controls the interaction with the points of a cage
 */
class MeshDeformationStyle : public vtkInteractorStyleTrackballCamera{

public:
        const double EPSILON = 1e-5;
        vtkSmartPointer<vtkPolyData> data;                  //Linking between coordinates and ids
        vtkSmartPointer<vtkPointPicker> pointPicker;        //The point picker
        vtkSmartPointer<vtkPropAssembly> assembly;              //Assembly of actors
        vtkSmartPointer<vtkActor> outlineActor;
        QVTKOpenGLNativeWidget* qvtkwidget;
        InfoPalette* infoPalette;
        DrawableMesh* model;
        DrawableMesh* cage;                                 //The cage
        ConstraintSolver* solver;
        ShapeOp::MatrixX3* modelPoints;
        ShapeOp::MatrixX3* cagePoints;
        ShapeOpBarycentricCoordinates* soCoords;
        std::vector<unsigned int> closenessConstraintsID;
        std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
        std::vector<AnnotationsConstraint*> semanticConstraints;
        bool coordsComputed;
        bool constrained;
        bool stretch;
        bool stretchBegun;
        bool shrinkBegun;
        int i = 0, minPos = -1, maxPos = -1;

        bool translation, rotation;                                //True if some point has been picked
        std::map<ulong, bool>* pointsSelectionStatus;              //Id of the picked point
        double clickPos[3];

        static MeshDeformationStyle* New();

        MeshDeformationStyle();

        vtkTypeMacro(MeshDeformationStyle, vtkInteractorStyleTrackballCamera)

        void drawBB(vtkSmartPointer<vtkPropAssembly> canvas);

        void OnMouseWheelForward() override;
        void OnMouseWheelBackward() override;

        void OnMouseMove() override;

        void OnRightButtonUp() override;

        void OnRightButtonDown() override;

        void OnLeftButtonUp() override;

        void OnLeftButtonDown() override;

        QVTKOpenGLNativeWidget *getQvtkwidget() const;

        void setQvtkwidget(QVTKOpenGLNativeWidget *value);

        vtkSmartPointer<vtkPropAssembly> getAssembly() const;

        void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);

        DrawableMesh *getModel() const;

        void setModel(DrawableMesh *value);

        DrawableMesh *getCage() const;

        void setCage(DrawableMesh *value);

        std::map<ulong, bool> *getSelectedPoints() const;

        void setSelectedPoints(std::map<ulong, bool> *value);

        vtkSmartPointer<vtkPolyData> getData() const;

        void setData(const vtkSmartPointer<vtkPolyData> &value);

        bool getCoordsComputed() const;

        void setCoordsComputed(bool value);

        void checkConstraintsSeamlessly();
        ShapeOpBarycentricCoordinates *getSoCoords() const;
        void setSoCoords(ShapeOpBarycentricCoordinates *value);
        ShapeOp::MatrixX3 *getModelPoints() const;
        void setModelPoints(ShapeOp::MatrixX3 *value);
        ShapeOp::MatrixX3 *getCagePoints() const;
        void setCagePoints(ShapeOp::MatrixX3 *value);
        std::vector<unsigned int> getClosenessConstraintsID() const;
        void setClosenessConstraintsID(const std::vector<unsigned int> &value);
        std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const;
        void setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value);
        ConstraintSolver *getSolver() const;
        void setSolver(ConstraintSolver *value);
        bool getConstrained() const;
        void setConstrained(bool value);

        bool getStretch() const;
        void setStretch(bool value);

        InfoPalette *getInfoPalette() const;
        void setInfoPalette(InfoPalette *value);

        std::vector<AnnotationsConstraint *> getSemanticConstraints() const;
        void setSemanticConstraints(const std::vector<AnnotationsConstraint *> &value);

protected:
        void applyDeformation();
};
#endif // MESHDEFORMATIONSTYLE_H
