#include <meshdeformationstyle.h>
#include <vtkOBBTree.h>
#include <vtkOutlineFilter.h>
#include <utilities.h>

using namespace std;
using namespace IMATI_STL;


std::vector<unsigned int> MeshDeformationStyle::getLaplacianConstraintsID() const
{
    return laplacianConstraintsID;
}

void MeshDeformationStyle::setLaplacianConstraintsID(const std::vector<unsigned int> &value)
{
    laplacianConstraintsID = value;
}

InfoPalette *MeshDeformationStyle::getInfoPalette() const
{
    return infoPalette;
}

void MeshDeformationStyle::setInfoPalette(InfoPalette *value)
{
    infoPalette = value;
}

std::vector<AnnotationsConstraint *> MeshDeformationStyle::getSemanticConstraints() const
{
    return semanticConstraints;
}

void MeshDeformationStyle::setSemanticConstraints(const std::vector<AnnotationsConstraint *> &value)
{
    semanticConstraints = value;
}

MeshDeformationStyle::MeshDeformationStyle(){
    this->translation = false;
    this->rotation = false;
    this->pointPicker = vtkSmartPointer<vtkPointPicker>::New();
    this->constrained = false;
    this->outlineActor = vtkSmartPointer<vtkActor>::New();
    this->stretchBegun = false;
    this->shrinkBegun = false;
}

void MeshDeformationStyle::drawBB(vtkSmartPointer<vtkPropAssembly> canvas)
{
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkOutlineFilter> outline = vtkSmartPointer<vtkOutlineFilter>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    canvas->RemovePart(outlineActor);
    outlineActor = vtkSmartPointer<vtkActor>::NewInstance(outlineActor);
    for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){
        if((*pit).second){
            points->InsertNextPoint(this->data->GetPoint((*pit).first));
        }
    }
    polyData->SetPoints(points);
    outline->SetInputData(polyData);
    outline->Update();
    outlineMapper->SetInputConnection(outline->GetOutputPort());
    outlineActor->GetProperty()->SetOpacity(1.0);
    outlineActor->GetProperty()->SetLineWidth(5.0);
    outlineActor->GetProperty()->SetColor(1,0,0);
    outlineActor->SetMapper(outlineMapper);
    canvas->AddPart(outlineActor);
    canvas->Modified();
}

void MeshDeformationStyle::OnMouseWheelForward()
{
    int mousePosition[2];
    this->Interactor->GetEventPosition(mousePosition);
    this->FindPokedRenderer(mousePosition[0], mousePosition[1]);
    if(this->Interactor->GetAltKey())
    {
        std::vector<IMATI_STL::Vertex*> selected;
        Eigen::MatrixXd cP;
        Eigen::Vector3d center;
        for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++)
            if(pit->second)
                selected.push_back(cage->getPoint(pit->first));
        cP.resize(3, selected.size());
        for(unsigned int i = 0; i < selected.size(); i++)
        {
            Eigen::Vector3d p = {selected[i]->x, selected[i]->y, selected[i]->z};
            cP.col(i) = p;
            center += p;
        }
        center/= selected.size();

        Eigen::Matrix3d scalingMatrix;
        scalingMatrix.setIdentity(3,3);
        scalingMatrix *= 1.1;
        cP.colwise() -= center;
        cP = scalingMatrix * cP;
        cP.colwise() += center;

        for(unsigned int i = 0; i < selected.size(); i++)
        {
            double newPosition[3] = {cP.col(i).x(), cP.col(i).y(), cP.col(i).z()};
            (*cagePoints).row(cage->getPointId(selected[i])) = cP.col(i);
            this->data->GetPoints()->SetPoint(cage->getPointId(selected[i]), newPosition);
            selected[i]->setValue(newPosition[0],newPosition[1], newPosition[2]);
        }

        this->data->Modified();
        applyDeformation();
        qvtkwidget->update();
    } else
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
}

void MeshDeformationStyle::OnMouseWheelBackward()
{
    if(this->Interactor->GetAltKey())
    {
        std::vector<IMATI_STL::Vertex*> selected;
        Eigen::MatrixXd cP;
        Eigen::Vector3d center;
        for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++)
            if(pit->second)
                selected.push_back(cage->getPoint(pit->first));
        cP.resize(3, selected.size());
        for(unsigned int i = 0; i < selected.size(); i++)
        {
            Eigen::Vector3d p = {selected[i]->x, selected[i]->y, selected[i]->z};
            cP.col(i) = p;
            center += p;
        }
        center/= selected.size();

        Eigen::Matrix3d scalingMatrix;
        scalingMatrix.setIdentity(3,3);
        scalingMatrix *= 0.9;
        cP.colwise() -= center;
        cP = scalingMatrix * cP;
        cP.colwise() += center;

        for(unsigned int i = 0; i < selected.size(); i++)
        {
            double newPosition[3] = {cP.col(i).x(), cP.col(i).y(), cP.col(i).z()};
            (*cagePoints).row(cage->getPointId(selected[i])) = cP.col(i);
            this->data->GetPoints()->SetPoint(cage->getPointId(selected[i]), newPosition);
            selected[i]->setValue(newPosition[0],newPosition[1], newPosition[2]);
        }

        this->data->Modified();
        applyDeformation();
        qvtkwidget->update();
    } else
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
}

bool MeshDeformationStyle::getStretch() const
{
    return stretch;
}

void MeshDeformationStyle::setStretch(bool value)
{
    stretch = value;
}

ShapeOpBarycentricCoordinates *MeshDeformationStyle::getSoCoords() const
{
    return soCoords;
}

void MeshDeformationStyle::setSoCoords(ShapeOpBarycentricCoordinates *value)
{
    soCoords = value;
}

ShapeOp::MatrixX3 *MeshDeformationStyle::getModelPoints() const
{
    return modelPoints;
}

void MeshDeformationStyle::setModelPoints(ShapeOp::MatrixX3 *value)
{
    modelPoints = value;
}

ShapeOp::MatrixX3 *MeshDeformationStyle::getCagePoints() const
{
    return cagePoints;
}

void MeshDeformationStyle::setCagePoints(ShapeOp::MatrixX3 *value)
{
    cagePoints = value;
}

std::vector<unsigned int> MeshDeformationStyle::getClosenessConstraintsID() const
{
    return closenessConstraintsID;
}

void MeshDeformationStyle::setClosenessConstraintsID(const std::vector<unsigned int> &value)
{
    closenessConstraintsID = value;
}

std::vector<std::shared_ptr<ShapeOp::Constraint> > MeshDeformationStyle::getConstraints() const
{
    return constraints;
}

void MeshDeformationStyle::setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value)
{
    constraints = value;
}

ConstraintSolver *MeshDeformationStyle::getSolver() const
{
    return solver;
}

void MeshDeformationStyle::setSolver(ConstraintSolver *value)
{
    solver = value;
}

bool MeshDeformationStyle::getConstrained() const
{
    return constrained;
}

void MeshDeformationStyle::setConstrained(bool value)
{
    constrained = value;
}

void MeshDeformationStyle::applyDeformation()
{
    if(coordsComputed){
        soCoords->deform();
        if(constrained){
            ShapeOp::Matrix3X mpt;
            mpt.resize(3, modelPoints->rows());
            mpt = modelPoints->transpose();
            for(unsigned int i = 0; i < closenessConstraintsID.size(); i++){
                auto c = static_cast<ShapeOp::ClosenessConstraint*>(constraints[closenessConstraintsID[i]].get());
                ShapeOp::Vector3 pos = modelPoints->row(c->getIds()[0]);
                c->setPosition(pos);
            }

            solver->setModelPointsTransposed(mpt);

            solver->solve(2);
            for (unsigned int i = 0; i < cagePoints->rows(); i++) {
                double newPosition[3] = {(*cagePoints)(i, 0), (*cagePoints)(i, 1), (*cagePoints)(i, 2)};
                this->data->GetPoints()->SetPoint(static_cast<vtkIdType>(i), newPosition);
                this->cage->setPointPosition(static_cast<vtkIdType>(i), newPosition);
            }
            this->data->Modified();
            soCoords->deform();

//            std::cout << "Errors list: [" << std::endl;
//            for(unsigned int i = 0; i < semanticConstraints.size(); i++){
//                std::cout << "Semantic constraints n°" << i << ": ";
//                std::cout << "Constraint error: " << semanticConstraints[i]->getError() << std::endl;
//            }

//            std::cout << "]" << std::endl<< std::endl << std::flush;
        }

        infoPalette->setConstraints(constraints);
        infoPalette->updateInfo();
        infoPalette->update();
        this->GetCurrentRenderer()->RemoveActor(assembly);
        this->model->setMeshModified(true);
        this->model->update();
        this->model->draw(assembly);
        this->assembly->Modified();
        this->GetCurrentRenderer()->AddActor(assembly);
        this->qvtkwidget->update();
    }
}

void MeshDeformationStyle::OnMouseMove(){

    /* While the mouse is moved on the display, the point position is updated
     * as the point obtained projecting the point selected in the camera plane
     * on the dragging plane */
    int mousePosition[2];
    double position[4];
    this->Interactor->GetEventPosition(mousePosition);
    this->ComputeDisplayToWorld(mousePosition[0], mousePosition[1], 0, position);
    double shift[3] = {position[0] - clickPos[0], position[1] - clickPos[1], position[2] - clickPos[2]};

    if(this->Interactor->GetAltKey() && (stretchBegun || shrinkBegun)){
        Eigen::Vector3d s = {shift[0], shift[1], shift[2]};
        Eigen::Vector3d v = {shift[0], shift[1], shift[2]};//= {0, 0, s.norm()};
        v.normalize();
        clickPos[0] = shift[0];
        clickPos[1] = shift[1];
        clickPos[2] = shift[2];
        if(shrinkBegun) v = -v;
        if(v.norm() < EPSILON)
            return;
        v.normalize();
        vector<Vertex*> selected;
        vector<Eigen::Vector3d> positions2D;
        unsigned int maxPos = INT_MAX, minPos = INT_MAX;
        double max = -DBL_MAX, min = DBL_MAX;
        vector<double> ts;
        unsigned int i = 0;
        Eigen::Vector3d xAxis = {1, 0, 0};
        Eigen::Vector3d yAxis = {0, 1, 0};
        Eigen::Vector3d zAxis = {0, 0, 1};
        for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){

            std::pair<ulong, bool> p = (*pit);
            if(p.second){
                double t;
                double *oldPosition = this->data->GetPoint(static_cast<vtkIdType>(p.first));
                Eigen::Vector3d point = {oldPosition[0], oldPosition[1], oldPosition[2]};
                Eigen::Vector3d projectedPoint = (point.dot(v) / v.dot(v)) * v;
                selected.push_back(cage->getPoint(p.first));
                positions2D.push_back(projectedPoint);
                if(std::abs(v(0)) > EPSILON)
                    t = projectedPoint(0) / v(0);
                else if(std::abs(v(1)) > EPSILON)
                    t = projectedPoint(1) / v(1);
                else
                    t = projectedPoint(2) / v(2);

                ts.push_back(t);

                if(t > max){
                    maxPos = i;
                    max = t;
                }
                if(t < min){
                    minPos = i;
                    min = t;
                }

                i++;
            }

        }

        for(i = 0; i < selected.size(); i++){
            double newPosition[3];
            if(v.dot(xAxis) < 0)
                newPosition[0] = selected[i]->x + v(0) * abs(ts[i] - max) * 0.05;
            else
                newPosition[0] = selected[i]->x + v(0) * abs(ts[i] - min) * 0.05;
            if(v.dot(yAxis) < 0)
                newPosition[1] = selected[i]->y + v(1) * abs(ts[i] - max) * 0.05;
            else
                newPosition[1] = selected[i]->y + v(1) * abs(ts[i] - min) * 0.05;
            if(v.dot(zAxis) < 0)
                newPosition[2] = selected[i]->z + v(2) * abs(ts[i] - max) * 0.05;
            else
                newPosition[2] = selected[i]->z + v(2) * abs(ts[i] - min) * 0.05;
            (*cagePoints)(cage->getPointId(selected[i]), 0) = newPosition[0];
            (*cagePoints)(cage->getPointId(selected[i]), 1) = newPosition[1];
            (*cagePoints)(cage->getPointId(selected[i]), 2) = newPosition[2];
            this->data->GetPoints()->SetPoint(cage->getPointId(selected[i]), newPosition);
            selected[i]->x = newPosition[0];
            selected[i]->y = newPosition[1];
            selected[i]->z = newPosition[2];
        }

        this->data->Modified();
        applyDeformation();
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];
        qvtkwidget->update();
    } else if (this->Interactor->GetControlKey() && translation){

        /* While the mouse is moved on the display, the point position is updated
         * as the point obtained projecting the point selected in the camera plane
         * on the dragging plane */

        for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){

            std::pair<ulong, bool> p = (*pit);
            if(p.second){
                double *oldPosition = this->data->GetPoint(static_cast<vtkIdType>(p.first));
                double newPosition[3] = {oldPosition[0] + shift[0], oldPosition[1] + shift[1], oldPosition[2] + shift[2]};
                (*cagePoints)(p.first, 0) = newPosition[0];
                (*cagePoints)(p.first, 1) = newPosition[1];
                (*cagePoints)(p.first, 2) = newPosition[2];
                this->data->GetPoints()->SetPoint(static_cast<vtkIdType>(p.first), newPosition);
                this->cage->setPointPosition(static_cast<vtkIdType>(p.first), newPosition);
            }
        }

        this->data->Modified();
        applyDeformation();

        qvtkwidget->update();
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];

    }else if(this->Interactor->GetControlKey() && rotation){
        Eigen::Vector3d mouseRay(shift[0], shift[1], shift[2]);
        int* sizes = this->GetCurrentRenderer()->GetRenderWindow()->GetSize();
        double ss[4];
        this->ComputeDisplayToWorld(sizes[0], sizes[1], 0, ss);
        Eigen::Vector3d maxPos(ss[0], ss[1], ss[2]);
        double rotationMagnitude = M_PI * mouseRay.norm() / maxPos.norm();
        Eigen::Vector3d barycenter(0, 0, 0);
        int n = 0;
        for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){
            std::pair<ulong, bool> p = (*pit);
            if(p.second){
                double *oldPosition = this->data->GetPoint(static_cast<vtkIdType>(p.first));
                barycenter(0) += oldPosition[0];
                barycenter(1) += oldPosition[1];
                barycenter(2) += oldPosition[2];
                n++;
            }
        }
        barycenter /= n;
        if(mouseRay.norm() > 0.001){
            double* pn = this->GetCurrentRenderer()->GetActiveCamera()->GetViewPlaneNormal();
            mouseRay.normalize();
            Eigen::Vector3d planeNormal(pn[0], pn[1], pn[2]);
            Eigen::Vector3d rotationAxis = -1 * mouseRay.cross(planeNormal);
            rotationAxis.normalize();
            AndreasStructures::Quaternion q(rotationMagnitude, rotationAxis(0), rotationAxis(1), rotationAxis(2));
            q.normalize();

            for(std::map<ulong, bool>::iterator pit = pointsSelectionStatus->begin(); pit != pointsSelectionStatus->end(); pit++){

                std::pair<ulong, bool> p = (*pit);
                if(p.second){
                    double *oldPosition = this->data->GetPoint(static_cast<vtkIdType>(p.first));
                    oldPosition[0] -= barycenter(0);
                    oldPosition[1] -= barycenter(1);
                    oldPosition[2] -= barycenter(2);
                    AndreasStructures::Quaternion q_;
                    q_.setW(0);
                    q_.setX(oldPosition[0]);
                    q_.setY(oldPosition[1]);
                    q_.setZ(oldPosition[2]);
                    AndreasStructures::Quaternion _q(q.getConjugate());
                    AndreasStructures::Quaternion np(q*q_*_q);
                    double newPosition[3] = {np.getX() + barycenter(0), np.getY() + barycenter(1), np.getZ() + barycenter(2)};
                    ShapeOp::Vector3 tmp = {newPosition[0], newPosition[1], newPosition[2]};
                    cagePoints->row(p.first) = tmp;
                    this->data->GetPoints()->SetPoint(static_cast<vtkIdType>(p.first), newPosition);
                    this->cage->setPointPosition(static_cast<vtkIdType>(p.first), newPosition);
                }
            }

            this->data->Modified();

            applyDeformation();

            qvtkwidget->update();
            clickPos[0] = position[0];
            clickPos[1] = position[1];
            clickPos[2] = position[2];
        }


    }else if(translation) //The interaction is stopped
        this->translation = false;
    else if(rotation)
        this->rotation = false;

    vtkInteractorStyleTrackballCamera::OnMouseMove();

}

void MeshDeformationStyle::OnRightButtonUp(){

    //The interaction is stopped
    this->translation = false;
    this->shrinkBegun = false;

    vtkInteractorStyleTrackballCamera::OnRightButtonUp();

}

void MeshDeformationStyle::OnRightButtonDown(){

    //If the user is trying to pick a point...
    if(this->Interactor->GetControlKey()){
        int mousePosition[2];
        double position[4];

        this->Interactor->GetEventPosition(mousePosition);
        this->FindPokedRenderer(mousePosition[0], mousePosition[1]);
        this->ComputeDisplayToWorld(mousePosition[0], mousePosition[1], 0, position);
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];

        this->translation = true;

        if(pointsSelectionStatus->size() == 0){

            //Some tolerance is set for the picking
            this->pointPicker->SetTolerance(0.01);
            this->pointPicker->Pick(mousePosition[0], mousePosition[1], 0, this->GetCurrentRenderer());
            //If some point has been picked...
            if(this->pointPicker->GetPointId() > 0 && this->pointPicker->GetPointId() < this->cage->V.numels()){

                this->pointsSelectionStatus->insert(std::make_pair(this->pointPicker->GetPointId(), true));
                cage->setSelectedPoints(*pointsSelectionStatus);

            }
        }
    } else if (this->Interactor->GetAltKey() && stretch){
        int mousePosition[2];
        double position[4];

        this->Interactor->GetEventPosition(mousePosition);
        this->FindPokedRenderer(mousePosition[0], mousePosition[1]);
        this->ComputeDisplayToWorld(mousePosition[0], mousePosition[1], 0, position);
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];
        this->shrinkBegun = true;
    } else
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
}

void MeshDeformationStyle::OnLeftButtonUp()
{
    //The interaction is stopped
    this->rotation = false;
    this->stretchBegun = false;

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void MeshDeformationStyle::OnLeftButtonDown()
{
    //If the user is trying to pick a point...
    if(this->Interactor->GetControlKey()){
        int mousePosition[2];
        double position[4];

        this->Interactor->GetEventPosition(mousePosition);
        this->FindPokedRenderer(mousePosition[0], mousePosition[1]);
        this->ComputeDisplayToWorld(mousePosition[0], mousePosition[1], 0, position);
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];
        this->rotation = true;
    } else if (this->Interactor->GetAltKey() && stretch){
        int mousePosition[2];
        double position[4];

        this->Interactor->GetEventPosition(mousePosition);
        this->FindPokedRenderer(mousePosition[0], mousePosition[1]);
        this->ComputeDisplayToWorld(mousePosition[0], mousePosition[1], 0, position);
        clickPos[0] = position[0];
        clickPos[1] = position[1];
        clickPos[2] = position[2];
        this->stretchBegun = true;
    }else
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

QVTKWidget *MeshDeformationStyle::getQvtkwidget() const
{
    return qvtkwidget;
}

void MeshDeformationStyle::setQvtkwidget(QVTKWidget *value)
{
    qvtkwidget = value;
}

vtkSmartPointer<vtkPropAssembly> MeshDeformationStyle::getAssembly() const
{
return assembly;
}

void MeshDeformationStyle::setAssembly(const vtkSmartPointer<vtkPropAssembly> &value)
{
assembly = value;
}

DrawableMesh *MeshDeformationStyle::getModel() const
{
return model;
}

void MeshDeformationStyle::setModel(DrawableMesh *value)
{
model = value;
}

DrawableMesh *MeshDeformationStyle::getCage() const
{
return cage;
}

void MeshDeformationStyle::setCage(DrawableMesh *value)
{
    cage = value;
}

std::map<ulong, bool> *MeshDeformationStyle::getSelectedPoints() const
{
    return pointsSelectionStatus;
}

void MeshDeformationStyle::setSelectedPoints(std::map<ulong, bool> *value)
{
    pointsSelectionStatus = value;
}

vtkSmartPointer<vtkPolyData> MeshDeformationStyle::getData() const
{
    return data;
}

void MeshDeformationStyle::setData(const vtkSmartPointer<vtkPolyData> &value)
{
    data = value;
}

bool MeshDeformationStyle::getCoordsComputed() const
{
    return coordsComputed;
}

void MeshDeformationStyle::setCoordsComputed(bool value)
{
    coordsComputed = value;
}

void MeshDeformationStyle::checkConstraintsSeamlessly()
{

}
