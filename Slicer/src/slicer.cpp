#include "slicer.h"
#include <utilities.h>
#include <contourcontainmenttreenode.h>

using namespace std;
using namespace IMATI_STL;

Slicer::Slicer(){
    transformationMatrix << 1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
}

void Slicer::slice(){

    /*******************************I punti estratti dal cutter sono i punti di intersezione tra piano ed edge del modello****************************/

    vector<vector<Point*> > boundaries;
    map<Edge*, bool> checkedEdges;

    for(Node* n = mesh->E.head(); n != nullptr; n = n->next()){
        Edge* e = static_cast<Edge*>(n->data);
        map<Edge*, bool>::iterator eit = checkedEdges.find(e);
        if(eit == checkedEdges.end()){
            checkedEdges[e] = true;
            Vertex* startingVertex = static_cast<Vertex*>(Utilities::edgePlaneIntersection(*e, normal, center));
            Vertex* v1 = startingVertex;
            if(*v1 != INFINITE_POINT){
                Triangle* t = e->t1;
                vector<Point*> boundary;
                boundary.push_back(v1);
                Edge* e_ = t->nextEdge(e);
                Vertex* v2;
                while(e_ != e){
                    checkedEdges[e_] = true;
                    v2 = static_cast<Vertex*>(Utilities::edgePlaneIntersection(*e_, normal, center));
                    if(*v2 != INFINITE_POINT){
                        if(*(boundary.back()) != (*v2)){
                            boundary.push_back(v2);
                            v1 = v2;
                        }
                        t = e_->oppositeTriangle(t);
                    }
                    e_ = t->nextEdge(e_);
                }

                boundaries.push_back(boundary);
            }
        }
    }


    for(vector<vector<Point*> >::iterator bit = boundaries.begin(); bit != boundaries.end(); ){

        double minAngle = DBL_MAX;
        unsigned int newStartingPosition = 0;
        vector<Point*> contour = static_cast<vector<Point*> >(*bit);
        //contour.pop_back();

        for(unsigned int i = 0; i < contour.size(); i++){
            Point* v1 = contour[static_cast<unsigned int>(Utilities::mod(static_cast<int>(i - 1), static_cast<int>(contour.size())))];
            Point* v2 = contour[i];
            Point* v3 = contour[(i + 1) % (contour.size())];
            double angle = v2->getAngle(v1, v3);
            if(angle< minAngle){
                newStartingPosition = i;
                minAngle = angle;
            }
        }

        rotate(contour.begin(), contour.begin() + newStartingPosition, contour.end());

        *bit= contour;
        if((*bit).size() < 5){
            boundaries.erase(bit);
        }else
            bit++;
    }


    for(unsigned int i = 0; i < boundaries.size(); i++)
        for (unsigned int j = i + 1; j < boundaries.size(); j++)
            if(Utilities::areBoundariesEqual(boundaries[i], boundaries[j]))
                boundaries.erase(boundaries.begin() + j--);
    vector<vector<Point*> > boundaries2D;
    for(unsigned int i = 0; i < boundaries.size(); i++){
        vector<Point*> boundary2D = Utilities::transformVertexList(boundaries[i], transformationMatrix.inverse());
        boundaries2D.push_back(boundary2D);
    }

    vector<ContourContainmentTreeNode* > nodes;
    unsigned int i;
    for(i = 0; i < boundaries2D.size(); i++){
        nodes.push_back(new ContourContainmentTreeNode());
        nodes.back()->setData(&(boundaries2D[i]));
    }

    for(i = 0; i < boundaries2D.size(); i++){

        for(unsigned int j = 0; j < boundaries2D.size(); j++){
            if(i == j)
                continue;

            if(Utilities::isPointInsidePolygon(boundaries2D[j][0], boundaries2D[i])){
                nodes[i]->addSon(nodes[j]);
                nodes[j]->addContainer(nodes[i]);
            }
        }

    }

    for(i = 0; i < nodes.size(); i++)
        if(nodes[i]->getContainers().size() == 0)
            nodes[i]->removeSonsCycles();


    for(i = 0; i < nodes.size(); i++)
        if(nodes[i]->getContainers().size() == 0){

            nodes[i]->reorderBoundaries(0);
            vector<Slice*> contained = nodes[i]->getContainedSlices();
            slices.insert(slices.end(), contained.begin(), contained.end());
            for(unsigned int j = 0; j < slices.size(); j++){
                slices[j]->setFromPlaneTransformation(transformationMatrix);
                slices[j]->triangulate();
            }

        }
}

IMATI_STL::Point Slicer::getNormal() const{
    return normal;
}

void Slicer::setNormal(const IMATI_STL::Point &value)
{
    normal = value;
}

IMATI_STL::Point Slicer::getCenter() const
{
    return center;
}

void Slicer::setCenter(const IMATI_STL::Point &value)
{
    center = value;
}

ExtendedTrimesh *Slicer::getMesh() const
{
    return mesh;
}

void Slicer::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

std::vector<Slice*> Slicer::getSlices() const
{
    return slices;
}

void Slicer::setSlices(const std::vector<Slice*> &value)
{
    slices = value;
}

double Slicer::getMaxSimplificationError() const
{
    return maxSimplificationError;
}

void Slicer::setMaxSimplificationError(double value)
{
    maxSimplificationError = value;
}

Eigen::Matrix4d Slicer::getTransformationMatrix() const
{
    return transformationMatrix;
}

void Slicer::setTransformationMatrix(const Eigen::Matrix4d &value)
{
    transformationMatrix = value;
}
