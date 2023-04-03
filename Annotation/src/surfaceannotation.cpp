#include "surfaceannotation.h"
#include <annotationutilities.h>
#include <utilities.h>
#include <spthread.h>
#include <rapidjson/writer.h>

using namespace std;
using namespace IMATI_STL;

SurfaceAnnotation::SurfaceAnnotation()
{
    type = AnnotationType::Surface;
}

SurfaceAnnotation::~SurfaceAnnotation()
{
    outlines.clear();
}

Annotation* SurfaceAnnotation::transfer(ExtendedTrimesh* targetMesh, short metric){

    SurfaceAnnotation* otherAnnotation = new SurfaceAnnotation(); //The transferred annotation
    Vertex* v, *initialVertex;                      //Some support variable

    sphereRay = targetMesh->bboxLongestDiagonal() / BBOX_SPHERE_RATIO;
    for(Node* n = targetMesh->V.head(); n != nullptr; n=n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;


    for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){

        std::vector<Vertex*> otherOutline;              //The outline of the transferred annotation

        if(oit->size() != 0)
        {
            bool alreadyUsed = true;
            vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
            std::vector<Vertex*>::iterator vit = outline.begin();
            Vertex* v1, *v2;

            outline.pop_back();

            do{
                v = static_cast<Vertex*>(*vit);
                vector<Vertex*> neighbors = targetMesh->getNeighboursInSphere(*v, sphereRay);
                vector<Triangle*> toCheckTriangles;
                Utilities::findFaces(toCheckTriangles, neighbors);
                v1 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
                if(v1 == nullptr)
                    sphereRay *= 2;
            }while(v1 == nullptr);
            sphereRay = targetMesh->bboxLongestDiagonal() / BBOX_SPHERE_RATIO;
            initialVertex = v1;
            //v1->info = &alreadyUsed;

            for(; vit != outline.end(); vit++){
                do{
                    v = static_cast<Vertex*>(*vit);
                    vector<Vertex*> neighbors = targetMesh->getNeighboursInSphere(*v, sphereRay);
                    vector<Triangle*> toCheckTriangles;
                    Utilities::findFaces(toCheckTriangles, neighbors);
                    v2 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
                    if(v2 == nullptr)
                        sphereRay *= 2;
                    if(sphereRay > targetMesh->bboxLongestDiagonal())
                    {
                        std::cerr << "Isolated vertex" << std::endl;
                        std::cerr << "(" << v1->x << "," << v1->y << "," << v1->z << ")" << std::endl;
                        exit(9);
                    }

                }while(v2 == nullptr);

                sphereRay = targetMesh->bboxLongestDiagonal() / BBOX_SPHERE_RATIO;
                if(v2->info == nullptr || !(*static_cast<bool*>(v2->info))){
                    vector<Vertex*> path = Utilities::dijkstra(v1, v2, metric, !POST_PROCESSING);
                    for (vector<Vertex*>::iterator pit = path.begin(); pit != path.end(); pit++)
                        (*pit)->info = &alreadyUsed;

                    otherOutline.insert(otherOutline.end(), path.begin(), path.end());
                    v1 = v2;
                }
            }

            v2 = initialVertex;
            initialVertex->info = nullptr;
            vector<Vertex*> path = Utilities::dijkstra(v1, v2, metric, false);
            otherOutline.insert(otherOutline.end(), path.begin(), path.end());

            for(unsigned int i = 0; i < otherOutline.size(); i++)
                otherOutline[i]->info = nullptr;


            while((otherOutline[0] == otherOutline[otherOutline.size() - 1]) && (otherOutline[1] == otherOutline[otherOutline.size() - 2])){
                otherOutline.erase(otherOutline.begin());
                otherOutline.erase(otherOutline.begin() + static_cast<long>(otherOutline.size()) - 1);
            }


            if(POST_PROCESSING){
                v = otherOutline[0];
                otherOutline.erase(otherOutline.begin());
                std::vector<Vertex*> crossedVertices;

                for(vector<Vertex*>::iterator vit1 = otherOutline.begin(); vit1 != otherOutline.end(); vit1++){
                    v1 = static_cast<Vertex*>(*vit1);
                    if(std::find(crossedVertices.begin(), crossedVertices.end(), v1) == crossedVertices.end())
                        crossedVertices.push_back(v1);
                    else
                        for(vector<Vertex*>::iterator vit2 = vit1 - 1; vit2 != otherOutline.begin(); vit2--){
                            v2 = static_cast<Vertex*>(*vit2);

                            if(v2 == v1){
                                otherOutline.erase(vit2, vit1);
                                vit1 = vit2;
                                break;
                            }
                        }
                }

                otherOutline.insert(otherOutline.begin(), v);
            }

            outline.push_back(*outline.begin());
            otherOutline.push_back(*otherOutline.begin());
        }

        otherAnnotation->addOutline(otherOutline);  //The new annotation outline is computed
    }

    //The Outline and inner vertex have been found, the tag and color are the same, so the process ends.
    otherAnnotation->setId(this->id);
    otherAnnotation->setMesh(targetMesh);
    otherAnnotation->setTag(this->tag);
    otherAnnotation->setColor(this->color);
    otherAnnotation->setAttributes(this->attributes);
    otherAnnotation->setHierarchyLevel(this->hierarchyLevel);
    for(Node* n = targetMesh->V.head(); n != nullptr; n=n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;

    return otherAnnotation;

}

Annotation* SurfaceAnnotation::parallelTransfer(ExtendedTrimesh* targetMesh, short metric){

    SurfaceAnnotation* otherAnnotation = new SurfaceAnnotation(); //The transferred annotation
    Vertex* v, *initialVertex;                      //Some support variable

    sphereRay = targetMesh->bboxLongestDiagonal() / BBOX_SPHERE_RATIO;

    for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
        std::vector<Vertex*> otherOutline;              //The outline of the transferred annotation
        bool alreadyUsed = true;
        vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
        std::vector<Vertex*>::iterator vit = outline.begin();
        Vertex* v1, *v2;

        outline.pop_back();

        do{
            v = static_cast<Vertex*>(*vit);
            vector<Vertex*> neighbors = targetMesh->getNeighboursInSphere(*v, sphereRay);
            vector<Triangle*> toCheckTriangles;
            Utilities::findFaces(toCheckTriangles, neighbors);
            v1 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
            if(v1 == nullptr)
                sphereRay *= 2;
            v1->info = &alreadyUsed;
        }while(v1 == nullptr);
        initialVertex = v1;
        v1->info = &alreadyUsed;

        vector<SPThread*> spTasks;
        vector<vector<Vertex*> *> paths;


        for(; vit != outline.end(); vit++){
            v = static_cast<Vertex*>(*vit);
            vector<Vertex*> neighbors = targetMesh->getNeighboursInSphere(*v, sphereRay);
            vector<Triangle*> toCheckTriangles;
            Utilities::findFaces(toCheckTriangles, neighbors);
            v2 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
            if(v2 != nullptr && v2->info == nullptr){
                if(spTasks.size() >= NUM_OF_THREADS){
                    for(unsigned int i = 0; i < NUM_OF_THREADS; i++){
                        spTasks[i]->waitThread();
                        otherOutline.insert(otherOutline.end(), paths[i]->begin(), paths[i]->end());
                    }
                    paths.clear();
                    spTasks.clear();
                }
                paths.push_back(new vector<Vertex*>());

                spTasks.push_back(new SPThread(v1, v2, paths[paths.size() - 1], metric));
                spTasks[spTasks.size() - 1]->startThread();
                v1 = v2;
            }
        }
        for(unsigned int i = 0; i < spTasks.size(); i++){
            spTasks[i]->waitThread();
            otherOutline.insert(otherOutline.end(), paths[i]->begin(), paths[i]->end());
        }
        v2 = initialVertex;
        vector<Vertex*> path = Utilities::dijkstra(v1, v2, metric, false);
        otherOutline.insert(otherOutline.end(), path.begin(), path.end());

        while((otherOutline[0] == otherOutline[otherOutline.size() - 1]) && (otherOutline[1] == otherOutline[otherOutline.size() - 2])){
            otherOutline.erase(otherOutline.begin());
            otherOutline.erase(otherOutline.begin() + static_cast<long>(otherOutline.size()) - 1);
        }

        v = otherOutline[0];
        otherOutline.erase(otherOutline.begin());
        std::vector<Vertex*> crossedVertices;

        for(vector<Vertex*>::iterator vit1 = otherOutline.begin(); vit1 != otherOutline.end(); vit1++){
            v1 = static_cast<Vertex*>(*vit1);
            if(std::find(crossedVertices.begin(), crossedVertices.end(), v1) == crossedVertices.end())
                crossedVertices.push_back(v1);
            else
                for(vector<Vertex*>::iterator vit2 = vit1 - 1; vit2 != otherOutline.begin(); vit2--){
                    v2 = static_cast<Vertex*>(*vit2);

                    if(v2 == v1){
                        otherOutline.erase(vit2, vit1);
                        vit1 = vit2;
                        break;
                    }
                }
        }


        otherOutline.insert(otherOutline.begin(), v);
        outline.push_back(*outline.begin());
        otherOutline.push_back(*otherOutline.begin());

        /*cout<<"OLD"<<endl<<flush;
        for(unsigned int j = 0; j < otherOutline.size(); j++)
            cout<<"("<<otherOutline[j]->x<<","<<otherOutline[j]->y<<","<<otherOutline[j]->z<<")"<<endl<<flush;*/
        otherAnnotation->addOutline(otherOutline);  //The new annotation outline is computed
    }
    //The Outline and inner vertex have been found, the tag and color are the same, so the process ends.
    otherAnnotation->setMesh(this->mesh);
    otherAnnotation->setTag(this->tag);
    otherAnnotation->setColor(this->color);


    return otherAnnotation;

}

void SurfaceAnnotation::print(ostream &os)
{
    Annotation::print(os);
    os << "type: Region" << std::endl << "boundaries:" << std::endl << "[" << std::endl;
    for(unsigned int i = 0; i < outlines.size(); i++){
        os << "[ " ;
        for(unsigned j = 0; j < outlines[i].size(); j++){
            os << mesh->getPointId(outlines[i][j]);
            if(j < outlines[i].size() - 1)
                os << ", ";
        }

        os << " ]," << std::endl;
    }
    os << "]" << std::endl;
}

void SurfaceAnnotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
    writer.StartObject();
    Annotation::printJson(writer);
    writer.Key("type");
    writer.String("Area");
    writer.Key("boundaries");
    writer.StartArray();
    for(unsigned int i = 0; i < outlines.size(); i++){
        writer.StartArray();
        for(unsigned j = 0; j < outlines[i].size(); j++)
            writer.Int(static_cast<int>(mesh->getPointId(outlines[i][j])));
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
}

bool SurfaceAnnotation::isPointInAnnotation(Vertex *p)
{
    vector<Vertex*> involved = getInvolvedVertices();
    return isPointOnBorder(p) || isPointInsideAnnotation(p);
}

bool SurfaceAnnotation::isPointInsideAnnotation(Vertex *p)
{
    vector<Vertex*> involved = getInnerVertices();
    for(unsigned int i = 0; i < involved.size(); i++){
        if(involved[i] == p)
            return true;
    }

    return false;
}


bool SurfaceAnnotation::isPointOnBorder(IMATI_STL::Vertex* p)
{
    for(unsigned int i = 0; i < outlines.size(); i++)
        for(unsigned int j = 0; j < outlines[i].size(); j++)
            if(outlines[i][j] == p)
                return true;

    return false;
}

bool SurfaceAnnotation::isPointOnBorder(Vertex *p, unsigned int &boundaryIndex)
{
    for(unsigned int i = 0; i < outlines.size(); i++)
        for(unsigned int j = 0; j < outlines[i].size(); j++)
            if(outlines[i][j] == p){
                boundaryIndex = i;
                return true;
            }
    return false;
}

bool SurfaceAnnotation::isTriangleInAnnotation(Triangle *t)
{
    vector<IMATI_STL::Triangle*> triangles = getTriangles();

    for(unsigned int i = 0; i < triangles.size(); i++)
        if(triangles[i] == t)
            return true;
    return false;
}

std::pair<unsigned int, unsigned int> SurfaceAnnotation::getAdjacentBoundary(SurfaceAnnotation *a)
{
    vector<vector<Vertex*> > aOutlines = a->getOutlines();
    std::vector<Triangle*> aTriangles = a->getTriangles();
    std::vector<Triangle*> thisTriangles = this->getTriangles();
    for(unsigned int i = 0; i < this->outlines.size(); i++)
        for(unsigned int j = 0; j < this->outlines[i].size(); j++)
            for(unsigned int k = 0; k < aOutlines.size(); k++){
                vector<Vertex*>::iterator vit = std::find(aOutlines[k].begin(), aOutlines[k].end(), this->outlines[i][j]);
                if(vit != aOutlines[k].end())
                    return make_pair(i, k);
            }

    return make_pair(this->outlines.size(), a->getOutlines().size());

}

bool SurfaceAnnotation::checkAdjacency(SurfaceAnnotation *a)
{
    vector<vector<Vertex*> > aOutlines = a->getOutlines();
    std::vector<Triangle*> aTriangles = a->getTriangles();
    std::vector<Triangle*> thisTriangles = this->getTriangles();
    vector<Triangle*>::iterator tit1 = std::find(aTriangles.begin(), aTriangles.end(), thisTriangles[0]);
    vector<Triangle*>::iterator tit2 = std::find(thisTriangles.begin(), thisTriangles.end(), aTriangles[0]);
    for(unsigned int i = 0; i < this->outlines.size(); i++)
        for(unsigned int j = 0; j < this->outlines[i].size(); j++)
            for(unsigned int k = 0; k < aOutlines.size(); k++){
                vector<Vertex*>::iterator vit = std::find(aOutlines[k].begin(), aOutlines[k].end(), this->outlines[i][j]);
                if(vit != aOutlines[k].end() && tit1 == aTriangles.end() && tit2 == thisTriangles.end())
                    return true;
            }



    return false;
}

std::vector<int> SurfaceAnnotation::getIsBoundaryStatus()
{
    vector<Vertex*> vertices;
    vector<int> isBoundary;
    unsigned int BOUNDARY_EDGE = 1;
    unsigned int ALREADY_USED = 9;

    if(outlines.size() == 0){

        isBoundary.resize(this->mesh->V.numels());
        fill(isBoundary.begin(), isBoundary.end(), 0);

    }else{
        for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
            vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
            for(vector<Vertex*>::iterator vit = outline.begin(); vit != outline.end(); vit++){
                Vertex* v = static_cast<Vertex*>(*vit);
                v->info = &BOUNDARY_EDGE;
            }
        }

        vector<IMATI_STL::Triangle*> triangles = Utilities::regionGrowing(outlines);
        for(unsigned int i = 0; i < triangles.size(); i++){
            Vertex* v = triangles[i]->v1();
            for(unsigned int j = 0; j < 3; j++){
                v = triangles[i]->nextVertex(v);
                if(v->info == nullptr || (*static_cast<unsigned int*>(v->info) != ALREADY_USED && *static_cast<unsigned int*>(v->info) != BOUNDARY_EDGE)){
                    isBoundary.push_back(0);
                    v->info = &ALREADY_USED;
                }
            }
        }

        for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++)
            for(vector<Vertex*>::iterator vit = (*oit).begin(); vit != (*oit).end(); vit++)
                isBoundary.push_back(1);

        for(unsigned int i = 0; i < triangles.size(); i++){
            Vertex* v = triangles[i]->v1();
            for(int j = 0; j < 3; j++){
                v = triangles[i]->nextVertex(v);
                v->info = nullptr;
            }
        }

    }
    return isBoundary;
}

vector<vector<Vertex *> > SurfaceAnnotation::getOutlines() const{
    return outlines;
}

void SurfaceAnnotation::setOutlines(const vector<vector<Vertex *> > value){
    outlines.clear();
    outlines.insert(outlines.end(), value.begin(), value.end());
}

void SurfaceAnnotation::addOutline(const vector<Vertex *> value){
    outlines.push_back(value);
}

vector<Triangle *> SurfaceAnnotation::getTriangles(){
    vector<Triangle*> annotationTriangles;
    if(outlines.size() == 0){
        for(Node* n = this->mesh->T.head(); n != nullptr; n = n->next()){
            Triangle* t = static_cast<Triangle*>(n->data);
            annotationTriangles.push_back(t);
        }
    } else
        annotationTriangles = Utilities::regionGrowing(outlines);

    return annotationTriangles;
}

Point *SurfaceAnnotation::getCenter()
{
    vector<Vertex*> involvedVertices = getInvolvedVertices();
    Point* center = new Point(0,0,0);
    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        *center += *(involvedVertices[i]);
    }

    *center /= involvedVertices.size();

    return center;
}

Point *SurfaceAnnotation::getOrientation()
{
    vector<Vertex*> involvedVertices = getInvolvedVertices();
    Eigen::MatrixXd eigenPoints;
    eigenPoints.resize(3, static_cast<Eigen::Index>(involvedVertices.size()));

    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        Eigen::Vector3d p = {involvedVertices[i]->x, involvedVertices[i]->y, involvedVertices[i]->z};
        eigenPoints.col(i) = p;
    }

    Eigen::Vector3d mean_vector = eigenPoints.rowwise().mean();
    eigenPoints.colwise() -= mean_vector;
    Eigen::Matrix3d U = eigenPoints.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    Point* orientation = new Point();
    orientation->setValue(U(0,2), U(1,2), U(2,2));
    return orientation;
}

std::vector<Vertex *> SurfaceAnnotation::getInvolvedVertices()
{
    vector<Vertex*> vertices = getInnerVertices();

    for(unsigned int i = 0; i < outlines.size(); i++)
        vertices.insert(vertices.end(), outlines[i].begin(), outlines[i].end());

    return vertices;
}

std::vector<Vertex *> SurfaceAnnotation::getInnerVertices()
{

    vector<Vertex*> vertices;
    unsigned int BOUNDARY_VERTEX = 1;
    unsigned int ALREADY_USED = 9;

    if(outlines.size() == 0){

        for(Node* n = this->mesh->V.head(); n != nullptr; n = n->next()){
            Vertex* v = static_cast<Vertex*>(n->data);
            vertices.push_back(v);
        }

    }else{
        for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
            vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
            for(vector<Vertex*>::iterator vit = outline.begin(); vit != outline.end(); vit++){
                Vertex* v = static_cast<Vertex*>(*vit);
                v->info = &BOUNDARY_VERTEX;
            }
        }

        vector<IMATI_STL::Triangle*> triangles = Utilities::regionGrowing(outlines);
        for(unsigned int i = 0; i < triangles.size(); i++){
            Vertex* v = triangles[i]->v1();
            for(unsigned int j = 0; j < 3; j++){
                v = triangles[i]->nextVertex(v);
                if(v->info == nullptr || (*static_cast<unsigned int*>(v->info) != ALREADY_USED && *static_cast<unsigned int*>(v->info) != BOUNDARY_VERTEX)){
                    vertices.push_back(v);
                    v->info = &ALREADY_USED;
                }
            }
        }

        for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++)
            for(vector<Vertex*>::iterator vit = (*oit).begin(); vit != (*oit).end(); vit++)
                (*vit)->info = nullptr;

        for(unsigned int i = 0; i < vertices.size(); i++)
            vertices[i]->info = nullptr;

    }

    return vertices;

}
