#include "greencoordinates.h"
#include <imatistl.h>
#include <fstream>
#include <gcthread.h>
#include <Eigen/Dense>
#include <utilities.h>
using namespace std;
using namespace IMATI_STL;
GreenCoordinates::GreenCoordinates(DrawableMesh* model, DrawableMesh* cage){

    long i = 0;
    Node* n;
    Vertex* v;
    Triangle* t;
    List *V, *T;
    ImatiSTL::init();
    this->model = model;
    this->cage = cage;

    /* Considering that the algorithm extension on the cage exterior updates
     * the coordinates only for the vertices outside the cage, it's neccessary
     * that the algorithm knows the position, inside the coordinates vector,
     * corresponding to the model vertex */
    V = &(this->model->V);
    FOREACHVVVERTEX(V, v, n)
        this->mverticesID[(long) v] = i++;

    i = 0;
    /* Considering that the algorithm updates the coordinates each time
     * that a triangle that contains the corresponding cage vertices is
     * found, it is neccessary that the algorithm knows the position,
     * inside the coordinates vector, corresponding to the cage vertex. */
    V = &(this->cage->V);
    FOREACHVVVERTEX(V, v, n)
        this->verticesID[(long) v] = i++;

    i = 0;
    /* Considering that the algorithm extension on the cage exterior updates
     * the coordinates only for the triangles outside the cage, it's neccessary
     * that the algorithm knows the position, inside the triangle coordinates vector,
     * corresponding to the cage triangle. Furthermore, considering that the algorithm
     * computes the scaling factors using the area of each triangle of the cage before
     * the deformation and the area of the same triangles after the deformation, it's
     * neccessary to keep track of the triangles before the deformation */
    T = &(this->cage->T);
    FOREACHVTTRIANGLE(T, t, n){
        trianglesID[(long) t] = i++;
        Point u, v;
        u = (*(t->v2()) - (*(t->v1())));
        v = (*(t->v3()) - (*(t->v1())));
        std::pair<Point, Point> tBefore(u, v);
        trianglesBefore.push_back(tBefore);
        scalingFactors.push_back(0);
    }

    //Initialization of the vertices and triangles coordinates vectors
    for(i = 0; i < model->V.numels(); i++){
        vector<double> vcoordLine;
        vector<double> tcoordLine;
        for(int j = 0; j < cage->V.numels(); j++)
            vcoordLine.push_back(0);

        for(int j = 0; j < cage->T.numels(); j++)
            tcoordLine.push_back(0);

        verticesCoords.push_back(vcoordLine);
        trianglesCoords.push_back(tcoordLine);
    }




	string s = this->coordsFileName;
	s.resize(model->getFilename().find_last_of("."));
	s.append(".coord");
	s.substr(s.find_first_of("/") + 1);
	coordsFileName = s.substr(s.find_first_of("/") + 1);
}

GreenCoordinates::~GreenCoordinates()
{
    model = nullptr;
    cage = nullptr;
    trianglesID.clear();
    trianglesBefore.clear();
    trianglesCoords.clear();
    scalingFactors.clear();
    mverticesID.clear();
    verticesID.clear();
    trianglesID.clear();
}

void GreenCoordinates::computeCoordinates(){

    /* To allow the avoidance of the computation of the coordinates, if they have already been computed (and stored)
     * the method checks if exist some file that contains them (the existance condition is based on the name
     * of the model)*/
    std::string sv = coordsFileName;
    sv.append("_vertices.coord");
    ifstream myfile (sv);

    if (!myfile.is_open()){

        /* To allow the reduction of computation time, the whole process is divided in subtastks,
         * each one on a separate thread, which are executed in parallel */
        long i = 0, begin = 0, end;
        std::vector<Vertex*> outerVertices;
        vector<MainThread*> threads;
        long verticesPerThread = model->V.numels() / NUM_THREADS + 1;
        mutex mtx;

        Node* ni;
        Vertex* v;
        for(ni = model->V.head(); ni != nullptr; ni = ni->next()){

            v = static_cast<Vertex*>(ni->data);
            if(i == begin){
                end = begin + verticesPerThread;
                if(end > model->V.numels())
                    end = model->V.numels();
                threads.push_back(new GCThread(cage, &verticesCoords, &trianglesCoords, verticesID, ni, begin, end, &outerVertices, &mtx));
                threads.back()->startThread();
                begin = end;
            }
            i++;

        }

        for(unsigned int t = 0; t < static_cast<unsigned int>(NUM_THREADS); t++)
            threads[t]->waitThread();

        cageExteriorExtension(outerVertices);

    }

}

void GreenCoordinates::cageExteriorExtension(std::vector<Vertex*> outerVertices){

    std::vector<Triangle*> outerFaces;
    vector<Triangle*> exitFaces;
    vector<pair<Triangle*, Vertex*> > exitToVert;
    unsigned long i;

    //Outer faces retrieval
    Utilities::findFaces(outerFaces, outerVertices);

    //We are interested on the cage faces which intersect some of the outer faces
    for(i = 0; i < outerFaces.size(); i++){

        Node* nc;
        Triangle* t;
        for(nc = this->cage->T.head(); nc != nullptr; nc = nc->next()){
            t = static_cast<Triangle*>(nc->data);
            if(t->intersects(outerFaces[i]))
                exitFaces.push_back(t);
        }
    }

    /* We link each vertex with a single face, that is the first face for which
     * the vertex lies in the subspace defined by the plane passing on the face
     * and its outward normal. If no face meets this requirement, the closer face
     * is linked with the vertex.*/
    for(i = 0; i < outerVertices.size(); i++){

        Vertex* eta = outerVertices[i];
        bool isLinked = false;
        vector<Triangle*>::iterator it = exitFaces.begin();
        while(!isLinked){

            Triangle* currentExitF;
            if(it != exitFaces.end()){

                currentExitF = static_cast<Triangle*>(*it);
                Point vf = (*eta) - currentExitF->getCenter();
                double side = currentExitF->getNormal() * vf;
                if(side >= 0){

                    exitToVert.push_back(pair<Triangle*, Vertex*>(currentExitF, eta));
                    isLinked = true;

                }
                it++;

            }else{

                currentExitF = Utilities::getCloserFace(exitFaces, eta);
                exitToVert.push_back(pair<Triangle*, Vertex*>(currentExitF, eta));
                isLinked = true;

            }

        }

    }

    /* Updating of the coordinates that corresponds to an outer vertex of the model
     * and the three vertices of the linked face */
    for(i = 0; i < exitToVert.size(); i++){

        pair<Triangle*, Vertex*> actualLink = exitToVert[i];
        Vertex* v = actualLink.second;
        Vertex* v1 = actualLink.first->v1();
        Vertex* v2 = actualLink.first->v2();
        Vertex* v3 = actualLink.first->v3();
        Point normal = actualLink.first->getNormal();
        Eigen::Vector4f coefficients = {static_cast<float>(v->x),
                                        static_cast<float>(v->y),
                                        static_cast<float>(v->z), 1};
        Eigen::Matrix4f systemMatrix;

        //System matrix instantiation
        systemMatrix << static_cast<float>(v1->x), static_cast<float>(v2->x), static_cast<float>(v3->x), static_cast<float>(normal.x),
                        static_cast<float>(v1->y), static_cast<float>(v2->y), static_cast<float>(v3->y), static_cast<float>(normal.y),
                        static_cast<float>(v1->z), static_cast<float>(v2->z), static_cast<float>(v3->z), static_cast<float>(normal.z),
                        1.0f,                         1.0f,                         1.0f,                         1.0f;
        systemMatrix.determinant();

        //System resolution
        Eigen::FullPivLU<Eigen::Matrix4f> lu(systemMatrix);
        Eigen::Vector4f results = lu.solve(coefficients);

        //Updating of the corresponding coordinates
        std::pair<long, long> id = *mverticesID.find((long) v);
        std::pair<long, long> id0 = *verticesID.find((long) v1);
        verticesCoords[id.second][id0.second] += results[0];
        std::pair<long, long> id1 = *verticesID.find((long) v2);
        verticesCoords[id.second][id1.second] += results[1];
        std::pair<long, long> id2 = *verticesID.find((long) v3);
        verticesCoords[id.second][id2.second] += results[2];
        std::pair<long, long> id3 = *trianglesID.find((long) actualLink.first);
        trianglesCoords[id.second][id3.second] += results[3];

    }

}

void GreenCoordinates::saveCoordinates(std::string filename){

    ofstream savefile;

    savefile.open(filename);
    savefile<<"Green Coordinates"<<endl<<flush;
    savefile<<model->V.numels()<<" "<<cage->V.numels()<<endl<<flush;
    for(int i = 0; i < verticesCoords.size(); i++){
        for(int j = 0; j < verticesCoords[0].size(); j++)
            savefile<<verticesCoords[i][j]<<" ";
        savefile<<std::endl;
    }

    for(int i = 0; i < trianglesCoords.size(); i++){
        for(int j = 0; j < trianglesCoords[0].size(); j++)
            savefile<<trianglesCoords[i][j]<<" ";
        savefile<<std::endl;
    }
    savefile.close();

}

void GreenCoordinates::loadCoordinates(std::string filename){

    ifstream loadfile;

    loadfile.open(filename);
    std::string line;
    int j;
    std::getline(loadfile, line);
    std::getline(loadfile, line);
    istringstream* iss = new istringstream(line);
    int modelVerticesNumber, cageVerticesNumber;
    (*iss) >> modelVerticesNumber;
    (*iss) >> cageVerticesNumber;
    for(unsigned int i = 0; i < modelVerticesNumber; i++){

        std::getline(loadfile, line);
        iss = new istringstream(line);
        double coord;
        j = 0;
        while((*iss) >> coord)
            verticesCoords[i][j++] = coord;
    }

    for(unsigned int i = 0; i < modelVerticesNumber; i++){
        std::getline(loadfile, line);
        iss = new istringstream(line);
        double coord;
        j = 0;
        while((*iss) >> coord)
            trianglesCoords[i][j++] = coord;
    }

    loadfile.close();

}

void GreenCoordinates::computeScalingFactors(){

    long i = 0;
    List* T = &(cage->T);
    Node* n;
    Triangle* t;

    FOREACHVTTRIANGLE(T, t, n){

        double numerator, denominator;
        Point u, v, u_, v_;
        std::pair<Point, Point> tBefore = trianglesBefore[i];

        //Vetors that define a triangle before the deformation
        u = tBefore.first;
        v = tBefore.second;

        //Vectors that define the same triangle after the deformation
        u_ = (*(t->v2()) - (*(t->v1())));
        v_ = (*(t->v3()) - (*(t->v1())));

        //The formula for the computation of the scaling factors can be found on the paper (see class description)
        numerator = sqrt( pow(u_.length(), 2) * pow(v.length(), 2) -
                          2 * (u_ * v_) * (u * v) +
                          pow(v_.length(), 2) * pow(u.length(), 2));

        denominator = sqrt(8) * ((u & v).length() / 2);

        scalingFactors[i++] = numerator / denominator;

    }

}

void GreenCoordinates::deform(){

    long i = 0, j;
    Node *ni, *nj;
    List *MV, *CV, *T;
    Vertex *vi, *vj;
    Triangle* t;

    computeScalingFactors();

    MV = &(this->model->V);
    CV = &(this->cage->V);
    T = &(this->cage->T);
    FOREACHVVVERTEX(MV, vi, ni){
        j = 0;
        Point v(0,0,0);
        FOREACHVVVERTEX(CV, vj, nj){
            v += (*vj) * verticesCoords[i][j];
            j++;
        }

        j = 0;
        FOREACHVTTRIANGLE(T, t, nj){
            v += t->getNormal() * trianglesCoords[i][j] * scalingFactors[j];
            j++;
        }

        vi->setValue(v.x, v.y, v.z);
        i++;

    }

}

vector<vector<double> > GreenCoordinates::getCoordinates(){

    return this->verticesCoords;

}
