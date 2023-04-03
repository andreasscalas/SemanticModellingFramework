#include <shapeopmeanvaluecoordinates.h>
#include <shapeopmvcthread.h>
#include <imatistl.h>
#include <fstream>

using namespace IMATI_STL;
using namespace std;

ShapeOpMeanValueCoordinates::ShapeOpMeanValueCoordinates(ExtendedTrimesh *mesh, ExtendedTrimesh *cage, ShapeOp::MatrixX3* meshVertices, ShapeOp::MatrixX3* cageVertices){

    Node* n;
    Vertex* v;
    this->mesh = mesh;
    this->cage = cage;
    this->meshVertices = meshVertices;
    this->cageVertices = cageVertices;

    /* Considering that the algorithm updates the coordinates each time
     * that a triangle that contains the corresponding cage vertices is
     * found, it is neccessary that the algorithm knows the position,
     * inside the coordinates vector, corresponding to the cage vertex. */
    List* V = &(cage->V);
    long i = 0;
    FOREACHVVVERTEX(V, v, n)
        verticesID[(long) v] = i++;

     /* For similar reasons, the size of the vector must be defined before the
      * computation of the coordinates. */
    coords = new ShapeOp::SparseMatrix();
    coords->resize(mesh->V.numels(), cage->V.numels());
    if(mesh->V.numels() == cage->V.numels())
        coords->setIdentity();
    else
        coords->setZero();
    string s = this->coordsFileName;
    s.resize(mesh->getFilename().find_last_of("."));
    s.append(".coord");
    s.substr(s.find_first_of("/") + 1);
    coordsFileName = s.substr(s.find_first_of("/") + 1);
}

ShapeOpMeanValueCoordinates::~ShapeOpMeanValueCoordinates()
{
    mesh = nullptr;
    cage = nullptr;
    verticesID.clear();
    delete coords;
}

void ShapeOpMeanValueCoordinates::computeCoordinates(){

    ifstream myfile(coordsFileName);

    /* If the coordinates file has not yet been created, the algorithm
     * proceeds to the computation phase. */
    if (!myfile.is_open() && mesh->V.numels() != cage->V.numels()){
        long i = 0, begin = 0, end;
        ShapeOp::MatrixXX tmpCoords;
        tmpCoords.setZero(coords->rows(), coords->cols());
        MainThread* threads[NUM_THREADS];
        long verticesPerThread = mesh->V.numels() / NUM_THREADS + 1;
        int t = 0;
        Node* ni;
        Vertex *v;
        Eigen::initParallel();
        //For each vertex of the model, the algorithm computes the corrisponding MVC
        List* V = &(mesh->V);
        std::mutex mtx;
        FOREACHVVVERTEX(V, v, ni){

            /* The vertices set is splitted into parts, each of them managed by a different
             * thread, to parallelize the execution and reduce the computation time*/
            if(i == begin){
                end = begin + verticesPerThread;
                threads[t] = new ShapeOpMVCThread(meshVertices, cageVertices, &(cage->T), &tmpCoords, verticesID, begin, end);
                static_cast<ShapeOpMVCThread*>(threads[t])->setMtx(&mtx);
                threads[t++]->startThread();
                begin = end;
            }
            i++;

        }

        for(int j = 0; j < NUM_THREADS; j++){
            threads[j]->waitThread();
        }


        std::vector<ShapeOp::Triplet> triplets;
        for(unsigned int i = 0; i < mesh->V.numels(); i++){
            for(unsigned int j = 0; j < cage->V.numels(); j++){
                if(abs(tmpCoords(i,j)) > 1e-3){
                    ShapeOp::Triplet tmp(i, j, tmpCoords(i,j));
                    triplets.push_back(tmp);
                }
            }
        }
        coords->setFromTriplets(triplets.begin(), triplets.end());
    }
    analyseMaxInfluenceThreshold();


}

void ShapeOpMeanValueCoordinates::saveCoordinates(std::string filename){

    ofstream savefile;
    savefile.open(filename);
    savefile<<"Mean Value Coordinates"<<endl<<flush;
    savefile<<mesh->V.numels()<<" "<<cage->V.numels()<<endl<<flush;
    ShapeOp::MatrixXX tmpCoords = coords->toDense();
    tmpCoords.resize(coords->rows(), coords->cols());
    for(long i = 0; i < coords->rows(); i++){
        for(long j = 0; j < coords->cols(); j++)
            savefile<<(tmpCoords)(i,j)<<" ";

        savefile<<std::endl<<flush;
    }
    savefile.close();

}

void ShapeOpMeanValueCoordinates::loadCoordinates(std::string filename){

    ifstream loadfile;
    loadfile.open(filename);
    std::string line;
    std::getline(loadfile, line);
    std::getline(loadfile, line);
    istringstream* iss = new istringstream(line);
    int modelVerticesNumber, cageVerticesNumber;
    ShapeOp::MatrixXX tmpCoords;
    (*iss) >> modelVerticesNumber;
    (*iss) >> cageVerticesNumber;
    tmpCoords.resize(modelVerticesNumber, cageVerticesNumber);
    std::vector<ShapeOp::Triplet> triplets;
    for(unsigned int i = 0; i < modelVerticesNumber; i++){

        int j = 0;
        std::getline(loadfile, line);
        iss = new istringstream(line);
        double coord;
        while((*iss) >> coord){
            tmpCoords(i,j) = coord;
            if(abs(coord) > 1e-4){
                ShapeOp::Triplet tmp(i,j,coord);
                triplets.push_back(tmp);
            }
            j++;
        }

    }

    coords->setFromTriplets(triplets.begin(), triplets.end());
    analyseMaxInfluenceThreshold();

    loadfile.close();

}


void ShapeOpMeanValueCoordinates::deform(){

    long i = 0;
    // For each vertex of the model, the new position of it is calculated using the coordinates properties.

    (*meshVertices) = (*coords) * (*cageVertices);
    for(Node* n = mesh->V.head(); n != nullptr; n = n->next()){
        Vertex *vi = static_cast<Vertex*>(n->data);
        vi->setValue((*meshVertices)(i, 0), (*meshVertices)(i, 1), (*meshVertices)(i, 2));
        i++;
    }

}



double ShapeOpMeanValueCoordinates::getThreshold() const
{
    return threshold;
}

void ShapeOpMeanValueCoordinates::setThreshold(double value)
{
    threshold = value;
}

void ShapeOpMeanValueCoordinates::analyseMaxInfluenceThreshold()
{
    Eigen::VectorXd maxValues;
    maxValues.resize(coords->outerSize());
    for(unsigned int i = 0; i < coords->outerSize(); i++)
    {
        double max = 0;
        for(Eigen::SparseMatrix<double>::InnerIterator it(*coords, i); it; ++it)
            if(it.value()>max)
                max = it.value();
        maxValues(i) = max;
    }
    threshold = maxValues.minCoeff();

    vector<Eigen::Triplet<bool> > triplets;
    for(unsigned int i = 0; i < coords->outerSize(); i++)
        for(Eigen::SparseMatrix<double>::InnerIterator it(*coords, i); it; ++it)
            if(it.value() > threshold - 1e-5)
            {
                Eigen::Triplet<bool> t(it.row(), it.col(), true);
                triplets.push_back(t);
            }

    maxInfluenceValues.resize(coords->rows(), coords->cols());
    maxInfluenceValues.setFromTriplets(triplets.begin(), triplets.end());

}


ShapeOp::SparseMatrix *ShapeOpMeanValueCoordinates::getCoordinates() const
{
    return coords;
}

std::vector<unsigned int> ShapeOpMeanValueCoordinates::getMaxInfluenceCageVertices(unsigned int vertexId)
{
    std::vector<unsigned int> cageVertices;
    for(Eigen::SparseMatrix<bool,Eigen::RowMajor>::InnerIterator it(maxInfluenceValues, vertexId); it; ++it)
        cageVertices.push_back(it.col());
    return cageVertices;
}
