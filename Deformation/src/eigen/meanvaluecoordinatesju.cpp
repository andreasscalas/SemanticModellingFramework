#include "meanvaluecoordinatesju.h"
using namespace IMATI_STL;
using namespace std;

MeanValueCoordinatesJu::MeanValueCoordinatesJu(DrawableMesh* model, DrawableMesh* cage){

    Node* n;
    Vertex* v;
    long i = 0;
    this->model = model;
    this->cage = cage;
	rowNum = model->V.numels();
	colNum = cage->V.numels();
	modelVertices.resize(rowNum, 3);
	cageVertices.resize(colNum, 3);
	coords.resize(rowNum, colNum);

    /* Considering that the algorithm updates the coordinates each time
     * that a triangle that contains the corresponding cage vertices is
     * found, it is neccessary that the algorithm knows the position,
     * inside the coordinates vector, corresponding to the cage vertex. */
    List* V = &(cage->V);
    FOREACHVVVERTEX(V, v, n)
         verticesID[(long) v] = i++;


     /* For similar reasons, the size of the vector must be defined before the
      * computation of the coordinates. */
	coords = Eigen::MatrixXd::Zero(rowNum, colNum);
    

    QString s = QString::fromStdString(model->getFilename());
    s.truncate(s.lastIndexOf('.'));
    s.append(".coord");
    coordsFileName = s.right(s.size() - s.lastIndexOf("/") - 1);
}

MeanValueCoordinatesJu::~MeanValueCoordinatesJu()
{
    model = nullptr;
    cage = nullptr;
    verticesID.clear();
	coords.resize(0,0);
}

void MeanValueCoordinatesJu::computeCoordinates(){

    ifstream myfile(coordsFileName.toStdString());

    /* If the coordinates file has not yet been created, the algorithm
     * proceeds to the computation phase. */
    if (!myfile.is_open()){
        long i = 0, begin = 0, end;
        MainThread* threads[NUM_THREADS];
        long verticesPerThread = rowNum / NUM_THREADS + 1;
        int t = 0;
        Node* ni;
        Vertex *v;

        //For each vertex of the model, the algorithm computes the corrisponding MVC
        List* V = &(model->V);
        FOREACHVVVERTEX(V, v, ni){

            /* The vertices set is splitted into parts, each of them managed by a different
             * thread, to parallelize the execution and reduce the computation time*/
            if(i == begin){
                end = begin + verticesPerThread;
                threads[t] = new MVCThread(cage, &coords, verticesID, ni, begin, end);
                threads[t++]->startThread();
                begin = end;
            }
            i++;

        }

        for(int j = 0; j < NUM_THREADS; j++){
            threads[j]->waitThread();
        }

    }

}

void MeanValueCoordinatesJu::saveCoordinates(QString filename){

    ofstream savefile;
    savefile.open(filename.toStdString());
    savefile<<"Mean Value Coordinates"<<endl<<flush;
    savefile<<rowNum<<" "<<colNum<<endl<<flush;
    for(long i = 0; i < rowNum; i++){

        for(long j = 0; j < colNum; j++)
            savefile<<coords(i,j)<<" ";

        savefile<<std::endl<<flush;
    }
    savefile.close();

}

void MeanValueCoordinatesJu::loadCoordinates(QString filename){

    ifstream loadfile;
    loadfile.open(filename.toStdString());
    std::string line;
    std::getline(loadfile, line);
    std::getline(loadfile, line);
    istringstream* iss = new istringstream(line);
    int modelVerticesNumber;
    (*iss) >> modelVerticesNumber;
    for(unsigned int i = 0; i < modelVerticesNumber; i++){

        int j = 0;
        std::getline(loadfile, line);
        iss = new istringstream(line);
        double coord;
        while((*iss) >> coord)
            coords(i,j++) = coord;

    }
    loadfile.close();

}


void MeanValueCoordinatesJu::deform(){

    long i = 0, j;
    Node *ni, *nj;
    Vertex *vi, *vj;
    // For each vertex of the model, the new position of it is calculated using the coordinates properties.
    List *MV = &(model->V), *CV = &(cage->V);
	/*j = 0;
	FOREACHVVVERTEX(CV, vj, nj) 
		cageVertices.row(j++) << vj->x, vj->y, vj->z;

	modelVertices = coords * cageVertices;

	FOREACHVVVERTEX(MV, vi, ni) { 
		vi->setValue(modelVertices(i, 0), modelVertices(i, 1), modelVertices(i, 2));
		i++;
	}*/
	

   FOREACHVVVERTEX(MV, vi, ni){
		j = 0;
        Point v(0, 0, 0);

        FOREACHVVVERTEX(CV, vj, nj){
            v += (*vj) * coords(i,j);
            j++;
        }

        vi->setValue(v.x, v.y, v.z);
        i++;

    }

}

Eigen::MatrixXd MeanValueCoordinatesJu::getCoordinates(){
    return this->coords;
}
