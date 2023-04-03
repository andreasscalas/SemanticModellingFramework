#include "mvcthread.h"

using namespace std;
using namespace IMATI_STL;
using namespace Eigen;

MVCThread::MVCThread(DrawableMesh* cage, Eigen::MatrixXd* coords, map<long, long> verticesID, Node* firstVertex, long begin, long end){
    this->cage = cage;
    this->coords = coords;
    this->firstVertex = firstVertex;
    this->begin = begin;
    this->end = end;
    this->verticesID = verticesID;
	numCol = cage->V.numels();

}

void MVCThread::startThread(){
     tid = new thread(MainThread::executeTaskHelper, this);
}

void MVCThread::executeTask(){

    long i = begin;
    for(Node* ni = firstVertex; i < end && ni != nullptr; ni = ni->next()){

        Node* nj;
        Triangle* t;
        Vertex* vj;
        Vertex* vi = (Vertex*) ni->data;
        Eigen::MatrixXd u;
        Eigen::VectorXd d;
		d.resize(numCol);
		u.resize(numCol,3);
        bool coincidentVertices = false, outside = false;
		int j = 0;

        /* Creation of the vectors corresponding to the segments connecting each
         * cage vertex with actual model vertex */
        List* V = &(cage->V);

        FOREACHVVVERTEX(V, vj, nj){
			Eigen::Vector3d v_i(vi->x, vi->y, vi->z);
			Eigen::Vector3d v_j(vj->x, vj->y, vj->z);
            Eigen::Vector3d v_ij = v_j - v_i;
            d(j) = v_ij.norm();
            if(d(j) < EPSILON){
				(*coords)(i, j) = 1;
                coincidentVertices = true;
                break;
			}
			else
				u.row(j) = v_ij / d(j);
			j++;
        }

        if(coincidentVertices){
            i++;
            continue;
        }

        double totalW = 0;

        List* T = &(cage->T);
        FOREACHVTTRIANGLE(T, t, nj){

            vector<double> l, teta, c, s;
            vector<Vertex*> v = {t->v1(), t->v2(), t->v3()}; //v?
            double h = 0;
            pair<long, long> id;

            for(int k = 0; k < 3; k++){
                pair<long, long> id1 = *verticesID.find((long) v[Utilities::mod((k - 1), 3)]);
                pair<long, long> id2 = *verticesID.find((long) v[Utilities::mod((k + 1), 3)]);
                l.push_back((u.row(id2.second) - u.row(id1.second)).norm());
                teta.push_back(((double) 2) * asin(l[k] / ((double) 2)));
                h += teta[k];
            }

            h /= 2;

            if(M_PI - h < EPSILON)
                for(int k = 0; k < 3; k++){
                    pair<long, long> id1 = *verticesID.find((long int) v[Utilities::mod((k - 1), 3)]);
                    pair<long, long> id2 = *verticesID.find((long int) v[Utilities::mod((k + 1), 3)]);
                    id = *verticesID.find((long) v[k]);
					(*coords)(i, id.second) += sin(teta[k]) * d(id1.second) * d(id2.second);
                }

            Matrix3d M;
            for(int k = 0; k < 3; k++){
                id = *verticesID.find((long int) v[k]);
                M.row(k) = u.row(id.second);
            }

            for(int k = 0; k < 3; k++){
                double sinkprev = sin(teta[Utilities::mod((k - 1), 3)]);
                double sinknext = sin(teta[Utilities::mod((k + 1), 3)]);
                double sinh = sin(h);
                double sinhminusteta = sin(h - teta[k]);
                c.push_back((((double) 2) * sinh * sinhminusteta) / (sinknext * sinkprev)-((double) 1) );
                double newCSquare = pow(c[k],((double) 2));
                if(newCSquare < 1)
                    s.push_back(Utilities::sign(M.determinant()) * sqrt(((double) 1) - newCSquare));
                else
                    s.push_back(0);
                if(abs(s[k]) <= EPSILON){
                    outside = true;
                    break;
                }
            }

            if(outside)
                continue;

            for(int k = 0; k < 3; k++){

                id = *verticesID.find((long) v[k]);
                double new_w = (teta[k] - c[Utilities::mod((k + 1), 3)] * teta[Utilities::mod((k - 1), 3)] -
                                c[Utilities::mod((k - 1), 3)] * teta[Utilities::mod((k + 1), 3)]);
                double sinknext = sin(teta[Utilities::mod((k + 1), 3)]);
                double skprev = s[Utilities::mod((k - 1), 3)];
                double tmp = new_w /(d[id.second] * sinknext * skprev);
				(*coords)(i, id.second) += tmp;
                totalW += tmp;
            }

        }

		for (j = 0; j < cage->V.numels(); j++) {
			(*coords)(i, j) /= totalW;
		}
        i++;
    }


}

void MVCThread::waitThread(){
    tid->join();
}

