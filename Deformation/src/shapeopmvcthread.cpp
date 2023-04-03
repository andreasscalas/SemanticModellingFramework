#include "shapeopmvcthread.h"
#include <utilities.h>

using namespace std;
using namespace ShapeOp;
using namespace IMATI_STL;

ShapeOpMVCThread::ShapeOpMVCThread(ShapeOp::MatrixX3 *modelPoints, ShapeOp::MatrixX3 *cagePoints, IMATI_STL::List* cageTriangles, ShapeOp::MatrixXX *coords, std::map<long, long> verticesID, long begin, long end)
{
    this->modelPoints = modelPoints;
    this->cagePoints = cagePoints;
    this->cageTriangles = cageTriangles;
    this->coords = coords;
    this->begin = begin;
    this->end = end;
    this->verticesID = verticesID;

}

void ShapeOpMVCThread::startThread(){
     tid = new thread(MainThread::executeTaskHelper, this);
}

void ShapeOpMVCThread::setMtx(std::mutex *value)
{
    mtx = value;
}

void ShapeOpMVCThread::executeTask(){

    for (long i = begin; i < end && i < modelPoints->rows(); i++) {

        Vector3 vi = (*modelPoints).row(i);
        MatrixX3 u;
        u.resize(cagePoints->rows(), 3);
        VectorX d(cagePoints->rows());
        bool coincident = false, outside = false;

        for(long j = 0; j < (*cagePoints).rows(); j++){

            Vector3 vj = (*cagePoints).row(j);
            Vector3 v_ij = vj - vi;
            double distance_ij = v_ij.norm();
            d(j) = distance_ij;
            if(distance_ij < EPSILON){
                (*coords)(i,j) = 1;
                coincident = true;
                break;
            }else
                u.row(j) = (v_ij / distance_ij);
        }

        if(coincident){
            i++;
            continue;
        }

        double totalW = 0;

        Triangle* t;
        Node* nj;
        FOREACHVTTRIANGLE(cageTriangles, t, nj){

            Vector3 l, teta, c, s;

            vector<Vertex*> v = {t->v1(), t->v2(), t->v3()}; //v?
            double h = 0;
            pair<long, long> id;

            for(int k = 0; k < 3; k++){
                pair<long, long> id1 = *verticesID.find((long) v[Utilities::mod((k - 1), 3)]);
                pair<long, long> id2 = *verticesID.find((long) v[Utilities::mod((k + 1), 3)]);
                l(k) = (u.row(id2.second) - u.row(id1.second)).norm();
                teta(k) = (2.0 * asin(l(k) / 2.0));
                h += teta(k);
            }

            h /= 2;

            if(M_PI - h < EPSILON)
                for(int k = 0; k < 3; k++){
                    pair<long, long> id1 = *verticesID.find((long int) v[Utilities::mod((k - 1), 3)]);
                    pair<long, long> id2 = *verticesID.find((long int) v[Utilities::mod((k + 1), 3)]);
                    id = *verticesID.find((long) v[k]);
                    (*coords)(i,id.second) += sin(teta(k)) * d(id1.second) * d(id2.second);
                }

            Matrix33 M;
            for(int k = 0; k < 3; k++){
                id = *verticesID.find((long) (v[k]));
                M(k,0) = u(id.second,0);
                M(k,1) = u(id.second,1);
                M(k,2) = u(id.second,2);
            }

            for(int k = 0; k < 3; k++){
                double sinkprev = sin(teta(Utilities::mod((k - 1), 3)));
                double sinknext = sin(teta(Utilities::mod((k + 1), 3)));
                double sinh = sin(h);
                double sinhminusteta = sin(h - teta(k));
                c(k) = ((2.0 * sinh * sinhminusteta) / (sinknext * sinkprev) - 1.0);
                double newCSquare = pow(c(k), 2.0);
                if(newCSquare < 1)
                    s(k) = Utilities::sign(M.determinant()) * sqrt(1.0 - newCSquare);
                else
                    s(k) = 0;
                if(abs(s(k)) <= EPSILON){
                    outside = true;
                    break;
                }
            }

            if(outside)
                continue;

            for(int k = 0; k < 3; k++){
                id = *verticesID.find((long) v[k]);
                double new_w = (teta(k) - c(Utilities::mod((k + 1), 3)) * teta(Utilities::mod((k - 1), 3)) -
                                c(Utilities::mod((k - 1), 3)) * teta(Utilities::mod((k + 1), 3)));
                double sinknext = sin(teta(Utilities::mod((k + 1), 3)));
                double skprev = s(Utilities::mod((k - 1), 3));
                double tmp = new_w / (d(id.second) * sinknext * skprev);
                (*coords)(i, id.second) += tmp;
                totalW += tmp;
            }

        }

        for(int j = 0; j < cagePoints->rows(); j++)
            (*coords)(i,j) /= totalW;

    }

}

void ShapeOpMVCThread::waitThread(){
    tid->join();
}

