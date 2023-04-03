#include "mvcthread.h"
#include <Eigen/Dense>
#include <utilities.h>

using namespace std;
using namespace IMATI_STL;
using namespace Eigen;

MVCThread::MVCThread(DrawableMesh *model, Eigen::MatrixX3d &cagePoints, List *cageTriangles, MatrixXd *coords, std::map<Vertex *, unsigned long> verticesID, long begin, long end)
{
    this->model = model;
    this->cagePoints = cagePoints;
    this->cageTriangles = cageTriangles;
    this->coords = coords;
    this->begin = begin;
    this->end = end;
    this->verticesID = verticesID;
}

void MVCThread::startThread(){
     tid = new thread(MainThread::executeTaskHelper, this);
}

std::mutex* MVCThread::getMtx() const
{
    return mtx;
}

void MVCThread::setMtx(const std::mutex *value)
{
    mtx = value;
}

void MVCThread::executeTask(){

    for (unsigned long i = begin; i < end && i < model->V.numels(); i++) {

        Eigen::Vector3d vi = {model->getPoint(i)->x, model->getPoint(i)->y, model->getPoint(i)->z};
        Eigen::MatrixX3d u;
        u.resize(cagePoints.rows(), 3);
        Eigen::VectorXd d(cagePoints.rows());
        bool coincident = false, outside = false;

        for(unsigned long j = 0; j < cagePoints.rows(); j++){

            Eigen::Vector3d vj = cagePoints.row(j);
            Eigen::Vector3d v_ij = vj - vi;
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

            Eigen::Vector3d l, teta, c, s;

            vector<Vertex*> v = {t->v1(), t->v2(), t->v3()}; //v?
            double h = 0;
            pair<IMATI_STL::Vertex*, unsigned long> id;

            for(int k = 0; k < 3; k++){
                pair<IMATI_STL::Vertex*, unsigned long> id1 = *verticesID.find(v[Utilities::mod((k - 1), 3)]);
                pair<IMATI_STL::Vertex*, unsigned long> id2 = *verticesID.find(v[Utilities::mod((k + 1), 3)]);
                l(k) = (u.row(id2.second) - u.row(id1.second)).norm();
                teta(k) = (2.0 * asin(l(k) / 2.0));
                h += teta(k);
            }

            h /= 2;

            if(M_PI - h < EPSILON)
                for(int k = 0; k < 3; k++){
                    pair<IMATI_STL::Vertex*, unsigned long> id1 = *verticesID.find(v[Utilities::mod((k - 1), 3)]);
                    pair<IMATI_STL::Vertex*, unsigned long> id2 = *verticesID.find(v[Utilities::mod((k + 1), 3)]);
                    id = *verticesID.find(v[k]);
                    (*coords)(i,id.second) += sin(teta(k)) * d(id1.second) * d(id2.second);
                }

            Eigen::Matrix3d M;
            for(int k = 0; k < 3; k++){
                id = *verticesID.find((v[k]));
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
                id = *verticesID.find(v[k]);
                double new_w = (teta(k) - c(Utilities::mod((k + 1), 3)) * teta(Utilities::mod((k - 1), 3)) -
                                c(Utilities::mod((k - 1), 3)) * teta(Utilities::mod((k + 1), 3)));
                double sinknext = sin(teta(Utilities::mod((k + 1), 3)));
                double skprev = s(Utilities::mod((k - 1), 3));
                double tmp = new_w / (d(id.second) * sinknext * skprev);
                (*coords)(i, id.second) += tmp;
                totalW += tmp;
            }

        }

        for(int j = 0; j < cagePoints.rows(); j++)
            (*coords)(i,j) /= totalW;

    }


}

void MVCThread::waitThread(){
    tid->join();
}

