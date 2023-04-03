#include "gcthread.h"
#include <utilities.h>

using namespace std;
using namespace IMATI_STL;

GCThread::GCThread(DrawableMesh* cage, vector<vector<double> >* verticesCoords,
                   vector<vector<double> >* trianglesCoords, map<unsigned long, unsigned long> verticesID,
                   Node* firstVertex, unsigned long begin, unsigned long end, vector<Vertex*>* outerVertices,
                   mutex* mtx){

    this->cage = cage;
    this->verticesCoords = verticesCoords;
    this->trianglesCoords = trianglesCoords;
    this->firstVertex = firstVertex;
    this->begin = begin;
    this->end = end;
    this->verticesID = verticesID;
    this->outerVertices = outerVertices;
    this->mtx = mtx;

}

void GCThread::startThread(){ tid = new thread(MainThread::executeTaskHelper, this); }

void GCThread::executeTask(){

    //The pseudocode of this method can be found in the GC paper (see GC class description)
    unsigned long i = begin;
    for(Node* ni = firstVertex; i < end && ni != nullptr; ni = ni->next()){

        Vertex* vi = static_cast<Vertex*>(ni->data);
        unsigned long j = 0;
        double sumV = 0;

        Node* nt;
        Triangle* t;
        for(nt = this->cage->T.head(); nt != nullptr; nt = nt->next()){

            t = static_cast<IMATI_STL::Triangle*>(nt->data);
            vector<Vertex*> vj = {t->v1(), t->v2(), t->v3()};
            vector<Point> pj = {*(t->v1()), *(t->v2()), *(t->v3())};
            vector<Point> q, N;
            vector<double>  I, II;
            vector<short> s;
            Point p, NII(0,0,0), w;
            double I_ = 0;
            short l;

            for(l = 0; l < 3; l++)
                pj[static_cast<unsigned long>(l)] -= (*vi);

            p = t->getNormal() * (t->getNormal() * pj[0]);

            for(l = 0; l < 3; l++){
                Point zero(0, 0, 0);
                s.push_back(static_cast<short>(Utilities::sign(((pj[static_cast<unsigned long>(l)] - p) & (pj[static_cast<unsigned long>(Utilities::mod(l + 1, 3))] - p)) * t->getNormal())));
                I.push_back(GCTriInt(p, pj[static_cast<unsigned long>(l)], pj[static_cast<unsigned long>(Utilities::mod(l + 1, 3))], zero));
                II.push_back(GCTriInt(zero, pj[static_cast<unsigned long>(Utilities::mod(l + 1, 3))], pj[static_cast<unsigned long>(l)], zero));
                q.push_back(pj[static_cast<unsigned long>(Utilities::mod(l + 1, 3))] & pj[static_cast<unsigned long>(l)]);
                N.push_back(q[static_cast<unsigned long>(l)] / q[static_cast<unsigned long>(l)].length());
            }

            for(short k = 0; k < 3; k++)
                I_ += s[static_cast<unsigned long>(k)] * I[static_cast<unsigned long>(k)];

            I_ = -1 * abs(I_);

            (*trianglesCoords)[i][j] = -I_;

            for(short k = 0; k < 3; k++)
                NII += N[static_cast<unsigned long>(k)] * II[static_cast<unsigned long>(k)];

            w = t->getNormal() * I_ + NII;

            if(w.length() > EPSILON)
                for(l = 0; l < 3; l++){
                    std::pair<unsigned long, unsigned long> id = *verticesID.find(reinterpret_cast<unsigned long>(vj[static_cast<unsigned long>(l)]));
                    double newWeight = (N[static_cast<unsigned long>(Utilities::mod(l + 1, 3))] * w)/(N[static_cast<unsigned long>(Utilities::mod(l + 1, 3))] * pj[static_cast<unsigned long>(l)]);
                    (*verticesCoords)[i][id.second] += newWeight;
                    sumV += newWeight;
                }

            j++;
        }

        if(sumV <= THRESHOLD){
            mtx->lock();
            outerVertices->push_back(vi);
            mtx->unlock();
        }
        i++;

    }

}

double GCThread::GCTriInt(Point p, Point v1, Point v2, Point eta){

    //The pseudocode of this method can be found in the GC paper (see GC class description)
    Point v2_v1 = v2 - v1;
    Point p_v1 = p - v1;
    Point v1_p = v1 - p;
    Point v2_p = v2 - p;
    double alfa = acos((v2_v1 * p_v1) / (v2_v1.length() * p_v1.length()));
    double beta = acos((v1_p * v2_p) / (v1_p.length() * v2_p.length()));
    double lambda = pow((p_v1).length(), 2) * pow(sin(alfa), 2);
    double c = pow((p - eta).length(), 2);
    double teta[2] = {M_PI - alfa, M_PI - alfa - beta};
    double I[2];

    /* The subdivision in parts has been done only to improve readability (from part0 to part 7 the variables keep
     * track of the step results of the formula) */
    for(short i = 0; i < 2; i++){
        double S = sin(teta[i]);
        double C = cos(teta[i]);
        double part0 = (-Utilities::sign(S));
        double part1 = (static_cast<double>(part0)) / (2.0);
        double part7 = (sqrt(c) * C) / sqrt(lambda + pow(S, 2) * c);
        double part2 = 2 * sqrt(c) * ((atan(part7)));
        double part3 = (2 * sqrt(lambda) * pow(S, 2)) / pow(1 - C, 2);
        double part4 = (1 - (2 * c * C) / ((c * (1 + C) + lambda + sqrt(pow(lambda, 2) + lambda * c * pow(S, 2)))));
        I[i] = part1 * (part2 + sqrt(lambda) * log10(part3 * part4));
    }
    double part5 = ((-1.0) / (4.0 * M_PI));
    double part6 = abs(I[0] - I[1] - sqrt(c) * beta);

    return part5 * part6;

}

void GCThread::waitThread(){ tid->join(); }
