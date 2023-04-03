#ifndef MVCTHREAD_H
#define MVCTHREAD_H

#include <vector>
#include <mainthread.h>
#include <drawablemesh.h>
#include <barycentriccoordinates.h>
#include <mutex>
#include <Eigen/Core>

/**
 * @brief The MVCThread class class that defines the behaviour of a thread which compute a subset of MVC
 */
class MVCThread : public MainThread{

    private:
        DrawableMesh* model;                        //The model on which the GC must be computed
        std::map<IMATI_STL::Vertex*, unsigned long> verticesID;            //ID of the vertices
        std::mutex* mtx;
        Eigen::MatrixX3d cagePoints;
        Eigen::MatrixXd* coords;
        IMATI_STL::List* cageTriangles;
        unsigned long begin;                                 //Index of the first vertex of the model of which compute the GC
        unsigned long end;                                   //Index of the last vertex of the model of which compute the GC
        const double EPSILON = 0.000000005;         //Constant that defines a value sufficently little that can be considered as 0

        /**
         * @brief executeTask method that allows the execution of the main task of the thread
         */
		void executeTask();;

    public:

        /**
         * @brief MVCThread main constructor of the class. To succesfully start, a thread need a set of information
         * @param cage the cage on which the GC must be computed
         * @param coords vector of coordinates of the vertices of the cage
         * @param verticesID ID of the vertices of the cage
         * @param firstVertex First vertex of the model of which compute the GC
         * @param begin Index of the first vertex of the model of which compute the GC
         * @param end Index of the last vertex of the model of which compute the GC
         */
        MVCThread(DrawableMesh* cage, std::vector<std::vector<double> >* coords, std::map<IMATI_STL::Vertex*, unsigned long> verticesID, IMATI_STL::Node* firstVertex, unsigned long begin, unsigned long end);

        MVCThread(DrawableMesh *model, Eigen::MatrixX3d &cagePoints, IMATI_STL::List *cageTriangles, Eigen::MatrixXd *coords, std::map<IMATI_STL::Vertex *, unsigned long> verticesID, long begin, long end);
        /**
         * @brief startThread method that allows the starting of the thread execution
         */
        void startThread();

        /**
         * @brief waitThread method that allows the request for waiting the end of the thread execution
         */void waitThread();

        std::mutex *getMtx() const;
        void setMtx(const std::mutex *value);
};

#endif // MVCTHREAD_H
