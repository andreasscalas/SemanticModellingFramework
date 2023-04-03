#ifndef MVCTHREAD_H
#define MVCTHREAD_H

#include <thread>
#include <vector>
#include <Eigen/Dense>
#include "mainthread.h"
#include "drawablemesh.h"
#include "barycentriccoordinates.h"
#include "utilities.h"

/**
 * @brief The MVCThread class class that defines the behaviour of a thread which compute a subset of MVC
 */
class MVCThread : public MainThread{

    private:
        DrawableMesh* cage;                         //The cage on which the GC must be computed
		Eigen::MatrixXd* coords;				//The coordinates linked to the vertices
        std::map<long, long> verticesID;            //ID of the vertices
        IMATI_STL::Node* firstVertex;               //First vertex of the model of which compute the GC
        long begin;                                 //Index of the first vertex of the model of which compute the GC
        long end;                                   //Index of the last vertex of the model of which compute the GC
        const double EPSILON = 0.000000005;         //Constant that defines a value sufficently little that can be considered as 0
		int numRow, numCol;

        /**
         * @brief executeTask method that allows the execution of the main task of the thread
         */
        void executeTask();

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
        MVCThread(DrawableMesh* cage, Eigen::MatrixXd* coords, std::map<long, long> verticesID, IMATI_STL::Node* firstVertex, long begin, long end);

        /**
         * @brief startThread method that allows the starting of the thread execution
         */
        void startThread();

        /**
         * @brief waitThread method that allows the request for waiting the end of the thread execution
         */void waitThread();

};

#endif // MVCTHREAD_H
