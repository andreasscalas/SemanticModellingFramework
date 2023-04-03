#ifndef GCTHREAD_H
#define GCTHREAD_H

#include <vector>
#include <mutex>
#include <mainthread.h>
#include <drawablemesh.h>


/**
 * @brief The GCThread class class that defines the behaviour of a thread which compute a subset of GC
 */
class GCThread : public MainThread{

    private:

        std::mutex* mtx;                                                 //A mutex for the race condition over outerVertices
        DrawableMesh* cage;                                         //The cage on which the GC must be computed
        std::map<unsigned long, unsigned long> verticesID;                            //ID of the vertices
        IMATI_STL::Node* firstVertex;                               //First vertex of the model of which compute the GC
        unsigned long begin;                                                 //Index of the first vertex of the model of which compute the GC
        unsigned long end;                                                   //Index of the last vertex of the model of which compute the GC
        std::vector<std::vector<double> >* verticesCoords;          //Vector of coordinates of the vertices of the cage
        std::vector<std::vector<double> >* trianglesCoords;         //Vector of coordinates of the triangles of the cage
        std::vector<IMATI_STL::Vertex*>* outerVertices;             //Vector of vertices that are outside the cage
        const double EPSILON = 0.00001;                             //Constant that defines a value sufficently little that can be considered as 0
        const double THRESHOLD = 0.5;                               //Constant that defines when a vertex can be identified as an outer vertex

        /**
         * @brief executeTask method that allows the execution of the main task of the thread
         */
        void executeTask();

        /**
         * @brief GCTriInt method that computes a scalar that are used for computing the BC
         * @param p First point
         * @param v1 Second point
         * @param v2 Third point
         * @param eta Fourth point
         * @return a scalar
         */
        double GCTriInt(IMATI_STL::Point p, IMATI_STL::Point v1, IMATI_STL::Point v2, IMATI_STL::Point eta);

    public:

        /**
         * @brief GCThread main constructor of the class. To succesfully start, a thread needs a set of information
         * @param cage the cage on which the GC must be computed
         * @param verticesCoords Vector of coordinates of the vertices of the cage
         * @param trianglesCoords Vector of coordinates of the triangles of the cage
         * @param verticesID ID of the vertices of the cage
         * @param firstVertex First vertex of the model of which compute the GC
         * @param begin Index of the first vertex of the model of which compute the GC
         * @param end Index of the last vertex of the model of which compute the GC
         * @param outerVertices Vector of vertices that are outside the cage
         * @param mtx A mutex for the race condition over outerVertices
         */
        GCThread(DrawableMesh* cage, std::vector<std::vector<double> >* verticesCoords,
                 std::vector<std::vector<double> >* trianglesCoords, std::map<unsigned long, unsigned long> verticesID,
                 IMATI_STL::Node* firstVertex, unsigned long begin, unsigned long end, std::vector<IMATI_STL::Vertex*>* outerVertices,
                 std::mutex* mtx);

        /**
         * @brief startThread method that allows the starting of the thread execution
         */
        void startThread();

        /**
         * @brief waitThread method that allows the request for waiting the end of the thread execution
         */
        void waitThread();

};

#endif // GCTHREAD_H
