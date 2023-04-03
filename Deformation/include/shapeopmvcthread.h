#ifndef SHAPEOPMVCTHREAD_H
#define SHAPEOPMVCTHREAD_H

#include <mutex>
#include <mainthread.h>
#include <imatistl.h>
#include <Types.h>

class ShapeOpMVCThread : public MainThread
{
private:
    ShapeOp::MatrixX3* modelPoints;
    ShapeOp::MatrixX3* cagePoints;
    ShapeOp::MatrixXX* coords;
    std::map<long, long> verticesID;            //ID of the vertices
    IMATI_STL::List* cageTriangles;
    long begin;                                 //Index of the first vertex of the model of which compute the GC
    long end;                                   //Index of the last vertex of the model of which compute the GC
    const double EPSILON = 0.000000005;         //Constant that defines a value sufficently little that can be considered as 0
    std::mutex* mtx;

    /**
     * @brief executeTask method that allows the execution of the main task of the thread
     */
    void executeTask();

public:

    /**
     * @brief ShapeOpMVCThread main constructor of the class. To succesfully start, a thread need a set of information
     * @param modelPoints matrix containing the points in the mesh to be deformed
     * @param cagePoints matrix containing the points in the cage
     * @param cageTriangles list of triangles composing the cage
     * @param coords the matrix of Barycentric Coordinates, to be computed in the different threads
     * @param verticesID map between vertices and their ID
     * @param begin first row in the matrix interested by the BC computation
     * @param end last row in the matrix interested by the BC computation
     */
    ShapeOpMVCThread(ShapeOp::MatrixX3 *modelPoints, ShapeOp::MatrixX3 *cagePoints, IMATI_STL::List* cageTriangles, ShapeOp::MatrixXX* coords, std::map<long, long> verticesID, long begin, long end);

    /**
     * @brief startThread method that allows the starting of the thread execution
     */
    void startThread();

    /**
     * @brief waitThread method that allows the request for waiting the end of the thread execution
     */void waitThread();
    void setMtx(std::mutex *value);
};

#endif // SHAPEOPMVCTHREAD_H
