#ifndef SPTHREAD_H
#define SPTHREAD_H

#include <mainthread.h>
#include <vector>
#include <imatistl.h>


class SPThread : public MainThread{

    private:

        IMATI_STL::Vertex* v1, *v2;        //Vertices between which the path will be computed
        std::vector<IMATI_STL::Vertex*>* path;  //The computed path
        short int metric; //The metric to be used

        /**
         * @brief Method that allows the execution of the main task of the thread
         */
        void executeTask();

    public:

        /**
         * @brief Main constructor of the class. To succesfully start, a thread need a set of information
         * @param v1 First vertex of the path
         * @param v2 Last vertex of the path
         * @param path The computed path
         * @param metric The metric used for weighting the arcs of paths
         */
        SPThread(IMATI_STL::Vertex* v1, IMATI_STL::Vertex* v2, std::vector<IMATI_STL::Vertex*>* path, const short int metric);

        /**
         * @brief method that allows the starting of the thread execution
         */
        void startThread();

        /**
         * @brief Method that allows the request for waiting the end of the thread execution
         */
        void waitThread();

};

#endif // SPTHREAD_H
