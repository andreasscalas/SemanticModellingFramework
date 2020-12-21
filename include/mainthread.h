#ifndef MAINTHREAD_H
#define MAINTHREAD_H

#include <thread>

/**
 * @brief The MainThread class abstract class to define an interface for different types of threads
 */
class MainThread{

    public:
    virtual ~MainThread(){}
        virtual void startThread() = 0;
        virtual void waitThread() = 0;

    protected:
        std::thread* tid;                           //Thread ID
        virtual void executeTask() = 0;
        static void executeTaskHelper(void* context){
            (static_cast<MainThread*>(context))->executeTask();
        }

};


#endif // MAINTHREAD_H
