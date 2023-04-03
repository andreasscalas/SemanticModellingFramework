#ifndef SLICINGTHREAD_H
#define SLICINGTHREAD_H

#include <mainthread.h>
#include <extendedtrimesh.h>
#include <slice.h>
#include <vector>
#include <mutex>

class SlicingThread : public MainThread{

    public:
        SlicingThread(unsigned int first, unsigned int last, unsigned int slicesNumber, ExtendedTrimesh* mesh,
                      std::vector<std::pair<Slice*, double> >* slices_height, std::mutex* mtx);


        ~SlicingThread() override;
        void startThread() override;
        void waitThread() override;

    protected:

        void executeTask() override;
        unsigned int first;
        unsigned int last;
        unsigned int slicesNumber;
        ExtendedTrimesh* mesh;
        std::vector<std::pair<Slice*, double> >* slices_height;
        std::mutex* mtx;

};

#endif // SLICINGTHREAD_H
