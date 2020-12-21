//=============================================================================

#ifndef GRAPHENE_STOP_WATCH_H
#define GRAPHENE_STOP_WATCH_H


//== INCLUDES =================================================================

#ifdef _WIN32
#  include <windows.h>
#else // Unix
#  include <sys/time.h>
#endif

#include <iostream>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== CLASS DEFINITION =========================================================

/// \addtogroup utility
/// @{


/// Simple class for a stop watch
class Stop_watch
{
public:

    /// Constructor
    Stop_watch() 
        : elapsed_(0.0), running_(false)
    {
#ifdef _WIN32 // Windows
        QueryPerformanceFrequency(&freq_);
#endif
    }


    /// Start time measurement
    void start()
    {
        elapsed_ = 0.0;
        cont();
    }


    /// Continue measurement, accumulates elapased times
    void cont()
    {
#ifdef _WIN32 // Windows
        QueryPerformanceCounter(&starttime_);
#else // Linux
        gettimeofday(&starttime_, 0);
#endif
        running_ = true;
    }


    /// Stop time measurement, return elapsed time in ms
    double stop()
    {
#ifdef _WIN32 // Windows
        QueryPerformanceCounter(&endtime_);
        elapsed_ += ((double)(endtime_.QuadPart - starttime_.QuadPart)
                     / (double)freq_.QuadPart * 1000.0f);
#else // Unix
        gettimeofday(&endtime_, 0);
        elapsed_ += ((endtime_.tv_sec  - starttime_.tv_sec )*1000.0 +
                     (endtime_.tv_usec - starttime_.tv_usec)*0.001);
#endif
        running_ = false;
        return elapsed();
    }


    /// Return elapsed time in ms (watch has to be stopped).
    double elapsed() const
    {
        if (running_)
        {
            std::cerr << "Stop_watch: stop watch before calling elapsed()\n";
        }
        return elapsed_;
    }


private:

    double  elapsed_;
    bool    running_;

#ifdef _WIN32 // Windows
    LARGE_INTEGER starttime_, endtime_;
    LARGE_INTEGER freq_;
#else // Unix
    timeval starttime_, endtime_;
#endif
};


//=============================================================================


/// output a timer to a stream
inline std::ostream&
operator<<(std::ostream& _os, const Stop_watch& _timer)
{
    _os << _timer.elapsed() << " ms";
    return _os;
}


//=============================================================================
/// @}
//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_STOP_WATCH_H defined
//=============================================================================

