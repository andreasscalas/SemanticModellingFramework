//=============================================================================
// code based on Petru Marginean's Logging in C++ article:
// http://www.drdobbs.com/cpp/logging-in-c/201804215?pgno=1
//=============================================================================
#ifndef GRAPHENE_LOGGER_H
#define GRAPHENE_LOGGER_H
//=============================================================================


#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>
#else
#include <sys/time.h>
#endif

//=============================================================================


namespace graphene {


//=============================================================================

// Logging levels. Log_info is the default level so that warnings and errors get
// logged but not debug information. Define these in graphene namespace for ease
// of use.
enum Log_level
{
    Log_error,
    Log_warning,
    Log_info,
    Log_debug
};


//=============================================================================


namespace utility {


//=============================================================================


inline std::string current_time();


//=============================================================================


/// \addtogroup utility
/// @{

/// A simple and lightweight logging class
class Logger
{

public:

    Logger();
    virtual ~Logger();

    std::ostringstream& get(Log_level level = Log_info);
    static Log_level&   log_level();
    static std::string  to_string(Log_level level);
    static Log_level    from_string(const std::string& level);

protected:

    std::ostringstream os;


private:

    Logger(const Logger&);
    Logger& operator=(const Logger&);

};


//=============================================================================
/// @{
//=============================================================================


inline Logger::Logger()
{
}


//-----------------------------------------------------------------------------


inline std::ostringstream& Logger::get(Log_level level)
{
//    os << "- " << current_time() << " ";
    os << to_string(level) << ": ";
    os << std::string(level > Log_debug ? level - Log_debug : 0, '\t');
    return os;
}


//-----------------------------------------------------------------------------


inline Logger::~Logger()
{
    std::cout << os.str() << std::flush;
}


//-----------------------------------------------------------------------------


inline Log_level& Logger::log_level()
{
    static Log_level level = Log_debug;
    return level;
}


//-----------------------------------------------------------------------------


inline std::string Logger::to_string(Log_level level)
{
    static const char* const buffer[] = { "error", "warning", "info", "debug" };
    return buffer[level];
}


//-----------------------------------------------------------------------------


inline Log_level Logger::from_string(const std::string& level)
{
    if (level == "debug")   return Log_debug;
    if (level == "info")    return Log_info;
    if (level == "warning") return Log_warning;
    if (level == "error")   return Log_error;

    Logger().get(Log_warning) << "Unknown logging level '" << level << "'. Using Log_info level as default.";
    return Log_info;
}


//-----------------------------------------------------------------------------


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)


inline std::string current_time()
{
    const int MAX_LEN = 200;
    char buffer[MAX_LEN];
    if (GetTimeFormatA(LOCALE_USER_DEFAULT, 0, 0,
            "HH':'mm':'ss", buffer, MAX_LEN) == 0)
        return "Error in current_time()";

    char result[100] = {0};
    static DWORD first = GetTickCount();
    sprintf(result, "%s.%03ld", buffer, (long)(GetTickCount() - first) % 1000);
    return result;
}

#else


inline std::string current_time()
{
    char buffer[11];
    time_t t;
    time(&t);
    tm* r = localtime(&t);
    strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[100] = {0};
    sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000);
    return result;
}

#endif // WIN32


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif //GRAPHENE_LOGGER_H
//=============================================================================
