
#ifndef GRAPHENE_CL_CONTEXT_H
#define GRAPHENE_CL_CONTEXT_H

#include <string>
#include <iostream>

#ifdef _WIN32

#include <CL/cl.h>
#include <CL/cl_gl.h>


namespace cl
{


class CL_context
{
    cl_context context_;

    cl_command_queue cmd_queue_;

    cl_device_id device_id_;


    std::string version_, vendor_, name_;
public:
    CL_context() :
        context_(0),
        cmd_queue_(0),
        device_id_(0)
    {}

    ~CL_context()
    {
        if (cmd_queue_)
            clReleaseCommandQueue(cmd_queue_);

        if (context_)
            clReleaseContext(context_);
    }

    const cl_context& context() { return context_; }
    const cl_command_queue& cmd_queue() { return cmd_queue_; }
    const cl_device_id& device_id() { return device_id_; }

    const std::string& version() const { return version_; }
    const std::string& vendor() const { return vendor_; }
    const std::string& name() const { return name_; }

    bool create();

};

} // namespace cl

#elif __linux


#include <CL/cl.h>
#include <CL/cl_gl.h>


namespace cl
{


class CL_context
{

    cl_context context_;

    cl_command_queue cmd_queue_;

    cl_device_id device_id_;


    std::string version_, vendor_, name_;
public:
    CL_context() :
        context_(0),
        cmd_queue_(0),
        device_id_(0)
    {}

    ~CL_context()
    {
        if (cmd_queue_)
            clReleaseCommandQueue(cmd_queue_);

        if (context_)
            clReleaseContext(context_);
    }

    const cl_context& context() { return context_; }
    const cl_command_queue& cmd_queue() { return cmd_queue_; }
    const cl_device_id& device_id() { return device_id_; }

    const std::string& version() const { return version_; }
    const std::string& vendor() const { return vendor_; }
    const std::string& name() const { return name_; }

    bool create();
};


} // namespace cl
#endif

#endif
