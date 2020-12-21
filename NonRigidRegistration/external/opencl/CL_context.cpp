

#include "CL_context.h"

#ifdef _WIN32

#include <GL/glew.h>
#include <GL/wglew.h>

namespace cl
{

bool CL_context::create()
{
    cl_int err;
    cl_uint num_platforms=0;
    cl_platform_id* platforms, selected_platform;
    cl_device_id* devices;
    size_t size;

    unsigned int i;

    err = clGetPlatformIDs(0, NULL, &num_platforms);
    if (err != CL_SUCCESS)
    {
        std::cout << "CL_context::create: [WARNING] Could not get platform count!" << std::endl;
        return false;
    }


    //for this: make sure gl context is current
    cl_context_properties ctx_props[] = {CL_CONTEXT_PLATFORM, (cl_context_properties) 0,
                                         CL_WGL_HDC_KHR, (cl_context_properties) wglGetCurrentDC(),
                                         CL_GL_CONTEXT_KHR,(cl_context_properties) wglGetCurrentContext(),
                                         0
                                        };

    platforms = new cl_platform_id[num_platforms];
    err = clGetPlatformIDs(num_platforms, platforms, NULL);
    for (i=0; i < num_platforms; ++i)
    {

        selected_platform = platforms[i];
        ctx_props[1] = (cl_context_properties) selected_platform;

        context_ = clCreateContextFromType(ctx_props, CL_DEVICE_TYPE_GPU, NULL, NULL, &err);

        if (context_)
            break;

    }
    delete[] platforms;


    if (err != CL_SUCCESS || context_ == 0)
    {
        std::cout << "CL_context::create: [WARNING] Could not create CL context!" << std::endl;
        return false;
    }


    err = clGetContextInfo(context_, CL_CONTEXT_DEVICES, 0, NULL, &size);
    if (err != CL_SUCCESS)
    {
        std::cout << "CL_context::create: [WARNING] Could not get device count for CL context!" << std::endl;
        return false;
    }
    devices = new cl_device_id[size/sizeof(cl_device_info)];
    clGetContextInfo(context_, CL_CONTEXT_DEVICES, size, devices, NULL);
    //select first/only device
    device_id_ = devices[0];
    delete[] devices;

    cmd_queue_ = clCreateCommandQueue(context_, device_id_, 0, &err);

    if (err != CL_SUCCESS || cmd_queue_ == 0)
    {
        std::cout << "CL_context::create: [WARNING] Could not create command-queue!" << std::endl;
        return false;
    }

    if (selected_platform != 0)
    {
        char buf[2048];
        //get some infos
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VENDOR, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VENDOR, size, buf, &size);
        vendor_ = buf;

        clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, size, buf, &size);
        name_ = buf;

        clGetPlatformInfo(selected_platform, CL_PLATFORM_VERSION, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VERSION, size, buf, &size);
        version_ = buf;
    }

    return true;
}

}
#elif __linux

#include <GL/glxew.h>


namespace cl
{

bool CL_context::create(){
    cl_int err;
    cl_uint num_platforms=0;
    cl_platform_id* platforms, selected_platform=0;
    cl_device_id* devices;
    size_t size;

    unsigned int i;

    err = clGetPlatformIDs(0, NULL, &num_platforms);
    if (err != CL_SUCCESS)
    {
        std::cout << "CL_context::create: [ERROR] Could not get platform count! ErrorID " << err << std::endl;
        return false;
    }


    //for this: make sure gl context is current
    cl_context_properties ctx_props[] = {CL_CONTEXT_PLATFORM, (cl_context_properties) 0,
                                         CL_GLX_DISPLAY_KHR, (cl_context_properties) glXGetCurrentDisplay(),
                                         CL_GL_CONTEXT_KHR,(cl_context_properties) glXGetCurrentContext(),
                                         0
                                        };

    platforms = new cl_platform_id[num_platforms];
    err = clGetPlatformIDs(num_platforms, platforms, NULL);
    for (i=0; i < num_platforms; ++i)
    {
        selected_platform = platforms[i];
        ctx_props[1] = (cl_context_properties) selected_platform;

        context_ = clCreateContextFromType(ctx_props, CL_DEVICE_TYPE_GPU, NULL, NULL, &err);

        if (context_)
            break;

    }
    delete[] platforms;


    //context_ = clCreateContextFromType(ctx_props, CL_DEVICE_TYPE_GPU, NULL, NULL, &err);

    if (err != CL_SUCCESS || context_ == 0)
    {
        std::cout << "CL_context::create: [WARNING] Could not create CL context!" << std::endl;
        return false;
    }


    err = clGetContextInfo(context_, CL_CONTEXT_DEVICES, 0, NULL, &size);
    if (err != CL_SUCCESS)
    {
        std::cout << "CL_context::create: [WARNING] Could not get device count for CL context!" << std::endl;
        return false;
    }
    devices = new cl_device_id[size/sizeof(cl_device_info)];
    clGetContextInfo(context_, CL_CONTEXT_DEVICES, size, devices, NULL);
    //select first/only device
    device_id_ = devices[0];
    delete[] devices;

    cmd_queue_ = clCreateCommandQueue(context_, device_id_, 0, &err);

    if (err != CL_SUCCESS || cmd_queue_ == 0)
    {
        std::cout << "CL_context::create: [WARNING] Could not create command-queue!" << std::endl;
        return false;
    }

    if (selected_platform != 0)
    {
        char buf[2048];
        //get some infos
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VENDOR, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VENDOR, size, buf, &size);
        vendor_ = buf;

        clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_NAME, size, buf, &size);
        name_ = buf;

        clGetPlatformInfo(selected_platform, CL_PLATFORM_VERSION, 0, NULL, &size);
        clGetPlatformInfo(selected_platform, CL_PLATFORM_VERSION, size, buf, &size);
        version_ = buf;
    }


    return true;
}

}

#endif
