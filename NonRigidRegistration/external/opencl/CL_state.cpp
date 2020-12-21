

#include "CL_state.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#ifdef HAVE_OCL



namespace cl {

CL_state* CL_state::instance_ = NULL;

CL_state* CL_state::instance()
{
    static Guard g;
    if (instance_ == NULL)
    {
        instance_ = new CL_state;
    }
    return instance_;
}


CL_state::CL_state() :
    kernels_(KERNEL_MAX, 0)
{
}

CL_state::~CL_state()
{
    size_t i;
    for (i=0; i < kernels_.size(); ++i)
    {
        if (kernels_[i])
            clReleaseKernel(kernels_[i]);
    }

    for (i=0; i < programs_.size(); ++i)
    {
        clReleaseProgram(programs_[i]);
    }
}

bool CL_state::init(const char *argv_0)
{
    if (ocl_context_.create())
    {
        std::cout << ocl_context_.version() << std::endl;
    }
    else
    {
        std::cout << "CL_state::Constructor: [ERROR] Could not create OpenCL context!" << std::endl;
        return false;
    }

    return load_programs(argv_0);
}

const cl_kernel& CL_state::kernel(Kernel_type k)
{
    return kernels_[k];
}

bool CL_state::load_programs(const char *argv_0)
{
    // path to shader directory
    std::string full_path(argv_0);
#ifdef _WIN32
    full_path = full_path.substr(0, full_path.find_last_of('\\')+1);
    full_path += "kernels\\";
#elif __APPLE__
    full_path = full_path.substr(0, full_path.find_last_of('/')+1);
    full_path += "../";
    full_path += "kernels/";
#else
    full_path = full_path.substr(0, full_path.find_last_of('/')+1);
    full_path += "kernels/";
#endif

    cl_program program;

    program = load_program(full_path+"blendshapes.cl");
    if (program)
    {
        kernels_[BLENDSHAPE_KERNEL] = get_kernel(program, "blend");
        programs_.push_back(program);
    }
    else
    {
        return false;
    }

    program = load_program(full_path+"correspondences.cl");
    if (program)
    {
        kernels_[CORRESPONDENCES_PS2MESH_KERNEL] = get_kernel(program, "ps2mesh");
        kernels_[CORRESPONDENCES_MESH2PS_KERNEL] = get_kernel(program, "mesh2ps");
        programs_.push_back(program);
    }
    else
    {
        return false;
    }

    return true;
}

cl_program CL_state::load_program(const std::string& filename)
{
    cl_program program=0;
    cl_int err;
    std::string str;
    std::stringstream ss;
    size_t size;

    // read file to string
    std::ifstream ifs(filename.c_str());

    if (!ifs)
    {
        std::cout << "CL_state::load_program: [ERROR] No such file\"" << filename << "\"." << std::endl;
        return 0;
    }

    ss << ifs.rdbuf();
    str = ss.str();
    const char* source = str.c_str();
    size = ss.str().size();

    //create cl program
    program = clCreateProgramWithSource(ocl_context_.context(), 1, &source, &size, &err);

    if (err != CL_SUCCESS)
    {
        std::cout << "CL_state::load_program: [WARNING] Could not create program for \"" << filename << "\"." << std::endl;
        return 0;
    }

    //compile program
    err = clBuildProgram(program, 1, &(ocl_context_.device_id()), NULL, NULL, NULL);

    if (err != CL_SUCCESS)
    {
        clGetProgramBuildInfo(program, ocl_context_.device_id(), CL_PROGRAM_BUILD_LOG, 0, NULL, &size);
        str.resize(size);
        clGetProgramBuildInfo(program, ocl_context_.device_id(), CL_PROGRAM_BUILD_LOG, size, &str[0], NULL);
        std::cout << "CL_state::load_program: [WARNING] Could not build program for \"" << filename << "\". Log:" << std::endl;
        std::cout << str << std::endl;

        return 0;
    }

    return program;
}

cl_kernel CL_state::get_kernel(cl_program program, const char *kernel_name)
{
    cl_int err;
    cl_kernel kernel;
    kernel = clCreateKernel(program, kernel_name, &err);
    if (err != CL_SUCCESS)
    {
        std::cout << "CL_state::get_kernel: [WARNING] Could not create kernel with name \"" << kernel_name << "\"." << std::endl;
        return 0;
    }
    return kernel;
}

} //namespace cl

#endif
