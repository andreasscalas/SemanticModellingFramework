#ifndef GRAPHENE_CL_STATE_H
#define GRAPHENE_CL_STATE_H

#ifdef HAVE_OCL

#include "CL_context.h"


#include <vector>
#include <string>
namespace cl {

enum Kernel_type
{
    BLENDSHAPE_KERNEL = 0,
    CORRESPONDENCES_PS2MESH_KERNEL,
    CORRESPONDENCES_MESH2PS_KERNEL,
    KERNEL_MAX
};

class CL_state
{

    //singleton stuff ---------------------------------------------------------
public:
    static CL_state* instance();


private:
    static CL_state* instance_;
    CL_state();
    CL_state(const CL_state&) {}
    ~CL_state();

    /*!
         * \brief The Guard class
         * Guard to make sure all data are properly deleted.
         */
    class Guard
    {
    public:
        ~Guard()
        {
            if (CL_state::instance_ != NULL)
            {
                delete CL_state::instance_;
                CL_state::instance_ = NULL;
            }
        }
    };

    friend class Guard;

    //singleton stuff end------------------------------------------------------

    CL_context ocl_context_;

    //compiled opencl programs
    std::vector<cl_program> programs_;
    //compiled opencl kernels
    std::vector<cl_kernel> kernels_;

public:

    ///loads and compiles kernels and initializes other stuff
    bool init(const char* argv_0);

    bool valid() { return ocl_context_.context() != 0; }

    CL_context& ocl_context() { return ocl_context_; }
    const CL_context& ocl_context() const { return ocl_context_; }

    ///return the kernel of type 'k' to use it other classes
    const cl_kernel& kernel(Kernel_type k);
private:

    ///load opencl programs and kernels
    bool load_programs(const char* argv_0);

    ///load a specific program
    cl_program load_program(const std::string &filename);

    /// get kernel with "kernel_name" for program "program"
    cl_kernel get_kernel(cl_program program, const char* kernel_name);


};


} //namespace cl

#endif

#endif
