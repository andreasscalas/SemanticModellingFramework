//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================
#ifndef GRAPHENE_REGISTRATION_H
#define GRAPHENE_REGISTRATION_H
//=============================================================================


//== INCLUDES =================================================================

#include <vector>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/geometry/Vector.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace geometry {

//=============================================================================


enum Registration_method {RIGID_REGISTRATION, CONFORMAL_REGISTRATION, R_TRANS_SCALE_ANISO_XYZ, R_TRANS_SCALE_ANISO_XZ };

template <typename Scalar>
Mat4<Scalar> registration(const std::vector<Vector<Scalar,3> >& _src,
                          const std::vector<Vector<Scalar,3> >& _dst,
                          Registration_method _mapping = RIGID_REGISTRATION,
                          double* relative_change = NULL,
                          bool no_rotation = false,
                          bool no_translation = false)
{
    assert(_src.size() == _dst.size());
    const int n = _src.size();
    assert(n>2);


    double error_before = 0.0;
    if (relative_change != NULL)
    {
        for ( unsigned int i = 0; i < n; ++i )
        {
            error_before += sqrnorm( _dst[i] - _src[i] );
        }
    }


    // compute barycenters
    Vector<Scalar,3> scog(0.0,0.0,0.0), dcog(0.0,0.0,0.0);
    if (!no_translation)
    {
        for (int i=0; i<n; ++i)
        {
            scog += _src[i];
            dcog += _dst[i];
        }
        scog /= (Scalar) n;
        dcog /= (Scalar) n;
    }

    // build matrix
    Mat4d M;
    {
        double  xx(0.0), xy(0.0), xz(0.0), yx(0.0), yy(0.0), yz(0.0), zx(0.0), zy(0.0), zz(0.0);
        Vector<Scalar,3>   sp, dp;

        for (int i=0; i<n; ++i)
        {
            sp = _src[i]; sp -= scog;
            dp = _dst[i]; dp -= dcog;
            xx += sp[0] * dp[0];
            xy += sp[0] * dp[1];
            xz += sp[0] * dp[2];
            yx += sp[1] * dp[0];
            yy += sp[1] * dp[1];
            yz += sp[1] * dp[2];
            zx += sp[2] * dp[0];
            zy += sp[2] * dp[1];
            zz += sp[2] * dp[2];
        }

        M(0,0) =  xx + yy + zz;
        M(1,1) =  xx - yy - zz;
        M(2,2) = -xx + yy - zz;
        M(3,3) = -xx - yy + zz;
        M(1,0) = M(0,1) = yz - zy;
        M(2,0) = M(0,2) = zx - xz;
        M(2,1) = M(1,2) = xy + yx;
        M(3,0) = M(0,3) = xy - yx;
        M(3,1) = M(1,3) = zx + xz;
        M(3,2) = M(2,3) = yz + zy;
    }


    // symmetric eigendecomposition
    Mat4d   V = Mat4d::identity();
    unsigned int iter(50);
    {
        int     i, j, k;
        double  theta, t, c, s, ss, g, h, tau, tM;

        while (--iter)
        {
            // find largest off-diagonal element
            i=0; j=1; ss=fabs(M(0,1));
            if ( (s=fabs(M(0,2))) > ss) { ss=s; i=0; j=2; }
            if ( (s=fabs(M(0,3))) > ss) { ss=s; i=0; j=3; }
            if ( (s=fabs(M(1,2))) > ss) { ss=s; i=1; j=2; }
            if ( (s=fabs(M(1,3))) > ss) { ss=s; i=1; j=3; }
            if ( (s=fabs(M(2,3))) > ss) { ss=s; i=2; j=3; }

            // converged?
            if (ss < 1e-10) break;

            // compute Jacobi rotation
            theta = 0.5 * (M(j,j) - M(i,i)) / M(i,j);
            t     = (theta<0.0 ? -1.0 : 1.0) / (fabs(theta) + sqrt(1.0+theta*theta));
            c     = 1.0 / sqrt(1.0 + t*t);
            s     = t*c;
            tau   = s/(1.0+c);
            tM    = t*M(i,j);

#define rot(a, s, t, i, j, k, l) \
{ g=a(i,j); h=a(k,l); a(i,j)=g-s*(h+g*t); a(k,l)=h+s*(g-h*t); }

            M(i,j)  = 0.0;
            for (k=  0; k<i; ++k)  rot(M, s, tau, k, i, k, j);
            for (k=i+1; k<j; ++k)  rot(M, s, tau, i, k, k, j);
            for (k=j+1; k<4; ++k)  rot(M, s, tau, i, k, j, k);
            for (k=  0; k<4; ++k)  rot(V, s, tau, k, i, k, j);
            M(i,i) -= tM;
            M(j,j) += tM;
        }
    }


    // did it work?
    if (!iter)
    {
        std::cerr << "registration: Jacobi did not converge\n";
        return Mat4<Scalar>::identity();
    }


    // eigenvector wrt largest eigenvalue -> quaternion
    Vec4d q;
    {
        int imax=0;
        double s, ss = M(imax,imax);
        if ( (s=M(1,1)) > ss) { ss=s; imax=1; }
        if ( (s=M(2,2)) > ss) { ss=s; imax=2; }
        if ( (s=M(3,3)) > ss) { ss=s; imax=3; }
        q = Vec4d( V(0,imax), V(1,imax), V(2,imax), V(3,imax) );
        q.normalize();
    }


    // rotation part
    Scalar
    ww(q[0]*q[0]), xx(q[1]*q[1]), yy(q[2]*q[2]), zz(q[3]*q[3]),
    wx(q[0]*q[1]), wy(q[0]*q[2]), wz(q[0]*q[3]),
    xy(q[1]*q[2]), xz(q[1]*q[3]), yz(q[2]*q[3]);
    Mat4<Scalar> T;
    if (no_rotation)
    {
        T(0,0) = 1.0;
        T(1,0) = 0.0;
        T(2,0) = 0.0;
        T(3,0) = 0.0;
        T(0,1) = 0.0;
        T(1,1) = 1.0;
        T(2,1) = 0.0;
        T(3,1) = 0.0;
        T(0,2) = 0.0;
        T(1,2) = 0.0;
        T(2,2) = 1.0;
        T(3,2) = 0.0;
    }
    else
    {
        T(0,0) = ww + xx - yy - zz;
        T(1,0) = 2.0*(xy + wz);
        T(2,0) = 2.0*(xz - wy);
        T(3,0) = 0.0;
        T(0,1) = 2.0*(xy - wz);
        T(1,1) = ww - xx + yy - zz;
        T(2,1) = 2.0*(yz + wx);
        T(3,1) = 0.0;
        T(0,2) = 2.0*(xz + wy);
        T(1,2) = 2.0*(yz - wx);
        T(2,2) = ww - xx - yy + zz;
        T(3,2) = 0.0;
    }


    // scaling
    if (_mapping == CONFORMAL_REGISTRATION ||
        _mapping == R_TRANS_SCALE_ANISO_XYZ ||
        _mapping == R_TRANS_SCALE_ANISO_XZ)
    {
        Vector<Scalar,3>  sp, dp;
        Scalar  nom(0), denom(0);
        Scalar  nomX(0), denomX(0);
        Scalar  nomY(0), denomY(0);
        Scalar  nomZ(0), denomZ(0);

        for (int i=0; i<n; ++i)
        {
            sp = _src[i]; sp -= scog;
            dp = _dst[i]; dp -= dcog;

            sp = linear_transform(T, sp);

            nom   += dot(sp,dp);
            denom += dot(sp,sp);

            nomX   += sp[0]*dp[0];
            denomX += sp[0]*sp[0];
            nomY   += sp[1]*dp[1];
            denomY += sp[1]*sp[1];
            nomZ   += sp[2]*dp[2];
            denomZ += sp[2]*sp[2];
        }

        const Scalar scaling = nom / denom;
        const float scalingX = nomX / denomX;
        const float scalingY = nomY / denomY;
        const float scalingZ = nomZ / denomZ;

        if (_mapping == CONFORMAL_REGISTRATION)
        {
            for (int i=0; i<3; ++i)
            {
                for (int j=0; j<3; ++j)
                {
                    T(i,j) *= scaling;
                }
            }
        }
        else if (_mapping == R_TRANS_SCALE_ANISO_XYZ)
        {
            /*
            std::cerr << "In registration: SCALING X: " << scalingX << std::endl;
            std::cerr << "In registration: SCALING Y: " << scalingY << std::endl;
            std::cerr << "In registration: SCALING Z: " << scalingZ << std::endl;
            */
            for (int j=0; j<3; ++j)
            {
                T(0,j) *= scalingX;
                T(1,j) *= scalingY;
                T(2,j) *= scalingZ;
            }
        }
        else if (_mapping == R_TRANS_SCALE_ANISO_XZ)
        {
            std::cerr << "In registration: SCALING X: " << scalingX << std::endl;
            std::cerr << "In registration: SCALING Z: " << scalingZ << std::endl;
            for (int j=0; j<3; ++j)
            {
                T(0,j) *= scalingX;
                T(2,j) *= scalingZ;
            }
        }
    }


    // translation part
    if (no_translation)
    {
        T(0,3) = 0.0;
        T(1,3) = 0.0;
        T(2,3) = 0.0;
        T(3,3) = 1.0;
    }
    else
    {
        T(0,3) = dcog[0] - T(0,0)*scog[0] - T(0,1)*scog[1] - T(0,2)*scog[2];
        T(1,3) = dcog[1] - T(1,0)*scog[0] - T(1,1)*scog[1] - T(1,2)*scog[2];
        T(2,3) = dcog[2] - T(2,0)*scog[0] - T(2,1)*scog[1] - T(2,2)*scog[2];
        T(3,3) = 1.0;
    }

    double error_after = 0.0;
    if (relative_change != NULL)
    {
        Vector<Scalar,3> sp;
        for ( unsigned int i = 0; i < n; ++i )
        {
            sp = _src[i];
            sp = affine_transform(T, sp);

            error_after += sqrnorm( _dst[i] - sp );
        }
    }


    if (error_after > error_before)
    {
        std::cerr << "registration ERROR: Error afterwards is larger than error before!" << std::endl;
    }
    if (relative_change != NULL)
    {
        *relative_change = (error_before - error_after) / error_before;
    }

    return T;
}


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_REGISTRATION_H
//=============================================================================
