#include "Util.h"

#include <iostream>
#include <iterator>
#include <omp.h>
#include <Eigen/Core>

namespace ik {

//===========================================================================//

void setNumThreads(int n, bool out)
{
#ifndef _WIN32
	cpu_set_t my_set;
	CPU_ZERO(&my_set);
	for(int i = 0; i < n; ++i)
		CPU_SET(i, &my_set);

	sched_setaffinity(0, sizeof(cpu_set_t), &my_set);
	omp_set_num_threads(n);
	Eigen::initParallel();

	if(out)
	{
		std::cout << " Number of processors available: " << omp_get_num_procs()
			<< " MAX number of OpenMP threads: "   << omp_get_max_threads() << std::endl;
	}
#endif
}

//===========================================================================//

std::string cat(const std::vector<std::string> &v)
{
	std::ostringstream s;
	std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(s));
	return s.str();
}

//===========================================================================//

bool contains_string(const std::string &a, const std::string &b)
{
    std::size_t found = a.find(b);
    return found != std::string::npos;
}

//===========================================================================//

ColorHSV rgb2hsv(const ColorRGB &in)
{
	ColorHSV    out;
	double      min, max, delta;

	min = in.r < in.g ? in.r : in.g;
	min = min  < in.b ? min  : in.b;

	max = in.r > in.g ? in.r : in.g;
	max = max  > in.b ? max  : in.b;

	out.v = max;                                // v
	delta = max - min;

	if( max > 0.0 )
	{
		out.s = (delta / max);                  // s
	}
	else
	{
		// r = g = b = 0                        // s = 0, v is undefined
		out.s = 0.0;
		out.h = NAN;                            // its now undefined
		return out;
	}

	if( in.r >= max )
	{
		out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
	}
	else if( in.g >= max )
	{
		out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
	}
	else
	{
		out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan
	}

	out.h *= 60.0;                              // degrees

	if( out.h < 0.0 )
	{
		out.h += 360.0;
	}

	return out;
}

//===========================================================================//

ColorRGB hsv2rgb(const ColorHSV &in)
{
	double      hh, p, q, t, ff;
	long        i;
	ColorRGB    out;

	if(in.s <= 0.0)
	{
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}

	hh = in.h;

	if(hh >= 360.0)
	{
		hh = 0.0;
	}

	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;

	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch(i)
	{
		case 0:
			out.r = in.v;
			out.g = t;
			out.b = p;
			break;

		case 1:
			out.r = q;
			out.g = in.v;
			out.b = p;
			break;

		case 2:
			out.r = p;
			out.g = in.v;
			out.b = t;
			break;

		case 3:
			out.r = p;
			out.g = q;
			out.b = in.v;
			break;

		case 4:
			out.r = t;
			out.g = p;
			out.b = in.v;
			break;

		case 5:
		default:
			out.r = in.v;
			out.g = p;
			out.b = q;
			break;
	}

	return out;
}

//===========================================================================//

} // namespace ik

