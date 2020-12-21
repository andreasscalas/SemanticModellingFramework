#ifndef HT_UTIL_H
#define HT_UTIL_H

//#include <util/ModelFile.h>
//#include "SkeletonFile.h"
//#include <util/PostureFile.h>

#include <vector>
#include <string>

namespace ik {

    // set scheduler cpu affinity, number of omp threads, init parallel eigen3
    void setNumThreads(int n, bool out = false);

    // concatenate strings contained in the input vector
    std::string cat(const std::vector<std::string> &v);

    // check whether a contains b
    bool contains_string(const std::string &a, const std::string &b);

    // rgb color
    struct ColorRGB
    {
        double r; // percent
        double g; // percent
        double b; // percent

        ColorRGB() {}

        ColorRGB(double r, double g, double b)
            : r(r), g(g), b(b) {}
    };

    // hsv color
    struct ColorHSV
    {
        double h; // angle in degrees
        double s; // percent
        double v; // percent

        ColorHSV() {}

        ColorHSV(double h, double s, double v)
            : h(h), s(s), v(v) {}
    };

    // color space conversions
    ColorHSV rgb2hsv(const ColorRGB &in);
    ColorRGB hsv2rgb(const ColorHSV &in);

} // namespace ik

#endif

