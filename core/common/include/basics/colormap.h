/**
 * @file colormap.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef SRC_COLORMAP_H
#define SRC_COLORMAP_H

#include <map>
#include "basics/basics.h"

namespace common
{
    struct ColorARGB
    {
        double a;
        double r;
        double g;
        double b;
        ColorARGB(const double& A, const double& R, const double& G, const double& B)
                : a(A), r(R), g(G), b(B) {}
        ColorARGB() : a(0.0), r(0.0), g(0.0), b(0.0) {}

        ColorARGB set_a(const double _a) { return ColorARGB(_a, r, g, b); }
    };

    extern std::map<std::string, ColorARGB> cmap;
    extern std::map<double, ColorARGB> jet_map;
    extern std::map<double, ColorARGB> autumn_map;

    /**
     * @brief Get the color in colormap by value using LUT
     *
     * @param val input value
     * @param m color map
     * @return ColorARGB color
     */
    ColorARGB GetColorByValue(const double& val,
                              const std::map<double, ColorARGB>& m);

    /**
     * @brief Get the Jet colormap By value using mapping function
     *
     * @param val input value
     * @param max maximum value
     * @param min minimum value
     * @return ColorARGB color
     */
    ColorARGB GetJetColorByValue(const double& val,
                                 const double& max,
                                 const double& min);
}  // namespace common

#endif //SRC_COLORMAP_H
