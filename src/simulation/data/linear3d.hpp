#pragma once

#include "vec3d.hpp"

namespace ElectronOptics::Simulation::Data {    
    class Linear3d {
    public:
        // f(x, y, z) = a + b * x + c * y + d * z;
        Linear3d(double a, double b, double c, double d) : a(a), b(b), c(c), d(d), gradient(b, c, d) {}
        Linear3d() : Linear3d(0, 0, 0, 0) {}
        double eval(vec3d pos) const {
            return a + b * pos.x + c * pos.y + d * pos.z;
        }

        double a, b, c, d;
        vec3d gradient;
    };
} // namespace ElectronOptics::Simulation::Data