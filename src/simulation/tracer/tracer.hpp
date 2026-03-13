#pragma once

#include <simulation/data/vec3d.hpp>
#include <simulation/data/potentialMeshOcttree.hpp>

namespace ElectronOptics::Simulation::Tracer {

    struct SimulationParameters {
        double timeStep;
        size_t maxSteps;
        double electronCharge;
        double electronMass;
    };

    class Tracer {
    public:
        Tracer(const Data::PotentialMeshOcttree& octtree, const SimulationParameters& params) : m_octtree(octtree), m_params(params) {}

        std::vector<vec3d> traceElectronPath(const vec3d& initialPosition, const vec3d& initialVelocity) const;
    private:
        const Data::PotentialMeshOcttree& m_octtree;
        const SimulationParameters& m_params;
    };
} // namespace ElectronOptics::Simulation::Tracer