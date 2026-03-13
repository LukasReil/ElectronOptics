#include "tracer.hpp"

std::vector<vec3d> ElectronOptics::Simulation::Tracer::Tracer::traceElectronPath(const vec3d &initialPosition, const vec3d &initialVelocity) const {

    vec3d currentPosition = initialPosition;
    vec3d currentVelocity = initialVelocity;

    std::vector<vec3d> path;
    path.reserve(m_params.maxSteps);

    for (size_t step = 0; step < m_params.maxSteps; step++) {
        path.push_back(currentPosition);
        
        auto tetrahedronOpt = m_octtree.getTetrahedronAtPosition(currentPosition);
        vec3d electricField{0};
        // If the position is outside the mesh, zero electric field is assumed
        // because it is either outside the bounding box or inside an electrode.
        if (tetrahedronOpt) {
            electricField = tetrahedronOpt->getElectricFieldAtPosition(currentPosition);
        }
        // * 1e6 for converting from V/mm to V/m and from m/s² to mm/s², since the geometry is in mm
        vec3d acceleration = electricField * m_params.electronCharge / m_params.electronMass * 1e6;
        
        // Update velocity and position using simple Euler integration
        // TODO: Swich to RK4
        currentVelocity += acceleration * m_params.timeStep;
        currentPosition += currentVelocity * m_params.timeStep;
    }

    return path;
}