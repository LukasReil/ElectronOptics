#pragma once

#include "simulation/data/potentialMesh.hpp"

namespace ElectronOptics::Simulation::Solver {

    class FEMSolver {
    public:
        FEMSolver(Data::PotentialMesh& inputMesh);
        
        void solve();
    private:
        Data::PotentialMesh& m_inputMesh;
    };

} // namespace ElectronOptics::Simulation::Solver