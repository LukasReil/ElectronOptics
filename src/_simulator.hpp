#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace ElectronOptics {


    struct ElectronSource {
        glm::vec3 centerPosition;
        glm::vec3 direction;
        float energy;
        
        float positionSpread;
        float beamRadius;
        float energySpread;
    };

    struct VerticalLens {
        glm::vec3 position;
        float strength;
        float width;
    };

    
    struct Electron {
        glm::vec3 pos;
        glm::vec3 vel;

        static constexpr float CHARGE{-1.0f};
        static constexpr float MASS{1.0f};
    };

    class Simulator {
    public:
        Simulator(float dt, float simTime, size_t numElectronsPerSource)
            : DT(dt), SIM_TIME(simTime), NUM_ELECTRONS_PER_SOURCE(numElectronsPerSource) {}

        void addSource(const ElectronSource& source) {
            m_sources.push_back(source);
        }

        void addLens(const VerticalLens& lens) {
            m_lenses.push_back(lens);
        }

        void updateSource(size_t index, const ElectronSource& source) {
            m_sources[index] = source;
        }

        void updateLens(size_t index, const VerticalLens& lens) {
            m_lenses[index] = lens;
        }

        std::vector<std::vector<glm::vec3>> simulate();
        
    private:
        const float DT;
        const float SIM_TIME;
        const size_t NUM_ELECTRONS_PER_SOURCE;
        std::vector<ElectronSource> m_sources;
        std::vector<VerticalLens> m_lenses;

        void integrateRK4(Electron& e);
        glm::vec3 lensField(const VerticalLens& lens, const glm::vec3& pos);
        glm::vec3 acceleration(const glm::vec3& pos);
    };

} // namespace ElectronOptics