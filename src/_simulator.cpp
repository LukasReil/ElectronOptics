#include "_simulator.hpp"

#include <random>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

using namespace ElectronOptics;
glm::vec3 Simulator::lensField(const VerticalLens& lens, const glm::vec3& pos)
{
    glm::vec3 deltaPos = pos - lens.position;
    float r2 = deltaPos.x*deltaPos.x + deltaPos.z*deltaPos.z;
    float gaussian = std::exp(-(deltaPos.y)*(deltaPos.y)/(lens.width*lens.width));

    glm::vec3 E;
    E.x = -lens.strength * deltaPos.x * gaussian;
    E.y = 0.0f;
    E.z = -lens.strength * deltaPos.z * gaussian;

    return E;
}

glm::vec3 Simulator::acceleration(const glm::vec3& pos) {

    glm::vec3 field(0,0,0);

    for (const auto& lens : m_lenses) {
        field += lensField(lens, pos);
    }


    return field * Electron::CHARGE / Electron::MASS;
}

void Simulator::integrateRK4(Electron& e) {
    auto f = [this](const Electron& s)
    {
        Electron d;
        d.pos = s.vel;
        d.vel = acceleration(s.pos);
        return d;
    };

    Electron k1 = f(e);

    Electron s2 = e;
    s2.pos += 0.5f * DT * k1.pos;
    s2.vel += 0.5f * DT * k1.vel;
    Electron k2 = f(s2);

    Electron s3 = e;
    s3.pos += 0.5f * DT * k2.pos;
    s3.vel += 0.5f * DT * k2.vel;
    Electron k3 = f(s3);

    Electron s4 = e;
    s4.pos += DT * k3.pos;
    s4.vel += DT * k3.vel;
    Electron k4 = f(s4);

    e.pos += DT/6.0f * (k1.pos + 2.0f*k2.pos + 2.0f*k3.pos + k4.pos);
    e.vel += DT/6.0f * (k1.vel + 2.0f*k2.vel + 2.0f*k3.vel + k4.vel);
}

std::vector<std::vector<glm::vec3>> Simulator::simulate() {

    std::vector<std::vector<glm::vec3>> results;
    std::mt19937 rng(42);

    for (const auto& source : m_sources) {
        std::normal_distribution<float> posDist(0.0f, source.positionSpread);
        std::normal_distribution<float> beamRadius(0.0f, source.beamRadius);
        std::normal_distribution<float> energyDist(source.energy, source.energySpread);

        for(size_t i = 0; i < NUM_ELECTRONS_PER_SOURCE; ++i) {
            Electron e;
            e.pos = source.centerPosition + glm::vec3(posDist(rng), posDist(rng), posDist(rng));

            glm::vec3 sourceDirection = glm::normalize(source.direction);
            glm::vec3 sourceRight;
            glm::vec3 sourceUp;
            if (sourceDirection == glm::vec3(0,1,0) || sourceDirection == glm::vec3(0,-1,0)) {
                sourceRight = glm::vec3(0,0,1);
            } else {
                sourceRight = glm::normalize(glm::cross(sourceDirection, glm::vec3(0,1,0)));
            }

            sourceUp = glm::normalize(glm::cross(sourceRight, sourceDirection));

            // spdlog::info("Source direction: {}, right: {}, up: {}", glm::to_string(sourceDirection), glm::to_string(sourceRight), glm::to_string(sourceUp));



            float energy = energyDist(rng);


            e.vel = glm::vec3(source.direction) * energy + beamRadius(rng) * sourceRight + beamRadius(rng) * sourceUp;

            std::vector<glm::vec3> traj;

            int steps = SIM_TIME / DT;
            for(int s=0; s<steps; ++s) {            
                integrateRK4(e);
                traj.push_back(e.pos);
            }

            results.push_back(traj);
        }
    }

    return results;

}
