#pragma once

#include <glm/glm.hpp>
#include "renderer.hpp"

namespace ElectronOptics {

    class ClickRayCaster {
    public:
        static size_t castRay(glm::vec2 mousePos, glm::vec3 camPos, glm::vec3 camForward, const std::vector<ModelRenderInfo>& models, const std::vector<ModelRenderInfo>& ignore = {});
    };

} // namespace ElectronOptics