#include "clickRayCaster.hpp"

#include <glm/gtc/matrix_transform.hpp>

using namespace ElectronOptics;

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

bool rayAABB(const Ray& ray, const AABB& aabb, float& tHit) {
    glm::vec3 invDir = 1.0f / ray.direction;
    glm::vec3 t0 = (aabb.min - ray.origin) * invDir;
    glm::vec3 t1 = (aabb.max - ray.origin) * invDir;

    glm::vec3 tMin = glm::min(t0, t1);
    glm::vec3 tMax = glm::max(t0, t1);

    float tEnter = glm::max(glm::max(tMin.x, tMin.y), tMin.z);
    float tExit  = glm::min(glm::min(tMax.x, tMax.y), tMax.z);

    if (tEnter > tExit || tExit < 0.0f) return false;

    tHit = tEnter > 0 ? tEnter : tExit;
    return true;
}

AABB calculateAABB(const ModelRenderInfo& modelInfo) {
    
    glm::vec3 min(FLT_MAX, FLT_MAX, FLT_MAX);
    glm::vec3 max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for(const auto& vertex : modelInfo.model.getVertices()) {
        glm::vec4 worldPos = modelInfo.modelMatrix * glm::vec4(vertex, 1.0f);
        min = glm::min(min, glm::vec3(worldPos));
        max = glm::max(max, glm::vec3(worldPos));
    }

    return {min, max};
}

size_t ClickRayCaster::castRay(glm::vec2 mousePos, glm::vec3 camPos, glm::vec3 camForward, const std::vector<ModelRenderInfo> &models, const std::vector<ModelRenderInfo>& ignore) {
    
    glm::mat4 view = glm::lookAt(camPos, camPos + camForward, glm::vec3(0,1,0));
    glm::ivec4 viewport;
    glGetIntegerv(GL_VIEWPORT, &viewport[0]);
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), static_cast<float>(viewport[2]) / static_cast<float>(viewport[3]), 0.1f, 100.0f);

    // Convert to Normalized Device Coordinates [-1, 1]
    float ndcX = (2.0f * mousePos.x) / viewport[2] - 1.0f;
    float ndcY = 1.0f - (2.0f * mousePos.y) / viewport[3]; // Y is flipped

    // Unproject two points at different depths to get ray direction
    glm::vec4 nearNDC = { ndcX, ndcY, -1.0f, 1.0f };
    glm::vec4 farNDC  = { ndcX, ndcY,  1.0f, 1.0f };

    glm::mat4 invVP = glm::inverse(projection * view);

    glm::vec4 nearWorld = invVP * nearNDC;
    glm::vec4 farWorld  = invVP * farNDC;

    nearWorld /= nearWorld.w;
    farWorld  /= farWorld.w;

    Ray ray;
    ray.origin    = glm::vec3(nearWorld);
    ray.direction = glm::normalize(glm::vec3(farWorld) - glm::vec3(nearWorld));


    size_t closestModelIndex = models.size();
    float closestT = FLT_MAX;

    for (size_t i = 0; i < models.size(); ++i) {
        // Skip ignored models
        if (std::find(ignore.begin(), ignore.end(), models[i]) != ignore.end()) {
            continue;
        }

        AABB aabb = calculateAABB(models[i]);
        float tHit;
        if (rayAABB(ray, aabb, tHit)) {
            if (tHit < closestT) {
                closestT = tHit;
                closestModelIndex = i;
            }
        }
    }

    return closestModelIndex;
}