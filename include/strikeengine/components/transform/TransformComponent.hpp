#pragma once

#include "strikeengine/ecs/Component.hpp"

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>


namespace StrikeEngine {

    struct TransformComponent final : public Component {
        glm::dvec3 position{0,0,0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 scale{1.0};

        [[nodiscard]] glm::dmat4 getMatrix() const {
            const glm::dmat4 translationMatrix = glm::translate(glm::dmat4(1.0), position);
            const glm::dmat4 rotationMatrix = glm::toMat4(orientation);
            const glm::dmat4 scaleMatrix = glm::scale(glm::dmat4(1.0), scale);
            return translationMatrix * rotationMatrix * scaleMatrix;
        }
    };

} // namespace StrikeEngine
