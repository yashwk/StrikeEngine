#pragma once

#include "nlohmann/json.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace glm {

    inline void from_json(const nlohmann::json& j, dvec3& v) {
        j.at(0).get_to(v.x);
        j.at(1).get_to(v.y);
        j.at(2).get_to(v.z);
    }

    inline void from_json(const nlohmann::json& j, dquat& q) {
        j.at(0).get_to(q.w);
        j.at(1).get_to(q.x);
        j.at(2).get_to(q.y);
        j.at(3).get_to(q.z);
    }

    inline void from_json(const nlohmann::json& j, dmat3& m) {
        m[0] = j.at(0).get<dvec3>();
        m[1] = j.at(1).get<dvec3>();
        m[2] = j.at(2).get<dvec3>();
    }

} // namespace glm
