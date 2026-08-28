#include <strikeengine/kernel/profiles/MotorProfileDatabase.hpp>
#include <strikeengine/models/physics/propulsion/ThrustCurve.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace StrikeEngine::Kernel {

    bool MotorProfileDatabase::loadProfile(const std::string& file_path) {
        std::ifstream f(file_path);
        if (!f.is_open()) {
            return false;
        }

        try {
            nlohmann::json data = nlohmann::json::parse(f);

            PropulsionConfig cfg;
            for (const auto& stageJson : data.at("stages")) {
                StageConfig stage;
                for (const auto& pointJson :
                     stageJson.value("thrust_curve", nlohmann::json::array())) {
                    Models::ThrustDataPoint point;
                    point.time_s = pointJson.at("time_s").get<double>();
                    point.thrust_n = pointJson.at("thrust_n").get<double>();
                    stage.thrustCurve.push_back(point);
                }
                stage.vacuumIsp       = stageJson.value("vacuum_isp", stage.vacuumIsp);
                stage.seaLevelIsp     = stageJson.value("sea_level_isp", stage.seaLevelIsp);
                stage.propellantMassKg = stageJson.value("propellant_mass_kg", stage.propellantMassKg);
                stage.dryMassKg       = stageJson.value("dry_mass_kg", stage.dryMassKg);
                cfg.stages.push_back(stage);
            }
            _propulsion = cfg;
        } catch (const std::exception&) {
            // Any load failure (syntax, missing required key, wrong type)
            // makes the profile unloadable; createVehicle translates this
            // into a friendly std::runtime_error.
            return false;
        }

        return true;
    }

    const PropulsionConfig& MotorProfileDatabase::propulsion() const {
        return _propulsion;
    }

} // namespace StrikeEngine::Kernel
