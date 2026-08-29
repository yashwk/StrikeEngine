#include <strikeengine/kernel/systems/GuidanceSystem.hpp>
#include <strikeengine/models/guidance/GuidanceModels.hpp>

#include <cmath>
#include <cstdio>

using namespace StrikeEngine::Kernel;

int main()
{
    std::printf("=== guidance_test: PN, APN, moving target, and handoff ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    const StrikeEngine::Models::Vec3 relativePosition{1000.0, 0.0, 0.0};
    const StrikeEngine::Models::Vec3 closingVelocity{-100.0, 10.0, 0.0};
    const auto pn = StrikeEngine::Models::proportionalNavigation(
        relativePosition, closingVelocity, 3.0);
    check(pn.valid && std::abs(pn.closingSpeed - 100.0) < 1e-12,
          "PN reports valid closing geometry");
    check(std::abs(pn.acceleration[1] - 3.0) < 1e-12 &&
              std::abs(pn.acceleration[0]) < 1e-12,
          "PN commands the expected lateral acceleration for a moving target");

    const auto apn = StrikeEngine::Models::augmentedProportionalNavigation(
        relativePosition, StrikeEngine::Models::Vec3{-100.0, 0.0, 0.0},
        StrikeEngine::Models::Vec3{0.0, 2.0, 0.0}, 3.5);
    check(apn.valid && std::abs(apn.acceleration[1] - 3.5) < 1e-12,
          "APN adds the normal target-acceleration feed-forward term");

    NavigationBlock nav;
    nav.size = 1;
    nav.estPx = {0.0}; nav.estPy = {0.0}; nav.estPz = {0.0};
    nav.estVx = {100.0}; nav.estVy = {0.0}; nav.estVz = {0.0};
    nav.estQw = {1.0}; nav.estQx = {0.0}; nav.estQy = {0.0}; nav.estQz = {0.0};
    nav.estWx = {0.0}; nav.estWy = {0.0}; nav.estWz = {0.0};

    EntityStatusBlock status;
    status.size = 1;
    status.isAlive = {true};

    GuidanceBlock guidance;
    guidance.mode = {GuidanceMode::ProportionalNavigation};
    guidance.targetX = {1000.0}; guidance.targetY = {0.0}; guidance.targetZ = {0.0};
    guidance.targetVx = {0.0}; guidance.targetVy = {10.0}; guidance.targetVz = {0.0};
    guidance.maxAccel = {0.0};
    guidance.navigationConstant = {3.5};
    guidance.waypointGain = {20.0};
    guidance.commandedAccelX = {0.0}; guidance.commandedAccelY = {0.0};
    guidance.commandedAccelZ = {0.0};

    SeekerBlock seeker;
    seeker.size = 1;
    seeker.type = {SeekerType::None};
    seeker.isLocked = {false};
    seeker.targetRangeRate = {0.0};
    seeker.targetAzimuthRate = {0.0};
    seeker.targetElevationRate = {0.0};
    ControlBlock control;
    GuidanceSystem system;
    system.update(status, nav, seeker, guidance, control, 0.01);
    check(std::abs(guidance.commandedAccelY[0] - 3.5) < 1e-12,
          "kernel PN mode applies the configured navigation constant");

    seeker.type[0] = SeekerType::RF;
    seeker.isLocked[0] = true;
    seeker.targetRangeRate[0] = -100.0;
    seeker.targetAzimuthRate[0] = 0.02;
    seeker.targetElevationRate[0] = 0.0;
    system.update(status, nav, seeker, guidance, control, 0.01);
    check(guidance.commandedAccelZ[0] > 0.0 &&
              std::abs(guidance.commandedAccelZ[0] - 7.0) < 1e-12,
          "locked seeker hands guidance to filtered-rate APN");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
