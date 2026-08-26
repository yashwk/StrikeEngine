#include <strikeengine/kernel/systems/SeekerSystem.hpp>
#include <cmath>
#include <cstdio>
#include <string>

using namespace StrikeEngine::Kernel;

namespace {

constexpr double pi = 3.14159265358979323846;

void makeBlocks(PhysicsBlock& physics, EntityStatusBlock& status, SeekerBlock& seeker)
{
    physics.size = 2;
    physics.px = {0.0, 100.0};
    physics.py = {0.0, 0.0};
    physics.pz = {1000.0, 1000.0};
    physics.vx = {0.0, 0.0}; physics.vy = {0.0, 0.0}; physics.vz = {0.0, 0.0};
    physics.qw = {1.0, 1.0}; physics.qx = {0.0, 0.0};
    physics.qy = {0.0, 0.0}; physics.qz = {0.0, 0.0};
    physics.active = {true, true};

    status.size = 2;
    status.allegiance = {Allegiance::Friendly, Allegiance::Hostile};
    status.isAlive = {true, true};
    status.rcsProfileId = {"", std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/flat_rcs.json"};

    seeker.size = 2;
    seeker.type = {SeekerType::RF, SeekerType::None};
    seeker.transmitterPowerW = {1.0e6, 0.0};
    seeker.antennaGainDb = {30.0, 0.0};
    seeker.wavelengthM = {0.03, 0.0};
    seeker.noiseFloorW = {1.0e-12, 1.0};
    seeker.snrThresholdDb = {13.0, 0.0};
    seeker.sensitivityW = {1.0e-9, 1.0};
    seeker.wavelengthBand = {0, 0};
    seeker.fieldOfViewHalfAngleRad = {pi / 4.0, pi / 4.0};
    seeker.gimbalAzimuthLimitRad = {pi / 3.0, pi / 3.0};
    seeker.gimbalElevationLimitRad = {pi / 3.0, pi / 3.0};
    seeker.lockHysteresisDb = {3.0, 3.0};
    seeker.lockDropoutTimeSec = {0.10, 0.10};
    seeker.measurementLatencySec = {0.0, 0.0};
    seeker.isLocked = {false, false};
    seeker.lockedTargetId = {0, 0};
    seeker.targetRange = {0.0, 0.0};
    seeker.targetRangeRate = {0.0, 0.0};
    seeker.targetAzimuth = {0.0, 0.0};
    seeker.targetElevation = {0.0, 0.0};
    seeker.targetAzimuthRate = {0.0, 0.0};
    seeker.targetElevationRate = {0.0, 0.0};
    seeker.previousAzimuth = {0.0, 0.0};
    seeker.previousElevation = {0.0, 0.0};
    seeker.lockLostTimeSec = {0.0, 0.0};
    seeker.hasPreviousLos = {false, false};
}

}

int main()
{
    std::printf("=== seeker_test: FOV, gimbal, hysteresis, and LOS rates ===\n");
    int failures = 0;
    auto check = [&](bool condition, const char* message) {
        if (condition) std::printf("  [PASS] %s\n", message);
        else { std::printf("  [FAIL] %s\n", message); ++failures; }
    };

    PhysicsBlock physics;
    EntityStatusBlock status;
    SeekerBlock seeker;
    makeBlocks(physics, status, seeker);
    SeekerSystem system;

    system.update(physics, status, seeker, 0.01);
    check(seeker.isLocked[0] && seeker.lockedTargetId[0] == 1,
          "front target is acquired");

    physics.px[1] = -100.0;
    system.update(physics, status, seeker, 0.01);
    check(!seeker.isLocked[0], "target behind the seeker is rejected by the FOV");

    physics.px[1] = 100.0;
    physics.py[1] = 100.0 * std::tan(50.0 * pi / 180.0);
    system.update(physics, status, seeker, 0.01);
    check(!seeker.isLocked[0], "target outside the FOV cone is rejected");

    seeker.fieldOfViewHalfAngleRad[0] = pi / 2.0;
    seeker.gimbalAzimuthLimitRad[0] = 30.0 * pi / 180.0;
    system.update(physics, status, seeker, 0.01);
    check(!seeker.isLocked[0], "target beyond the azimuth gimbal stop is rejected");

    physics.py[1] = 0.0;
    seeker.fieldOfViewHalfAngleRad[0] = pi / 4.0;
    seeker.gimbalAzimuthLimitRad[0] = pi / 3.0;
    system.update(physics, status, seeker, 0.01);
    check(seeker.isLocked[0], "target is reacquired inside seeker limits");

    physics.py[1] = 20.0;
    system.update(physics, status, seeker, 0.01);
    check(seeker.targetAzimuthRate[0] > 0.0,
          "tracked azimuth produces a positive filtered LOS rate");

    seeker.measurementLatencySec[0] = 0.05;
    const double publishedAzimuth = seeker.targetAzimuth[0];
    physics.py[1] = 30.0;
    system.update(physics, status, seeker, 0.01);
    check(std::abs(seeker.targetAzimuth[0] - publishedAzimuth) < 1e-12,
          "configured seeker latency delays the new angle measurement");
    for (int step = 0; step < 5; ++step) system.update(physics, status, seeker, 0.01);
    check(seeker.targetAzimuth[0] > 0.0,
          "delayed seeker measurement becomes available after its latency");

    seeker.noiseFloorW[0] = 3.0e-3; // below acquisition SNR, above hold SNR
    system.update(physics, status, seeker, 0.01);
    check(seeker.isLocked[0] && seeker.lockLostTimeSec[0] > 0.0,
          "same-target lock is held through a short signal dropout");

    seeker.noiseFloorW[0] = 1.0e-1;
    system.update(physics, status, seeker, 0.06);
    system.update(physics, status, seeker, 0.06);
    check(!seeker.isLocked[0], "lock is released after the dropout timeout");

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
