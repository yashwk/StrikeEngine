#pragma once

#include <array>
#include <functional>
#include <memory>

#include <strikeengine/models/terrain/GlobalTerrain.hpp>

namespace StrikeEngine::Kernel {

    /**
     * @brief Optional local-earth corrections for the truth model.
     *
     * In local mode, world X/Y/Z are interpreted as ENU (east, north, up)
     * for earth corrections. With useEcefTruth, PhysicsBlock position and
     * velocity are absolute ECEF values and the reference latitude/longitude
     * anchor terrain callbacks in a local ENU view. The default remains the
     * legacy flat-earth constant-gravity model.
     */
    struct EarthEnvironmentConfig {
        // When enabled, PhysicsBlock position/velocity are absolute ECEF
        // metres and m/s. The default remains local ENU-style coordinates.
        bool useEcefTruth = false;
        bool useWgs84Gravity = false;
        bool useSphericalGravity = false;
        // Opt-in central + J2 zonal-harmonic gravity. Takes precedence over
        // normal/spherical gravity and is expressed in ECEF or local ENU.
        bool includeJ2Gravity = false;
        bool includeCoriolis = false;
        bool includeCentrifugal = false;
        bool includeTransportRate = false;
        // In ECEF truth mode, model the gyro as measuring the INERTIAL body
        // rate (earth rotation included) and compensate it in the IMU/INS.
        // Ignored in local flat-earth mode. Default false preserves behavior.
        bool includeEarthRateGyro = false;
        double referenceLatitudeRad = 0.0;
        double referenceLongitudeRad = 0.0;
    };

    /**
     * @brief Flat-earth environment hooks used by the CPU truth model.
     *
     * Terrain elevation is expressed in world metres above the reference
     * datum. Wind is the world-frame air-mass velocity at the queried position
     * and time; the aerodynamic model uses vehicle velocity relative to this
     * air mass. Default callbacks describe the existing zero-wind, flat-ground
     * environment.
     */
    struct EnvironmentConfig {
        EarthEnvironmentConfig earth{};

        std::function<double(double x, double y)> terrainElevation =
            [](double, double) { return 0.0; };

        // Optional geodetic terrain database. When present it takes
        // precedence over terrainElevation and is sampled at the vehicle's
        // WGS84 latitude/longitude in both local and ECEF truth modes.
        std::shared_ptr<const Models::TerrainSource> globalTerrain;

        std::function<std::array<double, 3>(double x, double y, double z, double time)>
            windVelocity = [](double, double, double, double) {
                return std::array<double, 3>{0.0, 0.0, 0.0};
            };

        // Kinetic-impact (body-contact) band in metres. Opposing-allegiance
        // entities that close to within this distance dispatch TargetImpact
        // (report-only; lethality stays warhead-governed). Matches the sim
        // viewport's kinetic-hit display radius (15 m). <= 0 disables
        // kinetic-impact detection.
        double kineticImpactRadiusM = 15.0;

        // Report-only kinetic contact: when true, TargetImpact is dispatched
        // once per opposing pair per contact episode instead of every step
        // the pair stays inside the band. Default false = legacy event flood.
        bool kineticImpactLatchEnabled = false;

        // Ground-impact detection: when true, the terrain is also sampled at
        // the segment midpoint so a ridge between two above-ground endpoint
        // samples is detected (legacy false = endpoint-only crossing test).
        bool sweptGroundImpactEnabled = false;

        // When true, a ground impact also zeroes the body angular rates
        // (legacy false leaves the rotation state untouched). Position,
        // velocity and acceleration are always zeroed.
        bool groundImpactZeroRates = false;
    };

} // namespace StrikeEngine::Kernel
