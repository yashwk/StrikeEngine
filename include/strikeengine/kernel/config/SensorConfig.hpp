#pragma once

namespace StrikeEngine::Kernel {

    struct SensorConfig {
        bool imuEnabled = true;
        bool gpsEnabled = true;
        // Realistic tactical-INS noise floors (tight enough for the strapdown
        // EKF to track attitude over long engagements). Gyro noise is ~0.01
        // deg/s (1.7e-4 rad/s); accelerometer ~0.02 m/s^2; a coarse 1-sigma
        // gyro/accel bias.
        double accelNoiseStdDev = 0.02;
        double accelBiasStdDev  = 0.005;
        double gyroNoiseStdDev  = 0.0002;
        double gyroBiasStdDev   = 0.00005;
        double gpsPosNoiseStdDev = 1.5;
        double gpsVelNoiseStdDev = 0.15;
        double gpsUpdateRateHz = 1.0;
        // Normalized innovation gate for each scalar GPS position/velocity
        // measurement. <= 0 disables rejection; the default is a 5-sigma gate.
        double gpsInnovationGateSigma = 5.0;
        double imuLeverArmX = 0.0;
        double imuLeverArmY = 0.0;
        double imuLeverArmZ = 0.0;

        // --- Barometric altitude aiding (opt-in; off = legacy GPS/INS) ---
        bool   baroEnabled = false;
        double baroNoiseStdDev = 1.0;      // m, white
        double baroBiasStdDev = 0.0;       // m/sqrt(s) random-walk bias drift
        double baroUpdateRateHz = 1.0;

        // --- Magnetometer heading aiding (opt-in; tilted-dipole MVP model) --
        bool   magEnabled = false;
        double magNoiseStdDev = 50e-9;     // T per axis (50 nT)
        double magUpdateRateHz = 10.0;
        double magDisturbanceGateRel = 0.25; // reject when |B| disagrees by more

        // --- GPS realism (all default to legacy behavior) ------------------
        double gpsLatencySec = 0.0;        // measurement delay; 0 = same-tick
        double gpsLeverArmX = 0.0;         // antenna offset, body frame (m)
        double gpsLeverArmY = 0.0;
        double gpsLeverArmZ = 0.0;
        bool   gpsFixConsistencyEnabled = false; // whole-fix chi-square gate

        // --- INS algorithm options (default = legacy single-sample) --------
        bool   insConingCompensationEnabled = false; // two-sample coning/sculling
        bool   insAdaptiveQEnabled = false;          // dynamics-scaled process noise
        double insAdaptiveQGain = 1.0;

        // --- Alignment realism (0 = perfect legacy alignment from truth) ---
        double initialAttitudeErrorDeg = 0.0;  // per-axis 1-sigma
        double initialPositionErrorM = 0.0;    // per-axis 1-sigma
        double initialVelocityErrorMps = 0.0;  // per-axis 1-sigma
    };

} // namespace StrikeEngine::Kernel
