#pragma once
#include <cmath>

namespace StrikeEngine::Models {

    struct AeroWrench {
        double force_x, force_y, force_z;
        double torque_x, torque_y, torque_z;
    };

    class AeroModel {
    public:
        virtual ~AeroModel() = default;

        /**
         * @brief Computes aerodynamic forces and moments (Wrench).
         * @param vx, vy, vz Velocity vector components (m/s)
         * @param qx, qy, qz, qw Orientation quaternion (body to world)
         * @param wx, wy, wz Angular velocity (rad/s)
         * @param finPitch, finYaw, finRoll Control surface deflections (rad)
         * @param density Atmospheric density (kg/m^3)
         * @param speedOfSound Speed of sound (m/s)
         * @param referenceArea Reference area (m^2)
         * @return Computed aerodynamic wrench in world frame.
         */
        virtual AeroWrench computeWrench(
            double vx, double vy, double vz,
            double qx, double qy, double qz, double qw,
            double wx, double wy, double wz,
            double finPitch, double finYaw, double finRoll,
            double density, double speedOfSound, double referenceArea) const = 0;
    };

    class BasicAeroModel : public AeroModel {
    public:
        BasicAeroModel(double constantCd = 0.3, double constantCl = 0.1) 
            : cd(constantCd), cl(constantCl) {}

        AeroWrench computeWrench(
            double vx, double vy, double vz,
            double qx, double qy, double qz, double qw,
            double wx, double wy, double wz,
            double finPitch, double finYaw, double finRoll,
            double density, double speedOfSound, double referenceArea) const override 
        {
            double speedSq = vx * vx + vy * vy + vz * vz;
            if (speedSq < 1e-6) {
                return {0, 0, 0, 0, 0, 0};
            }

            double speed = std::sqrt(speedSq);
            double dir_x = vx / speed;
            double dir_y = vy / speed;
            double dir_z = vz / speed;

            double dynamicPressure = 0.5 * density * speedSq;
            
            // Drag
            double dragMag = dynamicPressure * cd * referenceArea;
            double drag_x = -dir_x * dragMag;
            double drag_y = -dir_y * dragMag;
            double drag_z = -dir_z * dragMag;

            // Simplistic Lift: Upward relative to velocity, ignoring true body AoA for MVP
            // Body Up = q * (0, 1, 0) * q^-1.
            // For MVP, we will simplify: lift is in the world UP direction if flying horizontally,
            // or we just cross velocity with right vector.
            // To be precise: body up vector.
            // q * (0,1,0) = (2(xy - wz), 1 - 2(x^2 + z^2), 2(yz + wx))
            double body_up_x = 2.0 * (qx * qy - qw * qz);
            double body_up_y = 1.0 - 2.0 * (qx * qx + qz * qz);
            double body_up_z = 2.0 * (qy * qz + qw * qx);

            // Lift direction is perpendicular to velocity, in the plane of velocity and body_up
            // Right = Velocity x BodyUp
            double right_x = dir_y * body_up_z - dir_z * body_up_y;
            double right_y = dir_z * body_up_x - dir_x * body_up_z;
            double right_z = dir_x * body_up_y - dir_y * body_up_x;

            // LiftDir = Right x Velocity
            double lift_dir_x = right_y * dir_z - right_z * dir_y;
            double lift_dir_y = right_z * dir_x - right_x * dir_z;
            double lift_dir_z = right_x * dir_y - right_y * dir_x;

            double lift_dir_mag = std::sqrt(lift_dir_x * lift_dir_x + lift_dir_y * lift_dir_y + lift_dir_z * lift_dir_z);
            if (lift_dir_mag > 1e-6) {
                lift_dir_x /= lift_dir_mag;
                lift_dir_y /= lift_dir_mag;
                lift_dir_z /= lift_dir_mag;
            } else {
                lift_dir_x = lift_dir_y = lift_dir_z = 0.0;
            }

            double liftMag = dynamicPressure * cl * referenceArea;
            double lift_x = lift_dir_x * liftMag;
            double lift_y = lift_dir_y * liftMag;
            double lift_z = lift_dir_z * liftMag;

            double force_x = drag_x + lift_x;
            double force_y = drag_y + lift_y;
            double force_z = drag_z + lift_z;

            // Simplified control surface torques (World frame mapped)
            // Pitch fin applies pitch torque, Yaw fin applies yaw torque.
            // A true aero model would compute body torques and rotate to world.
            // For MVP, we will compute in body frame and rotate to world.
            double q_inv_w = qw, q_inv_x = -qx, q_inv_y = -qy, q_inv_z = -qz;
            
            // Torque coefficients
            double CM_delta = 1.5; // Pitch/Yaw moment per radian
            double Cl_delta = 0.5; // Roll moment per radian
            
            double body_torque_x = dynamicPressure * referenceArea * 1.0 * (Cl_delta * finRoll);
            double body_torque_y = dynamicPressure * referenceArea * 1.0 * (CM_delta * finPitch); // Pitch
            double body_torque_z = dynamicPressure * referenceArea * 1.0 * (CM_delta * finYaw);   // Yaw

            // Add damping torque (-k * W)
            double damping_k = 0.1 * dynamicPressure;
            body_torque_x -= damping_k * wx;
            body_torque_y -= damping_k * wy;
            body_torque_z -= damping_k * wz;

            // Rotate torque to world frame: q * t * q_inv
            double ix = qw * body_torque_x + qy * body_torque_z - qz * body_torque_y;
            double iy = qw * body_torque_y + qz * body_torque_x - qx * body_torque_z;
            double iz = qw * body_torque_z + qx * body_torque_y - qy * body_torque_x;
            double iw = -qx * body_torque_x - qy * body_torque_y - qz * body_torque_z;

            double world_torque_x = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy);
            double world_torque_y = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz);
            double world_torque_z = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx);

            return {force_x, force_y, force_z, world_torque_x, world_torque_y, world_torque_z};
        }

    private:
        double cd;
        double cl;
    };

} // namespace StrikeEngine::Models
