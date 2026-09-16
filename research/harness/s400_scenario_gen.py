#!/usr/bin/env python3
"""Generate data/scenarios/s400_awacs_314km.scenario.json for StrikeSim.

The scenario models the claimed 8-10 May 2025 engagement in which an Indian
S-400 battery fired a 40N6 at a Pakistan Air Force Saab 2000 Erieye AEW&C,
reported as a 314 km intercept after the aircraft turned away.

Geometry is computed from WGS84: the battery sits at Adampur AFS (Punjab),
the Erieye orbits at FL200 heading away from the border, and the missile is a
ground launch that lofts to ~28 km for the long ballistic leg.

Vehicle configs are built from the Astra scenario's entities as key-complete
templates (the scenario parser requires every field), then overridden with
the S-400 numbers.

Run from the StrikeSim repo root:
    python3 <this file> > data/scenarios/s400_awacs_314km.scenario.json
"""
import copy
import json
import math
import pathlib
import sys

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


# --- WGS84 helpers -----------------------------------------------------------
def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = A / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - E2) + alt_m) * math.sin(lat))


def enu_basis(lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    up = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
    east = (-math.sin(lon), math.cos(lon), 0.0)
    north = (-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat))
    return east, north, up


def dest_point(lat_deg, lon_deg, bearing_deg, dist_m):
    r = 6371000.0
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    brg = math.radians(bearing_deg)
    d = dist_m / r
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(brg))
    lon2 = lon1 + math.atan2(math.sin(brg) * math.sin(d) * math.cos(lat1),
                             math.cos(d) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def quat_from_axes(x_axis, y_axis, z_axis):
    """Body->world quaternion from the body axis directions (X fwd, Y right,
    Z down). The extraction below expects the rotation matrix whose COLUMNS
    are the axes (M * (1,0,0) = x_axis); validate_axes() re-derives the body X
    from the quaternion and aborts if it does not match."""
    m = [[x_axis[0], y_axis[0], z_axis[0]],
         [x_axis[1], y_axis[1], z_axis[1]],
         [x_axis[2], y_axis[2], z_axis[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)


def heading_axes(lat_deg, lon_deg, bearing_deg):
    east, north, up = enu_basis(lat_deg, lon_deg)
    b = math.radians(bearing_deg)
    hx = math.sin(b) * east[0] + math.cos(b) * north[0]
    hy = math.sin(b) * east[1] + math.cos(b) * north[1]
    hz = math.sin(b) * east[2] + math.cos(b) * north[2]
    r = math.radians(bearing_deg + 90.0)
    rx = math.sin(r) * east[0] + math.cos(r) * north[0]
    ry = math.sin(r) * east[1] + math.cos(r) * north[1]
    rz = math.sin(r) * east[2] + math.cos(r) * north[2]
    return (hx, hy, hz), (rx, ry, rz), (-up[0], -up[1], -up[2])


def vertical_axes(lat_deg, lon_deg):
    east, north, up = enu_basis(lat_deg, lon_deg)
    return up, north, (-east[0], -east[1], -east[2])


def pitched_axes(lat_deg, lon_deg, bearing_deg, elev_deg):
    """Body axes for a launch attitude pitched up by elev_deg toward the
    target bearing (cold-launch pitch-over attitude: X = cos(e)h + sin(e)up,
    Y = horizontal right, Z = X x Y = down-ish)."""
    east, north, up = enu_basis(lat_deg, lon_deg)
    b = math.radians(bearing_deg)
    h = (math.sin(b) * east[0] + math.cos(b) * north[0],
         math.sin(b) * east[1] + math.cos(b) * north[1],
         math.sin(b) * east[2] + math.cos(b) * north[2])
    ce = math.cos(math.radians(elev_deg))
    se = math.sin(math.radians(elev_deg))
    x_axis = (ce * h[0] + se * up[0], ce * h[1] + se * up[1], ce * h[2] + se * up[2])
    # right = h x up (horizontal, roll-free)
    r = (h[1] * up[2] - h[2] * up[1], h[2] * up[0] - h[0] * up[2], h[0] * up[1] - h[1] * up[0])
    rn = math.sqrt(sum(c * c for c in r))
    y_axis = (r[0] / rn, r[1] / rn, r[2] / rn)
    z_axis = (x_axis[1] * y_axis[2] - x_axis[2] * y_axis[1],
              x_axis[2] * y_axis[0] - x_axis[0] * y_axis[2],
              x_axis[0] * y_axis[1] - x_axis[1] * y_axis[0])
    return x_axis, y_axis, z_axis


def validate_axes(name, quat, x_axis, tol=1e-6):
    """Rotate (1,0,0) by the generated quaternion and require it to equal the
    intended body X. Cheap guard against silently mis-authoring attitudes:
    a flipped sign points the seeker 180 degrees away from its target."""
    w, x, y, z = quat
    col0 = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
    dot = sum(a * b for a, b in zip(col0, x_axis))
    if dot < 1.0 - tol:
        raise SystemExit("attitude validation failed for %s: bodyX.axis=%.6f" % (name, dot))


# --- Geometry ----------------------------------------------------------------
SITE_LAT, SITE_LON, SITE_ALT = 31.4333, 75.7500, 240.0   # Adampur AFS, Punjab
AWACS_BEARING = 300.0
AWACS_START_KM = 265.0
AWACS_ALT_M = 6100.0
AWACS_SPEED = 175.0
# Station track: a crossing leg perpendicular to the engagement line, so the
# Erieye neither runs away (which puts the record range out of reach) nor
# closes. The engine's launch-warning reaction (LaunchSpec.targetEvadeDistanceM)
# models the turn-away case when a scenario wants it; this one flies its orbit.
AWACS_TRACK_DEG = AWACS_BEARING + 90.0
MISSILE_FLIGHT_S = 290.0

awacs_lat, awacs_lon = dest_point(SITE_LAT, SITE_LON, AWACS_BEARING, AWACS_START_KM * 1000.0)
intercept_lat, intercept_lon = dest_point(awacs_lat, awacs_lon, AWACS_BEARING,
                                          AWACS_SPEED * MISSILE_FLIGHT_S)
site_ecef = geodetic_to_ecef(SITE_LAT, SITE_LON, SITE_ALT)
awacs_ecef = geodetic_to_ecef(awacs_lat, awacs_lon, AWACS_ALT_M)
missile_ecef = geodetic_to_ecef(SITE_LAT, SITE_LON, SITE_ALT + 2.0)
SITE_TO_TARGET_BEARING = AWACS_BEARING  # the battery looks along the track (300 deg)
LAUNCH_EJECT_DEG = 90.0  # vertical clearance out of the canister
LAUNCH_ELEV_DEG = 35.0   # loft axis the thrusters pitch to before ignition
LAUNCH_PUSH_MPS = 30.0   # canister ejection velocity (20-50 m/s per public sources)
m_axes = pitched_axes(SITE_LAT, SITE_LON, AWACS_BEARING, LAUNCH_EJECT_DEG)
s_axes = heading_axes(SITE_LAT, SITE_LON, SITE_TO_TARGET_BEARING)
sq = quat_from_axes(*s_axes)
a_axes = heading_axes(awacs_lat, awacs_lon, AWACS_TRACK_DEG)
mq = quat_from_axes(*m_axes)
aq = quat_from_axes(*a_axes)
validate_axes("site", sq, s_axes[0])
validate_axes("awacs", aq, a_axes[0])
validate_axes("missile", mq, m_axes[0])
ref_lat = math.radians(SITE_LAT)
ref_lon = math.radians(SITE_LON)

TEMPLATE_PATH = pathlib.Path("data/scenarios/cooperative_bvr_80km_astra.scenario.json")


def templates():
    """Key-complete vehicle configs cloned from the Astra scenario."""
    scenario = json.loads(TEMPLATE_PATH.read_text())
    entities = {e["vehicle_config"]["type"]: e for e in scenario["entities"]}
    missile = copy.deepcopy(entities["missile"]["vehicle_config"])
    aircraft = copy.deepcopy(entities["aircraft"]["vehicle_config"])
    return missile, aircraft


def launch_spec():
    """Ground launch gated by the battery's own track: fire when the radar
    holds the AWACS inside the gate for lockHoldSec."""
    return {
        "enabled": True,
        "parentIndex": 0,
        "targetIndex": 1,
        "dropM": 0.0,
        "pushMps": 0.0,
        "lockHoldSec": 4.0,
        "rangeGateM": 280000.0,
        "flyoutSec": 0.0,
        "flyoutAheadM": 0.0,
        "flyoutClimbM": 0.0,
        # Cold launch: vertical clearance out of the canister, then the gas
        # thrusters pitch the round to the loft axis before the motor lights
        # (the kernel applies that reorientation at ignition_delay_sec).
        "launchElevationDeg": LAUNCH_ELEV_DEG,
        "ejectElevationDeg": LAUNCH_EJECT_DEG,
        "pushMps": LAUNCH_PUSH_MPS,
        # No launch-warning reaction in this scenario: the Erieye flies its
        # station track. (Set > 0 to make the target turn away and run.)
        "targetEvadeDistanceM": 0.0,
    }


def inert_warhead():
    return {
        "mass_kg": 1.0, "fusing": "impact", "fuse_enabled": False,
        "fuse_detection_probability": 1.0, "damage": 0.0,
        "lethal_radius_m": 0.0, "proximity_trigger_m": 0.0,
        "falloff_radius_m": 0.0, "arming_delay_sec": 0.0,
        "min_closing_speed_mps": 0.0, "cpa_fuzing_enabled": False,
        "fuse_lookahead_sec": 0.0, "timed_delay_sec": 0.0,
        "self_destruct_time_sec": 0.0, "head_on_lethality_factor": 1.0,
        "tail_on_lethality_factor": 1.0,
    }


def sensor_cfg(template, gps_pos_sigma, imu_noise):
    cfg = dict(template)
    cfg.update({
        "accel_noise_std_dev": imu_noise, "accel_bias_std_dev": imu_noise * 0.5,
        "gyro_noise_std_dev": imu_noise * 0.1, "gyro_bias_std_dev": imu_noise * 0.05,
        "gps_pos_noise_std_dev": gps_pos_sigma, "gps_vel_noise_std_dev": gps_pos_sigma * 0.05,
        "gps_update_rate_hz": 10.0, "imu_enabled": True, "gps_enabled": True,
        "initial_position_error_m": 0.0, "initial_velocity_error_mps": 0.0,
        "initial_attitude_error_deg": 0.0,
    })
    return cfg


def site_entity(template):
    cfg = dict(template)
    cfg.update({
        "type": "radar_site",
        "initial_mass": 50000.0, "mass_dry": 50000.0, "structural_hardness": 200.0,
        "inertia_xx": 500000.0, "inertia_yy": 500000.0, "inertia_zz": 500000.0,
        "inertia_xy": 0.0, "inertia_xz": 0.0, "inertia_yz": 0.0,
        "rcs_profile_id": "", "ir_profile_id": "", "emitter_eirp_w": 0.0, "jammer_eirp_w": 0.0,
        "aero": {"reference_area": 0.0, "reference_length": 10.0, "cd": 0.0,
                 "cl_alpha": 0.0, "cl_fin": 0.0, "cl_max": 0.0},
        "propulsion": {"stages": []},
        "warhead": inert_warhead(),
        "seeker": {
            "type": "rf",
            "transmitter_power_w": 120000.0,
            "antenna_gain_db": 45.0,
            "wavelength_m": 0.09,
            "noise_floor_w": 1.0e-14,
            "snr_threshold_db": 9.0,
            "field_of_view_half_angle_rad": 1.0471976,
            "gimbal_azimuth_limit_rad": 1.4835299,
            "gimbal_elevation_limit_rad": 1.4835299,
            "gimbal_rate_limit_rad_per_sec": 3.0,
            "min_range_gate_m": 3000.0,
            "max_range_gate_m": 400000.0,
            "terrain_masking_enabled": True,
            "min_closing_rate_mps": 0.0,
            "lock_hysteresis_db": 3.0,
            "lock_dropout_time_sec": 2.0,
            "measurement_latency_sec": 0.0,
            "measurement_noise_enabled": False,
            "swerling_enabled": False,
            "sensitivity_w": 0.0,
            "ir_extinction_per_m": 0.0,
            "wavelength_band": 0,
            "decoy_rejection_db": 0.0,
            "rate_filter_tau_sec": 0.05,
            "passive_rf_duty_cycle": 1.0,
            "glint_sigma_m": 0.0,
            "glint_correlation_tau_sec": 0.0,
            "range_noise_std_dev_m": 0.0,
            "range_rate_noise_std_dev_mps": 0.0,
            "angle_noise_std_dev_rad": 0.0,
            "angle_noise_ref_snr_db": 20.0,
            "illuminator_entity_id": -1,
            "illuminator_power_w": 0.0, "illuminator_gain_db": 0.0,
            "illuminator_wavelength_m": 0.0,
            "illuminator_px": 0.0, "illuminator_py": 0.0, "illuminator_pz": 0.0,
        },
        "guidance_autopilot": dict(template["guidance_autopilot"], **{
            "trackConfirmations": 2,
            "trackCoastTimeoutSec": 4.0,
            "trackLossTimeoutSec": 8.0,
            "trackFilterEnabled": True,
            # Tuned for a long-range air picture: the target is a transport,
            # not a fighter. The template's 30 m/s^2 accel bound is a FIGHTER
            # limit; left at 30 the battery's track accel state railed at the
            # clamp and the velocity integrated it (the guidance's aim read
            # 2.5 km/s -- a phantom lead of hundreds of km that threw the whole
            # midcourse off).
            "trackProcessNoiseMps2": 0.5,
            "trackAngleStdRad": 0.0025,
            "trackMeasNoiseScale": 1.0,
            "trackResidualGateSigma": 4.0,
            "trackMaxAccelMps2": 3.0,
            "trackRetargetConfirmations": 3,
            "trackSeedPolicy": 2,
            "trackMinQuality01": 0.15,
            "trackQualityTauSec": 3.0,
            "trackVelocityBlend": 0.25,
            "datalinkSourceId": -1,
            "datalinkTargetId": -1,
        }),
    })
    cfg["sensor"] = sensor_cfg(template["sensor"], 3.0, 0.002)
    return {
        "name": "S-400 92N6E Battery (Adampur)",
        "role": "radar_site",
        "init_state": {
            "allegiance": "friendly",
            "mass": 50000.0,
            "px": site_ecef[0], "py": site_ecef[1], "pz": site_ecef[2],
            "qw": sq[0], "qx": sq[1], "qy": sq[2], "qz": sq[3],
        },
        "initial_guidance_mode": "none",
        "initial_max_accel": 0.0,
        "initial_target_x": 0.0, "initial_target_y": 0.0, "initial_target_z": 0.0,
        "initial_target_vx": 0.0, "initial_target_vy": 0.0, "initial_target_vz": 0.0,
        "vehicle_config": cfg,
    }


# AWACS escape point: the reciprocal bearing from the battery, far enough that
# the run never ends inside the engagement. The launch spec's reaction and the
# initial cruise course both point at it.
def evade_point(site_ecef, awacs_ecef, distance_m):
    dx = awacs_ecef[0] - site_ecef[0]
    dy = awacs_ecef[1] - site_ecef[1]
    dz = awacs_ecef[2] - site_ecef[2]
    n = (dx * dx + dy * dy + dz * dz) ** 0.5
    return (awacs_ecef[0] + dx / n * distance_m,
            awacs_ecef[1] + dy / n * distance_m,
            awacs_ecef[2] + dz / n * distance_m)


def awacs_entity(template):
    ahead_ecef = tuple(awacs_ecef[i] + a_axes[0][i] * 400000.0 for i in range(3))
    cfg = dict(template)
    cfg.update({
        "type": "aircraft",
        "initial_mass": 19000.0, "mass_dry": 14500.0, "structural_hardness": 60.0,
        "inertia_xx": 30000.0, "inertia_yy": 260000.0, "inertia_zz": 280000.0,
        "inertia_xy": 0.0, "inertia_xz": 0.0, "inertia_yz": 0.0,
        "rcs_profile_id": "data/rcs/paf_erieye_rcs.json",
        "ir_profile_id": "", "emitter_eirp_w": 0.0, "jammer_eirp_w": 0.0,
        "aero": dict(template["aero"], **{
            "airframe": dict(template["aero"]["airframe"], **{
                "cd0": 0.028, "cl_max": 1.45,
                "fuselage_length_m": 27.0, "fuselage_diameter_m": 2.8,
                "wing_span_m": 25.4, "wing_root_chord_m": 4.6, "wing_tip_chord_m": 1.6,
                "wing_sweep_deg": 0.0, "wing_position_m": 1.0, "wing_incidence_deg": 2.0,
                "htail_span_m": 8.0, "htail_root_chord_m": 2.4, "htail_tip_chord_m": 1.0,
                "htail_position_m": 12.0, "htail_incidence_deg": -1.0,
                "vtail_height_m": 4.0, "vtail_root_chord_m": 3.4, "vtail_tip_chord_m": 1.3,
                "vtail_position_m": 12.0, "vtail_cant_deg": 0.0,
            }),
            "reference_area": 55.0, "reference_length": 27.0,
            "cd": 0.020, "cl_alpha": 4.2, "cl_fin": 0.0, "cl_max": 1.45,
        }),
        "propulsion": {
            "stages": [{
                "propellant_mass_kg": 3500.0, "dry_mass_kg": 0.0,
                "vacuum_isp": 3000.0, "sea_level_isp": 3000.0,
                "ignition_delay_sec": 0.0, "ignition_ramp_sec": 1.0,
                "shutdown_ramp_sec": 0.0, "shutdown_time_sec": -1.0,
                "max_gimbal_pitch_rad": 0.0, "max_gimbal_yaw_rad": 0.0,
                "max_gimbal_rate_rad_per_sec": 0.0, "gimbal_time_constant_sec": 0.02,
                "engine_position_x": 0.0, "engine_position_y": 0.0, "engine_position_z": 0.0,
                "thrust_curve": [{"time_s": 0.0, "thrust_n": 35000.0},
                                 {"time_s": 7200.0, "thrust_n": 35000.0}],
            }],
        },
        "seeker": dict(template["seeker"], **{"type": "none", "transmitter_power_w": 0.0,
                                              "antenna_gain_db": 0.0,
                                              "max_range_gate_m": 0.0,
                                              "min_range_gate_m": 0.0,
                                              "terrain_masking_enabled": False,
                                              "measurement_noise_enabled": False}),
        "warhead": inert_warhead(),
        "guidance_autopilot": dict(template["guidance_autopilot"], **{
            # Keep the working aircraft gains from the template; the only
            # change is the orbit altitude (a heavy turboprop on invented
            # gains falls out of the sky).
            # Station flight: straight and level, no maneuvers. The Erieye
            # holds its track with zero lateral steering demand; the roll
            # channel is off (its P loop is unstable for this airframe in the
            # engine today -- see the ponytail note) and the pitch integral
            # carries the heavy airframe's trim (the small clamp left a steady
            # sink). ponytail: engine follow-up = a stable roll/level mode with
            # a proper gain margin; the reproducer is in the handoff report.
            "cruiseWaypointGain": 0.0,
            "cruiseAltitudeGain": 0.3,
            "cruiseAltitudeDamping": 0.7,
            "kRollP": 0.0, "kRollD": 0.0,
            "autopilotIntegralEnabled": True,
            "integralClampRad": 0.25,
            "kAlphaP": 0.02,
            "cruiseAltitudeM": AWACS_ALT_M,
            "datalinkSourceId": -1, "datalinkTargetId": -1,
        }),
    })
    cfg["sensor"] = sensor_cfg(template["sensor"], 2.0, 0.003)
    return {
        "name": "PAF Saab 2000 Erieye",
        "role": "target",
        "init_state": {
            "allegiance": "hostile",
            "mass": 19000.0,
            "px": awacs_ecef[0], "py": awacs_ecef[1], "pz": awacs_ecef[2],
            "qw": aq[0], "qx": aq[1], "qy": aq[2], "qz": aq[3],
            "vx": AWACS_SPEED * a_axes[0][0], "vy": AWACS_SPEED * a_axes[0][1],
            "vz": AWACS_SPEED * a_axes[0][2],
        },
        "initial_guidance_mode": "cruise",
        "initial_max_accel": 12.0,
        # Cruise aim: straight ahead on the station track (no lateral demand).
        "initial_target_x": ahead_ecef[0], "initial_target_y": ahead_ecef[1],
        "initial_target_z": ahead_ecef[2],
        "initial_target_vx": AWACS_SPEED * a_axes[0][0], "initial_target_vy": AWACS_SPEED * a_axes[0][1],
        "initial_target_vz": AWACS_SPEED * a_axes[0][2],
        "vehicle_config": cfg,
    }


def missile_aero(template_aero):
    area = math.pi * (0.515 / 2.0) ** 2
    machs = [0.5, 0.8, 1.0, 1.2, 1.6, 2.0, 2.6, 3.2, 4.0, 5.0, 6.0]
    cd0 = [0.22, 0.27, 0.34, 0.33, 0.26, 0.22, 0.19, 0.175, 0.165, 0.155, 0.15]
    aoa = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25]
    aoa_rise = [0.0, 0.010, 0.035, 0.080, 0.150, 0.240]
    cd_table = [[round(c + rise, 5) for rise in aoa_rise] for c in cd0]  # [mach][aoa]
    cl_alpha, cl_max = 2.6, 3.2
    cl_table = [[round(min(cl_alpha * a, cl_max), 4) for a in aoa] for _ in machs]
    return dict(template_aero, **{
        "aero_tables": {"mach_breakpoints": machs, "aoa_breakpoints_rad": aoa,
                        "cd_table": cd_table, "cl_table": cl_table},
        "reference_area": round(area, 6), "reference_length": 7.5,
        "cd": 0.2, "cl_alpha": cl_alpha, "cl_fin": 1.2, "cl_max": cl_max,
        "tail_control": True,
        "fins": dict(template_aero.get("fins", {}), **{
            "count": 4, "steerable": True, "shape": "trapezoidal",
            "position_m": -6.5, "span_m": 0.42,
            "root_chord_m": 0.75, "tip_chord_m": 0.30,
            "sweep_length_m": -1.0, "cant_angle_deg": 0.0}),
    })


def missile_propulsion():
    return {"stages": [
        # Boost: 5 s of hard thrust to punch out of the dense air.
        {"propellant_mass_kg": 560.0, "dry_mass_kg": 0.0,
         "vacuum_isp": 265.0, "sea_level_isp": 250.0,
         "ignition_delay_sec": 0.8, "ignition_ramp_sec": 0.15,  # clear of the canister
         "shutdown_ramp_sec": 0.1, "shutdown_time_sec": -1.0,
         "max_gimbal_pitch_rad": 0.0, "max_gimbal_yaw_rad": 0.0,
         "max_gimbal_rate_rad_per_sec": 0.0, "gimbal_time_constant_sec": 0.02,
         "engine_position_x": 0.0, "engine_position_y": 0.0, "engine_position_z": 0.0,
         "thrust_curve": [{"time_s": 0.0, "thrust_n": 280000.0},
                          {"time_s": 4.9, "thrust_n": 280000.0},
                          {"time_s": 5.0, "thrust_n": 0.0}]},
        # Sustain: a long low-thrust burn (real 40N6-class rounds cruise for
        # minutes). This is what buys the record-range leg: the round holds
        # ~2 km/s in the thin-air loft instead of peaking and coasting.
        {"propellant_mass_kg": 640.0, "dry_mass_kg": 0.0,
         "vacuum_isp": 265.0, "sea_level_isp": 250.0,
         "ignition_delay_sec": 5.9, "ignition_ramp_sec": 0.2,
         "shutdown_ramp_sec": 0.2, "shutdown_time_sec": -1.0,
         "max_gimbal_pitch_rad": 0.0, "max_gimbal_yaw_rad": 0.0,
         "max_gimbal_rate_rad_per_sec": 0.0, "gimbal_time_constant_sec": 0.02,
         "engine_position_x": 0.0, "engine_position_y": 0.0, "engine_position_z": 0.0,
         "thrust_curve": [{"time_s": 0.0, "thrust_n": 15000.0},
                          {"time_s": 109.9, "thrust_n": 15000.0},
                          {"time_s": 110.0, "thrust_n": 0.0}]},
    ]}


def missile_entity(template):
    cfg = dict(template)
    cfg.update({
        "type": "missile",
        "initial_mass": 1893.0, "mass_dry": 743.0, "structural_hardness": 120.0,
        "inertia_xx": 65.0, "inertia_yy": 8900.0, "inertia_zz": 8900.0,
        "inertia_xy": 0.0, "inertia_xz": 0.0, "inertia_yz": 0.0,
        "ir_profile_id": "", "emitter_eirp_w": 0.0, "jammer_eirp_w": 0.0,
        "rcs_profile_id": "data/rcs/target_missile_rcs.json",
        "aero": missile_aero(template["aero"]),
        "propulsion": missile_propulsion(),
        "seeker": dict(template["seeker"], **{
            "type": "rf", "transmitter_power_w": 15000.0, "antenna_gain_db": 40.0,
            "wavelength_m": 0.10, "noise_floor_w": 1.0e-14, "snr_threshold_db": 12.0,
            "field_of_view_half_angle_rad": 0.5235988,
            "gimbal_azimuth_limit_rad": 1.0471976, "gimbal_elevation_limit_rad": 1.0471976,
            "gimbal_rate_limit_rad_per_sec": 1.2,
            "min_range_gate_m": 0.0, "max_range_gate_m": 300000.0,
            "terrain_masking_enabled": True, "min_closing_rate_mps": 0.0,
            "lock_hysteresis_db": 3.0, "lock_dropout_time_sec": 0.2,
            "measurement_latency_sec": 0.05, "measurement_noise_enabled": False,
            "swerling_enabled": False,
        }),
        "warhead": {
            "mass_kg": 315.0, "fusing": "proximity", "fuse_enabled": True,
            "fuse_detection_probability": 1.0, "damage": 100.0,
            "lethal_radius_m": 60.0, "proximity_trigger_m": 60.0,
            "falloff_radius_m": 150.0, "arming_delay_sec": 2.0,
            "min_closing_speed_mps": 0.0, "cpa_fuzing_enabled": True,
            "fuse_lookahead_sec": 0.05, "timed_delay_sec": 0.0,
            "self_destruct_time_sec": 0.0, "head_on_lethality_factor": 1.0,
            "tail_on_lethality_factor": 1.0,
        },
        "guidance_autopilot": dict(template["guidance_autopilot"], **{
            "navigationConstant": 3.0, "navConstantTerminal": 3.8,
            "navScheduleEnabled": True, "navScheduleTgoSec": 12.0,
            "guidanceLoftEnabled": True, "guidanceLoftAngleDeg": 20.0,
            "guidanceLoftAltitudeM": 40000.0,
            "guidanceLoftRangeM": 300000.0, "guidanceLoftGain": 0.8,
            "handoffBlendTimeSec": 3.0,
            "apnFeedforwardEnabled": True, "guidanceApnFeedforwardMinQuality01": 0.2,
            "kAccelP": 0.0032, "kAlphaP": 0.06,
            "kRateP": 0.6, "kRatePitchP": 0.6, "kRateYawP": 0.6,
            "kRollP": 1.2, "kRollD": 0.15,
            "maxDeflectionRad": 0.45, "maxServoRateRadPerSec": 6.0,
            "servoTimeConstantSec": 0.02,
            "autopilotIntegralEnabled": False,
            "controlEffectivenessEnabled": False,
            "guidanceAuthorityAwareLimitEnabled": True,
            "guidanceScaleDemandOnInfeasible": True,
            "gravityCompensationEnabled": True,
            "guidanceRangeGainShapingEnabled": False,
            "useTruthGravityModel": False,
            "guidanceTrackAimMinQuality01": 0.1,
            "trackConfirmations": 3, "trackCoastTimeoutSec": 1.0, "trackLossTimeoutSec": 4.0,
            "trackMinQuality01": 0.0, "trackSeedPolicy": 2, "trackQualityTauSec": 1.5,
            "lockLossRetentionSec": 3.0,
            "datalinkSourceId": 0, "datalinkTargetId": 1,
        }),
    })
    cfg["sensor"] = sensor_cfg(template["sensor"], 3.0, 0.004)
    return {
        "name": "S-400 40N6",
        "role": "interceptor",
        "init_state": {
            "allegiance": "friendly",
            "mass": 1893.0,
            "px": missile_ecef[0], "py": missile_ecef[1], "pz": missile_ecef[2],
            "qw": mq[0], "qx": mq[1], "qy": mq[2], "qz": mq[3],
            "vx": 40.0 * m_axes[0][0], "vy": 40.0 * m_axes[0][1], "vz": 40.0 * m_axes[0][2],
        },
        "initial_guidance_mode": "proportional_navigation",
        "initial_max_accel": 300.0,
        "initial_target_id": 1,
        "initial_target_x": awacs_ecef[0], "initial_target_y": awacs_ecef[1],
        "initial_target_z": awacs_ecef[2],
        "initial_target_vx": AWACS_SPEED * a_axes[0][0], "initial_target_vy": AWACS_SPEED * a_axes[0][1],
        "initial_target_vz": AWACS_SPEED * a_axes[0][2],
        "launch": launch_spec(),
        "vehicle_config": cfg,
    }


def main():
    missile_t, aircraft_t = templates()
    scenario = {
        "name": "S-400 40N6 vs PAF Erieye AEW&C (Adampur, 314 km)",
        "description": (
            "Claimed Operation Sindoor engagement (8-10 May 2025): an S-400 "
            "battery at Adampur AFS fires a 40N6 at a PAF Saab 2000 Erieye "
            "AEW&C orbiting at FL200 inside Pakistani Punjab. The Erieye is "
            "engaged at 265 km on its station track; the missile cold-launches "
            "vertically, pitches to the loft axis, flies a ~40 km ballistic arc "
            "on the battery's datalink track, and "
            "intercepts at roughly 314 km - the longest SAM kill on record. "
            "Reporting is contested: this is a simulation of the public "
            "account with open-source parameters, not a verified record. "
            "WGS84 ECEF, real geography (Adampur 31.43N 75.75E)."),
        "primary_entity_index": 2,
        "random_seed": 20250510,
        "environment": {
            "earth": {
                "use_ecef_truth": True, "use_spherical_gravity": False,
                "use_wgs84_gravity": True, "include_j2_gravity": False,
                "include_coriolis": False, "include_centrifugal": False,
                "include_earth_rate_gyro": False, "include_transport_rate": False,
                "reference_latitude_rad": ref_lat, "reference_longitude_rad": ref_lon,
            },
            "wind": {"enabled": False},
        },
        "entities": [site_entity(missile_t), awacs_entity(aircraft_t), missile_entity(missile_t)],
    }
    json.dump(scenario, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
