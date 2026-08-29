#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

namespace StrikeEngine::Models {

    struct ThrustDataPoint {
        double time_s;
        double thrust_n;
    };

    class ThrustCurve {
    public:
        ThrustCurve() = default;
        explicit ThrustCurve(const std::vector<ThrustDataPoint>& points) : curve(points) {}

        bool validate(std::string* error = nullptr) const {
            if (curve.empty()) return true;
            if (curve.size() < 2) {
                if (error) *error = "a non-empty thrust curve requires at least two points";
                return false;
            }
            if (!std::isfinite(curve.front().time_s) || curve.front().time_s != 0.0) {
                if (error) *error = "the first thrust-curve time must be finite and exactly 0 s";
                return false;
            }

            bool hasPositiveThrust = false;
            for (std::size_t i = 0; i < curve.size(); ++i) {
                const auto& point = curve[i];
                if (!std::isfinite(point.time_s) || !std::isfinite(point.thrust_n) ||
                    point.time_s < 0.0 || point.thrust_n < 0.0) {
                    if (error) *error = "thrust-curve times must be finite and non-negative, and thrust must be finite and non-negative";
                    return false;
                }
                if (point.thrust_n > 0.0) hasPositiveThrust = true;
                if (i > 0 && point.time_s <= curve[i - 1].time_s) {
                    if (error) *error = "thrust-curve times must be strictly increasing";
                    return false;
                }
            }
            if (!hasPositiveThrust) {
                if (error) *error = "a non-empty thrust curve must contain positive thrust";
                return false;
            }
            return true;
        }

        double lastPositiveTime() const {
            double result = 0.0;
            for (const auto& point : curve) {
                if (point.thrust_n > 0.0) result = point.time_s;
            }
            return result;
        }

        double evaluate(double currentTime) const {
            if (curve.empty()) return 0.0;
            if (currentTime <= curve.front().time_s) return curve.front().thrust_n;
            if (currentTime >= curve.back().time_s) return curve.back().thrust_n;

            auto it = std::lower_bound(curve.begin(), curve.end(), currentTime,
                [](const ThrustDataPoint& p, double time) { return p.time_s < time; });

            const auto& p2 = *it;
            const auto& p1 = *(--it);

            double t1 = p1.time_s, thrust1 = p1.thrust_n;
            double t2 = p2.time_s, thrust2 = p2.thrust_n;
            double fraction = (currentTime - t1) / (t2 - t1);
            return thrust1 + fraction * (thrust2 - thrust1);
        }

    private:
        std::vector<ThrustDataPoint> curve;
    };

} // namespace StrikeEngine::Models
