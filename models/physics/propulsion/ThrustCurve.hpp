#pragma once
#include <vector>
#include <algorithm>

namespace StrikeEngine::Models {

    struct ThrustDataPoint {
        double time_s;
        double thrust_n;
    };

    class ThrustCurve {
    public:
        ThrustCurve() = default;
        explicit ThrustCurve(const std::vector<ThrustDataPoint>& points) : curve(points) {}

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
