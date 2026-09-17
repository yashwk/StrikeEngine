#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <algorithm>

namespace StrikeEngine::Models {

    constexpr double kFinPi = 3.14159265358979323846;

    enum class FinShape : uint8_t {
        Trapezoidal,
        Elliptical,
        Delta,
        Cranked,
        FreeForm
    };

    enum class FinAirfoil : uint8_t {
        FlatPlate,
        DoubleWedge,
        Biconvex,
        Hexagonal
    };

    struct PlanformPolygon {
        std::vector<std::array<double, 2>> points;
    };

    struct PlanformMetrics {
        double area = 0.0;
        double span = 0.0;
        double mac = 0.0;
        double macLead = 0.0;
        double cpz = 0.0;
        double Yma = 0.0;
    };

    inline double polygonSpan(const PlanformPolygon& p) {
        if (p.points.empty()) return 0.0;
        double lo = p.points.front()[1], hi = lo;
        for (const auto& pt : p.points) {
            lo = std::min(lo, pt[1]);
            hi = std::max(hi, pt[1]);
        }
        return hi - lo;
    }

    inline double polygonMaxChord(const PlanformPolygon& p) {
        if (p.points.empty()) return 0.0;
        double lo = p.points.front()[0], hi = lo;
        for (const auto& pt : p.points) {
            lo = std::min(lo, pt[0]);
            hi = std::max(hi, pt[0]);
        }
        return hi - lo;
    }

    struct PlanformBands {
        std::vector<double> y;
        std::vector<double> lead;
        std::vector<double> trail;
        bool valid = false;
    };

    inline PlanformBands planformBands(const PlanformPolygon& p)
    {
        PlanformBands b;
        const std::size_t n = p.points.size();
        if (n < 3) return b;

        std::vector<double> ys;
        ys.reserve(n);
        for (const auto& pt : p.points) ys.push_back(pt[1]);
        std::sort(ys.begin(), ys.end());
        ys.erase(std::unique(ys.begin(), ys.end(),
                             [](double a, double c) { return std::abs(a - c) < 1e-12; }),
                 ys.end());
        if (ys.size() < 2) return b;

        auto leadTrailAt = [&](double y, double& lead, double& trail) {
            double lo = 1e300, hi = -1e300;
            bool found = false;
            for (std::size_t i = 0; i < n; ++i) {
                const auto& a = p.points[i];
                const auto& c = p.points[(i + 1) % n];
                const double y0 = a[1], y1 = c[1];
                if (std::abs(y1 - y0) < 1e-15) continue;
                const double t = (y - y0) / (y1 - y0);
                if (t < -1e-12 || t > 1.0 + 1e-12) continue;
                const double x = a[0] + t * (c[0] - a[0]);
                lo = std::min(lo, x);
                hi = std::max(hi, x);
                found = true;
            }
            if (!found) return false;
            lead = lo;
            trail = hi;
            return true;
        };

        for (double y : ys) {
            double lead = 0.0, trail = 0.0;
            if (!leadTrailAt(y, lead, trail)) return b;
            b.y.push_back(y);
            b.lead.push_back(lead);
            b.trail.push_back(trail);
        }
        b.valid = b.y.size() >= 2;
        return b;
    }

    inline PlanformMetrics computePlanformMetrics(const PlanformPolygon& p)
    {
        PlanformMetrics m;
        m.span = polygonSpan(p);

        const PlanformBands b = planformBands(p);
        if (!b.valid) return m;

        double intC = 0.0, intC2 = 0.0, intLeadC = 0.0, intYc = 0.0;
        for (std::size_t i = 0; i + 1 < b.y.size(); ++i) {
            const double h = b.y[i + 1] - b.y[i];
            if (h <= 1e-15) continue;
            const double c0 = b.trail[i] - b.lead[i];
            const double c1 = b.trail[i + 1] - b.lead[i + 1];
            const double dC = c1 - c0;
            const double L0 = b.lead[i];
            const double dL = b.lead[i + 1] - b.lead[i];
            intC += h * 0.5 * (c0 + c1);
            intC2 += h * (c0 * c0 + c0 * c1 + c1 * c1) / 3.0;
            intLeadC += h * (L0 * c0 + (L0 * dC + dL * c0) * 0.5
                             + dL * dC / 3.0);
            intYc += h * (b.y[i] * c0 + (b.y[i] * dC + h * c0) * 0.5 + h * dC / 3.0);
        }
        if (intC < 1e-15) return m;
        m.area = intC;
        m.mac = intC2 / intC;
        m.macLead = intLeadC / intC;
        m.cpz = m.macLead + 0.25 * m.mac;
        m.Yma = intYc / intC;
        return m;
    }

    inline PlanformPolygon trapezoidPolygon(
        double rootChord, double tipChord, double span, double sweep)
    {
        PlanformPolygon p;
        p.points = {{0.0, 0.0},
                    {rootChord, 0.0},
                    {sweep + tipChord, span},
                    {sweep, span}};
        return p;
    }

    inline PlanformPolygon deltaPolygon(double rootChord, double span, double sweep) {
        PlanformPolygon p;
        p.points = {{0.0, 0.0}, {rootChord, 0.0}, {sweep, span}};
        return p;
    }

    inline PlanformPolygon crankedPolygon(
        double rootChord, double tipChord, double span, double sweep,
        double crankFraction, double crankChordFactor)
    {
        const double t = std::clamp(crankFraction, 0.05, 0.95);
        const double yCrank = t * span;
        const double leadCrank = sweep * t;
        const double chordCrank = rootChord + (tipChord - rootChord) * t;
        PlanformPolygon p;
        p.points = {{0.0, 0.0},
                    {rootChord, 0.0},
                    {sweep + tipChord, span},
                    {sweep, span},
                    {leadCrank + chordCrank * crankChordFactor, yCrank},
                    {leadCrank, yCrank}};
        return p;
    }

    inline PlanformPolygon ellipticalPolygon(
        double rootChord, double tipChord, double span, double sweep,
        int samples = 96)
    {
        PlanformPolygon p;
        const int n = std::max(8, samples);
        p.points.reserve(static_cast<std::size_t>(2 * n + 2));
        auto chordAt = [&](double t) {
            const double e = std::sqrt(std::max(0.0, 1.0 - t * t));
            return tipChord + (rootChord - tipChord) * e;
        };
        for (int i = 0; i <= n; ++i) {
            const double t = static_cast<double>(i) / n;
            p.points.push_back({sweep * t, t * span});
        }
        for (int i = n; i >= 0; --i) {
            const double t = static_cast<double>(i) / n;
            p.points.push_back({sweep * t + chordAt(t), t * span});
        }
        return p;
    }

    inline PlanformPolygon explicitPolygon(
        const std::vector<std::array<double, 2>>& points)
    {
        PlanformPolygon p;
        if (points.empty()) return p;
        double yMin = points.front()[1];
        for (const auto& pt : points) yMin = std::min(yMin, pt[1]);
        p.points.reserve(points.size());
        for (const auto& pt : points) {
            p.points.push_back({pt[0], pt[1] - yMin});
        }
        return p;
    }

    struct FinsGeometry {
        FinShape shape;
        FinAirfoil airfoil = FinAirfoil::FlatPlate;
        double n = 0.0;
        double rootChord = 0.0;
        double tipChord = 0.0;
        double span = 0.0;
        double sweepLength = 0.0;
        double positionM = 0.0;
        double cantRad = 0.0;
        double rocketRadius = 0.0;
        double referenceArea = 0.0;
        double referenceLength = 0.0;

        double thicknessRatio = 0.0;
        double maxThicknessLocation = 0.5;
        double leadingEdgeRadius = 0.0;
        double trailingEdgeThickness = 0.0;
        double midChordLandFraction = 0.33;

        double Af = 0.0;
        double AR = 0.0;
        double gammaC = 0.0;
        double Yma = 0.0;
        double cpz = 0.0;
        double cpLeverArmM = 0.0;
        double liftInterferenceFactor = 0.0;
        double rollGeometricalConstant = 0.0;
        double rollDampingInterferenceFactor = 0.0;
        double rollForcingInterferenceFactor = 0.0;
        double finNumCorrection = 0.0;
        bool steerable = true;

        double clAlphaSingle(double mach) const {
            const double m2 = mach * mach;
            double beta;
            if (mach < 0.8)            beta = std::sqrt(1.0 - m2);
            else if (mach < 1.1)       beta = std::sqrt(1.0 - 0.8 * 0.8);
            else                       beta = std::sqrt(m2 - 1.0);
            const double clalpha2D = 2.0 * kFinPi / beta;
            const double planform = 2.0 * kFinPi * AR / (clalpha2D * std::cos(gammaC));
            const double d = 2.0 / planform;
            return clalpha2D * planform * (Af / referenceArea) * std::cos(gammaC)
                   / (2.0 + planform * std::sqrt(1.0 + d * d));
        }

        double clAlpha(double mach) const {
            return finNumCorrection * liftInterferenceFactor * clAlphaSingle(mach);
        }

        double rollForcingPerRad(double mach) const {
            return rollForcingInterferenceFactor * n * (Yma + rocketRadius)
                   * clAlphaSingle(mach) / referenceLength;
        }

        double rollDampingCoeff(double mach) const {
            return 2.0 * rollDampingInterferenceFactor * n * clAlphaSingle(mach)
                   * std::cos(cantRad) * rollGeometricalConstant
                   / (referenceArea * referenceLength * referenceLength);
        }
    };

    inline double rollForcingInterferenceFactor(double tau) {
        const double t = tau;
        const double t2 = t * t;
        const double asinV = std::asin((t2 - 1.0) / (t2 + 1.0));
        const double c = (t2 + 1.0) * (t2 + 1.0) / (t2 * (t - 1.0) * (t - 1.0));
        return (1.0 / (kFinPi * kFinPi)) * (
              (kFinPi * kFinPi / 4.0) * ((t + 1.0) * (t + 1.0) / t2)
            + kFinPi * c * asinV
            - (2.0 * kFinPi * (t + 1.0)) / (t * (t - 1.0))
            + c * asinV * asinV
            - (4.0 * (t + 1.0)) / (t * (t - 1.0)) * asinV
            + (8.0 / ((t - 1.0) * (t - 1.0))) * std::log((t2 + 1.0) / (2.0 * t))
        );
    }

    inline double finNumCorrection(double n) {
        if (n == 5) return 2.37;
        if (n == 6) return 2.74;
        if (n == 7) return 2.99;
        if (n == 8) return 3.24;
        return n / 2.0;
    }

    inline std::shared_ptr<FinsGeometry> buildFinsGeometry(
        FinShape shape, int count,
        double rootChord, double tipChord, double span,
        double sweepLength, double positionM, double cantAngleDeg,
        const std::vector<std::array<double, 2>>& shapePoints,
        double referenceArea, std::string* error = nullptr,
        bool steerable = true,
        FinAirfoil airfoil = FinAirfoil::FlatPlate,
        double thicknessRatio = 0.0,
        double maxThicknessLocation = 0.5,
        double leadingEdgeRadius = 0.0,
        double trailingEdgeThickness = 0.0,
        double crankFraction = 0.5,
        double crankChordFactor = 0.5,
        double midChordLandFraction = 0.33)
    {
        auto fail = [&](const char* msg) {
            if (error) *error = msg;
            return nullptr;
        };
        if (count < 3) return fail("fin count must be >= 3");
        if (referenceArea <= 0.0) return fail("reference_area must be > 0");
        if (thicknessRatio < 0.0 || thicknessRatio > 0.5) {
            return fail("fin thickness ratio must be in [0, 0.5]");
        }

        const double radius = std::sqrt(referenceArea / kFinPi);
        const double sweep = (sweepLength < 0.0) ? (rootChord - tipChord) : sweepLength;

        PlanformPolygon polygon;
        if (shape == FinShape::Trapezoidal) {
            if (rootChord <= 0.0) return fail("trapezoidal root_chord must be > 0");
            if (tipChord < 0.0)   return fail("trapezoidal tip_chord must be >= 0");
            if (span <= 0.0)      return fail("trapezoidal span must be > 0");
            polygon = trapezoidPolygon(rootChord, tipChord, span, sweep);
        } else if (shape == FinShape::Elliptical) {
            if (rootChord <= 0.0) return fail("elliptical root_chord must be > 0");
            if (span <= 0.0)      return fail("elliptical span must be > 0");
            polygon = ellipticalPolygon(rootChord, tipChord, span, sweep);
        } else if (shape == FinShape::Delta) {
            if (rootChord <= 0.0) return fail("delta root_chord must be > 0");
            if (span <= 0.0)      return fail("delta span must be > 0");
            polygon = deltaPolygon(rootChord, span, sweep);
        } else if (shape == FinShape::Cranked) {
            if (rootChord <= 0.0) return fail("cranked root_chord must be > 0");
            if (span <= 0.0)      return fail("cranked span must be > 0");
            if (crankFraction <= 0.0 || crankFraction >= 1.0) {
                return fail("cranked crank_fraction must be in (0, 1)");
            }
            polygon = crankedPolygon(rootChord, tipChord, span, sweep,
                                     crankFraction, crankChordFactor);
        } else {
            if (shapePoints.size() < 3) return fail("free-form needs >= 3 shape points");
            polygon = explicitPolygon(shapePoints);
        }

        const PlanformMetrics metrics = computePlanformMetrics(polygon);
        if (metrics.area < 1e-9) return fail("fin planform area is too small");

        auto g = std::make_shared<FinsGeometry>();
        g->shape = shape;
        g->airfoil = airfoil;
        g->n = static_cast<double>(count);
        g->rocketRadius = radius;
        g->referenceArea = referenceArea;
        g->referenceLength = 2.0 * radius;
        g->positionM = positionM;
        g->cantRad = cantAngleDeg * kFinPi / 180.0;
        g->steerable = steerable;
        g->thicknessRatio = thicknessRatio;
        g->maxThicknessLocation = std::clamp(maxThicknessLocation, 0.05, 0.95);
        g->leadingEdgeRadius = std::max(0.0, leadingEdgeRadius);
        g->trailingEdgeThickness = std::max(0.0, trailingEdgeThickness);
        g->midChordLandFraction = std::clamp(midChordLandFraction, 0.0, 0.9);
        g->sweepLength = sweep;
        g->cpLeverArmM = 0.0;

        const double tau = (span + radius) / radius;
        g->liftInterferenceFactor = 1.0 + 1.0 / tau;
        g->rollForcingInterferenceFactor = rollForcingInterferenceFactor(tau);
        g->finNumCorrection = finNumCorrection(g->n);

        if (shape == FinShape::Trapezoidal) {
            g->span = span;
            g->rootChord = rootChord;
            g->tipChord = tipChord;
            const double Yr = rootChord + tipChord;
            g->Af = Yr * span / 2.0;
            g->AR = 2.0 * span * span / g->Af;
            g->gammaC = std::atan((sweep + 0.5 * tipChord - 0.5 * rootChord) / span);
            g->Yma = (span / 3.0) * (rootChord + 2.0 * tipChord) / Yr;
            const double lambda = tipChord / rootChord;
            g->rollGeometricalConstant = (
                (rootChord + 3.0 * tipChord) * span * span * span
              + 4.0 * (rootChord + 2.0 * tipChord) * radius * span * span
              + 6.0 * (rootChord + tipChord) * span * radius * radius
            ) / 12.0;
            g->rollDampingInterferenceFactor = 1.0 + (
                ((tau - lambda) / tau) - ((1.0 - lambda) / (tau - 1.0)) * std::log(tau)
            ) / (
                ((tau + 1.0) * (tau - lambda)) / 2.0
              - ((1.0 - lambda) * (tau * tau * tau - 1.0)) / (3.0 * (tau - 1.0))
            );
            g->cpz = (sweep / 3.0) * ((rootChord + 2.0 * tipChord) / (rootChord + tipChord))
                   + (1.0 / 6.0) * (rootChord + tipChord
                                   - rootChord * tipChord / (rootChord + tipChord));
        } else if (shape == FinShape::Elliptical) {
            g->span = span;
            g->rootChord = rootChord;
            g->tipChord = tipChord;
            g->Af = kFinPi * rootChord * span / 4.0;
            g->AR = 2.0 * span * span / g->Af;
            g->gammaC = 0.0;
            g->Yma = span / (3.0 * kFinPi) * std::sqrt(9.0 * kFinPi * kFinPi - 64.0);
            g->rollGeometricalConstant = rootChord * span
                * (3.0 * kFinPi * span * span + 32.0 * radius * span
                   + 12.0 * kFinPi * radius * radius) / 48.0;
            const double rs2 = radius * radius;
            const double s2 = span * span;
            if (span > radius) {
                g->rollDampingInterferenceFactor = 1.0 + rs2 * (
                    2.0 * rs2 * std::sqrt(s2 - rs2) * std::log(
                        (2.0 * span * std::sqrt(s2 - rs2) + 2.0 * s2) / radius)
                    - 2.0 * rs2 * std::sqrt(s2 - rs2) * std::log(2.0 * span)
                    + 2.0 * span * span * span
                    - kFinPi * radius * s2
                    - 2.0 * rs2 * span
                    + kFinPi * radius * rs2
                ) / (2.0 * s2 * (span / 3.0 + kFinPi * radius / 4.0) * (s2 - rs2));
            } else if (span < radius) {
                g->rollDampingInterferenceFactor = 1.0 - rs2 * (
                    2.0 * span * span * span
                    - kFinPi * s2 * radius
                    - 2.0 * span * rs2
                    + kFinPi * radius * rs2
                    + 2.0 * rs2 * std::sqrt(rs2 - s2) * std::atan(span / std::sqrt(rs2 - s2))
                    - kFinPi * rs2 * std::sqrt(rs2 - s2)
                ) / (2.0 * span * (rs2 - s2) * (s2 / 3.0 + kFinPi * span * radius / 4.0));
            } else {
                g->rollDampingInterferenceFactor = (28.0 - 3.0 * kFinPi) / (4.0 + 3.0 * kFinPi);
            }
            g->cpz = 0.288 * rootChord;
        } else {
            double tipLo = 0.0, tipHi = 0.0;
            {
                double yMax = 0.0;
                for (const auto& pt : polygon.points) yMax = std::max(yMax, pt[1]);
                tipLo = 1e30; tipHi = -1e30;
                for (const auto& pt : polygon.points) {
                    if (pt[1] >= yMax - 1e-9) {
                        tipLo = std::min(tipLo, pt[0]);
                        tipHi = std::max(tipHi, pt[0]);
                    }
                }
                if (tipLo > tipHi) { tipLo = 0.0; tipHi = 0.0; }
            }
            g->span = metrics.span;
            g->rootChord = rootChord > 0.0 ? rootChord : polygonMaxChord(polygon);
            g->tipChord = std::max(0.0, tipHi - tipLo);
            g->Af = metrics.area;
            g->AR = 2.0 * g->span * g->span / g->Af;
            g->Yma = metrics.Yma;
            g->cpz = metrics.cpz;
            g->gammaC = std::atan((sweep + 0.5 * g->tipChord - 0.5 * g->rootChord)
                                  / std::max(g->span, 1e-9));
            g->rollGeometricalConstant = g->rootChord * g->span
                * (3.0 * kFinPi * g->span * g->span + 32.0 * radius * g->span
                   + 12.0 * kFinPi * radius * radius) / 48.0;
            g->rollDampingInterferenceFactor = 1.0;
        }

        g->cpLeverArmM = positionM - g->cpz;
        return g;
    }

}
