#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <memory>

namespace StrikeEngine::Models {

    constexpr double kFinPi = 3.14159265358979323846;

    enum class FinShape : uint8_t { Trapezoidal, Elliptical, FreeForm };

    /**
     * Precomputed geometric/aerodynamic parameters of a set of fins, ported
     * from RocketPy's _geometry.py / fins model (Diederich planform +
     * Prandtl-Glauert Mach correction + fin-body interference).
     *
     * Conventions (matches AeroModel body frame: X forward/nose, Y right,
     * Z down):
     *   - cpLeverArmM is the SIGNED body-X coordinate of the fin's center of
     *     pressure relative to the CG (positive = nose/canard, negative =
     *     tail). A negative lever arm yields the restoring (stabilizing)
     *     moment for positive alpha/beta.
     *   - clAlpha(mach) is the multi-fin lift slope (1/rad), already
     *     including the fin-count correction and lift-interference factor.
     */
    struct FinsGeometry {
        FinShape shape;
        double n = 0.0;              // fin count (>=3)
        double rootChord = 0.0;
        double tipChord = 0.0;       // trapezoidal only
        double span = 0.0;
        double sweepLength = 0.0;    // trapezoidal only
        double positionM = 0.0;      // root LE axial offset from CG (+ nose)
        double cantRad = 0.0;
        double rocketRadius = 0.0;
        double referenceArea = 0.0;
        double referenceLength = 0.0;

        double Af = 0.0;             // planform area of one fin
        double AR = 0.0;             // aspect ratio
        double gammaC = 0.0;         // mid-chord sweep angle (rad)
        double Yma = 0.0;            // spanwise mean-aero-chord position
        double cpz = 0.0;            // CP distance from root LE (tailward)
        double cpLeverArmM = 0.0;    // signed body-X CP coordinate (= positionM - cpz)
        double liftInterferenceFactor = 0.0;
        double rollGeometricalConstant = 0.0;
        double rollDampingInterferenceFactor = 0.0;
        double rollForcingInterferenceFactor = 0.0;
        double finNumCorrection = 0.0;

        // Single-fin lift slope vs the fin's local angle of attack, before the
        // fin-count and interference corrections.
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

    // Roll forcing interference factor: a function of tau only, shared by all
    // three fin shapes (RocketPy). tau = (span + radius) / radius.
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

    /**
     * Builds the FinsGeometry for a fin set. Validates shape-specific inputs
     * (fail-fast). Returns nullptr and sets `error` on invalid input.
     *
     * positionM: fin root leading edge axial offset from CG, along body +X
     * (nose positive; tail fins are negative). cantAngleDeg: fin cant.
     */
    inline std::shared_ptr<FinsGeometry> buildFinsGeometry(
        FinShape shape, int count,
        double rootChord, double tipChord, double span,
        double sweepLength, double positionM, double cantAngleDeg,
        const std::vector<std::array<double, 2>>& shapePoints,
        double referenceArea, std::string* error = nullptr)
    {
        auto fail = [&](const char* msg) {
            if (error) *error = msg;
            return nullptr;
        };
        if (count < 3) return fail("fin count must be >= 3");
        if (referenceArea <= 0.0) return fail("reference_area must be > 0");

        const double radius = std::sqrt(referenceArea / kFinPi);

        auto g = std::make_shared<FinsGeometry>();
        g->shape = shape;
        g->n = static_cast<double>(count);
        g->rocketRadius = radius;
        g->referenceArea = referenceArea;
        g->referenceLength = 2.0 * radius;
        g->positionM = positionM;
        g->cantRad = cantAngleDeg * kFinPi / 180.0;
        g->rootChord = rootChord;
        g->tipChord = tipChord;

        const double tau = (span + radius) / radius;
        g->liftInterferenceFactor = 1.0 + 1.0 / tau;
        g->rollForcingInterferenceFactor = rollForcingInterferenceFactor(tau);
        g->finNumCorrection = finNumCorrection(g->n);

        if (shape == FinShape::Trapezoidal) {
            if (rootChord <= 0.0) return fail("trapezoidal root_chord must be > 0");
            if (tipChord < 0.0)   return fail("trapezoidal tip_chord must be >= 0");
            if (span <= 0.0)      return fail("trapezoidal span must be > 0");
            double sweep = (sweepLength < 0.0) ? (rootChord - tipChord) : sweepLength;
            g->sweepLength = sweep;
            g->span = span;

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
        }
        else if (shape == FinShape::Elliptical) {
            if (rootChord <= 0.0) return fail("elliptical root_chord must be > 0");
            if (span <= 0.0)      return fail("elliptical span must be > 0");
            g->span = span;
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
        }
        else { // FreeForm
            if (shapePoints.size() < 3) return fail("free-form needs >= 3 shape points");
            // infer dimensions
            double root = std::abs(shapePoints.front()[0] - shapePoints.back()[0]);
            double yMin = shapePoints[0][1], yMax = shapePoints[0][1];
            for (const auto& p : shapePoints) {
                yMin = std::min(yMin, p[1]);
                yMax = std::max(yMax, p[1]);
            }
            const double sp = yMax - yMin;
            if (root <= 0.0) return fail("free-form root chord must be > 0");
            if (sp <= 0.0)   return fail("free-form span must be > 0");
            g->rootChord = root;
            g->span = sp;

            double Af = 0.0;
            for (std::size_t i = 0; i + 1 < shapePoints.size(); ++i) {
                Af += (shapePoints[i][1] + shapePoints[i + 1][1])
                      * (shapePoints[i][0] - shapePoints[i + 1][0]);
            }
            Af = std::abs(Af) / 2.0;
            if (Af < 1e-6) return fail("free-form fin area is too small");
            g->Af = Af;
            g->AR = 2.0 * sp * sp / Af;

            const int ppl = 40;
            std::vector<double> lead(ppl, 1e30), trail(ppl, -1e30), chlen(ppl, 0.0);
            for (std::size_t p = 1; p < shapePoints.size(); ++p) {
                double x1 = shapePoints[p - 1][0], y1 = shapePoints[p - 1][1];
                double x2 = shapePoints[p][0],     y2 = shapePoints[p][1];
                int pi = static_cast<int>(y1 / sp * (ppl - 1));
                int ci = static_cast<int>(y2 / sp * (ppl - 1));
                pi = std::max(0, std::min(ppl - 1, pi));
                ci = std::max(0, std::min(ppl - 1, ci));
                if (pi > ci) std::swap(pi, ci);
                for (int i = pi; i <= ci; ++i) {
                    double y = i * sp / (ppl - 1);
                    double x = (y1 != y2)
                        ? std::max(std::min(x1, x2),
                             std::min(std::max(x1, x2),
                               (y - y2) / (y1 - y2) * x1 + (y1 - y) / (y1 - y2) * x2))
                        : x1;
                    lead[i] = std::min(lead[i], x);
                    trail[i] = std::max(trail[i], x);
                    chlen[i] += (y1 < y2) ? -x : x;
                }
            }
            for (int i = 0; i < ppl; ++i) {
                if (!std::isfinite(lead[i]) && !std::isfinite(trail[i])) {
                    lead[i] = trail[i] = 0.0;
                }
                if (chlen[i] < 0.0) chlen[i] = 0.0;
                if (!std::isfinite(chlen[i])) chlen[i] = 0.0;
                chlen[i] = std::min(chlen[i], trail[i] - lead[i]);
            }

            double totalArea = 0.0, macLen = 0.0, macLead = 0.0, macSpan = 0.0;
            double cosGammaSum = 0.0, rollConst = 0.0, rollNum = 0.0, rollDen = 0.0;
            const double dy = sp / (ppl - 1);
            for (int i = 0; i < ppl; ++i) {
                const double chord = trail[i] - lead[i];
                const double y = i * dy;
                macLen += chord * chord;
                macSpan += y * chord;
                macLead += lead[i] * chord;
                totalArea += chord;
                rollConst += chlen[i] * (radius + y) * (radius + y);
                rollNum += radius * radius * radius * chord / ((radius + y) * (radius + y));
                rollDen += (radius + y) * chord;
                if (i > 0) {
                    double dx = (trail[i] + lead[i]) / 2.0 - (trail[i - 1] + lead[i - 1]) / 2.0;
                    cosGammaSum += dy / std::hypot(dx, dy);
                }
            }
            macLen *= dy; macSpan *= dy; macLead *= dy;
            totalArea *= dy; rollConst *= dy; rollNum *= dy; rollDen *= dy;
            macLen /= totalArea; macSpan /= totalArea; macLead /= totalArea;
            const double cosGamma = cosGammaSum / (ppl - 1);

            g->gammaC = std::acos(std::max(-1.0, std::min(1.0, cosGamma)));
            g->Yma = macSpan;
            g->cpz = macLead + 0.25 * macLen;
            g->rollGeometricalConstant = rollConst;
            g->rollDampingInterferenceFactor = 1.0 + rollNum / rollDen;
        }

        g->cpLeverArmM = positionM - g->cpz;
        return g;
    }

} // namespace StrikeEngine::Models
