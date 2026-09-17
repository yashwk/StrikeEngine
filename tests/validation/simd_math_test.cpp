// Runtime-dispatched SIMD covariance kernels. The contract under test is that
// every tier a machine supports reproduces the scalar triple-loop result
// bit-for-bit, so vectorising the EKF must not re-baseline a seeded run.
#include <strikeengine/kernel/math/SimdMath.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace StrikeEngine::Kernel;

namespace {

int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

constexpr int kN = static_cast<int>(kCovarianceSize);
constexpr std::size_t kCount = kCovarianceSize * kCovarianceSize;
using Matrix = std::array<double, kCount>;

std::uint64_t bitsOf(double value)
{
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

// The exact loop nest the kernels replaced: value accumulates over k in
// ascending order, then the second product reads the intermediate and the
// transposed factor. Contraction is disabled so the reference rounds the same
// way a baseline build does, independent of the host's instruction set.
#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("fp-contract=off")
#endif
void referencePropagate(const double* f, const double* p, double* out)
{
    double temp[kCount];
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            double value = 0.0;
            for (int k = 0; k < kN; ++k) {
                value += f[r * kN + k] * p[k * kN + c];
            }
            temp[r * kN + c] = value;
        }
    }
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            double value = 0.0;
            for (int k = 0; k < kN; ++k) {
                value += temp[r * kN + k] * f[c * kN + k];
            }
            out[r * kN + c] = value;
        }
    }
}
#if defined(__GNUC__)
#pragma GCC pop_options
#endif

std::uint32_t rngState = 0x5EED1234u;
double nextRandom()
{
    rngState = rngState * 1664525u + 1013904223u;
    const double unit = static_cast<double>(rngState >> 8) / static_cast<double>(1u << 24);
    return unit * 2.0 - 1.0;
}

Matrix randomMatrix()
{
    Matrix m{};
    for (std::size_t i = 0; i < kCount; ++i) {
        m[i] = nextRandom();
    }
    return m;
}

// A transition matrix of the shape the INS actually builds: identity plus
// dt-scaled position/velocity, attitude and bias blocks. Random dense input
// would not exercise the near-identity structure that dominates in flight.
Matrix insLikeTransition(double dt)
{
    Matrix m{};
    m.fill(0.0);
    for (int i = 0; i < kN; ++i) {
        m[static_cast<std::size_t>(i) * kN + i] = 1.0;
    }
    for (int axis = 0; axis < 3; ++axis) {
        m[static_cast<std::size_t>(axis) * kN + (axis + 3)] = dt;
        m[static_cast<std::size_t>(axis + 6) * kN + (axis + 12)] = -dt;
    }
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            m[static_cast<std::size_t>(row + 3) * kN + (column + 6)] = -0.3 * dt;
            m[static_cast<std::size_t>(row + 3) * kN + (column + 9)] = -0.7 * dt;
        }
    }
    return m;
}

// A covariance is symmetric positive semi-definite; build it that way so the
// comparison covers representative magnitudes rather than arbitrary noise.
Matrix randomCovariance()
{
    Matrix a = randomMatrix();
    Matrix c{};
    for (int r = 0; r < kN; ++r) {
        for (int col = 0; col < kN; ++col) {
            double value = 0.0;
            for (int k = 0; k < kN; ++k) {
                value += a[r * kN + k] * a[col * kN + k];
            }
            c[static_cast<std::size_t>(r) * kN + col] = value;
        }
    }
    return c;
}

int compareBitwise(const Matrix& expected, const Matrix& actual)
{
    int differing = 0;
    for (std::size_t i = 0; i < kCount; ++i) {
        if (bitsOf(expected[i]) != bitsOf(actual[i])) {
            ++differing;
        }
    }
    return differing;
}

} // namespace

int main()
{
    std::printf("=== simd_math_test: runtime-dispatched covariance kernels ===\n");

    const SimdTier detected = simdTier();
    std::printf("  detected tier: %s (kCovarianceSize = %zu)\n",
                simdTierName(detected), kCovarianceSize);

    check(simdTierName(SimdTier::Scalar) != nullptr, "scalar tier has a name");
    check(simdTierName(SimdTier::Avx2) != nullptr, "avx2 tier has a name");
    check(simdTierName(SimdTier::Avx512) != nullptr, "avx512 tier has a name");
    check(simdTierName(SimdTier::Scalar) != simdTierName(SimdTier::Avx2),
          "tier names are distinct");
    check(SimdTier::Scalar < SimdTier::Avx2 && SimdTier::Avx2 < SimdTier::Avx512,
          "tiers are ordered so a fallback is always available");
    check(simdTier() == detected, "tier detection is stable across calls");

    // Cases: random dense, the INS transition shape, and the identity (whose
    // propagation must return the covariance unchanged).
    Matrix cases[3];
    cases[0] = randomMatrix();
    cases[1] = insLikeTransition(0.01);
    cases[2] = Matrix{};
    cases[2].fill(0.0);
    for (int i = 0; i < kN; ++i) {
        cases[2][static_cast<std::size_t>(i) * kN + i] = 1.0;
    }
    const char* caseNames[3] = {"random dense", "INS-shaped", "identity"};

    const SimdTier tiers[3] = {SimdTier::Scalar, SimdTier::Avx2, SimdTier::Avx512};
    const char* tierNames[3] = {"scalar", "avx2", "avx512"};

    for (int c = 0; c < 3; ++c) {
        const Matrix covariance = randomCovariance();
        Matrix expected{};
        referencePropagate(cases[c].data(), covariance.data(), expected.data());

        for (int t = 0; t < 3; ++t) {
            // Never invoke a tier the CPU cannot run.
            if (tiers[t] > detected) {
                continue;
            }
            Matrix actual{};
            propagateCovariance15(cases[c].data(), covariance.data(), actual.data(), tiers[t]);
            char label[128];
            std::snprintf(label, sizeof(label),
                          "%s x %s matches the triple loop bit-for-bit",
                          tierNames[t], caseNames[c]);
            check(compareBitwise(expected, actual) == 0, label);
        }
    }

    // In-place operation is how the INS calls it; it must equal the
    // out-of-place result exactly.
    {
        const Matrix transition = insLikeTransition(0.005);
        const Matrix covariance = randomCovariance();
        Matrix outOfPlace{};
        referencePropagate(transition.data(), covariance.data(), outOfPlace.data());

        Matrix inPlace = covariance;
        propagateCovariance15(transition.data(), inPlace.data(), inPlace.data());
        check(compareBitwise(outOfPlace, inPlace) == 0,
              "in-place propagation equals the out-of-place result");
    }

    // Every lower tier must agree with the fastest one, not just the reference.
    if (detected != SimdTier::Scalar) {
        const Matrix transition = insLikeTransition(0.01);
        const Matrix covariance = randomCovariance();
        Matrix scalar{};
        Matrix fast{};
        propagateCovariance15(transition.data(), covariance.data(), scalar.data(), SimdTier::Scalar);
        propagateCovariance15(transition.data(), covariance.data(), fast.data(), detected);
        check(compareBitwise(scalar, fast) == 0,
              "the detected tier agrees with the scalar tier bit-for-bit");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILURES", failures);
    return failures == 0 ? 0 : 1;
}
