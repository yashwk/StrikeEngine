#pragma once

#include <cstddef>

namespace StrikeEngine::Kernel {

    /**
     * @brief Error-state dimension the vectorised covariance kernels are sized for.
     *
     * The kernels are fixed-width (they hold a whole 15-element row in vector
     * registers), so a change to the filter's error-state size must be caught
     * at compile time rather than silently reading past a row.
     */
    inline constexpr std::size_t kCovarianceSize = 15;

    /**
     * @brief Runtime CPU feature tier for the hand-vectorised kernels.
     *
     * Every tier is compiled into the shipped binary and the process picks one
     * at first use from the CPU it is actually running on. A binary built for
     * baseline x86-64 (SSE2) therefore still uses AVX2/AVX-512 where present
     * and stays correct, and no faster, everywhere else. The tiers are ordered
     * so a lower one is always a valid fallback for a higher one.
     */
    enum class SimdTier {
        Scalar = 0,
        Avx2 = 1,
        Avx512 = 2
    };

    /// Feature tier this process uses. Detected once and cached.
    SimdTier simdTier();

    /// Stable tier name for diagnostics and tests ("scalar"/"avx2"/"avx512").
    const char* simdTierName(SimdTier tier);

    /**
     * @brief Error-state covariance propagation: out = transition * covariance * transition^T.
     *
     * Inputs and output are 15x15 row-major doubles (kCovarianceSize). Every
     * tier produces the same result bit-for-bit as the scalar reference: the
     * product accumulates over the inner index in ascending order and no tier
     * contracts to FMA, which is exactly how the pre-vectorised triple loop
     * rounded on a baseline build. Callers may rely on that to keep a seeded
     * run reproducible across machines.
     *
     * `out` may alias `covariance` (the in-place filter update) but must not
     * alias `transition`, which is still being read.
     *
     * `tier` selects the kernel explicitly and defaults to the detected one.
     * Passing a tier above what `simdTier()` reports is undefined (the CPU
     * cannot run it); tests use it to compare every tier a machine supports.
     */
    void propagateCovariance15(const double* transition,
                               const double* covariance,
                               double* out,
                               SimdTier tier = simdTier());

} // namespace StrikeEngine::Kernel
