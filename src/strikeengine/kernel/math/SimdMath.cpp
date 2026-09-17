#include <strikeengine/kernel/math/SimdMath.hpp>

#include <cstdlib>
#include <cstring>

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define STRIKEENGINE_SIMD_X86 1
#endif

namespace StrikeEngine::Kernel {
namespace {

constexpr int kN = static_cast<int>(kCovarianceSize);
constexpr std::size_t kCount = kCovarianceSize * kCovarianceSize;

// Every kernel below must reproduce the scalar result bit-for-bit. A baseline
// x86-64 build (SSE2) has no FMA and rounds each multiply and add separately,
// so contraction is disabled here: enabling it only in the AVX paths would
// silently change the filter output and re-baseline seeded engagements.
#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("fp-contract=off")
#endif

void transpose15(const double* in, double* out)
{
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            out[c * kN + r] = in[r * kN + c];
        }
    }
}

// out = a * b. Each output column accumulates over k in ascending order, which
// is what makes the vector paths (which parallelise across columns, not across
// the reduction) numerically identical to this reference.
void matmulScalar(const double* a, const double* b, double* out)
{
    for (int r = 0; r < kN; ++r) {
        double acc[kN];
        for (int c = 0; c < kN; ++c) {
            acc[c] = 0.0;
        }
        const double* ar = a + r * kN;
        for (int k = 0; k < kN; ++k) {
            const double f = ar[k];
            const double* bk = b + k * kN;
            for (int c = 0; c < kN; ++c) {
                acc[c] += f * bk[c];
            }
        }
        for (int c = 0; c < kN; ++c) {
            out[r * kN + c] = acc[c];
        }
    }
}

void propagateScalar(const double* transition, const double* covariance, double* out)
{
    double temp[kCount];
    double transposed[kCount];
    transpose15(transition, transposed);
    matmulScalar(transition, covariance, temp);
    matmulScalar(temp, transposed, out);
}

#if defined(STRIKEENGINE_SIMD_X86)

// 15 columns do not divide by the vector width, so the trailing lanes are
// loaded and stored under a mask. Masked accesses never touch the bytes past
// column 14, so no padding or out-of-range read is needed.
__attribute__((target("avx2")))
void matmulAvx2(const double* a, const double* b, double* out)
{
    const __m256i tail = _mm256_set_epi64x(0, -1, -1, -1);
    for (int r = 0; r < kN; ++r) {
        __m256d a0 = _mm256_setzero_pd();
        __m256d a1 = _mm256_setzero_pd();
        __m256d a2 = _mm256_setzero_pd();
        __m256d a3 = _mm256_setzero_pd();
        const double* ar = a + r * kN;
        for (int k = 0; k < kN; ++k) {
            const __m256d f = _mm256_set1_pd(ar[k]);
            const double* bk = b + k * kN;
            a0 = _mm256_add_pd(a0, _mm256_mul_pd(f, _mm256_loadu_pd(bk)));
            a1 = _mm256_add_pd(a1, _mm256_mul_pd(f, _mm256_loadu_pd(bk + 4)));
            a2 = _mm256_add_pd(a2, _mm256_mul_pd(f, _mm256_loadu_pd(bk + 8)));
            a3 = _mm256_add_pd(a3, _mm256_mul_pd(f, _mm256_maskload_pd(bk + 12, tail)));
        }
        double* orow = out + r * kN;
        _mm256_storeu_pd(orow, a0);
        _mm256_storeu_pd(orow + 4, a1);
        _mm256_storeu_pd(orow + 8, a2);
        _mm256_maskstore_pd(orow + 12, tail, a3);
    }
}

__attribute__((target("avx512f")))
void matmulAvx512(const double* a, const double* b, double* out)
{
    const __mmask8 tail = 0x7F;
    for (int r = 0; r < kN; ++r) {
        __m512d lo = _mm512_setzero_pd();
        __m512d hi = _mm512_setzero_pd();
        const double* ar = a + r * kN;
        for (int k = 0; k < kN; ++k) {
            const __m512d f = _mm512_set1_pd(ar[k]);
            const double* bk = b + k * kN;
            lo = _mm512_add_pd(lo, _mm512_mul_pd(f, _mm512_loadu_pd(bk)));
            hi = _mm512_add_pd(hi, _mm512_mul_pd(f, _mm512_maskz_loadu_pd(tail, bk + 8)));
        }
        double* orow = out + r * kN;
        _mm512_storeu_pd(orow, lo);
        _mm512_mask_storeu_pd(orow + 8, tail, hi);
    }
}

void propagateAvx2(const double* transition, const double* covariance, double* out)
{
    double temp[kCount];
    double transposed[kCount];
    transpose15(transition, transposed);
    matmulAvx2(transition, covariance, temp);
    matmulAvx2(temp, transposed, out);
}

void propagateAvx512(const double* transition, const double* covariance, double* out)
{
    double temp[kCount];
    double transposed[kCount];
    transpose15(transition, transposed);
    matmulAvx512(transition, covariance, temp);
    matmulAvx512(temp, transposed, out);
}

#endif // STRIKEENGINE_SIMD_X86

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

SimdTier detectTier()
{
    SimdTier hardware = SimdTier::Scalar;
#if defined(STRIKEENGINE_SIMD_X86) && defined(__GNUC__)
    // libgcc's feature table accounts for OS-enabled vector state (XCR0), so
    // this is false on a CPU whose AVX-512 the kernel has not turned on.
    __builtin_cpu_init();
    if (__builtin_cpu_supports("avx512f")) {
        hardware = SimdTier::Avx512;
    } else if (__builtin_cpu_supports("avx2")) {
        hardware = SimdTier::Avx2;
    }
#endif

    // Override for exercising and benchmarking the lower tiers on one machine.
    // It can only lower the tier, never raise it above what the CPU reported,
    // so a typo cannot turn into an illegal-instruction crash.
    if (const char* override = std::getenv("STRIKEENGINE_SIMD")) {
        SimdTier requested = hardware;
        if (std::strcmp(override, "scalar") == 0) {
            requested = SimdTier::Scalar;
        } else if (std::strcmp(override, "avx2") == 0) {
            requested = SimdTier::Avx2;
        } else if (std::strcmp(override, "avx512") == 0) {
            requested = SimdTier::Avx512;
        }
        if (requested < hardware) {
            hardware = requested;
        }
    }
    return hardware;
}

} // namespace

SimdTier simdTier()
{
    static const SimdTier tier = detectTier();
    return tier;
}

const char* simdTierName(SimdTier tier)
{
    switch (tier) {
        case SimdTier::Avx512: return "avx512";
        case SimdTier::Avx2:   return "avx2";
        default:               return "scalar";
    }
}

void propagateCovariance15(const double* transition,
                           const double* covariance,
                           double* out,
                           SimdTier tier)
{
#if defined(STRIKEENGINE_SIMD_X86)
    switch (tier) {
        case SimdTier::Avx512: propagateAvx512(transition, covariance, out); return;
        case SimdTier::Avx2:   propagateAvx2(transition, covariance, out);   return;
        default: break;
    }
#else
    (void)tier;
#endif
    propagateScalar(transition, covariance, out);
}

} // namespace StrikeEngine::Kernel
