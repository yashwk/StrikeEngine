#pragma once

#include <cstddef>
#include <utility>

namespace StrikeEngine::Kernel {

    /**
     * @brief Resizes @p v to @p n entries only when it is currently shorter.
     *
     * Data-block growth must never shrink a live array: the blocks are grown
     * from several call sites (entity creation, the systems' own defensive
     * sizing) and a caller that passes a smaller count than a vector already
     * holds must not discard state.
     */
    template <typename Vector, typename Value>
    inline void growTo(Vector& v, std::size_t n, const Value& defaultValue)
    {
        if (v.size() < n) v.resize(n, defaultValue);
    }

    /// Overload for vectors of non-default-constructible-with-value types
    /// (e.g. nested vectors), where value-initialisation is the default.
    template <typename Vector>
    inline void growTo(Vector& v, std::size_t n)
    {
        if (v.size() < n) v.resize(n);
    }

} // namespace StrikeEngine::Kernel
