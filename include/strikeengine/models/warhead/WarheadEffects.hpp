#pragma once

namespace StrikeEngine::Models {

    /**
     * @brief Kill probability of a warhead vs miss distance with an optional
     * linear fragmentation/overpressure falloff band.
     *
     *  - d <= lethalRadiusM                   -> 1.0 (guaranteed kill)
     *  - falloffRadiusM <= 0 or falloffRadiusM <= lethalRadiusM
     *                                         -> flat law: 1.0 inside lethal,
     *                                            else 0.0 (no falloff band)
     *  - lethalRadiusM < d <= falloffRadiusM  -> linear decay from 1.0 at the
     *                                            lethal edge to 0.0 at the
     *                                            falloff edge
     *  - d > falloffRadiusM                   -> 0.0
     *
     * @param missDistanceM   Distance from the detonation point to the target (m).
     * @param lethalRadiusM   Inner guaranteed-kill radius (m).
     * @param falloffRadiusM  Outer edge of the falloff band (m); <= 0 disables
     *                        the band entirely.
     */
    inline double warheadKillProbability(double missDistanceM, double lethalRadiusM, double falloffRadiusM)
    {
        if (missDistanceM <= lethalRadiusM) return 1.0;
        if (falloffRadiusM <= lethalRadiusM) return 0.0;
        if (missDistanceM >= falloffRadiusM) return 0.0;
        return (falloffRadiusM - missDistanceM) / (falloffRadiusM - lethalRadiusM);
    }

} // namespace StrikeEngine::Models
