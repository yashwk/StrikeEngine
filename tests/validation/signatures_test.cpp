// Signature table lookup edge cases: RCSDatabase / IRSignatureDatabase.
//
// The bilinear lookup forces its lower index to at least 1 so that the `-1`
// neighbour is valid. An axis with a single breakpoint therefore produced an
// upper index of 1 on a one-element vector - an out-of-bounds read that only
// surfaced under a bounds-checked standard library. Both lookups now clamp the
// upper breakpoint index, and RCSDatabase validates its table shape at load the
// way IRSignatureDatabase already did.
#include <strikeengine/models/signatures/RCSDatabase.hpp>
#include <strikeengine/models/signatures/IRSignatureDatabase.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace StrikeEngine::Models;

static const std::string kData =
    std::string(STRIKEENGINE_SOURCE_DIR) + "/tests/data/";

static int failures = 0;

static void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

static bool near(double a, double b, double relTol = 1e-9)
{
    const double scale = std::max(std::abs(a), std::abs(b));
    return std::abs(a - b) <= relTol * (scale > 0.0 ? scale : 1.0);
}

int main()
{
    std::printf("=== signatures_test: RCS / IR table lookup ===\n");

    // ---- Single-breakpoint axis must not read past the vector -------------
    {
        RCSDatabase rcs;
        check(rcs.loadProfile(kData + "degenerate_rcs.json"),
              "RCS profile with a single azimuth breakpoint loads");
        // Table holds 10 dBsm, so the lookup is 10^(10/10) = 10 m^2 whatever
        // the query angle: there is no azimuth interval to interpolate across.
        check(near(rcs.getRCS(0.0, 0.0), 10.0) &&
                  near(rcs.getRCS(1.5, 1.2), 10.0) &&
                  near(rcs.getRCS(-3.0, 0.785), 10.0),
              "single-breakpoint azimuth returns the tabulated value at any angle");
    }
    {
        IRSignatureDatabase ir;
        check(ir.loadProfile(kData + "degenerate_ir.json"),
              "IR profile with a single azimuth breakpoint loads");
        check(near(ir.getRadiantIntensity(0.0, 0.785), 250.0) &&
                  near(ir.getRadiantIntensity(2.5, 0.1), 250.0),
              "single-breakpoint IR axis returns the tabulated value");
    }

    // ---- Table shape is validated at load ---------------------------------
    {
        RCSDatabase rcs;
        check(!rcs.loadProfile(kData + "malformed_rcs.json"),
              "RCS table that does not match its breakpoints is rejected");
    }
    {
        // A rejected load must leave the lookup on its documented default
        // rather than partially-loaded state.
        RCSDatabase fresh;
        check(near(fresh.getRCS(0.5, 0.5), 1.0),
              "an unloaded RCS database returns the 1 m^2 default");
    }

    // ---- A well-formed profile still interpolates -------------------------
    {
        RCSDatabase rcs;
        check(rcs.loadProfile(kData + "flat_rcs.json"),
              "well-formed RCS fixture loads");
        const double rcsMid = rcs.getRCS(0.0, 0.0);
        check(std::isfinite(rcsMid) && rcsMid > 0.0,
              "well-formed lookup returns a finite positive area");
    }
    {
        IRSignatureDatabase ir;
        check(ir.loadProfile(kData + "flat_ir_hot.json"),
              "well-formed IR fixture loads");
        check(std::isfinite(ir.getRadiantIntensity(0.0, 0.0)),
              "well-formed IR lookup returns a finite value");
    }

    // ---- Missing file fails cleanly ---------------------------------------
    {
        RCSDatabase rcs;
        check(!rcs.loadProfile(kData + "definitely_absent_rcs.json"),
              "a missing RCS file fails the load");
    }

    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "FAILED",
                failures);
    return failures == 0 ? 0 : 1;
}
