
#ifndef __mixr_base_random_IRng_HPP__
#define __mixr_base_random_IRng_HPP__

#include <cstdint>

namespace mixr {
namespace base {

//------------------------------------------------------------------------------
// Interface: IRng
// Description: Pure interface for a hierarchical, splittable pseudo-random
//              number generator (PRNG).
//
// Design (ADR-007):
//   - Every stochastic component in the simulation kernel consumes an IRng*
//     obtained via split() from a parent stream. This guarantees that:
//     a) The full run is deterministic given a single top-level seed.
//     b) Adding or removing components cannot shift the sequence of any
//        other component (independence by construction).
//   - No global RNG is ever called from sim code paths.
//   - std::random_device is forbidden in sim code (use only at top-level
//     seeding, outside IRng hierarchy).
//
// Usage:
//   IRng* root = new PcgRng(seed);
//   IRng* weapon_stream   = root->split(0);
//   IRng* sensor_stream   = root->split(1);
//   // Each child stream is statistically independent of the parent and siblings.
//------------------------------------------------------------------------------
class IRng
{
public:
    virtual ~IRng() = default;

    // Returns a uniformly-distributed 64-bit unsigned integer covering
    // the full [0, 2^64-1] range.
    virtual uint64_t next_u64() = 0;

    // Returns a uniformly-distributed double in [lo, hi).
    // Default: [0.0, 1.0).
    virtual double uniform(double lo = 0.0, double hi = 1.0) = 0;

    // Returns a normally-distributed double with the given mean and stddev.
    // Implemented via Box-Muller transform internally.
    virtual double normal(double mean = 0.0, double stddev = 1.0) = 0;

    // Creates a new, independent child sub-stream seeded deterministically
    // from this generator's current state and the given child_id.
    //
    // Contract:
    //   - Calling split(n) on two generators that have the same seed and
    //     have been advanced identically will produce two child generators
    //     that generate the same sequence.
    //   - Different child_id values produce statistically independent streams.
    //   - Caller owns the returned pointer.
    virtual IRng* split(uint64_t child_id) = 0;
};

} // namespace base
} // namespace mixr

#endif // __mixr_base_random_IRng_HPP__
