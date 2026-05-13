#ifndef __mixr_models_common_Gmti_HPP__
#define __mixr_models_common_Gmti_HPP__

// ---------------------------------------------------------------------------
// Class: Gmti  (Ground-Moving-Target-Indication radar mode)
//
// IRng* noise-injection pattern (ADR-007):
//
//   All stochastic components that draw random values MUST hold an IRng*
//   obtained from the simulation's hierarchical sub-stream tree.  Never call
//   a global RNG (rand(), std::rand(), std::mt19937, std::random_device) from
//   simulation code paths.
//
//   Pattern to mirror in future stochastic components:
//     1.  Add an `IRng* rng_` member, default nullptr.
//     2.  Expose `setRng(IRng*)` — called by the owning player/simulation
//         after seedRng(master_seed) partitions the stream tree.
//     3.  Guard every random draw with `if (rng_ != nullptr)` so the
//         component degrades gracefully (deterministically zero-noise) when
//         no RNG has been injected.
//     4.  Do NOT call split() inside the component; the parent hands a
//         pre-split stream whose child_id encodes the component's slot.
//
//   Example call-site (owning simulation, player slot index 3, sensor 0):
//     IRng* sim_root  = new PcgRng(master_seed);
//     IRng* p3_stream = sim_root->split(3);          // player 3
//     IRng* s0_stream = p3_stream->split(0);         // sensor 0 on player 3
//     gmti->setRng(s0_stream);                       // Gmti owns the pointer
// ---------------------------------------------------------------------------

#include "mixr/models/system/Radar.hpp"
#include "mixr/base/osg/Vec3d"
#include "mixr/base/random/IRng.hpp"

namespace mixr {
namespace base { class List; }
namespace models {

//------------------------------------------------------------------------------
// Class: Gmti
// Description: Very simple, Ground-Moving-Target-Indication (GMTI) mode radar.
//
//   Extends the base Radar with optional Gaussian azimuth/elevation measurement
//   noise so that downstream track filters see realistic, reproducible scatter.
//   Noise is injected only when an IRng* sub-stream has been supplied via
//   setRng(); without one the sensor behaves as before (deterministic, zero
//   noise) for backwards compatibility.
//------------------------------------------------------------------------------
class Gmti : public Radar
{
   DECLARE_SUBCLASS(Gmti, Radar)

public:
   Gmti();

   const base::Vec3d& getPoi() const                                { return poiVec; }
   void setPoi(const double x, const double y, const double z);
   void setPoi(const base::Vec3d& newPos);

   // --- IRng noise-injection interface (ADR-007) ---

   // Transfer ownership of a pre-split IRng sub-stream to this component.
   // The stream must have been obtained via parent->split(child_id) by the
   // owning simulation AFTER seedRng(master_seed) has been called.
   // Passing nullptr disables noise (component reverts to deterministic).
   // Gmti takes ownership and deletes the previous stream if any.
   void setRng(base::IRng* rng);

   // Standard deviation of the additive Gaussian angle noise (radians).
   // Default: 0.0 (no noise until explicitly set).
   void setNoiseStdRad(double stdRad);
   double getNoiseStdRad() const { return noiseStdRad_; }

   // Last drawn noise samples — exposed for golden-trace testing only.
   double getLastAzNoise()  const { return lastAzNoise_; }
   double getLastElNoise()  const { return lastElNoise_; }

protected:
   void dynamics(const double dt) override;

private:
   base::Vec3d poiVec;    // Point Of Interest vector  (m) [ x, y, z ] NED

   // IRng sub-stream for Gaussian az/el noise draws (ADR-007).
   // Owned by this object; nullptr => noise disabled.
   base::IRng* rng_{nullptr};
   double      noiseStdRad_{0.0};

   // Last noise values (useful for unit tests / golden traces).
   double lastAzNoise_{0.0};
   double lastElNoise_{0.0};

private:
   // slot table helper methods
   bool setSlotPoi(base::List* const);
};

}
}

#endif
