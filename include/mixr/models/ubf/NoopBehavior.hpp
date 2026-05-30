
#ifndef __mixr_models_ubf_NoopBehavior_HPP__
#define __mixr_models_ubf_NoopBehavior_HPP__

#include "mixr/base/ubf/AbstractBehavior.hpp"

namespace mixr {
namespace models {

//------------------------------------------------------------------------------
// Class: NoopBehavior
//
// Description: A behavior that always returns nullptr (no recommended action).
//              Used as the placeholder behavior in BVR-2v2 blue1's UBF agent
//              slot in sprint-11 (T-E34) to prove the wire path compiles and
//              the Agent::controller() loop runs without affecting aircraft
//              behavior. Will be replaced by BeamMissileBehavior in T-E35.
//
//              Because genAction() returns nullptr, the existing Autopilot
//              configuration on the player continues to drive the aircraft
//              unchanged (the Agent does not displace the Autopilot slot —
//              it sits alongside it under the player's `components`).
//
// Factory name: NoopBehavior
//------------------------------------------------------------------------------
class NoopBehavior final : public base::ubf::AbstractBehavior
{
   DECLARE_SUBCLASS(NoopBehavior, base::ubf::AbstractBehavior)

public:
   NoopBehavior();

   base::ubf::AbstractAction* genAction(const base::ubf::AbstractState* const state,
                                        const double dt) override;
};

}
}

#endif
