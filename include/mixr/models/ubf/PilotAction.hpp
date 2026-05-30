
#ifndef __mixr_models_ubf_PilotAction_HPP__
#define __mixr_models_ubf_PilotAction_HPP__

#include "mixr/base/ubf/AbstractAction.hpp"

namespace mixr {
namespace base { class Component; }
namespace models {
class Autopilot;

//------------------------------------------------------------------------------
// Class: PilotAction
//
// Description: UBF action that issues commands to the actor Player's
//              Autopilot. Each field is independently opt-in via a
//              setCmd*() call; fields not set are NOT pushed to the
//              autopilot when execute() runs.
//
//              This gives behaviors (e.g. BeamMissileBehavior in T-E35)
//              the freedom to command a heading change only, leaving the
//              previously-configured altitude/velocity hold in place.
//
// Factory name: PilotAction
//
// Slots: none (constructed by behaviors at runtime, not from EDL).
//
// Threading: execute() runs on the BG thread (Agent::controller -> Agent::
//   updateData), the same thread MIXR ships Autopilot::process() on, so
//   the setter calls are safe.
//------------------------------------------------------------------------------
class PilotAction final : public base::ubf::AbstractAction
{
   DECLARE_SUBCLASS(PilotAction, base::ubf::AbstractAction)

public:
   PilotAction();

   // Returns true if execute() drove at least one autopilot setter on the actor.
   bool execute(base::Component* actor) override;

   // -------- Command setters (opt-in) ---------------------------------------
   // Calling a setCmd*() marks that field as "set"; execute() will then push
   // it. Unset fields are not touched on the Autopilot.

   void setCmdHeadingDeg(double deg)        { cmdHeading_deg = deg; hasCmdHeading = true; }
   void setCmdAltitudeFt(double ft)         { cmdAltitude_ft = ft;  hasCmdAltitude = true; }
   void setCmdVelocityKts(double kts)       { cmdVelocity_kts = kts; hasCmdVelocity = true; }
   void setCmdNavMode(bool navOn)           { cmdNavMode = navOn;   hasCmdNavMode = true; }

   // -------- Inspection (for tests and the recorder hook) -------------------
   bool hasHeading() const                  { return hasCmdHeading; }
   bool hasAltitude() const                 { return hasCmdAltitude; }
   bool hasVelocity() const                 { return hasCmdVelocity; }
   bool hasNavMode() const                  { return hasCmdNavMode; }

   double getCmdHeadingDeg() const          { return cmdHeading_deg; }
   double getCmdAltitudeFt() const          { return cmdAltitude_ft; }
   double getCmdVelocityKts() const         { return cmdVelocity_kts; }
   bool   getCmdNavMode() const             { return cmdNavMode; }

private:
   // Locate the actor Player's Autopilot via the Player's pilot pointer (set
   // up by Player::setPilot()). Falls back to a depth-first findByType on the
   // actor's component subtree if no Player relationship is available (e.g.
   // unit tests that hand an Autopilot directly as the actor).
   Autopilot* resolveAutopilot(base::Component* actor) const;

   double cmdHeading_deg{};
   double cmdAltitude_ft{};
   double cmdVelocity_kts{};
   bool   cmdNavMode{};

   bool   hasCmdHeading{};
   bool   hasCmdAltitude{};
   bool   hasCmdVelocity{};
   bool   hasCmdNavMode{};
};

}
}

#endif
