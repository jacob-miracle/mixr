
#ifndef __mixr_models_ubf_BeamMissileBehavior_HPP__
#define __mixr_models_ubf_BeamMissileBehavior_HPP__

#include "mixr/base/ubf/AbstractBehavior.hpp"

#include <cmath>
#include <string>

namespace mixr {

namespace base {
class Length;
class Angle;
class Time;
class Number;
namespace ubf {
class AbstractAction;
class AbstractState;
}
}

namespace models {

class PilotState;

//------------------------------------------------------------------------------
// Class: BeamMissileBehavior
//
// Description:
//    Defensive UBF behavior — turn perpendicular ("beam") or 180 deg ("cold
//    abort") to the line-of-sight of the closest incoming missile so the
//    weapon either runs out of burn-energy reaching us or misses
//    geometrically.
//
//    This is the canonical defensive tactic that has physics in our engine:
//    MIXR's missile model carries no atmospheric drag, so a dive-to-bleed-
//    energy tactic cannot work (probe verified pre-sprint-11; see
//    docs/sprint-11-planning/final-plan.md §1). Beaming exploits proportional
//    navigation lead by forcing the missile into a high-angle off-axis
//    pursuit that bleeds latax.
//
// Factory name: BeamMissileBehavior
//
// Slots:
//    triggerRange  <Length|Number>   ! Engage defensive when nearest incoming
//                                    ! weapon's slant range falls below this
//                                    ! threshold.  Number form is metres.
//                                    ! (default: 15000 m)
//    beamAngle     <Angle|Number>    ! Angle to add to the missile's LOS
//                                    ! bearing to produce commanded heading.
//                                    ! 90 deg = beam, 180 deg = cold abort.
//                                    ! Number form is degrees.
//                                    ! (default: 90 deg)
//    recoverTime   <Time|Number>     ! After threat clears, hold the defensive
//                                    ! heading for this duration before
//                                    ! relinquishing control (returning
//                                    ! nullptr so the arbiter falls back to
//                                    ! lower-voted behaviors).  Number form is
//                                    ! seconds.  (default: 5 s)
//    holdAlt       <Length|Number>   ! Altitude to hold while defensive.
//                                    ! Number form is feet.  If the slot is
//                                    ! NOT set the behavior captures ownship
//                                    ! altitude at trigger time so the
//                                    ! defensive turn does NOT dive (drag-free
//                                    ! engine, dive-defensive is impotent).
//
//    vote          <Integer>         ! Inherited from AbstractBehavior.
//                                    ! Default chosen high (100) so the
//                                    ! defensive vote wins the Arbiter
//                                    ! against any cruise behavior at default
//                                    ! vote=1.
//
// State machine (per BG tick via Agent::controller()):
//    CRUISE     — no incoming inside triggerRange. genAction() returns nullptr.
//                  Other (lower-voted) behaviors in the Arbiter chain are free
//                  to drive the aircraft.
//    DEFENSIVE  — nearest incoming inside triggerRange. genAction() returns a
//                  PilotAction setting cmdHeading=(LOS+beamAngle)%360,
//                  cmdAltitude=holdAlt_ft (slot value if set, else captured
//                  ownship altitude at trigger time), navMode=false.  Heading
//                  is recomputed each tick so the defensive turn tracks the
//                  threat geometry as range/bearing evolves.
//    RECOVERING — threat just cleared (no incoming inside triggerRange).
//                  genAction() continues to emit the LAST defensive
//                  PilotAction for recoverTime_s seconds, then transitions
//                  back to CRUISE.  This prevents oscillation between defensive
//                  and cruise if a transient track loss flickers the threat
//                  list.
//
// References:
//    docs/sprint-11-planning/final-plan.md §4 (defensive tactic = beam/cold abort)
//    mixr/include/mixr/base/ubf/AbstractBehavior.hpp (base contract)
//    mixr/include/mixr/models/ubf/PilotState.hpp (threat list source)
//    mixr/include/mixr/models/ubf/PilotAction.hpp (autopilot command sink)
//------------------------------------------------------------------------------
class BeamMissileBehavior final : public base::ubf::AbstractBehavior
{
   DECLARE_SUBCLASS(BeamMissileBehavior, base::ubf::AbstractBehavior)

public:
   enum class Phase {
      CRUISE,
      DEFENSIVE,
      RECOVERING
   };

   // --- Defaults (exposed as static constexpr so unit tests can pin the
   // canonical values WITHOUT instantiating the class — instantiation
   // requires linking libmixr_base.a for the Object/Slottable machinery,
   // which the standalone header-only test path deliberately avoids). ----
   static constexpr double kDefaultTriggerRangeM = 15000.0;
   static constexpr double kDefaultBeamAngleDeg  =    90.0;
   static constexpr double kDefaultRecoverTimeS  =     5.0;
   static constexpr int    kDefaultVote          =   100;

   BeamMissileBehavior();

   // UBF contract: returns a pre-ref'd AbstractAction (caller unrefs) or
   // nullptr when no recommended action.
   base::ubf::AbstractAction* genAction(const base::ubf::AbstractState* const state,
                                        const double dt) override;

   // Reset clears the runtime state machine so a scenario restart begins
   // from CRUISE with no stale captured heading/altitude.
   void reset() override;

   // ---- Configuration accessors (used by tests, defaults, and future tuning) -
   double getTriggerRangeM() const   { return triggerRange_m; }
   double getBeamAngleDeg() const    { return beamAngle_deg; }
   double getRecoverTimeS() const    { return recoverTime_s; }
   bool   hasHoldAltOverride() const { return hasHoldAltSlot; }
   double getHoldAltFt() const       { return holdAltSlot_ft; }

   // ---- Runtime inspection (used by tests and the future recorder hook) ----
   Phase  getPhase() const           { return phase; }
   bool   isDefensiveActive() const  { return phase != Phase::CRUISE; }
   double getActiveHeadingDeg() const { return activeHeading_deg; }
   double getActiveAltitudeFt() const { return activeAltitude_ft; }

   // ---- Direct setters (used by tests and slot helpers) -------------------
   void setTriggerRangeM(double v)   { triggerRange_m = v; }
   void setBeamAngleDeg(double v)    { beamAngle_deg = v; }
   void setRecoverTimeS(double v)    { recoverTime_s = v; }
   void setHoldAltFt(double v)       { holdAltSlot_ft = v; hasHoldAltSlot = true; }
   void clearHoldAltOverride()       { hasHoldAltSlot = false; holdAltSlot_ft = 0.0; }

   // ---- Pure geometry helper (header-only so unit tests can include it
   // without linking against libmixr_*.a) ----------------------------------
   //
   // Returns (losBearingDeg + beamAngleDeg) wrapped to [0, 360).
   // Used both by genAction() and by the unit test under
   // tests/integration/beam_missile_test.cpp.
   static double computeBeamHeading(double losBearingDeg, double beamAngleDeg);

private:
   // ---- Slot configuration --------------------------------------------------
   double triggerRange_m  {kDefaultTriggerRangeM};   // 15 km — slightly inside typical Aam first-shot range
   double beamAngle_deg   {kDefaultBeamAngleDeg};    // 90 deg = beam; set 180 for cold abort
   double recoverTime_s   {kDefaultRecoverTimeS};    // 5 s defensive-hold after threat clears
   double holdAltSlot_ft  {0.0};                     // populated only when hasHoldAltSlot==true
   bool   hasHoldAltSlot  {false};

   // ---- State machine -------------------------------------------------------
   Phase  phase             {Phase::CRUISE};
   double activeHeading_deg {0.0};
   double activeAltitude_ft {0.0};
   double timeInRecovery_s  {0.0};

   // ---- Behavior-state emission (T-E34 wire contract) ----------------------
   //
   // The recorder-side hook (REID_BEHAVIOR_STATE) is deferred to a follow-up
   // task (Agent.cpp + dataRecorderTokens.hpp are outside this task's
   // allowed_paths).  Until then, transitions are logged to std::cout as a
   // single line of JSON shaped exactly like the streamer's
   // projectBehaviorState() output so the headless log is the system of
   // record. See finish_notes for the rationale and the follow-up TODO.
   void emitTransition(const PilotState* const ps,
                       Phase from, Phase to);

   // ---- Slot helpers --------------------------------------------------------
   bool setSlotTriggerRange(const base::Length* const);
   bool setSlotTriggerRange(const base::Number* const);
   bool setSlotBeamAngle(const base::Angle* const);
   bool setSlotBeamAngle(const base::Number* const);
   bool setSlotRecoverTime(const base::Time* const);
   bool setSlotRecoverTime(const base::Number* const);
   bool setSlotHoldAlt(const base::Length* const);
   bool setSlotHoldAlt(const base::Number* const);
};

inline double BeamMissileBehavior::computeBeamHeading(double losBearingDeg,
                                                      double beamAngleDeg)
{
   double h = losBearingDeg + beamAngleDeg;
   // fmod can return a negative remainder for negative dividends; normalize
   // to [0, 360).
   h = std::fmod(h, 360.0);
   if (h < 0.0) h += 360.0;
   return h;
}

}
}

#endif
