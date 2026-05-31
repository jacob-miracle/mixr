
#include "mixr/models/ubf/BeamMissileBehavior.hpp"

#include "mixr/models/ubf/PilotState.hpp"
#include "mixr/models/ubf/PilotAction.hpp"
#include "mixr/models/player/Player.hpp"
#include "mixr/models/WorldModel.hpp"

#include "mixr/base/numeric/Number.hpp"
#include "mixr/base/units/length/Length.hpp"
#include "mixr/base/units/angle/Angle.hpp"
#include "mixr/base/units/time/Time.hpp"

#include "mixr/base/ubf/AbstractAction.hpp"
#include "mixr/base/ubf/AbstractState.hpp"

// T-E36: recorder hook — route behavior phase changes through the MIXR data
// recorder (REID_BEHAVIOR_STATE) instead of std::cout, so the event reaches the
// WebSocket streamer as a `behavior_state` JSON event.
#include "mixr/simulation/IDataRecorder.hpp"
#include "mixr/simulation/dataRecorderTokens.hpp"
#include "mixr/simulation/recorder_macros.hpp"

#include <algorithm>
#include <cmath>

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(BeamMissileBehavior, "BeamMissileBehavior")

// ---------------------------------------------------------------------------
// Slot table
// ---------------------------------------------------------------------------
BEGIN_SLOTTABLE(BeamMissileBehavior)
   "triggerRange",   // 1: range threshold (Length or Number == metres)
   "beamAngle",      // 2: heading offset from LOS (Angle or Number == degrees)
   "recoverTime",    // 3: post-threat hold duration (Time or Number == seconds)
   "holdAlt",        // 4: altitude to hold while defensive (Length or Number == feet)
END_SLOTTABLE(BeamMissileBehavior)

BEGIN_SLOT_MAP(BeamMissileBehavior)
   ON_SLOT(1, setSlotTriggerRange, base::Length)
   ON_SLOT(1, setSlotTriggerRange, base::Number)
   ON_SLOT(2, setSlotBeamAngle,    base::Angle)
   ON_SLOT(2, setSlotBeamAngle,    base::Number)
   ON_SLOT(3, setSlotRecoverTime,  base::Time)
   ON_SLOT(3, setSlotRecoverTime,  base::Number)
   ON_SLOT(4, setSlotHoldAlt,      base::Length)
   ON_SLOT(4, setSlotHoldAlt,      base::Number)
END_SLOT_MAP()

// ---------------------------------------------------------------------------
// Constructor / copy / destruct
// ---------------------------------------------------------------------------
BeamMissileBehavior::BeamMissileBehavior()
{
   STANDARD_CONSTRUCTOR()

   // Defensive behaviors must beat cruise-level behaviors in the Arbiter.
   // The base class clamps the slot value to (0, 65535]; we set the default
   // here so users who omit the slot still get the intended priority.
   setVote(kDefaultVote);
}

void BeamMissileBehavior::copyData(const BeamMissileBehavior& org, const bool)
{
   BaseClass::copyData(org);
   triggerRange_m   = org.triggerRange_m;
   beamAngle_deg    = org.beamAngle_deg;
   recoverTime_s    = org.recoverTime_s;
   holdAltSlot_ft   = org.holdAltSlot_ft;
   hasHoldAltSlot   = org.hasHoldAltSlot;

   phase            = org.phase;
   activeHeading_deg = org.activeHeading_deg;
   activeAltitude_ft = org.activeAltitude_ft;
   timeInRecovery_s = org.timeInRecovery_s;
}

void BeamMissileBehavior::deleteData()
{
   // Nothing owned.
}

// ---------------------------------------------------------------------------
// reset — drop the runtime state machine back to CRUISE at scenario start.
// Configuration (slots) is intentionally preserved; only the per-run state
// resets.
// ---------------------------------------------------------------------------
void BeamMissileBehavior::reset()
{
   BaseClass::reset();
   phase             = Phase::CRUISE;
   activeHeading_deg = 0.0;
   activeAltitude_ft = 0.0;
   timeInRecovery_s  = 0.0;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

constexpr double kMetresPerFoot = 0.3048;

// Pick the closest entry in `incoming` (smallest slant range).  Returns
// incoming.end() when the list is empty.
auto pickNearest(const std::vector<PilotState::IncomingWeapon>& incoming)
{
   return std::min_element(incoming.begin(), incoming.end(),
      [](const PilotState::IncomingWeapon& a,
         const PilotState::IncomingWeapon& b) {
         return a.range_m < b.range_m;
      });
}

} // namespace

// ---------------------------------------------------------------------------
// genAction — invoked by Agent::controller() each BG tick.
//
// Algorithm:
//   1. dynamic_cast the AbstractState* to PilotState; if not one (or null),
//      return nullptr.
//   2. Sort/scan incoming weapons; pick the nearest by slant range.
//   3. State machine:
//        CRUISE     -> if nearest.range_m < triggerRange_m:
//                          transition to DEFENSIVE
//                          capture holdAlt (slot-override OR ownship altitude)
//                          fall through to emit the defensive action
//                      else: return nullptr
//        DEFENSIVE  -> if threat still inside triggerRange_m:
//                          recompute heading from CURRENT LOS bearing (the
//                          threat keeps moving)
//                          emit action
//                      else:
//                          transition to RECOVERING with timer=0
//                          continue emitting last action
//        RECOVERING -> emit last action; advance timer by dt.
//                      When timer >= recoverTime_s:
//                          transition to CRUISE
//                          return nullptr on THIS tick (relinquish control)
//                      else: emit cached action
//   4. Build a PilotAction:
//        cmdHeading_deg  = activeHeading_deg
//        cmdAltitude_ft  = activeAltitude_ft
//        cmdNavMode      = false       (Autopilot's heading-hold path)
//      Heading-hold and altitude-hold modes are toggled on inside
//      PilotAction::execute() whenever the corresponding cmd is set.
// ---------------------------------------------------------------------------
base::ubf::AbstractAction* BeamMissileBehavior::genAction(
   const base::ubf::AbstractState* const state,
   const double dt)
{
   const auto* ps = dynamic_cast<const PilotState*>(state);
   if (ps == nullptr) {
      // We can't run without ownship + threat data.  Silently no-op so the
      // arbiter falls back to lower-voted behaviors.
      return nullptr;
   }

   const auto& incoming = ps->getIncomingWeapons();
   const auto nearestIt = pickNearest(incoming);
   const bool haveNearest = (nearestIt != incoming.end());

   const bool threatInside = haveNearest
                             && (nearestIt->range_m < triggerRange_m);

   const Phase prev = phase;

   // --- State transitions ----------------------------------------------------
   if (threatInside) {
      // (Re)compute heading and altitude every tick a threat is inside the
      // gate.  This keeps the defensive turn pointing perpendicular to the
      // CURRENT LOS as the geometry evolves.
      activeHeading_deg = computeBeamHeading(nearestIt->losBearing_deg,
                                             beamAngle_deg);
      if (hasHoldAltSlot) {
         activeAltitude_ft = holdAltSlot_ft;
      } else {
         // Capture ownship altitude only at trigger transition; on subsequent
         // defensive ticks keep the same captured altitude (so the turn does
         // not drift up or down with autopilot tracking error).
         if (prev == Phase::CRUISE) {
            activeAltitude_ft = ps->getOwnAltitudeM() / kMetresPerFoot;
         }
      }
      phase            = Phase::DEFENSIVE;
      timeInRecovery_s = 0.0;
   } else if (prev == Phase::DEFENSIVE) {
      // Threat just cleared — start recovery timer, keep last action.
      phase            = Phase::RECOVERING;
      timeInRecovery_s = 0.0;
   } else if (prev == Phase::RECOVERING) {
      timeInRecovery_s += dt;
      if (timeInRecovery_s >= recoverTime_s) {
         phase = Phase::CRUISE;
      }
   }
   // (CRUISE with no threat: phase stays CRUISE, nothing to do.)

   // --- Emit a transition log if the phase changed --------------------------
   if (phase != prev) {
      emitTransition(ps, prev, phase);
   }

   // --- Action selection ----------------------------------------------------
   if (phase == Phase::CRUISE) {
      return nullptr;
   }

   // DEFENSIVE or RECOVERING — emit a PilotAction pushing the cached cmds.
   auto* action = new PilotAction();
   action->setCmdHeadingDeg(activeHeading_deg);
   action->setCmdAltitudeFt(activeAltitude_ft);
   action->setCmdNavMode(false);
   action->setVote(getVote());
   return action;
}

// ---------------------------------------------------------------------------
// Transition emit (T-E36) — route the behavior phase change through the MIXR
// data recorder as REID_BEHAVIOR_STATE.
//
// The recorder (DataRecorder::recordDataImp) packs the sample into a
// pb::BehaviorStateChangeMsg and the WebSocket streamer projects it to a
// `behavior_state` JSON event via streamer::projectBehaviorState(), which is
// what flips the frontend HUD badge CRUISE <-> DEFENSIVE.  This replaces the
// interim std::cout wire-shape T-E35 used before REID_BEHAVIOR_STATE existed.
//
// Sample layout (see dataRecorderTokens.hpp / DataRecord.proto):
//    obj[0] => ownship Player (DataRecorder::genPlayerId fills the numeric id,
//              so the projection's player_id matches the aircraft's
//              player_position track and the per-player badge actually updates)
//    val[0] => to-state code (0=CRUISE, 1=DEFENSIVE, 2=RECOVERING)
//    val[1] => triggerRange (m);  val[2] => beamAngle (deg)
//    val[3] => nearest incoming slant range (m), or <0 when no threat tracked
//              (kept NaN-free so the JSON projection stays valid)
//
// We reach the recorder via the canonical getWorldModel()->getDataRecorder()
// path.  getOwnship() is const and the recorder only READS the player
// (genPlayerId takes a const Player*), so the const_cast that yields the
// non-const recorder pointer the SAMPLE macro requires is safe.
// ---------------------------------------------------------------------------
void BeamMissileBehavior::emitTransition(const PilotState* const ps,
                                         Phase from, Phase to)
{
   (void)from;  // 'from' is implicit in the recorder stream (the prior to_state)

   if (ps == nullptr) return;
   const Player* const own = ps->getOwnship();
   if (own == nullptr) return;
   WorldModel* const wm = const_cast<Player*>(own)->getWorldModel();
   if (wm == nullptr) return;

   double nearestRange = -1.0;  // <0 sentinel: no incoming weapon tracked
   const auto& incoming = ps->getIncomingWeapons();
   const auto it = pickNearest(incoming);
   if (it != incoming.end()) nearestRange = it->range_m;

   const double toCode = static_cast<double>(static_cast<int>(to));

   BEGIN_RECORD_DATA_SAMPLE( wm->getDataRecorder(), REID_BEHAVIOR_STATE )
      SAMPLE_1_OBJECT( own )
      SAMPLE_4_VALUES( toCode, triggerRange_m, beamAngle_deg, nearestRange )
   END_RECORD_DATA_SAMPLE()
}

// ---------------------------------------------------------------------------
// Slot setters
// ---------------------------------------------------------------------------

// triggerRange ---------------------------------------------------------------
bool BeamMissileBehavior::setSlotTriggerRange(const base::Length* const x)
{
   if (x == nullptr) return false;
   const double m = x->getValueInMeters();
   if (m <= 0.0) return false;
   triggerRange_m = m;
   return true;
}

bool BeamMissileBehavior::setSlotTriggerRange(const base::Number* const x)
{
   if (x == nullptr) return false;
   const double m = x->asDouble();
   if (m <= 0.0) return false;
   triggerRange_m = m;
   return true;
}

// beamAngle ------------------------------------------------------------------
bool BeamMissileBehavior::setSlotBeamAngle(const base::Angle* const x)
{
   if (x == nullptr) return false;
   beamAngle_deg = x->getValueInDegrees();
   return true;
}

bool BeamMissileBehavior::setSlotBeamAngle(const base::Number* const x)
{
   if (x == nullptr) return false;
   beamAngle_deg = x->asDouble();
   return true;
}

// recoverTime ----------------------------------------------------------------
bool BeamMissileBehavior::setSlotRecoverTime(const base::Time* const x)
{
   if (x == nullptr) return false;
   const double s = x->getValueInSeconds();
   if (s < 0.0) return false;
   recoverTime_s = s;
   return true;
}

bool BeamMissileBehavior::setSlotRecoverTime(const base::Number* const x)
{
   if (x == nullptr) return false;
   const double s = x->asDouble();
   if (s < 0.0) return false;
   recoverTime_s = s;
   return true;
}

// holdAlt --------------------------------------------------------------------
bool BeamMissileBehavior::setSlotHoldAlt(const base::Length* const x)
{
   if (x == nullptr) return false;
   holdAltSlot_ft = x->getValueInFeet();
   hasHoldAltSlot = true;
   return true;
}

bool BeamMissileBehavior::setSlotHoldAlt(const base::Number* const x)
{
   if (x == nullptr) return false;
   holdAltSlot_ft = x->asDouble();
   hasHoldAltSlot = true;
   return true;
}

}
}
