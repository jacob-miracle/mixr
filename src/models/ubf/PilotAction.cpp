
#include "mixr/models/ubf/PilotAction.hpp"

#include "mixr/models/player/Player.hpp"
#include "mixr/models/system/Autopilot.hpp"
#include "mixr/models/system/Pilot.hpp"

#include "mixr/base/Component.hpp"
#include "mixr/base/Pair.hpp"

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(PilotAction, "PilotAction")
EMPTY_SLOTTABLE(PilotAction)

PilotAction::PilotAction()
{
   STANDARD_CONSTRUCTOR()
}

void PilotAction::copyData(const PilotAction& org, const bool)
{
   BaseClass::copyData(org);
   cmdHeading_deg  = org.cmdHeading_deg;
   cmdAltitude_ft  = org.cmdAltitude_ft;
   cmdVelocity_kts = org.cmdVelocity_kts;
   cmdNavMode      = org.cmdNavMode;
   hasCmdHeading   = org.hasCmdHeading;
   hasCmdAltitude  = org.hasCmdAltitude;
   hasCmdVelocity  = org.hasCmdVelocity;
   hasCmdNavMode   = org.hasCmdNavMode;
}

void PilotAction::deleteData()
{
}

// ---------------------------------------------------------------------------
// Resolve the Autopilot the action should drive.
//
//   1. If actor IS an Autopilot, use it directly (unit-test convenience).
//   2. If actor is a Player, use its top-level pilot if that pilot is an
//      Autopilot (the common production path).
//   3. Otherwise, depth-first search the actor's component subtree for any
//      Autopilot child.
//
// We do NOT silently substitute the base Pilot type; only an Autopilot has
// the setCommandedHeadingD/Altitude/Velocity setters this action commands.
// ---------------------------------------------------------------------------
Autopilot* PilotAction::resolveAutopilot(base::Component* actor) const
{
   if (actor == nullptr) return nullptr;

   if (auto ap = dynamic_cast<Autopilot*>(actor)) {
      return ap;
   }
   if (auto player = dynamic_cast<Player*>(actor)) {
      if (auto ap = dynamic_cast<Autopilot*>(player->getPilot())) {
         return ap;
      }
   }
   if (auto pair = actor->findByType(typeid(Autopilot))) {
      return dynamic_cast<Autopilot*>(pair->object());
   }
   return nullptr;
}

// ---------------------------------------------------------------------------
// execute — push every set-* field into the Autopilot.
//
// Returns true if AT LEAST one setter was driven (i.e. the action was not a
// no-op). A pure no-op action returns false so the caller can avoid the
// behavior-state recorder emit until a behavior actually issues a command.
// ---------------------------------------------------------------------------
bool PilotAction::execute(base::Component* actor)
{
   Autopilot* ap = resolveAutopilot(actor);
   if (ap == nullptr) return false;

   bool drove = false;
   if (hasCmdNavMode) {
      ap->setNavMode(cmdNavMode);
      drove = true;
   }
   if (hasCmdHeading) {
      ap->setHeadingHoldMode(true);
      ap->setCommandedHeadingD(cmdHeading_deg);
      drove = true;
   }
   if (hasCmdAltitude) {
      ap->setAltitudeHoldMode(true);
      ap->setCommandedAltitudeFt(cmdAltitude_ft);
      drove = true;
   }
   if (hasCmdVelocity) {
      ap->setVelocityHoldMode(true);
      ap->setCommandedVelocityKts(cmdVelocity_kts);
      drove = true;
   }
   return drove;
}

}
}
