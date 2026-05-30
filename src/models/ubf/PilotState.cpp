
#include "mixr/models/ubf/PilotState.hpp"

#include "mixr/models/player/Player.hpp"
#include "mixr/models/player/weapon/IWeapon.hpp"
#include "mixr/models/WorldModel.hpp"

#include "mixr/base/List.hpp"
#include "mixr/base/Pair.hpp"
#include "mixr/base/PairStream.hpp"

#include <cmath>
#include <limits>

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(PilotState, "PilotState")
EMPTY_SLOTTABLE(PilotState)

PilotState::PilotState()
{
   STANDARD_CONSTRUCTOR()
}

void PilotState::copyData(const PilotState& org, const bool)
{
   BaseClass::copyData(org);
   ownship          = org.ownship;
   ownLat_deg       = org.ownLat_deg;
   ownLon_deg       = org.ownLon_deg;
   ownAlt_m         = org.ownAlt_m;
   ownHeading_deg   = org.ownHeading_deg;
   ownVelNED_mps    = org.ownVelNED_mps;
   ownSpeed_mps     = org.ownSpeed_mps;
   incomingWeapons  = org.incomingWeapons;
}

void PilotState::deleteData()
{
   ownship = nullptr;
   incomingWeapons.clear();
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

constexpr double kPi      = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kEarthR  = 6371008.8;   // mean Earth radius (m); haversine
                                         // matches streamer/json_projection.cpp

// Great-circle range (m) between two geodetic lat/lon points.
double haversineRangeM(double lat1, double lon1, double lat2, double lon2)
{
   const double dlat = (lat2 - lat1) * kDeg2Rad;
   const double dlon = (lon2 - lon1) * kDeg2Rad;
   const double a = std::sin(dlat * 0.5) * std::sin(dlat * 0.5)
                  + std::cos(lat1 * kDeg2Rad) * std::cos(lat2 * kDeg2Rad)
                    * std::sin(dlon * 0.5) * std::sin(dlon * 0.5);
   return 2.0 * kEarthR * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// Initial bearing (degrees true, 0..360) from point 1 to point 2.
double geodeticBearingDeg(double lat1, double lon1, double lat2, double lon2)
{
   const double lat1r = lat1 * kDeg2Rad;
   const double lat2r = lat2 * kDeg2Rad;
   const double dlon  = (lon2 - lon1) * kDeg2Rad;
   const double y     = std::sin(dlon) * std::cos(lat2r);
   const double x     = std::cos(lat1r) * std::sin(lat2r)
                      - std::sin(lat1r) * std::cos(lat2r) * std::cos(dlon);
   if (std::abs(x) < 1e-12 && std::abs(y) < 1e-12) return 0.0;
   double b = std::atan2(y, x) * kRad2Deg;
   if (b < 0.0) b += 360.0;
   return b;
}

} // namespace

// ---------------------------------------------------------------------------
// updateState — pulled by Agent::controller() each BG tick.
//
// Algorithm (sprint-11 plan §4 — world-model walk):
//   1. Resolve actor -> ownship Player.
//   2. Snapshot ownship kinematics (lat/lon/alt/heading/velocity).
//   3. Walk WorldModel::getPlayers().  For each IWeapon whose
//      getTargetPlayer() == ownship, compute range, range-rate, LOS bearing,
//      closing time.
//
// Cost is O(N) in the active player list; N is small in our scenarios.
// We deliberately do NOT cache between ticks because weapon Players can come
// and go each frame and the cost is negligible at sprint-11 scales.
// ---------------------------------------------------------------------------
void PilotState::updateState(const base::Component* const actor)
{
   // Default: clear, in case the actor is null/unrecognised this tick.
   ownship = nullptr;
   incomingWeapons.clear();

   const auto p = dynamic_cast<const Player*>(actor);
   if (p == nullptr) {
      // Allow base-class machinery (child-state propagation) to still run.
      BaseClass::updateState(actor);
      return;
   }
   ownship = p;

   // ---- ownship kinematics -----------------------------------------------
   ownLat_deg     = p->getLatitude();
   ownLon_deg     = p->getLongitude();
   ownAlt_m       = p->getAltitudeM();
   ownHeading_deg = p->getHeadingD();
   ownVelNED_mps  = p->getVelocity();
   ownSpeed_mps   = ownVelNED_mps.length();

   // ---- threat scan via WorldModel ---------------------------------------
   // Player::getWorldModel() is non-const because it may lazy-resolve the
   // container pointer; cast away const on `p` only for that resolution.
   auto* mutP = const_cast<Player*>(p);
   WorldModel* wm = mutP->getWorldModel();
   if (wm == nullptr) {
      // No simulation context yet; nothing else we can do.
      BaseClass::updateState(actor);
      return;
   }

   base::PairStream* plist = wm->getPlayers();
   if (plist == nullptr) {
      BaseClass::updateState(actor);
      return;
   }

   const base::List::Item* item = plist->getFirstItem();
   while (item != nullptr) {
      const auto pair = static_cast<const base::Pair*>(item->getValue());
      item = item->getNext();
      if (pair == nullptr) continue;
      const auto obj = const_cast<base::Object*>(pair->object());
      if (obj == nullptr) continue;

      // Only weapons are candidates for the MAW list.
      auto* wpn = dynamic_cast<IWeapon*>(obj);
      if (wpn == nullptr) continue;
      if (!wpn->isActive()) continue;

      // Filter: only weapons whose target is ownship.
      const Player* tgt = wpn->getTargetPlayer();
      if (tgt != p) continue;

      // Compute the geometry. Use haversine + great-circle bearing so the
      // streamer projection and the state observer agree on what "range to
      // missile" means for a given frame.
      const double wLat = wpn->getLatitude();
      const double wLon = wpn->getLongitude();
      const double wAlt = wpn->getAltitudeM();

      const double horizRange = haversineRangeM(ownLat_deg, ownLon_deg, wLat, wLon);
      const double dAlt       = wAlt - ownAlt_m;
      const double slantRange = std::sqrt(horizRange * horizRange + dAlt * dAlt);

      // Range-rate: project the relative velocity onto the unit LOS vector
      // (ownship -> weapon). Negative => closing.
      const base::Vec3d losNED = wpn->getVelocity() - ownVelNED_mps;
      // For range-rate, we need d|r|/dt = (r . v) / |r|. Build r in NED by
      // approximating dLat -> meters via Earth radius * dLat_rad and dLon ->
      // meters via Earth radius * cos(lat) * dLon_rad — same flat-earth
      // approximation MIXR's gaming-area NED projection uses internally for
      // tens of nm scales.
      const double dN_m = (wLat - ownLat_deg) * kDeg2Rad * kEarthR;
      const double dE_m = (wLon - ownLon_deg) * kDeg2Rad * kEarthR
                          * std::cos(ownLat_deg * kDeg2Rad);
      const double dD_m = -(wAlt - ownAlt_m);   // NED: down positive
      const base::Vec3d rNED(dN_m, dE_m, dD_m);
      const double rLen = rNED.length();
      double rangeRate = 0.0;
      if (rLen > 1e-3) {
         // d|r|/dt = (r . (v_target - v_self)) / |r|. losNED already encodes
         // the relative velocity.
         rangeRate = (rNED * losNED) / rLen;
      }

      double closingTime = std::numeric_limits<double>::infinity();
      if (rangeRate < 0.0) {
         closingTime = slantRange / -rangeRate;
      }

      IncomingWeapon entry;
      entry.weapon         = wpn;
      entry.range_m        = slantRange;
      entry.rangeRate_mps  = rangeRate;
      entry.losBearing_deg = geodeticBearingDeg(ownLat_deg, ownLon_deg, wLat, wLon);
      entry.closingTime_s  = closingTime;
      incomingWeapons.push_back(entry);
   }

   plist->unref();

   // Delegate to base so any child AbstractState components also get updated.
   BaseClass::updateState(actor);
}

// ---------------------------------------------------------------------------
// Test-only seams
// ---------------------------------------------------------------------------
void PilotState::setOwnKinematicsForTest(double lat_deg, double lon_deg,
                                         double alt_m, double heading_deg,
                                         const base::Vec3d& vel_ned_mps)
{
   ownLat_deg     = lat_deg;
   ownLon_deg     = lon_deg;
   ownAlt_m       = alt_m;
   ownHeading_deg = heading_deg;
   ownVelNED_mps  = vel_ned_mps;
   ownSpeed_mps   = vel_ned_mps.length();
}

void PilotState::clearIncomingForTest()
{
   incomingWeapons.clear();
}

void PilotState::addIncomingForTest(const IncomingWeapon& w)
{
   incomingWeapons.push_back(w);
}

}
}
