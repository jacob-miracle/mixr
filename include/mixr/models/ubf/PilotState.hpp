
#ifndef __mixr_models_ubf_PilotState_HPP__
#define __mixr_models_ubf_PilotState_HPP__

#include "mixr/base/ubf/AbstractState.hpp"
#include "mixr/base/osg/Vec3d"

#include <cstdint>
#include <vector>

namespace mixr {
namespace base { class Component; }
namespace models {
class Player;
class IWeapon;

//------------------------------------------------------------------------------
// Class: PilotState
//
// Description: UBF state for a fighter pilot.  Holds own-ship kinematics and
//              the per-tick list of incoming weapons targeting us (Missile
//              Approach Warning, derived from a WorldModel walk per the
//              sprint-11 probe finding that MIXR's RWR does not emit a
//              discrete launch event).
//
// Factory name: PilotState
//
// Slots: none (data is populated each tick from the actor Player).
//
// Threading:
//   updateState() is invoked from the UBF Agent::controller() on the
//   background thread (Agent::updateData()) once per BG tick. It does NOT
//   mutate any Player state — it only reads.
//
// References:
//   docs/sprint-11-planning/final-plan.md §1 (RWR + drag probe)
//   docs/sprint-11-planning/final-plan.md §4 (defensive = beam/cold abort)
//------------------------------------------------------------------------------
class PilotState final : public base::ubf::AbstractState
{
   DECLARE_SUBCLASS(PilotState, base::ubf::AbstractState)

public:
   // ---------------------------------------------------------------------------
   // IncomingWeapon — one entry per IWeapon in WorldModel currently targeting
   // ownship. Range, range-rate, line-of-sight bearing and predicted closing
   // time are precomputed here so behaviors can stay branch-free.
   // ---------------------------------------------------------------------------
   struct IncomingWeapon {
      IWeapon* weapon{};            // not owned; lifetime managed by WorldModel
      double   range_m{};           // slant range, metres
      double   rangeRate_mps{};     // d(range)/dt, m/s; negative => closing
      double   losBearing_deg{};    // initial great-circle bearing from ownship
                                    // to weapon, degrees true (0=N, 90=E)
      double   closingTime_s{};     // range_m / |rangeRate_mps| if closing, else +inf
   };

public:
   PilotState();

   // Refresh own-ship + threat tables from the actor Player. Called by the
   // Agent::controller() each BG tick via base::ubf::AbstractState::updateState().
   void updateState(const base::Component* const actor) override;

   // -------- Own-ship kinematics (populated per tick) ----------------------
   double getOwnLatitudeDeg() const   { return ownLat_deg; }
   double getOwnLongitudeDeg() const  { return ownLon_deg; }
   double getOwnAltitudeM() const     { return ownAlt_m; }
   double getOwnHeadingDeg() const    { return ownHeading_deg; }
   const base::Vec3d& getOwnVelocityNED() const { return ownVelNED_mps; }
   double getOwnSpeedMps() const      { return ownSpeed_mps; }

   const Player* getOwnship() const   { return ownship; }

   // -------- Threat list ----------------------------------------------------
   const std::vector<IncomingWeapon>& getIncomingWeapons() const
   {
      return incomingWeapons;
   }
   std::size_t getIncomingCount() const { return incomingWeapons.size(); }

   // ---------------------------------------------------------------------------
   // Test seam — let unit tests stub the state without a live WorldModel.
   // ---------------------------------------------------------------------------
   void setOwnKinematicsForTest(double lat_deg, double lon_deg, double alt_m,
                                double heading_deg, const base::Vec3d& vel_ned_mps);
   void clearIncomingForTest();
   void addIncomingForTest(const IncomingWeapon& w);

private:
   const Player* ownship{};      // resolved each tick from the actor; not owned

   double ownLat_deg{};
   double ownLon_deg{};
   double ownAlt_m{};
   double ownHeading_deg{};
   base::Vec3d ownVelNED_mps;
   double ownSpeed_mps{};

   std::vector<IncomingWeapon> incomingWeapons;
};

}
}

#endif
