
#ifndef __mixr_models_common_Missile_HPP__
#define __mixr_models_common_Missile_HPP__

#include "mixr/models/player/weapon/IWeapon.hpp"

#include <cstdint>   // T-E37: uint64_t for the proximity-fuze RNG child id

namespace mixr {
namespace base { class Length; class Number; }
namespace models {
class Player;
class Track;

//------------------------------------------------------------------------------
// Class: Missile
// Description: Base class for missiles; with a simple, default missile model
//
// Factory name: Missile
// Slots:
//   minSpeed          <Number>  ! Minimum Velocity (m/s)  (default: 0.0)
//   maxSpeed          <Number>  ! Maximum Velocity (m/s)  (default: 800.0)
//   speedMaxG         <Number>  ! Velocity we reach max G (default: 800.0)
//   maxg              <Number>  ! Max G's (at "speedMaxG" or above) (default: 4.0)
//   maxAccel          <Number>  ! Maximum Acceleration (m/s/s) (default: 50.0)
//   cmdPitch          <Number>  ! Command Pitch (rad) (default: 0.0)
//   cmdHeading        <Number>  ! Command Heading (rad) (default: 0.0)
//   cmdSpeed          <Number>  ! Command speed (m/s) (default: 0.0)
//   maxEffectiveRange <Length>  ! Max effective range; the proximity-fuze
//                               ! effectiveness ramps from 1.0 to 0.0 between
//                               ! 70% and 100% of this distance flown.
//                               ! Number form is meters.
//                               ! (default: 60 km, AIM-120D-class) (T-E37)
//
// Distance-degraded proximity-fuze Pk (ADR-007 / T-E37):
//   weaponDynamics() accumulates cumulative path length (flownDistance) every
//   tick.  At proximity-fuze evaluation (weaponGuidance, closest-point pass),
//   the warhead/fuze effectiveness is computed from how far the missile has
//   flown relative to maxEffectiveRange:
//
//       eff = clamp(1 - max(0, flown - 0.7*maxRange) / (0.3*maxRange), 0, 1)
//
//   A uniform U(0,1) is then drawn from a per-missile sub-stream obtained via
//   Simulation::split(child_id); the missile detonates as ENTITY_IMPACT only
//   when U < eff, otherwise it degrades to a DETONATION (result=5) miss.  The
//   draw is deterministic in (master seed, released-weapon id) so replications
//   are reproducible while still introducing seed-driven Pk variance.
//
//------------------------------------------------------------------------------
class Missile : public IWeapon
{
    DECLARE_SUBCLASS(Missile, IWeapon)

public:
    Missile();

    // get functions
    double getVpMin() const         { return vpMin; }
    double getVpMax() const         { return vpMax; }
    double getVpMaxG() const        { return vpMaxG; }
    double getMaxG() const          { return maxG; }
    double getMaxAccel() const      { return maxAccel; }
    double getMinAccel() const      { return 0.0; }

    // T-E37: distance-degraded proximity-fuze ---------------------------------
    double getFlownDistance() const     { return flownDistance; }     // cumulative path length flown since launch (m)
    double getMaxEffectiveRange() const { return maxEffectiveRange; } // (m)

    // AIM-120D-class default max effective range (m), used when the EDL slot
    // is omitted.  Exposed as a static constexpr so unit tests can pin the
    // canonical value WITHOUT instantiating the class (instantiation needs
    // libmixr_base.a for the Object/Slottable machinery, which the standalone
    // header-only test path deliberately avoids).
    static constexpr double kDefaultMaxEffectiveRangeM {60000.0};

    // Salt that namespaces this missile's proximity-fuze RNG sub-stream so a
    // fuze draw can never alias another root-level Simulation::split()
    // consumer that happens to key on the same released-weapon id.  "FUZE" in
    // the high 32 bits; the released-weapon id occupies the low 32.
    static constexpr std::uint64_t kFuzeRngChildSalt {0x46555A4500000000ULL};

    // Deterministic RNG child id for this missile's single proximity-fuze
    // roll (ADR-007).  child_id = salt ^ released-weapon id.  Header-only so a
    // unit test can reproduce the production draw byte-for-byte against a
    // PcgRng seeded with the same master.
    static std::uint64_t fuzeRngChildId(int weaponId)
    {
        return kFuzeRngChildSalt ^ static_cast<std::uint64_t>(static_cast<std::uint32_t>(weaponId));
    }

    // Pure proximity-fuze effectiveness model (T-E37):
    //   eff = clamp(1 - max(0, flown - 0.7*maxRange) / (0.3*maxRange), 0, 1)
    // -> 1.0 anywhere inside 70% of maxRange, ramping linearly to 0.0 at 100%.
    // Header-only/inline so unit tests can include this header without linking
    // libmixr_*.a (same pattern as BeamMissileBehavior::computeBeamHeading).
    static double fuzeEffectiveness(double flownDistance_m, double maxEffectiveRange_m);

    // set functions
    bool setVpMin(const double v);
    bool setVpMax(const double v);
    bool setVpMaxG(const double v);
    bool setMaxG(const double v);
    bool setMaxAccel(const double v);
    bool setMaxEffectiveRange(const double v);   // T-E37 (meters)

    const char* getDescription() const override;
    const char* getNickname() const override;
    int getCategory() const override;
    void atReleaseInit() override;

    virtual void setCmdPitchD(const double x)  { cmdPitch   = x * static_cast<double>(base::angle::D2RCC); }
    virtual void setCmdHdgD(const double x)    { cmdHeading = x * static_cast<double>(base::angle::D2RCC); }

    bool setTargetTrack(Track* const trk, const bool posTrkEnb) override;
    bool setTargetPlayer(Player* const tgt, const bool posTrkEnb) override;

    bool event(const int event, base::Object* const obj = nullptr) override;
    void reset() override;

protected:
   virtual bool setSlotVpMin(const base::Number* const msg);
   virtual bool setSlotVpMax(const base::Number* const msg);
   virtual bool setSlotVpMaxG(const base::Number* const msg);
   virtual bool setSlotMaxG(const base::Number* const msg);
   virtual bool setSlotMaxAccel(const base::Number* const msg);
   virtual bool setSlotMaxEffectiveRange(const base::Length* const msg);   // T-E37
   virtual bool setSlotMaxEffectiveRange(const base::Number* const msg);   // T-E37 (meters)
   virtual bool setSlotCmdPitch(const base::Number* const msg);
   virtual bool setSlotCmdHeading(const base::Number* const msg);
   virtual bool setSlotCmdVelocity(const base::Number* const msg);

   // Weapon interface
   void weaponGuidance(const double dt) override;
   void weaponDynamics(const double dt) override;

private:
    virtual bool calculateVectors(const Player* const tgt, const Track* const trk, base::Vec3d* const los, base::Vec3d* const vel, base::Vec3d* const posx) const;

   // ---
   // Default guidance & dynamics parameters
   // ---
   double trng {};          // target range             (m)
   double trngT {};         // target range (truth)     (m)
   double trdot {};         // target range rate        (m/s)
   double trdotT {};        // target range rate (truth)(m/s)
   double cmdPitch {};      // Commanded Pitch          (rad)
   double cmdHeading {};    // Commanded Heading        (rad)
   double cmdVelocity {};   // Commanded speed          (m/s)
   double vpMin {};         // Minimum Velocity         (m/s)
   double vpMax {};         // Maximum Velocity         (m/s)
   double maxAccel {};      // Max longitudual acceleration ((f/s)/s)
   double maxG {};          // Max lateral G's (pitch/yaw)  (gees)
   double vpMaxG {};        // Velocity for Max G's     (gees)

   // T-E37: distance-degraded proximity-fuze state
   double maxEffectiveRange {kDefaultMaxEffectiveRangeM}; // Max effective range (m); fuze degradation knee at 70%
   double flownDistance {};                               // Cumulative path length flown since launch (m)
};

// ---------------------------------------------------------------------------
// Inline pure helper (header-only so the T-E37 unit test can include this
// header without linking any libmixr_*.a).
// ---------------------------------------------------------------------------
inline double Missile::fuzeEffectiveness(double flownDistance_m, double maxEffectiveRange_m)
{
   // A non-positive max effective range disables the degradation entirely
   // (the missile is always fully effective) — keeps the model safe if an
   // EDL author zeroes the slot.
   if (maxEffectiveRange_m <= 0.0) return 1.0;

   const double knee {0.7 * maxEffectiveRange_m};   // start of degradation
   const double span {0.3 * maxEffectiveRange_m};   // 70% -> 100% ramp width
   const double over {flownDistance_m - knee};
   const double excess {over > 0.0 ? over : 0.0};   // max(0, flown - 0.7*max)
   double eff {1.0 - (excess / span)};
   if (eff < 0.0) eff = 0.0;
   if (eff > 1.0) eff = 1.0;
   return eff;
}

}
}

#endif
