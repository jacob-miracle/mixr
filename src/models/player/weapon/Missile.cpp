
#include "mixr/models/player/weapon/Missile.hpp"

#include "mixr/models/Track.hpp"
#include "mixr/models/system/trackmanager/TrackManager.hpp"
#include "mixr/models/WorldModel.hpp"

#include "mixr/simulation/IDataRecorder.hpp"

#include "mixr/base/List.hpp"
#include "mixr/base/numeric/Number.hpp"
#include "mixr/base/PairStream.hpp"
#include "mixr/base/osg/Matrixd"

#include "mixr/base/random/IRng.hpp"      // T-E37: per-missile proximity-fuze RNG sub-stream (ADR-007)
#include "mixr/base/units/lengths.hpp"    // T-E37: maxEffectiveRange as a base::Length slot

#include <cmath>
#include <cstdint>

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(Missile, "Missile")

BEGIN_SLOTTABLE(Missile)
"minSpeed",          //  1: Minimum Velocity           (m/s)
"maxSpeed",          //  2: Maximum Velocity           (m/s)
"speedMaxG",         //  3: Velocity we reach max G
"maxg",              //  4: Max G's (at "speedMaxG" or above)
"maxAccel",          //  5: Maximum Acceleration       (m/s/s)
"cmdPitch",          //  6: Command Pitch              (rad)
"cmdHeading",        //  7: Command Heading            (rad)
"cmdSpeed",          //  8: Command speed              (m/s)
"maxEffectiveRange", //  9: Max effective range (Length or meters) (T-E37)
END_SLOTTABLE(Missile)

BEGIN_SLOT_MAP(Missile)
   ON_SLOT(1, setSlotVpMin, base::Number)
   ON_SLOT(2, setSlotVpMax, base::Number)
   ON_SLOT(3, setSlotVpMaxG, base::Number)
   ON_SLOT(4, setSlotMaxG, base::Number)
   ON_SLOT(5, setSlotMaxAccel, base::Number)
   ON_SLOT(6, setSlotCmdPitch, base::Number)
   ON_SLOT(7, setSlotCmdHeading, base::Number)
   ON_SLOT(8, setSlotCmdVelocity, base::Number)
   ON_SLOT(9, setSlotMaxEffectiveRange, base::Length)    // T-E37: ( KiloMeters N )
   ON_SLOT(9, setSlotMaxEffectiveRange, base::Number)    // T-E37: meters
END_SLOT_MAP()

BEGIN_EVENT_HANDLER(Missile)
END_EVENT_HANDLER()

int Missile::getCategory() const               { return (MISSILE | GUIDED); }
const char* Missile::getDescription() const    { return "AAM"; }
const char* Missile::getNickname() const       { return "AAM"; }

Missile::Missile()
{
   STANDARD_CONSTRUCTOR()

   static base::String generic("GenericMissile");
   setType_old(&generic);
   setType("GenericMissile");

   setMaxTOF(60.0);
   setLethalRange(30.0f);
   setMaxBurstRng(150.0f);
   setTSG(1.0);
   setSOBT(0.0f);
   setEOBT(60.0f);

   setVpMin(0.0);
   setVpMax(800.0f);
   setVpMaxG(800.0f);
   setMaxG(4.0);
   setMaxAccel(50.0);
}

void Missile::copyData(const Missile& org, const bool)
{
   BaseClass::copyData(org);

   trng = org.trng;
   trngT = org.trngT;
   trdot = org.trdot;
   trdotT = org.trdotT;
   vpMin = org.vpMin;
   vpMax = org.vpMax;
   vpMaxG = org.vpMaxG;
   maxG = org.maxG;
   maxAccel = org.maxAccel;
   cmdPitch = org.cmdPitch;
   cmdHeading = org.cmdHeading;
   cmdVelocity = org.cmdVelocity;
   maxEffectiveRange = org.maxEffectiveRange;   // T-E37 (configured value carries to the flyout clone)
   flownDistance = org.flownDistance;           // T-E37 (reset() re-zeros this for the flyout)
}

void Missile::deleteData()
{
}

//------------------------------------------------------------------------------
// reset() -- Reset vehicle dynamics
//------------------------------------------------------------------------------
void Missile::reset()
{
   BaseClass::reset();

   // T-E37: each flyout starts its cumulative-distance integral at zero.
   // release() clones the initial weapon then calls reset() on the flyout, so
   // this is the canonical place to (re)zero the per-flight accumulator.  It
   // also keeps the determinism contract clean: a reset+rerun with the same
   // master seed reproduces the same flownDistance -> effectiveness -> roll.
   flownDistance = 0.0;
}

//------------------------------------------------------------------------------
// atReleaseInit() -- Init weapon data at release
//------------------------------------------------------------------------------
void Missile::atReleaseInit()
{
   // First the base class will setup the initial conditions
   BaseClass::atReleaseInit();

   if (getDynamicsModel() == nullptr) {
      // set initial commands
      cmdPitch = static_cast<double>(getPitch());
      cmdHeading = static_cast<double>(getHeading());
      cmdVelocity = vpMax;

      if (getTargetTrack() != nullptr) {
         // Set initial range and range dot
         base::Vec3d los = getTargetTrack()->getPosition();
         trng = los.length();
         trngT = trng;
      }
      else if (getTargetPlayer() != nullptr) {
         // Set initial range and range dot
         base::Vec3d los = getTargetPosition();
         trng = los.length();
         trngT = trng;
      }
      else {
         trng = 0.0;
      }

      // Range dot
      trdot = 0.0;
      trdotT = 0.0;
   }
}

//------------------------------------------------------------------------------
// calculateVectors() --
//------------------------------------------------------------------------------
bool Missile::calculateVectors(const Player* const tgt, const Track* const trk, base::Vec3d* const los, base::Vec3d* const vel, base::Vec3d* const posx) const
{
   if (trk != nullptr) {
      //los = trk->getPosition();
      //vel = trk->getVelocity();
      const Player* tgt0 = trk->getTarget();
      base::Vec3d p0 = getPosition();
      if (los != nullptr) *los = tgt0->getPosition() - p0;
      if (vel != nullptr) *vel = tgt0->getVelocity();
      if (posx != nullptr) *posx = tgt0->getPosition();
   }
   else if (tgt != nullptr) {
      base::Vec3d p0 = getPosition();
      if (los != nullptr) *los = tgt->getPosition() - p0;
      if (vel != nullptr) *vel = tgt->getVelocity();
      if (posx != nullptr) *posx = tgt->getPosition();
   }
   else {
      // no guidance until we have a target
      return false;
   }

   return true;
}

//------------------------------------------------------------------------------
// Slot functions
//------------------------------------------------------------------------------
bool Missile::setSlotVpMin(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      setVpMin(msg->asDouble());
      ok = true;
   }
   return ok;
}

bool Missile::setSlotVpMax(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      setVpMax(msg->asDouble());
      ok = true;
   }
   return ok;
}

bool Missile::setSlotVpMaxG(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      setVpMaxG(msg->asDouble());
      ok = true;
   }
   return ok;
}

bool Missile::setSlotMaxG(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      setMaxG(msg->asDouble());
      ok = true;
   }
   return ok;
}

bool Missile::setSlotMaxAccel(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      setMaxAccel(msg->asDouble());
      ok = true;
   }
   return ok;
}

bool Missile::setSlotCmdPitch(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      cmdPitch = msg->asDouble();
      ok = true;
   }
   return ok;
}

bool Missile::setSlotCmdHeading(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      cmdHeading = msg->asDouble();
      ok = true;
   }
   return ok;
}

bool Missile::setSlotCmdVelocity(const base::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      cmdVelocity = msg->asDouble();
      ok = true;
   }
   return ok;
}


// setTargetPlayer() -- sets a pointer to the target player
bool Missile::setTargetPlayer(Player* const tgt, const bool pt)
{
   // if our tgt has changed, reset ground truth vals for weaponGuidance's fuzing logic
   if (tgt != nullptr && tgt != getTargetPlayer()) {
      trngT = (tgt->getPosition()-getPosition()).length();
      trdotT=0.0;
   }
   return BaseClass::setTargetPlayer(tgt, pt);
}

// setTargetTrack() -- sets a pointer to the target track
bool Missile::setTargetTrack(Track* const trk, const bool pt)
{
   // if our track has changed, reset ground truth vals for weaponGuidance's fuzing logic
   if (trk != nullptr && trk != getTargetTrack()) {
      trngT = (trk->getPosition()).length();
      trdotT = 0.0;
   }
   return BaseClass::setTargetTrack(trk, pt);
}

//------------------------------------------------------------------------------
// weaponGuidance() -- default guidance; using Robot Aircraft (RAC) guidance
//------------------------------------------------------------------------------
void Missile::weaponGuidance(const double dt)
{
   // ---
   // Control velocity:  During burn time, accel to max velocity,
   //  after burn time, deaccelerate to min velocity.
   // ---
   if (isEngineBurnEnabled()) cmdVelocity = vpMax;
   else cmdVelocity = vpMin;

   // ---
   // If the target's already dead,
   //    then don't go away mad, just go away.
   // ---
   const Player* tgt = getTargetPlayer();
   const Track* trk = getTargetTrack();
   if (trk != nullptr) tgt = trk->getTarget();

   if (tgt != nullptr && !tgt->isActive()) return;

   base::Vec3d los; // Target Line of Sight
   base::Vec3d vel; // Target velocity

   // ---
   // Basic guidance
   // ---
   {
      // ---
      // Get position and velocity vectors from the target/track
      // ---
      base::Vec3d posx;
      calculateVectors(tgt, trk, &los, &vel, &posx);

      // compute range to target
      const double trng0 = trng;
      trng = los.length();

      // compute range rate,
      //double trdot0 = trdot;
      if (dt > 0)
         trdot = (trng - trng0)/dt;
      else
         trdot = 0.0;

      // Target total velocity
      const double totalVel = vel.length();

      // compute target velocity parallel to LOS,
      const double vtplos = (los * vel/trng);

      // ---
      // guidance - fly to intercept point
      // ---

      // if we have guidance ...
      if ( isGuidanceEnabled() && trng > 0) {

         // get missile velocity (must be faster than target),
         double v = vpMax;
         if (v < totalVel) v = totalVel + 1;

         // compute target velocity normal to LOS squared,
         const double tgtVp = totalVel;
         const double vtnlos2 = tgtVp*tgtVp - vtplos*vtplos;

         // and compute missile velocity parallex to LOS.
         const double vmplos = std::sqrt( v*v - vtnlos2 );

         // Now, use both velocities parallel to LOS to compute
         //  closure rate.
         const double vclos = vmplos - vtplos;

         // Use closure rate and range to compute time to intercept.
         double dt1 = 0;
         if (vclos > 0) dt1 = trng/vclos;

         // Use time to intercept to extrapolate target position.
         base::Vec3d p1 = (los + (vel * dt1));

         // Compute missile commanded heading and
         cmdHeading = std::atan2(p1.y(),p1.x());

         // commanded pitch.
         const double grng = std::sqrt(p1.x()*p1.x() + p1.y()*p1.y());
         cmdPitch = -std::atan2(p1.z(),grng);

      }
   }

   // ---
   // fuzing logic  (let's see if we've scored a hit)
   //  (compute range at closest point and compare to max burst radius)
   //  (use target truth data)
   // ---
   {
      // ---
      // Get position and velocity vectors from the target (truth)
      // (or default to the values from above)
      // ---
      if (tgt != nullptr) {
         calculateVectors(tgt, nullptr, &los, &vel, nullptr);
      }

      // compute range to target
      const double trng0 = trngT;
      trngT = los.length();

      // compute range rate,
      double trdot0 = trdotT;
      if (dt > 0)
         trdotT = (trngT - trng0)/dt;
      else
         trdotT = 0;

      // when we've just passed the target ...
      if (trdotT > 0 && trdot0 < 0 && !isDummy() && getTOF() > 2.0) {
         bool missed = true;   // assume the worst

         // compute relative velocity vector.
         const base::Vec3d velRel = (vel - getVelocity());

         // compute missile velocity squared,
         double vm2 = velRel.length2();
         if (vm2 > 0) {

            // relative range (dot) relative velocity
            const double rdv = los * velRel;

            // interpolate back to closest point
            const double ndt = -rdv/vm2;
            const base::Vec3d p0 = los + (velRel*ndt);

            // range squared at closest point
            const double r2 = p0.length2();

            // compare to burst radius squared
            if (r2 <= (getMaxBurstRng()*getMaxBurstRng()) ) {

               // ---
               // Geometric proximity hit.  Apply the distance-degraded
               // proximity-fuze effectiveness (ADR-007 / T-E37): a missile
               // that has flown past 70% of its maxEffectiveRange has a
               // degraded warhead/fuze and may fail to score even on a clean
               // geometric pass.
               // ---
               const double eff{fuzeEffectiveness(flownDistance, maxEffectiveRange)};

               // Roll U(0,1) from a per-missile deterministic RNG sub-stream.
               // Simulation::split(child_id) is pure in child_id, so a given
               // (master seed, released-weapon id) always yields the same draw
               // -> replications are reproducible while seeds vary the outcome.
               double u{0.0};
               if (WorldModel* wm = getWorldModel()) {
                  if (base::IRng* rng = wm->split(fuzeRngChildId(getID()))) {
                     u = rng->uniform();
                     delete rng;   // caller owns the split() stream (ADR-007)
                  }
               }

               // Detonate as an entity impact only when the fuze is effective.
               // U(0,1) < eff: eff==1.0 always hits (u<1.0); eff==0.0 never
               // hits (u>=0.0); 0<eff<1 hits with probability eff.
               if (u < eff) {

                  // We've detonated
                  missed = false;
                  setMode(Mode::DETONATED);
                  setDetonationResults( Detonation::ENTITY_IMPACT );

                  // compute location of the detonation relative to the target
                  base::Vec3d p0n = -p0;
                  if (tgt != nullptr) p0n = tgt->getRotMat() * p0n;
                  setDetonationLocation(p0n);

                  // Did we hit anyone?
                  checkDetonationEffect();

                  // Log the event
                  const double detRange = getDetonationRange();
                  if (isMessageEnabled(MSG_INFO)) {
                     std::cout << "DETONATE_ENTITY_IMPACT rng = " << detRange
                               << " flown = " << flownDistance
                               << " eff = " << eff << " u = " << u << std::endl;
                  }

                  BEGIN_RECORD_DATA_SAMPLE( getWorldModel()->getDataRecorder(), REID_WEAPON_DETONATION )
                     SAMPLE_3_OBJECTS( this, getLaunchVehicle(), getTargetPlayer() )
                     SAMPLE_2_VALUES( static_cast<int>(Detonation::ENTITY_IMPACT), detRange )
                  END_RECORD_DATA_SAMPLE()

               } else {
                  // Within burst radius but the distance-degraded fuze failed:
                  // leave 'missed' true so the miss path below records a
                  // DETONATION (result=5) and orphans the target.  The geometry
                  // was a hit; the effectiveness roll was not (T-E37).
                  if (isMessageEnabled(MSG_INFO)) {
                     std::cout << "FUZE_INEFFECTIVE (range-degraded) flown = "
                               << flownDistance << " eff = " << eff
                               << " u = " << u << std::endl;
                  }
               }
            }
         }

         // Did we miss the target?
         if (missed) {
            // We've detonated ...
            setMode(Mode::DETONATED);
            setDetonationResults( Detonation::DETONATION );

            // because we've just missed the target
            setTargetPlayer(nullptr,false);
            setTargetTrack(nullptr,false);

            // Log the event
            const double detRange = trngT;
            if (isMessageEnabled(MSG_INFO)) {
               std::cout << "DETONATE_OTHER rng = " << detRange << std::endl;
            }

            BEGIN_RECORD_DATA_SAMPLE( getWorldModel()->getDataRecorder(), REID_WEAPON_DETONATION )
               SAMPLE_3_OBJECTS( this, getLaunchVehicle(), getTargetPlayer() )
               SAMPLE_2_VALUES( static_cast<int>(Detonation::DETONATION), detRange )
            END_RECORD_DATA_SAMPLE()

         }

      }
   }
}

//------------------------------------------------------------------------------
// weaponDynamics -- default missile dynamics; using Robot Aircraft (RAC) dynamics
//------------------------------------------------------------------------------
void Missile::weaponDynamics(const double dt)
{
   static const double g = base::ETHG;              // Acceleration of Gravity

   // ---
   // Max turning G (Missiles: Use Gmax)
   // ---
   const double gmax = maxG;

   // ---
   // Computer max turn rate, max/min pitch rates
   // ---

   // Turn rate base on vp and g,s
   const double ra_max = gmax * g / getTotalVelocity();

   // Set max (pull up) pitch rate same as turn rate
   const double qa_max = ra_max;

   // Set min (push down) pitch rate
   const double qa_min = -qa_max;

   // ---
   // Get old angular values
   // ---
   const base::Vec3d oldRates = getAngularVelocities();
   //double pa1 = oldRates[IROLL];
   const double qa1 = oldRates[IPITCH];
   const double ra1 = oldRates[IYAW];

   // ---
   // Find pitch rate and update pitch
   // ---
   double qa = base::angle::aepcdRad(cmdPitch - static_cast<double>(getPitchR()));
   if(qa > qa_max) qa = qa_max;
   if(qa < qa_min) qa = qa_min;

   // Using Pitch rate, integrate pitch
   const double newTheta = static_cast<double>(getPitch() + (qa + qa1) * dt / 2.0);

   // Find turn rate
   double ra = base::angle::aepcdRad(cmdHeading - static_cast<double>(getHeadingR()));
   if(ra > ra_max) ra = ra_max;
   if(ra < -ra_max) ra = -ra_max;

   // Use turn rate integrate heading
   double newPsi = static_cast<double>(getHeading() + (ra + ra1) * dt / 2.0);
   if(newPsi > 2.0f*base::PI) newPsi -= static_cast<double>(2.0*base::PI);
   if(newPsi < 0.0f) newPsi += static_cast<double>(2.0*base::PI);

   // Roll angle proportional to max turn rate - filtered
   double pa = 0.0;
   const double newPhi = static_cast<double>( 0.98 * getRollR() + 0.02 * ((ra / ra_max) * (base::angle::D2RCC * 60.0)) );

   // Sent angular values
   setEulerAngles(newPhi, newTheta, newPsi);
   setAngularVelocities(pa, qa, ra);

   // Find Acceleration
   double vpdot = (cmdVelocity - getTotalVelocity());
   if(vpdot > maxAccel)  vpdot = maxAccel;
   if(vpdot < -maxAccel) vpdot = -maxAccel;

   // Set acceleration vector
   base::Vec3d aa(vpdot, 0.0, 0.0);
   base::Vec3d ae = aa * getRotMat();
   setAcceleration(ae);

   // Compute new velocity
   const double newVP = getTotalVelocity() + vpdot * dt;

   // Set acceleration vector
   //base::Vec3 ve0 = getVelocity();
   const base::Vec3d va(newVP, 0.0, 0.0);
   const base::Vec3d ve1 = va * getRotMat();
   setVelocity(ve1);
   setVelocityBody(newVP, 0.0, 0.0);

   // ---
   // T-E37: accumulate cumulative path length for distance-degraded Pk.
   // newVP is the speed applied to this frame's translation (BaseClass::
   // dynamics integrates position from this velocity), so newVP*dt is the
   // path increment for the frame.  Used by weaponGuidance()'s proximity-fuze
   // effectiveness term.
   // ---
   if (dt > 0.0) flownDistance += newVP * dt;
}

// setVpMin() -- set min Vp
bool Missile::setVpMin(const double v)
{
   vpMin =  v;
   return true;
}

// setVpMax() -- set max Vp
bool Missile::setVpMax(const double v)
{
   vpMax =  v;
   return true;
}

// setVpMaxG() -- Set Vp with max G's
bool Missile::setVpMaxG(const double v)
{
   vpMaxG =  v;
   return true;
}

// setMaxG() -- Set max G's (g)
bool Missile::setMaxG(const double v)
{
   maxG = v;
   return true;
}

// setMaxAccel() -- Max acceleration (m/s/s)
bool Missile::setMaxAccel(const double v)
{
   maxAccel =  v;
   return true;
}

// setMaxEffectiveRange() -- Max effective range (meters) (T-E37)
bool Missile::setMaxEffectiveRange(const double v)
{
   maxEffectiveRange = v;
   return true;
}

//------------------------------------------------------------------------------
// T-E37 slot functions: maxEffectiveRange
//------------------------------------------------------------------------------
// maxEffectiveRange as a typed length, e.g. ( KiloMeters 60 )
bool Missile::setSlotMaxEffectiveRange(const base::Length* const msg)
{
   bool ok{};
   if (msg != nullptr) {
      ok = setMaxEffectiveRange(msg->getValueInMeters());
   }
   return ok;
}

// maxEffectiveRange as a bare number (meters)
bool Missile::setSlotMaxEffectiveRange(const base::Number* const msg)
{
   bool ok{};
   if (msg != nullptr) {
      ok = setMaxEffectiveRange(msg->asDouble());
   }
   return ok;
}

}
}
