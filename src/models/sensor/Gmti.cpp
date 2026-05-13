
#include "mixr/models/sensor/Gmti.hpp"

#include "mixr/models/player/Player.hpp"
#include "mixr/models/system/Antenna.hpp"
#include "mixr/models/system/trackmanager/TrackManager.hpp"
#include "mixr/models/Emission.hpp"

#include "mixr/base/numeric/Integer.hpp"
#include "mixr/base/Pair.hpp"
#include "mixr/base/PairStream.hpp"

#include <cmath>

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(Gmti, "Gmti")
EMPTY_DELETEDATA(Gmti)

BEGIN_SLOTTABLE(Gmti)
    "poi",            // 1: Point-Of-Interest (POI): meters [ north east down ]
END_SLOTTABLE(Gmti)

BEGIN_SLOT_MAP(Gmti)
    ON_SLOT(1, setSlotPoi, base::List)
END_SLOT_MAP()

Gmti::Gmti()
{
    STANDARD_CONSTRUCTOR()
}

void Gmti::copyData(const Gmti& org, const bool)
{
    BaseClass::copyData(org);
    poiVec = org.poiVec;
    // Do NOT copy rng_ — each instance owns its own sub-stream (ADR-007).
    // The new instance starts with no RNG; caller must re-inject via setRng().
    rng_         = nullptr;
    noiseStdRad_ = org.noiseStdRad_;
    lastAzNoise_ = 0.0;
    lastElNoise_ = 0.0;
}

//------------------------------------------------------------------------------
// setRng() -- inject an IRng* sub-stream (ADR-007).
// Takes ownership; deletes any previously held stream.
//------------------------------------------------------------------------------
void Gmti::setRng(base::IRng* rng)
{
    delete rng_;
    rng_ = rng;
}

//------------------------------------------------------------------------------
// setNoiseStdRad() -- set the Gaussian az/el noise standard deviation (radians).
//------------------------------------------------------------------------------
void Gmti::setNoiseStdRad(double stdRad)
{
    noiseStdRad_ = (stdRad >= 0.0) ? stdRad : 0.0;
}

//------------------------------------------------------------------------------
// dynamics() --  Update dynamics
//------------------------------------------------------------------------------
void Gmti::dynamics(const double dt)
{
    BaseClass::dynamics(dt);

    // ---
    // Update the antenna's Reference position
    // ---
    if (getAntenna() != nullptr && getOwnship() != nullptr) {
        // Compute relative vector to POI
        base::Vec3d dpoi {getPoi() - getOwnship()->getPosition()};

        // rotate to ownship heading
        double sinHdg{getOwnship()->getSinHeading()};
        double cosHdg{getOwnship()->getCosHeading()};
        double x{dpoi[models::Player::INORTH] * cosHdg + dpoi[models::Player::IEAST] * sinHdg};
        double y{-dpoi[models::Player::INORTH] * sinHdg + dpoi[models::Player::IEAST] * cosHdg};
        double z{dpoi[models::Player::IDOWN]};

        // Compute az & el to POI
        double grng{std::sqrt(x*x + y*y)};
        double az{std::atan2(y,x)};
        double el{std::atan2(-z,grng)};

        // ---
        // Inject Gaussian azimuth/elevation measurement noise (ADR-007).
        //
        // Draw noise from the injected sub-stream if one is available.
        // When rng_ is nullptr (no stream injected) noise is zero so the
        // component behaves identically to the pre-refactor baseline.
        // ---
        if (rng_ != nullptr && noiseStdRad_ > 0.0) {
            lastAzNoise_ = rng_->normal(0.0, noiseStdRad_);
            lastElNoise_ = rng_->normal(0.0, noiseStdRad_);
        } else {
            lastAzNoise_ = 0.0;
            lastElNoise_ = 0.0;
        }
        az += lastAzNoise_;
        el += lastElNoise_;

        // Get current antenna limits and search volume
        double leftLim{}, rightLim{};
        double lowerLim{}, upperLim{};
        double width{}, height{};
        getAntenna()->getAzimuthLimits(&leftLim, &rightLim);
        getAntenna()->getElevationLimits(&lowerLim, &upperLim);
        getAntenna()->getScanVolume(&width, &height);

        // Limit to within search scan limits of antenna
        if (az < base::angle::aepcdRad(leftLim + width/2.0))
            az = base::angle::aepcdRad(leftLim + width/2.0);
        else if (az > base::angle::aepcdRad(rightLim - width/2.0))
            az = base::angle::aepcdRad(rightLim - width/2.0);

        if (el < base::angle::aepcdRad(lowerLim + height/2.0))
            el = base::angle::aepcdRad(lowerLim + height/2.0);
        else if (el > base::angle::aepcdRad(upperLim - height/2.0))
            el = base::angle::aepcdRad(upperLim - height/2.0);

        // Set the reference 'look' angles
        getAntenna()->setRefAzimuth(az);
        getAntenna()->setRefElevation(el);
    }

    BaseClass::dynamics(dt);
}

//------------------------------------------------------------------------------
// setPoi() -- Gmti Point Of Interest
//------------------------------------------------------------------------------
void Gmti::setPoi(const double x, const double y, const double z)
{
    poiVec.set(x, y, z);
}

void Gmti::setPoi(const base::Vec3d& newPoi)
{
    poiVec = newPoi;
}

//------------------------------------------------------------------------------
// setSlotPoi:  Set Slot POI Vector [ north east down ]
//------------------------------------------------------------------------------
bool Gmti::setSlotPoi(base::List* const numList)
{
    bool ok{};
    double values[3]{};
    const std::size_t n{numList->getNumberList(values, 3)};
    if (n == 3) {
        setPoi(values[0], values[1], values[2]);
        ok = true;
    }
    return ok;
}

}
}
