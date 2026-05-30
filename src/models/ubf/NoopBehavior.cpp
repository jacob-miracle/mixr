
#include "mixr/models/ubf/NoopBehavior.hpp"

#include "mixr/base/ubf/AbstractAction.hpp"
#include "mixr/base/ubf/AbstractState.hpp"

namespace mixr {
namespace models {

IMPLEMENT_SUBCLASS(NoopBehavior, "NoopBehavior")
EMPTY_SLOTTABLE(NoopBehavior)
EMPTY_COPYDATA(NoopBehavior)
EMPTY_DELETEDATA(NoopBehavior)

NoopBehavior::NoopBehavior()
{
   STANDARD_CONSTRUCTOR()
}

base::ubf::AbstractAction* NoopBehavior::genAction(
   const base::ubf::AbstractState* const /*state*/,
   const double /*dt*/)
{
   // Returning nullptr is the explicit "no recommended action" signal in the
   // UBF contract (Agent::controller checks for null before calling
   // action->execute()).
   return nullptr;
}

}
}
