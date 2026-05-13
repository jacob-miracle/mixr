
// PcgRng.cpp
//
// PcgRng is implemented entirely in PcgRng.hpp (header-only).
// This translation unit exists solely to:
//   1. Provide a compilation unit for the build system to include in libmixr_base.
//   2. Allow future non-inline additions (e.g. out-of-line virtuals, debug hooks)
//      without changing call-site headers.
//
// See: mixr/include/mixr/base/random/PcgRng.hpp

#include "mixr/base/random/PcgRng.hpp"

// Nothing else needed — all methods are inline in the header.
