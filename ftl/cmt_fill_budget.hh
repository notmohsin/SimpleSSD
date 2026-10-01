#ifndef __FTL_CMT_FILL_BUDGET__
#define __FTL_CMT_FILL_BUDGET__

#include <algorithm>
#include <cstdint>

namespace SimpleSSD {

namespace FTL {

// How many neighbor LPNs may be installed on one demand miss (excluding the
// demand entry itself).
//
// Random / first-in-window: only use already-free slots after reserving one
// for the demand mapping.  Never evict a full CMT to install unused neighbors.
// Sequential (previous demand LPN in the same translation window): allow up to
// windowSize-1 insertions, which may evict victims — DFTL-style page fill.
inline uint64_t windowFillBudget(uint64_t occupied, uint64_t capacity,
                                 uint64_t windowSize,
                                 bool sequentialWindow) {
  if (capacity <= 1 || windowSize <= 1) {
    return 0;
  }

  const uint64_t maxByWindow = windowSize - 1;

  if (sequentialWindow) {
    return std::min(maxByWindow, capacity - 1);
  }

  if (occupied >= capacity) {
    return 0;
  }

  const uint64_t freeSlots = capacity - occupied;

  if (freeSlots <= 1) {
    return 0;
  }

  return std::min(maxByWindow, freeSlots - 1);
}

}  // namespace FTL

}  // namespace SimpleSSD

#endif
