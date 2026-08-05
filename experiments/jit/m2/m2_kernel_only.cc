// M2 specialization-latency probe: the *smallest* translation unit that codegens
// one scene-specialized collision kernel. Timing `clang++ -O3 -march=native -c`
// on this file is the "clang" specialization cost at SCENE granularity -- the bar
// the T1 copy-and-patch / T2 min-pipeline specializers must beat (design doc E2).
// (Contrast the ~5-9 s full-robot FK+CC compile measured in M0.)
#include <vamp/vector.hh>

#include "specialized_kernel.hh"

using DataV = vamp::FloatVector<vamp::FloatVectorWidth>;

// Exported so the optimizer actually emits the specialized code.
extern "C" auto scene_kernel(DataV sx, DataV sy, DataV sz, DataV sr) -> bool
{
    return vamp::m2::scene_in_collision_spec<DataV>(sx, sy, sz, sr);
}
