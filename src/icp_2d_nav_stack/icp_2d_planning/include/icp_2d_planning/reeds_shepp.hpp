#pragma once

#include <vector>
#include <array>
#include <cmath>

namespace icp_planning {
namespace reeds_shepp {

enum class SegType : int8_t { LEFT = 0, STRAIGHT = 1, RIGHT = 2 };
enum class SegDir  : int8_t { FORWARD = 1, REVERSE = -1 };

struct Segment {
    SegType type;
    SegDir  dir;
    double  length;
};

struct RSPath {
    std::array<Segment, 5> segs;
    int    n_segs        {0};
    double total_length  {1e9};
};

// Pose sample with explicit travel direction (needed for direction encoding in output path)
struct SampledPose {
    double x, y, theta;
    SegDir dir;
};

// ── Public API ────────────────────────────────────────────────────────────────

double path_length(double x0, double y0, double t0,
                   double x1, double y1, double t1,
                   double r);

RSPath shortest_path(double x0, double y0, double t0,
                     double x1, double y1, double t1,
                     double r);

// Sample an RS path at spacing ds (metres).
// Each SampledPose carries the direction of the segment it belongs to.
std::vector<SampledPose>
sample_path(const RSPath & path,
            double x0, double y0, double t0,
            double r, double ds = 0.05);

}  // namespace reeds_shepp
}  // namespace icp_planning
