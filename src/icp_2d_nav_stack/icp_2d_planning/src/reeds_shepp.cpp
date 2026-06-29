// Reeds-Shepp curves.
// Reference: Reeds & Shepp (1990), Table 1.  Formulas cross-checked with
// the open-source implementation by Gärber & LaValle.

#include "icp_2d_planning/reeds_shepp.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace icp_planning {
namespace reeds_shepp {

static constexpr double PI     = M_PI;
static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr double INF    = std::numeric_limits<double>::infinity();

static inline double mod2pi(double a)
{
    double v = std::fmod(a, TWO_PI);
    if (v < 0.0) v += TWO_PI;
    return v;
}

static inline void polar(double x, double y, double & r, double & theta)
{
    r     = std::hypot(x, y);
    theta = std::atan2(y, x);
}

static void normalise_input(double x0, double y0, double t0,
                             double x1, double y1, double t1,
                             double r,
                             double & dx, double & dy, double & dt)
{
    double cos0 = std::cos(t0), sin0 = std::sin(t0);
    double rx = x1 - x0, ry = y1 - y0;
    dx = ( rx * cos0 + ry * sin0) / r;
    dy = (-rx * sin0 + ry * cos0) / r;
    dt = std::atan2(std::sin(t1 - t0), std::cos(t1 - t0));
}

// ── Path builders ─────────────────────────────────────────────────────────────

static RSPath make3(SegType t0, SegDir d0, double l0,
                    SegType t1, SegDir d1, double l1,
                    SegType t2, SegDir d2, double l2,
                    double r)
{
    RSPath p;
    p.segs[0] = {t0, d0, std::fabs(l0) * r};
    p.segs[1] = {t1, d1, std::fabs(l1) * r};
    p.segs[2] = {t2, d2, std::fabs(l2) * r};
    p.n_segs  = 3;
    p.total_length = (std::fabs(l0) + std::fabs(l1) + std::fabs(l2)) * r;
    return p;
}

static RSPath make4(SegType t0, SegDir d0, double l0,
                    SegType t1, SegDir d1, double l1,
                    SegType t2, SegDir d2, double l2,
                    SegType t3, SegDir d3, double l3,
                    double r)
{
    RSPath p;
    p.segs[0] = {t0, d0, std::fabs(l0) * r};
    p.segs[1] = {t1, d1, std::fabs(l1) * r};
    p.segs[2] = {t2, d2, std::fabs(l2) * r};
    p.segs[3] = {t3, d3, std::fabs(l3) * r};
    p.n_segs  = 4;
    p.total_length = (std::fabs(l0)+std::fabs(l1)+std::fabs(l2)+std::fabs(l3)) * r;
    return p;
}

using L = SegType;

// ── Word families ─────────────────────────────────────────────────────────────

// Family CSC: L+S+L+, R+S+R+, L+S+R+, R+S+L+
static void CSC(double x, double y, double phi, double /*r*/,
                std::vector<RSPath> & paths)
{
    // LSL: polar() gives the straight-segment length directly (no sqrt needed)
    {
        double u, t;
        polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
        double v = mod2pi(phi - t);
        paths.push_back(make3(L::LEFT, SegDir::FORWARD, t,
                               L::STRAIGHT, SegDir::FORWARD, u,
                               L::LEFT, SegDir::FORWARD, v, 1.0));
    }
    // RSR
    {
        double u, t;
        polar(x + std::sin(phi), y + 1.0 - std::cos(phi), u, t);
        double v = mod2pi(t - phi);
        paths.push_back(make3(L::RIGHT, SegDir::FORWARD, -t,
                               L::STRAIGHT, SegDir::FORWARD, u,
                               L::RIGHT, SegDir::FORWARD, -v, 1.0));
    }
    // LSR
    {
        double u1, t1;
        polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
        double u1sq = u1 * u1 - 4.0;
        if (u1sq >= 0.0) {
            double u = std::sqrt(u1sq);
            double t = mod2pi(t1 - std::atan2(2.0, u));
            double v = mod2pi(t - phi);
            paths.push_back(make3(L::LEFT, SegDir::FORWARD, t,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::RIGHT, SegDir::FORWARD, v, 1.0));
        }
    }
    // RSL
    {
        double u1, t1;
        polar(x - std::sin(phi), y + 1.0 + std::cos(phi), u1, t1);
        double u1sq = u1 * u1 - 4.0;
        if (u1sq >= 0.0) {
            double u = std::sqrt(u1sq);
            double t = mod2pi(t1 + std::atan2(2.0, u));
            double v = mod2pi(phi - t);
            paths.push_back(make3(L::RIGHT, SegDir::FORWARD, -t,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::LEFT, SegDir::FORWARD, v, 1.0));
        }
    }
}

// Family CCC: L+R-L+, L-R+L-, R+L-R+, R-L+R-
static void CCC(double x, double y, double phi,
                std::vector<RSPath> & paths)
{
    // L+R-L+
    {
        double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi);
        double u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.0) {
            double A = std::acos(std::clamp(u1 / 4.0, -1.0, 1.0));
            double t = mod2pi(PI / 2.0 + theta + A);
            double u = mod2pi(PI - 2.0 * A);
            double v = mod2pi(phi - t - u);
            paths.push_back(make3(L::LEFT, SegDir::FORWARD, t,
                                   L::RIGHT, SegDir::REVERSE, -u,
                                   L::LEFT, SegDir::FORWARD, v, 1.0));
        }
    }
    // L-R+L-
    {
        double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi);
        double u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.0) {
            double A = std::acos(std::clamp(u1 / 4.0, -1.0, 1.0));
            double t = mod2pi(PI / 2.0 + theta - A);
            double u = mod2pi(PI - 2.0 * A);
            double v = mod2pi(phi - t + u);
            paths.push_back(make3(L::LEFT, SegDir::REVERSE, -t,
                                   L::RIGHT, SegDir::FORWARD, u,
                                   L::LEFT, SegDir::REVERSE, -v, 1.0));
        }
    }
    // R+L-R+ and R-L+R-
    {
        double xi = x + std::sin(phi), eta = y + 1.0 - std::cos(phi);
        double u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.0 && u1 > 1e-9) {
            double u  = std::acos(std::clamp(1.0 - u1*u1/8.0, -1.0, 1.0));
            double arg = std::clamp(2.0 * std::sin(u) / u1, -1.0, 1.0);
            double A  = std::asin(arg);
            double t  = mod2pi(PI/2.0 - A + theta);
            double v  = mod2pi(t - phi - u);
            paths.push_back(make3(L::RIGHT, SegDir::FORWARD, t,
                                   L::LEFT, SegDir::REVERSE, -u,
                                   L::RIGHT, SegDir::FORWARD, v, 1.0));
            double t2 = mod2pi(PI/2.0 + A + theta);
            double v2 = mod2pi(t2 - phi + u);
            paths.push_back(make3(L::RIGHT, SegDir::REVERSE, -t2,
                                   L::LEFT, SegDir::FORWARD, u,
                                   L::RIGHT, SegDir::REVERSE, -v2, 1.0));
        }
    }
}

// Family CCSC: C|C(π/2)SC  (quarter-arc + straight + curve sandwich)
static void CCSC(double x, double y, double phi,
                 std::vector<RSPath> & paths)
{
    // L+R-(π/2)S+L+
    {
        double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
        double u1, theta;
        polar(xi - 2.0, eta, u1, theta);
        if (u1 >= 2.0) {
            double u = std::sqrt(u1 * u1 - 4.0);
            double A = std::atan2(2.0, u);
            double t = mod2pi(theta + A + PI / 2.0);
            double v = mod2pi(t + PI / 2.0 - phi);
            paths.push_back(make4(L::LEFT, SegDir::FORWARD, t,
                                   L::RIGHT, SegDir::REVERSE, -PI/2.0,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::LEFT, SegDir::FORWARD, v, 1.0));
        }
    }
    // R+L-(π/2)S+R+
    {
        double xi = x - std::sin(phi), eta = y + 1.0 + std::cos(phi);
        double u1, theta;
        polar(xi - 2.0, eta, u1, theta);
        if (u1 >= 2.0) {
            double u = std::sqrt(u1 * u1 - 4.0);
            double A = std::atan2(2.0, u);
            double t = mod2pi(theta + A + PI / 2.0);
            double v = mod2pi(t + PI / 2.0 + phi);
            paths.push_back(make4(L::RIGHT, SegDir::FORWARD, t,
                                   L::LEFT, SegDir::REVERSE, -PI/2.0,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::RIGHT, SegDir::FORWARD, -v, 1.0));
        }
    }
}

// Family CSCC: SC(π/2)|CC  (mirror of CCSC under time-reversal)
static void CSCC(double x, double y, double phi,
                 std::vector<RSPath> & paths)
{
    // S+R+(π/2)L-R+
    {
        double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
        double u1, theta;
        polar(xi - 2.0, eta, u1, theta);
        if (u1 >= 2.0) {
            double u = std::sqrt(u1 * u1 - 4.0);
            double A = std::atan2(2.0, u);
            double t = mod2pi(theta + A + PI / 2.0);
            double v = mod2pi(t - PI / 2.0 - phi);
            paths.push_back(make4(L::LEFT, SegDir::FORWARD, v,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::RIGHT, SegDir::FORWARD, PI/2.0,
                                   L::LEFT, SegDir::REVERSE, -t, 1.0));
        }
    }
    // S+L+(π/2)R-L+
    {
        double xi = x - std::sin(phi), eta = y + 1.0 + std::cos(phi);
        double u1, theta;
        polar(xi - 2.0, eta, u1, theta);
        if (u1 >= 2.0) {
            double u = std::sqrt(u1 * u1 - 4.0);
            double A = std::atan2(2.0, u);
            double t = mod2pi(theta + A + PI / 2.0);
            double v = mod2pi(phi - t + PI / 2.0);
            paths.push_back(make4(L::RIGHT, SegDir::FORWARD, -v,
                                   L::STRAIGHT, SegDir::FORWARD, u,
                                   L::LEFT, SegDir::FORWARD, PI/2.0,
                                   L::RIGHT, SegDir::REVERSE, t, 1.0));
        }
    }
}

// ── Collect all 48 candidates via four symmetry transforms ───────────────────

static void swap_lr(RSPath & p)
{
    for (int i = 0; i < p.n_segs; ++i) {
        if      (p.segs[i].type == SegType::LEFT)  p.segs[i].type = SegType::RIGHT;
        else if (p.segs[i].type == SegType::RIGHT) p.segs[i].type = SegType::LEFT;
    }
}

static void flip_dir(RSPath & p)
{
    for (int i = 0; i < p.n_segs; ++i)
        p.segs[i].dir = (p.segs[i].dir == SegDir::FORWARD) ? SegDir::REVERSE : SegDir::FORWARD;
}

using WordFn = void(*)(double, double, double, std::vector<RSPath>&);

static void apply_words(double x, double y, double phi,
                        std::vector<RSPath> & out)
{
    // CSC takes a 4th arg (r=1); adapt with a lambda stored as a local static
    CSC(x, y, phi, 1.0, out);
    CCC(x, y, phi, out);
    CCSC(x, y, phi, out);
    CSCC(x, y, phi, out);
}

static std::vector<RSPath> all_paths_unit(double x, double y, double phi)
{
    std::vector<RSPath> paths;
    paths.reserve(48);

    // Original
    apply_words(x, y, phi, paths);

    // Time-flip: negate all directions
    {
        std::vector<RSPath> tmp;
        apply_words(-x, y, -phi, tmp);
        for (auto & p : tmp) { flip_dir(p); paths.push_back(p); }
    }

    // Reflect: mirror x-axis, swap L/R
    {
        std::vector<RSPath> tmp;
        apply_words(x, -y, -phi, tmp);
        for (auto & p : tmp) { swap_lr(p); paths.push_back(p); }
    }

    // Time-flip + reflect
    {
        std::vector<RSPath> tmp;
        apply_words(-x, -y, phi, tmp);
        for (auto & p : tmp) { flip_dir(p); swap_lr(p); paths.push_back(p); }
    }

    return paths;
}

// ── Public API ────────────────────────────────────────────────────────────────

double path_length(double x0, double y0, double t0,
                   double x1, double y1, double t1,
                   double r)
{
    double dx, dy, dt;
    normalise_input(x0, y0, t0, x1, y1, t1, r, dx, dy, dt);
    const auto paths = all_paths_unit(dx, dy, dt);
    double best = INF;
    for (const auto & p : paths)
        if (p.total_length * r < best) best = p.total_length * r;
    return best;
}

RSPath shortest_path(double x0, double y0, double t0,
                     double x1, double y1, double t1,
                     double r)
{
    double dx, dy, dt;
    normalise_input(x0, y0, t0, x1, y1, t1, r, dx, dy, dt);
    const auto paths = all_paths_unit(dx, dy, dt);

    RSPath best;
    best.total_length = INF;
    for (const auto & p : paths) {
        double scaled = p.total_length * r;
        if (scaled < best.total_length) {
            best = p;
            best.total_length = scaled;
            for (int i = 0; i < p.n_segs; ++i)
                best.segs[i].length *= r;
        }
    }
    return best;
}

// ── Sample with per-pose direction tracking ───────────────────────────────────
std::vector<SampledPose>
sample_path(const RSPath & path,
            double x0, double y0, double t0,
            double r, double ds)
{
    std::vector<SampledPose> poses;
    poses.push_back({x0, y0, t0, SegDir::FORWARD});

    double x = x0, y = y0, t = t0;

    for (int si = 0; si < path.n_segs; ++si) {
        const Segment & seg = path.segs[si];
        if (seg.length < 1e-6) continue;

        int    steps     = std::max(1, static_cast<int>(seg.length / ds));
        double step      = seg.length / steps;
        double dir_sign  = (seg.dir == SegDir::FORWARD) ? 1.0 : -1.0;

        for (int i = 0; i < steps; ++i) {
            if (seg.type == SegType::STRAIGHT) {
                x += dir_sign * step * std::cos(t);
                y += dir_sign * step * std::sin(t);
            } else {
                double curve_sign = (seg.type == SegType::LEFT) ? 1.0 : -1.0;
                double dtheta = dir_sign * curve_sign * step / r;
                double tm = t + dtheta / 2.0;   // midpoint integration
                x += dir_sign * step * std::cos(tm);
                y += dir_sign * step * std::sin(tm);
                t += dtheta;
                t  = std::atan2(std::sin(t), std::cos(t));
            }
            poses.push_back({x, y, t, seg.dir});
        }
    }
    return poses;
}

}  // namespace reeds_shepp
}  // namespace icp_planning
