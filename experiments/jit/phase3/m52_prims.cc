// Idea #2: per-primitive collision math. (a) Which bucket do real MBM obstacles land in
// (general OBB vs z-aligned fast-path)? (b) How much does the fast-path already save, and is
// there a further (AABB) fast-path worth adding? (c) What share of the narrowphase is each type?
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include "mbm_envs.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
using Env = vamp::collision::Environment<DataV>;

static void add_prim(vamp::collision::Environment<float> &ef, const MbmPrim &p)
{
    if (p.kind == 0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px, p.py, p.pz, p.a));
    else if (p.kind == 1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b));
    else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b, p.c));
}

static void bucket(const char *name, const std::vector<MbmEnv> &mbm)
{
    std::size_t sp = 0, cap = 0, zcap = 0, cub = 0, zcub = 0, scenes = mbm.size();
    float worst_axis3z = 1.f;  // closest-to-but-not-1 axis_3_z among general cuboids
    std::size_t gen_could_be_z = 0;  // general cuboids whose axis_3 is ~ (0,0,1)
    for (const auto &me : mbm) {
        vamp::collision::Environment<float> ef;
        for (const auto &p : me.prims) add_prim(ef, p);
        sp += ef.spheres.size(); cap += ef.capsules.size(); zcap += ef.z_aligned_capsules.size();
        cub += ef.cuboids.size(); zcub += ef.z_aligned_cuboids.size();
        for (const auto &c : ef.cuboids) {
            if (std::abs(c.axis_3_x) < 1e-4f && std::abs(c.axis_3_y) < 1e-4f && std::abs(c.axis_3_z - 1.f) < 1e-4f) gen_could_be_z++;
            if (c.axis_3_z < 1.f) worst_axis3z = std::min(worst_axis3z, c.axis_3_z);
        }
    }
    std::printf("%-6s scenes=%2zu | spheres=%3zu  capsule gen/zalign=%zu/%zu  cuboid gen/zalign=%zu/%zu  (general-but-actually-z=%zu, worst axis3z=%.7f)\n",
                name, scenes, sp, cap, zcap, cub, zcub, gen_could_be_z, worst_axis3z);
}

// AABB fast-path candidate: box axis-aligned, half extents (hx,hy,hz) about center. No dots.
static inline DataV sphere_aabb(float cx, float cy, float cz, float hx, float hy, float hz,
                                const DataV &x, const DataV &y, const DataV &z, const DataV &rsq)
{
    auto ax = ((x - DataV::fill(cx)).abs() - DataV::fill(hx)).max(0.f);
    auto ay = ((y - DataV::fill(cy)).abs() - DataV::fill(hy)).max(0.f);
    auto az = ((z - DataV::fill(cz)).abs() - DataV::fill(hz)).max(0.f);
    return vamp::collision::dot_3(ax, ay, az, ax, ay, az) - rsq;
}

template <class Fn>
static double med_ns(std::size_t calls, Fn fn)
{
    std::vector<double> t;
    volatile float sk = 0;
    for (int rp = 0; rp < 11; ++rp) {
        auto a = std::chrono::steady_clock::now();
        float acc = fn();
        auto z = std::chrono::steady_clock::now();
        sk += acc;
        t.push_back(std::chrono::duration<double>(z - a).count() / calls * 1e9);
    }
    (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2];
}

int main()
{
    std::printf("== MBM obstacle bucket distribution ==\n");
    bucket("ur5", ur5_envs); bucket("panda", panda_envs); bucket("fetch", fetch_envs); bucket("baxter", baxter_envs);

    // per-primitive microbench: same yaw-rotated box tested as general-OBB / z-aligned / (if it
    // were) AABB. N sphere positions near the box. This isolates the raw per-call cost of each path.
    std::printf("\n== per-call cost (ns), yaw-rotated box, near queries ==\n");
    const std::size_t N = 20000;
    std::mt19937 rng(1); std::normal_distribution<float> nd(0.f, 0.4f);
    std::vector<DataV> X(N), Y(N), Z(N);
    for (std::size_t i = 0; i < N; ++i) { X[i] = DataV::fill(nd(rng)); Y[i] = DataV::fill(nd(rng)); Z[i] = DataV::fill(nd(rng)); }
    DataV rsq = DataV::fill(0.05f * 0.05f), rr = DataV::fill(0.05f);

    auto boxG = vamp::collision::factory::cuboid::flat(0, 0, 0, 0, 0, 1.2f, 0.2f, 0.3f, 0.15f);  // yaw box -> z-aligned bucket
    vamp::collision::Cuboid<DataV> cg(boxG);  // upcast to vector
    double tG = med_ns(N, [&] { float a = 0; for (std::size_t i = 0; i < N; ++i) a += vamp::collision::sphere_cuboid(cg, X[i], Y[i], Z[i], rsq)[{0, 0}]; return a; });
    double tZ = med_ns(N, [&] { float a = 0; for (std::size_t i = 0; i < N; ++i) a += vamp::collision::sphere_z_aligned_cuboid(cg, X[i], Y[i], Z[i], rsq)[{0, 0}]; return a; });
    double tA = med_ns(N, [&] { float a = 0; for (std::size_t i = 0; i < N; ++i) a += sphere_aabb(0, 0, 0, 0.2f, 0.3f, 0.15f, X[i], Y[i], Z[i], rsq)[{0, 0}]; return a; });

    auto capG = vamp::collision::factory::cylinder::center::flat(0, 0, 0, 0, 0, 1.2f, 0.05f, 0.4f);  // yaw cyl -> z-aligned
    vamp::collision::Capsule<DataV> cc(capG);
    double tcG = med_ns(N, [&] { float a = 0; for (std::size_t i = 0; i < N; ++i) a += vamp::collision::sphere_capsule(cc, X[i], Y[i], Z[i], rr)[{0, 0}]; return a; });
    double tcZ = med_ns(N, [&] { float a = 0; for (std::size_t i = 0; i < N; ++i) a += vamp::collision::sphere_z_aligned_capsule(cc, X[i], Y[i], Z[i], rr)[{0, 0}]; return a; });

    std::printf("cuboid: general=%.2f  z_aligned=%.2f (%.2fx)  AABB=%.2f (%.2fx vs z)\n", tG, tZ, tG / tZ, tA, tZ / tA);
    std::printf("capsule: general=%.2f  z_aligned=%.2f (%.2fx)\n", tcG, tcZ, tcG / tcZ);
    return 0;
}
