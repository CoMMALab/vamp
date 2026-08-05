// Static-sphere analysis: how many robot spheres are config-independent (depend on
// NO active joint) -> their FK is constant and their scene-collision is constant, so
// they can be hoisted out of the per-config hot loop entirely (checked once/query).
// This is the branch-free special case of the bounding-sphere FK gate, and it directly
// cuts FK for FK-dominated robots ("much of the Fetch is immobile").
//
// A sphere's joint-dependency signature = which joints move it (perturb each joint).
// signature==0 -> fully static. popcount(signature) -> how many joints it depends on
// (few-joint spheres move in a low-dim set -> also tighter reachability / pruning).
//
// CSV: robot,n_spheres,dim,n_static,static_frac,depends_on_1,mean_joints_per_sphere,max_joints
#include <array>
#include <bitset>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;

    template <typename Robot>
    void run(const char *name)
    {
        constexpr std::size_t ns = Robot::n_spheres, dim = Robot::dimension;
        std::mt19937 rng(0x11);
        std::uniform_real_distribution<float> u(0.F, 1.F);
        typename Robot::template Spheres<rake> sph;

        std::vector<std::uint64_t> sig(ns, 0);
        for (int base = 0; base < 6; ++base)
        {
            typename Robot::template ConfigurationBlock<rake> b0;
            for (std::size_t j = 0; j < dim; ++j)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = u(rng);
                b0[j] = DataV(ln.data());
            }
            Robot::template scale_configuration_block<rake>(b0);
            std::vector<std::array<float, 3>> p0(ns);
            Robot::template sphere_fk<rake>(b0, sph);
            for (std::size_t j = 0; j < ns; ++j)
                p0[j] = {sph.x[j].to_array()[0], sph.y[j].to_array()[0], sph.z[j].to_array()[0]};
            for (std::size_t jt = 0; jt < dim && jt < 64; ++jt)
            {
                auto b = b0;
                b[jt] = b[jt] + DataV(0.3F);
                Robot::template sphere_fk<rake>(b, sph);
                for (std::size_t j = 0; j < ns; ++j)
                {
                    float dx = sph.x[j].to_array()[0] - p0[j][0];
                    float dy = sph.y[j].to_array()[0] - p0[j][1];
                    float dz = sph.z[j].to_array()[0] - p0[j][2];
                    if (dx * dx + dy * dy + dz * dz > 1e-8F) sig[j] |= (1ULL << jt);
                }
            }
        }

        std::size_t n_static = 0, dep1 = 0, sumdeps = 0, maxdeps = 0;
        for (std::size_t j = 0; j < ns; ++j)
        {
            std::size_t d = std::bitset<64>(sig[j]).count();
            if (d == 0) ++n_static;
            if (d == 1) ++dep1;
            sumdeps += d;
            if (d > maxdeps) maxdeps = d;
        }
        std::printf("%s,%zu,%zu,%zu,%.4f,%zu,%.2f,%zu\n",
                    name, ns, dim, n_static, static_cast<double>(n_static) / ns, dep1,
                    static_cast<double>(sumdeps) / ns, maxdeps);
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    std::printf("robot,n_spheres,dim,n_static,static_frac,depends_on_1,mean_joints_per_sphere,max_joints\n");
    run<vamp::robots::UR5>("ur5");
    run<vamp::robots::Panda>("panda");
    run<vamp::robots::Baxter>("baxter");
    run<vamp::robots::Fetch>("fetch");
    return 0;
}
