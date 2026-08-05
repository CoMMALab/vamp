// M2 scene generator: emit scene_gen.hh (baked constexpr obstacle arrays,
// sorted by min_distance) for a random N-sphere scene. Pure std -- no vamp deps,
// so the "specialize the scene" compile we time later is small.
//
// usage: m2_gen <N> <extent> <seed> <out_header_path>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::fprintf(stderr, "usage: %s <N> <extent> <seed> <out_header>\n", argv[0]);
        return 2;
    }
    const std::size_t n = std::strtoul(argv[1], nullptr, 10);
    const float extent = std::strtof(argv[2], nullptr);
    const unsigned seed = static_cast<unsigned>(std::strtoul(argv[3], nullptr, 10));
    const char *out = argv[4];

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> pos(-extent, extent);
    std::uniform_real_distribution<float> rad(0.02F, 0.08F);

    struct Obs { float x, y, z, r, mind; };
    std::vector<Obs> obs(n);
    for (auto &o : obs)
    {
        o.x = pos(rng);
        o.y = pos(rng);
        o.z = pos(rng);
        o.r = rad(rng);
        o.mind = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z) - o.r;  // Sphere ctor formula
    }
    std::sort(obs.begin(), obs.end(), [](const Obs &a, const Obs &b) { return a.mind < b.mind; });

    std::FILE *f = std::fopen(out, "w");
    if (not f)
    {
        std::fprintf(stderr, "cannot open %s\n", out);
        return 1;
    }
    std::fprintf(f, "#pragma once\n#include <cstddef>\nnamespace scene_gen {\n");
    std::fprintf(f, "constexpr std::size_t N = %zu;\n", n);
    std::fprintf(f, "constexpr float SCENE_EXTENT = %.9gf;\n", extent);
    std::fprintf(f, "constexpr unsigned SCENE_SEED = %u;\n", seed);

    auto emit = [&](const char *name, float Obs::*field)
    {
        std::fprintf(f, "constexpr float %s[N] = {", name);
        for (std::size_t i = 0; i < n; ++i)
        {
            std::fprintf(f, "%s%.9gf", (i ? "," : ""), obs[i].*field);
        }
        std::fprintf(f, "};\n");
    };
    emit("OBS_X", &Obs::x);
    emit("OBS_Y", &Obs::y);
    emit("OBS_Z", &Obs::z);
    emit("OBS_R", &Obs::r);
    emit("OBS_MIND", &Obs::mind);
    std::fprintf(f, "}  // namespace scene_gen\n");
    std::fclose(f);
    return 0;
}
