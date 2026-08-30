#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <Eigen/Dense>
#include <vamp/vector.hh>

namespace vamp::robots
{
    namespace
    {
        // Pad and align vectors for easy loading.
        alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> lows{-10, -10, 0};
        alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> highs{10, 10, 5};
        static float radius = 0.2;
    }  // namespace

    struct Sphere
    {
        using npy_matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using state_vec = Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor>;
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        static constexpr auto n_spheres = 1;
        static constexpr auto resolution = 32;
        static constexpr std::size_t topple_out_dim = 6;

        static constexpr float &min_radius = radius;
        static constexpr float &max_radius = radius;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        template <std::size_t rake>
        using ConfigurationBlock = FloatVector<rake, 3>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        static constexpr std::array<std::string_view, dimension> joint_names = {"x", "y", "z"};
        static constexpr char *end_effector = "";

        template <std::size_t rake>
        struct Spheres
        {
            FloatVector<rake, 1> x;
            FloatVector<rake, 1> y;
            FloatVector<rake, 1> z;
            FloatVector<rake, 1> r;
        };

        inline static void set_radius(float new_radius) noexcept
        {
            radius = new_radius;
        }

        alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
            20,
            20,
            5};

        alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
            -10,
            -10,
            5};

        inline static void set_lows(std::array<float, 3> new_lows) noexcept
        {
            std::copy_n(new_lows.cbegin(), 3, lows.begin());
        }

        inline static void set_highs(std::array<float, 3> new_highs) noexcept
        {
            std::copy_n(new_highs.cbegin(), 3, highs.begin());
        }

        inline static void scale_configuration(Configuration &q) noexcept
        {
            Configuration clow(lows.data());
            Configuration chigh(highs.data());

            q = q * (chigh - clow) + clow;
        }

        inline static void descale_configuration(Configuration &q) noexcept
        {
            Configuration clow(lows.data());
            Configuration chigh(highs.data());

            q = (q - clow) / (chigh - clow);
        }

        template <std::size_t rake>
        inline static void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
        {
            q[0] = lows[0] + (q[0] * (highs[0] - lows[0]));
            q[1] = lows[1] + (q[1] * (highs[1] - lows[1]));
            q[2] = lows[2] + (q[2] * (highs[2] - lows[2]));
        }

        template <std::size_t rake>
        inline static void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
        {
            q[0] = (q[0] - lows[0]) / (highs[0] - lows[0]);
            q[1] = (q[1] - lows[1]) / (highs[1] - lows[1]);
            q[2] = (q[2] - lows[2]) / (highs[2] - lows[2]);
        }

        inline static auto space_measure() noexcept -> float
        {
            Configuration clow(lows.data());
            Configuration chigh(highs.data());
            return (chigh - clow).l2_norm();
        }

        static inline std::pair<std::vector<npy_matrix>, std::vector<npy_matrix>> load_matrices() noexcept
        {
            std::vector<npy_matrix> a;
            std::vector<npy_matrix> b;
            return {a, b};
        }

        static inline auto topple_nn_forward(std::vector<npy_matrix> weights, std::vector<npy_matrix> bias, std::array<float, 2 * dimension> x) {
            // convert x to eigen
            npy_matrix x_eigen(2 * dimension, 1);
            for (auto i = 0U; i < 2 * dimension; i++) {
                x_eigen(i) = x[i];
            }

            // forward pass
            npy_matrix z = x_eigen;
            for (auto i = 0U; i < weights.size() - 1; i++) {
                z = weights[i] * z + bias[i];
                // activation
                z = z.cwiseMax(0);
            }
            z = weights[weights.size() - 1] * z + bias[weights.size() - 1];

            // convert to array
            std::array<float, 134> y;
            for (int i = 0U; i < 134; i++) {
                y[i] = z(i);
            }
            return y;
        }

        static inline auto reconstruct_control_points(state_vec x0, state_vec x1, state_vec v0, state_vec v1, state_vec a0, state_vec a1, float t) {
            int n = topple_out_dim + 1;
            state_vec x_v_0 = x0 + v0 * t / n;
            state_vec x_a_0 = x_v_0 + (a0 * std::pow(t, 2) / (n - 1) + v0) * t / n;

            state_vec x_v_1 = x1 - v1 * t / n;
            state_vec x_a_1 = x_v_1 + (a1 * std::pow(t, 2) / (n - 1) - v1) * t / n;

            std::vector<state_vec> cp;
            cp.push_back(x_v_0);
            cp.push_back(x_a_0);
            cp.push_back(x_v_1);
            cp.push_back(x_a_1);

            return cp;
        }
        
        template <std::size_t rake>
        static auto fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept
        {
            return not sphere_environment_in_collision(environment, q[0], q[1], q[2], radius);
        }

        using Debug = std::
            pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

        template <std::size_t rake>
        static auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept -> Debug
        {
            Debug output;

            output.first.emplace_back(
                sphere_environment_get_collisions<decltype(q[0])>(environment, q[0], q[1], q[2], radius));

            return output;
        }

        template <std::size_t rake>
        static auto fkcc_attach(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept
        {
            return not sphere_environment_in_collision(environment, q[0], q[1], q[2], radius);
        }

        template <std::size_t rake>
        inline static void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
        {
            out.x[0] = q[0];
            out.y[0] = q[1];
            out.z[0] = q[2];
            out.r[0] = radius;
        }

        static auto eefk(const std::array<float, 3> &q) noexcept -> Eigen::Isometry3f
        {
            auto tf = Eigen::Isometry3f::Identity();
            tf.translation() = Eigen::Vector3f(q[0], q[1], q[2]);
            return tf;
        }

        template <std::size_t rake, typename InputVector, typename OutputVector>
        static inline auto bezier(const InputVector &x, const FloatVector<rake> t, OutputVector &y) noexcept
        {
            return;
        }
    };
}  // namespace vamp::robots
