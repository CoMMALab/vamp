#include <vamp_python_init.hh>

#include <vamp/collision/environment.hh>
#include <vamp/collision/filter.hh>
#include <vamp/collision/capt.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/stats.hh>

#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/ndarray.h>

namespace nb = nanobind;
namespace vc = vamp::collision;
namespace vf = vamp::collision::factory;
using namespace nb::literals;

void vamp::binding::init_environment(nanobind::module_ &pymodule)
{
    nb::class_<vc::Sphere<float>>(pymodule, "Sphere")
        .def(
            "__init__",
            [](vc::Sphere<float> *q, const std::array<float, 3> &center, float radius) noexcept
            { new (q) vc::Sphere<float>(vf::sphere::array(center, radius)); },
            "Constructor from center and radius.")
        .def_static("make_flat", &vf::sphere::flat)
        .def_static("make", &vf::sphere::array)
        .def_ro("x", &vc::Sphere<float>::x)
        .def_ro("y", &vc::Sphere<float>::y)
        .def_ro("z", &vc::Sphere<float>::z)
        .def_ro("r", &vc::Sphere<float>::r)
        .def_prop_ro(
            "position",
            [](vc::Sphere<float> &sphere) { return std::array<float, 3>{sphere.x, sphere.y, sphere.z}; })
        .def_ro("min_distance", &vc::Sphere<float>::min_distance)
        .def_rw("name", &vc::Sphere<float>::name);

    nb::class_<vc::Cylinder<float>>(pymodule, "Cylinder")
        .def(
            "__init__",
            [](vc::Cylinder<float> *q,
               const std::array<float, 3> &center,
               const std::array<float, 3> &euler_xyz,
               float radius,
               float length) noexcept
            { new (q) vc::Cylinder<float>(vf::cylinder::center::array(center, euler_xyz, radius, length)); },
            "Constructor from center, Euler XYZ orientation, radius, and length.")
        .def(
            "__init__",
            [](vc::Cylinder<float> *q,
               const std::array<float, 3> &endpoint1,
               const std::array<float, 3> &endpoint2,
               float radius) noexcept
            { new (q) vc::Cylinder<float>(vf::cylinder::endpoints::array(endpoint1, endpoint2, radius)); },
            "Constructor from endpoints and radius.")
        .def_ro("x1", &vc::Cylinder<float>::x1)
        .def_ro("y1", &vc::Cylinder<float>::y1)
        .def_ro("z1", &vc::Cylinder<float>::z1)
        .def_prop_ro("x2", &vc::Cylinder<float>::x2)
        .def_prop_ro("y2", &vc::Cylinder<float>::y2)
        .def_prop_ro("z2", &vc::Cylinder<float>::z2)
        .def_ro("xv", &vc::Cylinder<float>::xv)
        .def_ro("yv", &vc::Cylinder<float>::yv)
        .def_ro("zv", &vc::Cylinder<float>::zv)
        .def_ro("r", &vc::Cylinder<float>::r)
        .def_ro("rdv", &vc::Cylinder<float>::rdv)
        .def_ro("min_distance", &vc::Cylinder<float>::min_distance)
        .def_rw("name", &vc::Cylinder<float>::name);

    nb::class_<vc::Cuboid<float>>(pymodule, "Cuboid")
        .def(
            "__init__",
            [](vc::Cuboid<float> *q,
               const std::array<float, 3> &center,
               const std::array<float, 3> &euler_xyz,
               const std::array<float, 3> &half_extents) noexcept
            { new (q) vc::Cuboid<float>(vf::cuboid::array(center, euler_xyz, half_extents)); },
            "Constructor from center, Euler XYZ orientation, and XYZ half-extents.")
        .def_ro("x", &vc::Cuboid<float>::x)
        .def_ro("y", &vc::Cuboid<float>::y)
        .def_ro("z", &vc::Cuboid<float>::z)
        .def_ro("axis_1_x", &vc::Cuboid<float>::axis_1_x)
        .def_ro("axis_1_y", &vc::Cuboid<float>::axis_1_y)
        .def_ro("axis_1_z", &vc::Cuboid<float>::axis_1_z)
        .def_ro("axis_2_x", &vc::Cuboid<float>::axis_2_x)
        .def_ro("axis_2_y", &vc::Cuboid<float>::axis_2_y)
        .def_ro("axis_2_z", &vc::Cuboid<float>::axis_2_z)
        .def_ro("axis_3_x", &vc::Cuboid<float>::axis_3_x)
        .def_ro("axis_3_y", &vc::Cuboid<float>::axis_3_y)
        .def_ro("axis_3_z", &vc::Cuboid<float>::axis_3_z)
        .def_ro("axis_1_r", &vc::Cuboid<float>::axis_1_r)
        .def_ro("axis_2_r", &vc::Cuboid<float>::axis_2_r)
        .def_ro("axis_3_r", &vc::Cuboid<float>::axis_3_r)
        .def_ro("min_distance", &vc::Cuboid<float>::min_distance)
        .def_rw("name", &vc::Cuboid<float>::name);

    pymodule.def("make_heightfield", &vf::heightfield::array);

    nb::class_<vc::HeightField<float>>(pymodule, "HeightField")
        .def_ro("x", &vc::HeightField<float>::x)
        .def_ro("y", &vc::HeightField<float>::y)
        .def_ro("z", &vc::HeightField<float>::z)
        .def_ro("xs", &vc::HeightField<float>::xs)
        .def_ro("ys", &vc::HeightField<float>::ys)
        .def_ro("zs", &vc::HeightField<float>::zs)
        .def_ro("data", &vc::HeightField<float>::data);

    nb::class_<vc::Environment<float>>(pymodule, "Environment")
        .def(nb::init<>())
        .def("add_sphere", &vc::Environment<float>::add_sphere)
        .def("add_cuboid", &vc::Environment<float>::add_cuboid)
        .def("add_capsule", &vc::Environment<float>::add_capsule)
        .def("add_heightfield", &vc::Environment<float>::add_heightfield)
        .def(
            "add_pointcloud",
            [](vc::Environment<float> &e,
               const std::vector<collision::Point> &pc,
               float r_min,
               float r_max,
               float r_point)
            {
                auto start_time = std::chrono::steady_clock::now();
                e.pointclouds.emplace_back(pc, r_min, r_max, r_point);
                return vamp::utils::get_elapsed_nanoseconds(start_time);
            })
        .def(
            "attach",
            [](vc::Environment<float> &e, const vc::Attachment<float> &a)
            { e.attachments.emplace_back(a); },
            "Adds an attachment; multiple attachments (across end-effectors) may coexist.")
        .def("detach", [](vc::Environment<float> &e) { e.attachments.clear(); })
        .def_ro("attachments", &vc::Environment<float>::attachments)
        .def_ro("spheres", &vc::Environment<float>::spheres)
        .def_ro("cuboids", &vc::Environment<float>::cuboids)
        .def_ro("z_aligned_cuboids", &vc::Environment<float>::z_aligned_cuboids)
        .def_ro("capsules", &vc::Environment<float>::capsules)
        .def_ro("z_aligned_capsules", &vc::Environment<float>::z_aligned_capsules)
        .def_ro("heightfields", &vc::Environment<float>::heightfields)
        .def_ro("pointclouds", &vc::Environment<float>::pointclouds);

    pymodule.def(
        "filter_pointcloud",
        [](const std::vector<collision::Point> &pc,
           float min_dist,
           float max_range,
           const collision::Point &origin,
           const collision::Point &workcell_min,
           const collision::Point &workcell_max,
           bool cull) -> std::pair<std::vector<collision::Point>, std::size_t>
        {
            auto start_time = std::chrono::steady_clock::now();
            auto filtered =
                vc::filter_pointcloud(pc, min_dist, max_range, origin, workcell_min, workcell_max, cull);
            return {filtered, vamp::utils::get_elapsed_nanoseconds(start_time)};
        });

    pymodule.def(
        "filter_pointcloud",
        [](const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &pc,
           float min_dist,
           float max_range,
           const collision::Point &origin,
           const collision::Point &workcell_min,
           const collision::Point &workcell_max,
           bool cull) -> std::pair<std::vector<collision::Point>, std::size_t>
        {
            auto start_time = std::chrono::steady_clock::now();
            auto filtered =
                vc::filter_pointcloud(pc, min_dist, max_range, origin, workcell_min, workcell_max, cull);
            return {filtered, vamp::utils::get_elapsed_nanoseconds(start_time)};
        });

    nb::class_<vc::Attachment<float>>(pymodule, "Attachment")
        .def(
            "__init__",
            [](vc::Attachment<float> *q, Eigen::Matrix4f &tf, std::size_t end_effector) noexcept
            {
                Eigen::Isometry3f iso;
                iso.matrix() = tf;
                new (q) vc::Attachment<float>(iso);
                q->end_effector = end_effector;
            },
            "tf"_a,
            "end_effector"_a = 0,
            "Constructor for an attachment centered at a relative transform from the given "
            "end-effector (by index into the robot's end-effector list).")
        .def_rw("end_effector", &vc::Attachment<float>::end_effector)
        .def_prop_ro("relative_frame", [](vc::Attachment<float> &a) { return a.tf; })
        .def(
            "add_sphere",
            [](vc::Attachment<float> &a, collision::Sphere<float> &sphere)
            { a.spheres.emplace_back(sphere); })
        .def(
            "add_spheres",
            [](vc::Attachment<float> &a, std::vector<collision::Sphere<float>> &spheres)
            { a.spheres.insert(a.spheres.end(), spheres.cbegin(), spheres.cend()); })
        .def(
            "set_ee_pose",
            [](vc::Attachment<float> &a, Eigen::Matrix4f &tf)
            {
                Eigen::Isometry3f iso;
                iso.matrix() = tf;
                a.pose(iso);
            },
            "tf"_a)
        .def_ro("posed_spheres", &vc::Attachment<float>::posed_spheres);

    nb::class_<vc::CollisionStats>(
        pymodule,
        "CollisionStats",
        "Which robot link hit which obstacle while collision-checking, as a heuristic for what a planner kept running into. "
        "Make one with a robot's `collision_stats(environment)`, run planners, validators or simplifiers inside a `with` block, "
        "and read the matrix after. Only the first collision of a rejected block of samples is counted (the collision kernels "
        "stop there), and a block is one rejected edge. Rows are the robot's links with geometry then \"<attachment>\" (what the "
        "robot holds); columns are the environment's obstacles, in the order of its lists (spheres, capsules, z-aligned "
        "capsules, cuboids, z-aligned cuboids, heightfields, point clouds), then \"<self>\" (the robot hitting itself). Make "
        "the stats after the environment is built: its obstacles are named, and numbered, as they are then.")
        .def(
            "__init__",
            [](vc::CollisionStats *self, std::vector<std::string> link_names, const vc::Environment<float> &environment)
            { new (self) vc::CollisionStats(std::move(link_names), vc::obstacle_names(environment)); },
            "link_names"_a,
            "environment"_a)
        .def(
            "__enter__",
            [](vc::CollisionStats &stats) -> vc::CollisionStats &
            {
                stats.previous = vc::active_stats;
                vc::active_stats = &stats;
                return stats;
            },
            nb::rv_policy::reference,
            "Start counting on this thread.")
        .def(
            "__exit__",
            [](vc::CollisionStats &stats, nb::object, nb::object, nb::object)
            {
                vc::active_stats = stats.previous;
                stats.previous = nullptr;
            },
            "exc_type"_a = nb::none(),
            "exc_value"_a = nb::none(),
            "traceback"_a = nb::none(),
            "Stop counting.")
        .def("reset", &vc::CollisionStats::reset, "Set every count back to zero.")
        .def(
            "matrix",
            [](const vc::CollisionStats &stats)
            {
                const std::size_t rows = stats.link_names.size(), columns = stats.obstacle_names.size();
                auto *data = new std::uint64_t[rows * columns];
                std::copy(stats.counts.begin(), stats.counts.end(), data);
                nb::capsule owner(data, [](void *p) noexcept { delete[] static_cast<std::uint64_t *>(p); });
                return nb::ndarray<nb::numpy, std::uint64_t, nb::ndim<2>>(data, {rows, columns}, owner);
            },
            "The counts: a (links + 1) x (obstacles + 1) array of integers.")
        .def("pairs", &vc::CollisionStats::pairs, "The nonzero counts as (link, obstacle, count), the most counted first.")
        .def_ro("link_names", &vc::CollisionStats::link_names)
        .def_ro("obstacle_names", &vc::CollisionStats::obstacle_names)
        .def_ro("rejected", &vc::CollisionStats::rejected);
}
